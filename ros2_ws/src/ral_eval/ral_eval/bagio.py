# Copyright 2026 Resilient Autonomy Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
The one bag reader. metrics.py and plot_run.py both go through it.

A second deserialisation path would be a second set of assumptions about
which time a sample carries, and the two artifacts of one run would then be
able to disagree about when something happened.

Time
----
The recorder runs with --use-sim-time, so a message's bag LOG time is already
sim time. `stamp_of` still prefers the message's own stamp when it has one:
the header is stamped by the publisher at the instant the sample describes,
while the log time is stamped when the recorder received it, and for /scan and
/ground_truth/odom those differ by the transport delay. Messages without a
header (geometry_msgs/Twist on /cmd_vel is the one that matters) have only the
log time, and that is stated at the call site in metrics.py rather than hidden
here.

Order
-----
`read_bag` returns samples in the order the storage plugin yields them, which
is RECEIVE-time order, not stamp order. `stamp_of` substitutes publisher
stamps for log times and does NOT restore order afterwards. Measured on
runs/baseline_roomba_20261023T130602Z: 0 inversions over 75,198 messages. Any
consumer whose arithmetic depends on a monotonic series (path_length's dt)
must sort for itself, because a storage-plugin change could alter this
silently.

Missing data
------------
`read_bag` returns a per-topic status beside the samples, because an empty
list on its own cannot distinguish three different facts: the topic is not in
the bag, the topic is there and was never published, or the topic is there and
its message type could not be imported so NOTHING was read. The third case is
live on any machine without nav2_msgs, where
/navigate_to_pose/_action/feedback would otherwise be reported as "no feedback
inside the mission window" - a claim about the robot, drawn from a missing
Python package. None of the three is turned into a zero here: the caller
decides, and in metrics.py the decision is always a null metric with a reason.
"""

import math
import os

from ral_eval.frames import world_pose_to_map

from rclpy.serialization import deserialize_message

import rosbag2_py

from rosidl_runtime_py.utilities import get_message


def _reader(bag_dir: str) -> 'rosbag2_py.SequentialReader':
    if not os.path.isdir(bag_dir):
        raise FileNotFoundError(f'no bag directory at {bag_dir}')
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='', output_serialization_format=''))
    return reader


def bag_topics(bag_dir: str) -> 'dict[str, str]':
    """{topic: type name} as the bag itself records them."""
    reader = _reader(bag_dir)
    table = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    del reader
    return table


def read_bag(bag_dir: str, topics: 'list[str] | None' = None) -> 'tuple[dict, dict]':
    """
    Read a bag into ({topic: [(log_time_s, msg), ...]}, {topic: status}).

    Status is one of:
      'read'                  the topic is in the bag and was deserialised
      'absent'                the bag has no such topic
      'unresolved_type:<err>' the bag has the topic but its message type could
                              not be imported, so its messages were NOT read
    An empty list under a 'read' status means recorded and silent. An empty
    list under any other status means nothing was measured, and the caller
    must say so.

    `topics` restricts the read and guarantees a key for every name asked for,
    so a caller can tell "recorded but silent" from "never requested" without
    a second lookup.
    """
    reader = _reader(bag_dir)
    available = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    wanted = list(topics) if topics is not None else list(available)

    out = {name: [] for name in wanted}
    status = {}

    classes = {}
    for name in wanted:
        type_name = available.get(name)
        if type_name is None:
            status[name] = 'absent'
            continue
        try:
            classes[name] = get_message(type_name)
        except (ImportError, AttributeError, ValueError, ModuleNotFoundError) as exc:
            status[name] = f'unresolved_type:{type(exc).__name__}: {exc}'
            continue
        status[name] = 'read'

    if topics is not None and not classes:
        # An empty StorageFilter is "no filter" in rosbag2 storage, which would
        # deserialise every message in the bag only to discard it.
        del reader
        return out, status

    if topics is not None:
        # set_filter narrows the read at the storage layer; asking for a topic
        # the bag does not contain is not an error, it simply yields nothing.
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(classes)))

    while reader.has_next():
        name, data, log_time_ns = reader.read_next()
        message_class = classes.get(name)
        if message_class is None:
            continue
        out.setdefault(name, []).append(
            (log_time_ns * 1e-9, deserialize_message(data, message_class)))

    del reader
    return out, status


def stamp_of(msg, log_time_s: float) -> float:
    """
    Seconds from the message's own stamp, else the bag log time.

    `header.stamp` first, then a top-level `stamp` (the RAL messages), then
    the log time. An all-zero stamp is treated as UNSET and falls through to
    the log time: sim time 0 is before the simulator started, so a sample
    carrying it would be silently windowed out rather than reported.
    """
    for stamp in (getattr(getattr(msg, 'header', None), 'stamp', None),
                  getattr(msg, 'stamp', None)):
        if stamp is not None and (stamp.sec or stamp.nanosec):
            return stamp.sec + stamp.nanosec * 1e-9
    return log_time_s


def ral_stamp(msg) -> float:
    """
    Sim seconds from a RAL message's own top-level `stamp` field (no header).

    A named alias for `stamp_of`'s second case, kept so a call site states
    which convention it relies on: MissionState and FaultEvent carry `stamp`
    at the top level and have no header at all.
    """
    return msg.stamp.sec + msg.stamp.nanosec * 1e-9


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def truth_track(samples, wtm):
    """Truth as [(stamp, x_map, y_map, yaw_map)], converted once, stamp-sorted."""
    out = []
    for log_time, msg in samples:
        x, y, yaw = world_pose_to_map(msg.pose.pose.position.x,
                                      msg.pose.pose.position.y,
                                      _yaw(msg.pose.pose.orientation), wtm)
        out.append((stamp_of(msg, log_time), x, y, yaw))
    # path_length's dt arithmetic requires a monotonic series; the bag's own
    # order is receive order, which stamp_of does not preserve.
    out.sort(key=lambda sample: sample[0])
    return out
