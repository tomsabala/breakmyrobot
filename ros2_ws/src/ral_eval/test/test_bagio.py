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
The one bag reader, against bags written here.

Everything downstream reads through bagio, and the failures it used to hide
are the ones that look like measurements: a topic whose message type could not
be imported returned the same empty list as a topic that was recorded and
never published, so `recoveries` could be reported as null with the reason "no
action feedback inside the mission window" on a machine that simply lacked
nav2_msgs. Each test below names the bug it catches.
"""

import os

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ral_eval.bagio import read_bag, stamp_of
from ral_interfaces.msg import MissionState

from rclpy.serialization import serialize_message

import rosbag2_py


def _odom(stamp_sec: float = 0.0) -> Odometry:
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.header.stamp.sec = int(stamp_sec)
    msg.header.stamp.nanosec = int(round((stamp_sec - int(stamp_sec)) * 1e9))
    return msg


def _write_bag(path: str, topics: list, messages: list) -> str:
    """topics: [(name, type_name)]; messages: [(name, serialized, log_time_ns)]."""
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                            output_serialization_format='cdr'))
    for name, type_name in topics:
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=name, type=type_name, serialization_format='cdr'))
    for name, data, log_time_ns in messages:
        writer.write(name, data, log_time_ns)
    del writer                                          # finalises metadata.yaml
    return path


def test_stamp_of_prefers_a_set_stamp_and_falls_back_to_the_log_time():
    """An all-zero stamp is UNSET, not sim time 0: _window would drop it."""
    assert abs(stamp_of(_odom(5.5), 99.0) - 5.5) < 1e-9
    # header present but never filled in -> the log time, NOT 0.0, which would
    # be windowed out silently and give path_length a negative dt.
    assert abs(stamp_of(_odom(0.0), 99.0) - 99.0) < 1e-9
    # geometry_msgs/Twist has no header at all.
    assert abs(stamp_of(Twist(), 42.0) - 42.0) < 1e-9
    # MissionState carries its stamp at the TOP level, not in a header; before
    # this it silently got the recorder's time instead of its own.
    state = MissionState()
    state.stamp.sec = 7
    state.stamp.nanosec = 250000000
    assert abs(stamp_of(state, 99.0) - 7.25) < 1e-9


def test_read_bag_distinguishes_absent_silent_and_undecodable_topics(tmp_path):
    """All three used to yield [], so a missing package read as a robot fact."""
    bag = _write_bag(
        str(tmp_path / 'bag'),
        [('/a', 'nav_msgs/msg/Odometry'),
         ('/b', 'no_such_pkg/msg/Nope'),
         ('/c', 'nav_msgs/msg/Odometry')],
        [('/a', serialize_message(_odom(1.0)), 1_000_000_000),
         ('/a', serialize_message(_odom(2.0)), 2_000_000_000)])

    data, status = read_bag(bag, ['/a', '/b', '/c', '/missing'])

    assert len(data['/a']) == 2
    assert status['/a'] == 'read'
    assert status['/c'] == 'read' and data['/c'] == []
    assert status['/b'].startswith('unresolved_type:'), status['/b']
    assert data['/b'] == []
    assert status['/missing'] == 'absent'
    assert len({status['/b'], status['/c'], status['/missing']}) == 3


def test_read_bag_returns_receive_time_in_order_independent_of_header_stamps(tmp_path):
    """
    Freezes the measured baseline: 0 inversions over 75,198 messages.

    A storage plugin that started yielding stamp order would silently turn
    travel into teleport_excluded_m, because path_length's limit scales with
    dt. This fails loudly instead.
    """
    bag = _write_bag(
        str(tmp_path / 'bag'),
        [('/a', 'nav_msgs/msg/Odometry')],
        [('/a', serialize_message(_odom(3.0)), 1_000_000_000),
         ('/a', serialize_message(_odom(1.0)), 2_000_000_000),
         ('/a', serialize_message(_odom(2.0)), 3_000_000_000)])

    data, status = read_bag(bag, ['/a'])

    assert status['/a'] == 'read'
    log_times = [sample[0] for sample in data['/a']]
    assert log_times == sorted(log_times), log_times
    assert [round(t, 6) for t in log_times] == [1.0, 2.0, 3.0]
    # stamp_of substitutes the publisher stamps and does NOT re-sort.
    stamps = [round(stamp_of(msg, log), 6) for log, msg in data['/a']]
    assert stamps == [3.0, 1.0, 2.0], stamps


def test_read_bag_with_no_resolvable_topic_reads_nothing(tmp_path):
    """An empty StorageFilter is "no filter": it would deserialise the whole bag."""
    bag = _write_bag(
        str(tmp_path / 'bag'),
        [('/a', 'nav_msgs/msg/Odometry')],
        [('/a', serialize_message(_odom(1.0)), 1_000_000_000)])

    data, status = read_bag(bag, ['/missing'])

    assert data == {'/missing': []}
    assert status == {'/missing': 'absent'}
    assert os.path.isdir(bag)
