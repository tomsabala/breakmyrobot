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
The operator dashboard's gateway: ROS on one side, JSON over HTTP on the other.

WHAT THIS IS NOT: a robotics protocol. The browser link is labelled "dashboard
link" everywhere it is shown, because it carries a VIEW MODEL
(DASHBOARD_UI_SPEC.md section 4) and not ROS messages. Every number in that
view model is a transcription of a message this process received, or of a file
under runs/; nothing here computes a robotics quantity, and nothing here
invents one. The encoders transcode and never interpret.

Three things the browser cannot do are why this process exists at all:

  1. launch and stop the stack (`ros2 launch`, as a subprocess in this
     container),
  2. drive a mission through a real `MissionDriver` on a real `MissionNode` -
     the SAME node and the SAME drivers `run_scenario` uses, so an
     operator-issued mission takes the identical path a measured run does,
  3. read `runs/` - those are container paths (`run.json` records
     `/ws/runs/...`).

Threading, fixed. Breaking this is how two spinners end up on one node:

  telemetry  owns TelemetryNode and its SingleThreadedExecutor. The ONLY
             thread that spins TelemetryNode.
  http       ThreadingHTTPServer. Spins nothing; blocks on futures by polling
             `future.done()` at 10 ms.
  worker     one at a time. Owns MissionNode and the MissionDriver, and is the
             ONLY thread that calls rclpy.spin_once(mission_node). Used for
             bring-up/prepare and then for a mission.
  rtf        polls run_scenario.measure_rtf(world) every 10 s.

MissionNode is never added to the telemetry executor.

A dashboard mission writes NO run directory. It is a live operator action, not
a measured experiment; `runs/` carries `run_scenario` output only, and the
console says so in the idle state's detail.
"""

import argparse
import base64
import json
import os
import posixpath
import signal
import subprocess
import sys
import threading
import time
import traceback
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from ral_eval import frames
from ral_eval.grid import grid_from_pgm
from ral_eval.mission_driver import build_driver
from ral_eval.mission_node import MissionNode, goal_pose_stamped, injection_point
from ral_eval.robot_profile import load_profile, profile_path
from ral_eval.run_scenario import (
    CANCEL_WAIT_SIM,
    DISPATCH,
    POST_TERMINAL_SIM,
    STACK_READY_WALL_LIMIT,
    _target_of,
    measure_rtf,
    occupancy_grid_from_pgm,
    preflight_goal,
    read_occupied_thresh,
)
from ral_interfaces.msg import ActiveFaults, FaultEvent, FrameAlignment, MissionState
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

import yaml

# The only authored world. The dashboard offers no world selector for the same
# reason the robot popover shows one row: a selector whose other entries do not
# exist would be a control that cannot be honoured.
WORLD = 'ral_apartment'

# Per-topic minimum wall interval between broadcasts, in seconds. These are
# TRANSPORT limits on streams that are sampled anyway; an event is never
# dropped, because a dropped event is a hole in the record the operator reads
# (R-PF4). A topic absent from this table is never throttled.
RATE_LIMIT_S = {
    'truth': 0.05,          # 20 Hz
    'scan': 0.1,            # the sensor's own 10 Hz
    'scan_raw': 0.1,
    'cmd': 0.1,
    'fb': 0.2,              # 5 Hz
    'clock': 0.2,
    'map': 1.0,
    'costmap': 1.0,
}
NEVER_DROP = ('mission', 'fault_event', 'active', 'bumper', 'align', 'stack',
              'rtf', 'hello', 'dropped', 'reset')

# One client's backlog. On overflow the OLDEST rate-limited frame is dropped
# and a `dropped` frame is sent, so the console can hatch the gap rather than
# interpolate across it (R-TR5).
CLIENT_QUEUE_MAX = 256

# A fault service that is not there after this many WALL seconds did not apply
# the fault. run_scenario waits SERVICE_WAIT_SIM sim seconds; the dashboard has
# no mission clock to wait on when no stack is up, so this is its wall mirror.
SERVICE_WAIT_WALL = 5.0
SERVICE_CALL_WALL = 30.0

# Operator mission default, mirroring the scenario schema's own field name.
DEFAULT_TIMEOUT_SEC = 180.0

# The frames table the Reference screen shows. Prose about real artifacts, each
# naming the file or node that establishes it; no number is stated here, every
# number comes from /api/reference's `world_to_map` and `map` blocks.
FRAMES = [
    {'frame': 'world', 'established_by': 'gz (the SDF world origin)',
     'used_for': 'truth, spawn poses, obstacle and kidnap poses, scenario poses'},
    {'frame': 'map', 'established_by': 'slam_toolbox at mapping start, sealed into '
                                       'maps/apartment_frames.yaml',
     'used_for': 'AMCL, Nav2, /map, the goal as sent; world_to_map below'},
    {'frame': 'ekf_local', 'established_by': "ArduPilot EKF3 at its first GPS fix",
     'used_for': 'the car estimate; measured per run and published on /frame_alignment'},
]


# -- view-model encoders ------------------------------------------------------
# One per message type, each returning exactly the shape DASHBOARD_UI_SPEC.md
# section 4 declares. Pure functions over a message: no node, no clock, no
# state, so the tests can drive them with synthetic messages and a reader can
# see that nothing is computed.
#
# `stamp_sim_s` is always the message's own header stamp, EXCEPT for the four
# unstamped types (Twist, NavControllerOutput, State, Contacts). Those carry
# `stamp_source: 'received'` and the node's sim_now() at arrival, so the UI can
# never present a receive time as a publisher stamp (C1).

def _stamp_s(stamp) -> float:
    """builtin_interfaces/Time as float seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _yaw(orientation) -> float:
    return frames.yaw_of_quaternion(orientation)


def encode_truth(msg) -> dict:
    """nav_msgs/Odometry on /ground_truth/odom: the simulator's own pose."""
    pose = msg.pose.pose
    twist = msg.twist.twist
    # ground_truth_node marks an unknown twist with covariance[0] = -1.0.
    # Reporting 0.0 for it would state the body is stationary (C14).
    known = float(msg.twist.covariance[0]) != -1.0
    return {
        'stamp_sim_s': _stamp_s(msg.header.stamp),
        'frame': msg.header.frame_id,
        'pose': {'x': pose.position.x, 'y': pose.position.y, 'yaw': _yaw(pose.orientation)},
        'twist': {'linear_x': twist.linear.x if known else None,
                  'angular_z': twist.angular.z if known else None,
                  'known': known},
    }


def encode_estimate(msg, label: str, frame: str) -> dict:
    """Encode the robot's own pose estimate, whichever type it arrives in."""
    pose = msg.pose.pose
    covariance = msg.pose.covariance
    # A covariance trace exists only where the estimator publishes one. The
    # car's EKF3 sends no pose covariance at all, so the field is null and the
    # UI says `unavailable` with the profile's reason rather than showing 0.
    if isinstance(msg, PoseWithCovarianceStamped):
        trace = float(covariance[0]) + float(covariance[7]) + float(covariance[35])
    else:
        trace = None
    return {
        'stamp_sim_s': _stamp_s(msg.header.stamp),
        'frame': frame,
        'pose': {'x': pose.position.x, 'y': pose.position.y,
                 'yaw_rad': _yaw(pose.orientation)},
        'covariance_trace': trace,
        'label': label,
    }


def encode_scan(msg) -> dict:
    """
    sensor_msgs/LaserScan, with every no-return made explicit as null.

    A LaserScan reports a no-return as +inf, NaN, or a value outside
    [range_min, range_max] depending on the driver. All of them become null,
    because 0.0 would draw a point at the sensor and a clipped range would
    draw a wall that is not there (C13). Finite in-window ranges pass through
    bit-identical.
    """
    ranges = []
    for value in msg.ranges:
        value = float(value)
        if value != value or value in (float('inf'), float('-inf')):
            ranges.append(None)
        elif value < msg.range_min or value > msg.range_max:
            ranges.append(None)
        else:
            ranges.append(value)
    return {
        'stamp_sim_s': _stamp_s(msg.header.stamp),
        'frame': msg.header.frame_id,
        'angle_min': float(msg.angle_min),
        'angle_increment': float(msg.angle_increment),
        'range_min': float(msg.range_min),
        'range_max': float(msg.range_max),
        'ranges': ranges,
    }


def encode_plan(msg) -> dict:
    """nav_msgs/Path from Nav2's global planner."""
    return {
        'stamp_sim_s': _stamp_s(msg.header.stamp),
        'frame': msg.header.frame_id,
        'poses': [{'x': p.pose.position.x, 'y': p.pose.position.y} for p in msg.poses],
    }


def encode_cmd(msg, stamp_sim_s: float) -> dict:
    """geometry_msgs/Twist: unstamped, so the arrival time is labelled as such."""
    return {
        'stamp_sim_s': stamp_sim_s,
        'stamp_source': 'received',
        'linear_x': msg.linear.x,
        'angular_z': msg.angular.z,
    }


def encode_feedback(msg) -> dict:
    """Pass Nav2's own feedback numbers through; never recompute them."""
    feedback = msg.feedback
    return {
        'stamp_sim_s': _stamp_s(feedback.current_pose.header.stamp),
        'distance_remaining': float(feedback.distance_remaining),
        'number_of_recoveries': int(feedback.number_of_recoveries),
    }


def encode_wp(msg, stamp_sim_s: float) -> dict:
    """
    mavros_msgs/NavControllerOutput: wp_dist is uint16 METRES and truncates.

    The integer is passed through unchanged; the UI states the interval it
    means rather than pretending to a precision the FCU does not report.
    """
    return {'stamp_sim_s': stamp_sim_s, 'stamp_source': 'received',
            'wp_dist': int(msg.wp_dist)}


def encode_mavros_state(msg, stamp_sim_s: float) -> dict:
    """mavros_msgs/State: the autopilot's own link, mode and arming."""
    return {
        'stamp_sim_s': stamp_sim_s,
        'stamp_source': 'received',
        'connected': bool(msg.connected),
        'armed': bool(msg.armed),
        'guided': bool(msg.guided),
        'mode': msg.mode,
        'system_status': int(msg.system_status),
    }


def encode_alignment(msg) -> dict:
    """ral_interfaces/FrameAlignment: the MEASURED world -> ekf_local offset."""
    return {
        'stamp_sim_s': _stamp_s(msg.stamp),
        'parent_frame': msg.parent_frame,
        'child_frame': msg.child_frame,
        'x': msg.x,
        'y': msg.y,
        'yaw': msg.yaw,
        'residual_m': msg.residual_m,
        'residual_yaw_rad': msg.residual_yaw_rad,
        'source': msg.source,
    }


def encode_fault_event(msg) -> dict:
    """ral_interfaces/FaultEvent, verbatim. Every string is the manager's."""
    return {
        'stamp_sim_s': _stamp_s(msg.stamp),
        'id': msg.id,
        'kind': msg.kind,
        'injection_point': msg.injection_point,
        'target': msg.target,
        'state': msg.state,
        'detail': msg.detail,
    }


def encode_active(msg) -> dict:
    """ral_interfaces/ActiveFaults: the manager's authoritative snapshot."""
    return {'stamp_sim_s': _stamp_s(msg.stamp),
            'active': [encode_fault_event(event) for event in msg.active]}


def encode_mission(msg) -> dict:
    """ral_interfaces/MissionState, verbatim including an empty outcome."""
    goal = msg.goal
    return {
        'stamp_sim_s': _stamp_s(msg.stamp),
        'experiment_id': msg.experiment_id,
        'state': msg.state,
        'goal': {'frame_id': goal.header.frame_id,
                 'x': goal.pose.position.x,
                 'y': goal.pose.position.y,
                 'yaw_rad': _yaw(goal.pose.orientation)},
        'outcome': msg.outcome,
        'detail': msg.detail,
    }


def encode_bumper(msg, stamp_sim_s: float) -> dict:
    """ros_gz_interfaces/Contacts: who touched whom, as gz named them."""
    pairs = []
    for contact in msg.contacts:
        pairs.append(f'{contact.collision1.name}|{contact.collision2.name}')
    return {'stamp_sim_s': stamp_sim_s, 'stamp_source': 'received',
            'count': len(msg.contacts), 'pairs': pairs}


def _floats(value):
    """
    Restore the float typing JSON loses, because JSON has ONE number type.

    A browser sends 6.0 as `6`, Python parses it as an int, and rosidl
    refuses it with "The 'x' field must be of type 'float'". The scenario
    path never meets this because YAML keeps 6.0 a float. Every numeric field
    of every fault service is float64 - poses, sizes, forces, noise,
    probabilities - while `enabled` is a bool and `name` a string; bool is
    tested FIRST because in Python it is a subclass of int.
    """
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        return float(value)
    if isinstance(value, dict):
        return {key: _floats(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_floats(item) for item in value]
    return value


def _grid_bytes(data) -> bytes:
    if hasattr(data, 'tobytes'):
        return data.tobytes()
    return bytes(int(value) & 0xFF for value in data)


def encode_grid(msg) -> dict:
    """nav_msgs/OccupancyGrid as base64 int8, so -1 stays unknown, not free."""
    info = msg.info
    return {
        'frame': msg.header.frame_id,
        'resolution': float(info.resolution),
        'width': int(info.width),
        'height': int(info.height),
        'origin': {'x': info.origin.position.x, 'y': info.origin.position.y,
                   'yaw': _yaw(info.origin.orientation)},
        'data_b64': base64.b64encode(_grid_bytes(msg.data)).decode('ascii'),
    }


# -- broadcast ----------------------------------------------------------------

class Broadcaster:
    """
    Fan one view-model frame out to every connected browser.

    Rate limiting is PER TOPIC and applies to sampled streams only; the event
    topics are in NEVER_DROP and always go out. A client whose queue overflows
    loses its oldest droppable frame and is told, so the timeline can show a
    gap instead of a smooth line that was never measured.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._clients = []
        self._last_sent = {}
        self.latest = {}

    def register(self) -> 'ClientQueue':
        client = ClientQueue()
        with self._lock:
            self._clients.append(client)
        return client

    def unregister(self, client) -> None:
        with self._lock:
            if client in self._clients:
                self._clients.remove(client)

    def client_count(self) -> int:
        with self._lock:
            return len(self._clients)

    def reset(self) -> None:
        """
        Forget every per-stack frame; a new stack starts from silence.

        `latest` exists so a console joining mid-run is not blank. After a
        stack stops, those frames describe a robot that is no longer running,
        and replaying them to the next client would show the PREVIOUS robot's
        mission, faults and trails under the new one. Connected clients get a
        `reset` so they drop the same history.
        """
        self.latest = {topic: frame for topic, frame in self.latest.items()
                       if topic == 'hello'}
        self._last_sent = {}
        self.publish('reset', {'reason': 'the stack was stopped; per-stack telemetry, '
                                         'mission state and fault history are cleared'})

    def publish(self, topic: str, payload) -> None:
        now = time.monotonic()
        limit = RATE_LIMIT_S.get(topic)
        if limit is not None and topic not in NEVER_DROP:
            last = self._last_sent.get(topic)
            if last is not None and now - last < limit:
                return
            self._last_sent[topic] = now
        frame = json.dumps({'topic': topic, 'payload': payload},
                           allow_nan=False, default=str)
        self.latest[topic] = frame
        with self._lock:
            clients = list(self._clients)
        for client in clients:
            client.put(topic, frame)


class ClientQueue:
    """One SSE connection's bounded backlog."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._frames = []
        self._dropped = 0
        self.closed = False

    def put(self, topic: str, frame: str) -> None:
        with self._lock:
            if len(self._frames) >= CLIENT_QUEUE_MAX:
                index = next((i for i, (t, _) in enumerate(self._frames)
                              if t not in NEVER_DROP), None)
                if index is None:
                    # Nothing droppable: the backlog is all events, which are
                    # evidence. Keep them and let the socket apply the
                    # pressure rather than silently losing a fault.
                    pass
                else:
                    self._frames.pop(index)
                    self._dropped += 1
            self._frames.append((topic, frame))
            self._event.set()

    def drain(self, timeout: float):
        """Return (frames, dropped_since_last_drain); blocks up to `timeout`."""
        self._event.wait(timeout)
        with self._lock:
            frames_out = [frame for _, frame in self._frames]
            dropped = self._dropped
            self._frames = []
            self._dropped = 0
            self._event.clear()
        return frames_out, dropped


# -- telemetry ----------------------------------------------------------------

class TelemetryNode(Node):
    """
    Subscribes to everything the console draws, and publishes nothing.

    The static subscriptions exist for the gateway's whole life so a console
    connecting before any stack is up still receives the latched
    /mission_state, /faults/active and /fault_events of a stack somebody else
    started. The profile-driven ones are created when a stack starts and
    destroyed when it stops, because their types and QoS come from the robot
    profile and a car's estimate topic has no meaning while a roomba is up.

    Only the telemetry thread spins this node. attach_profile/detach_profile
    are therefore executed inside a one-shot timer callback, so entity
    creation and destruction happen where the executor runs.
    """

    def __init__(self, broadcaster: Broadcaster) -> None:
        super().__init__(
            'ral_dashboard',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.out = broadcaster
        self.profile = None
        self._dynamic = []
        self._pending = []
        self._pending_lock = threading.Lock()
        self.bringup_seen = {}
        self.bringup_at_wall = {}
        self._bringup_t0 = None
        self.bumper_available = None
        self.bumper_reason = ''

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        event_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(Clock, '/clock', self._on_clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/ground_truth/odom', self._on_truth, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_raw', self._on_scan_raw,
                                 qos_profile_sensor_data)
        self.create_subscription(MissionState, '/mission_state', self._on_mission, event_qos)
        self.create_subscription(FaultEvent, '/fault_events', self._on_fault_event, event_qos)
        self.create_subscription(ActiveFaults, '/faults/active', self._on_active, latched)

        # The contact path. ros_gz_interfaces is an exec dependency of this
        # package; without it the contact row states that it is unavailable
        # rather than reporting a zero that was never measured (R-P8.3).
        try:
            from ros_gz_interfaces.msg import Contacts
        except ImportError as exc:
            self.bumper_available = False
            self.bumper_reason = (f'ros_gz_interfaces is not importable here ({exc}), so '
                                  '/bumper cannot be decoded; contact state is unavailable')
        else:
            self.bumper_available = True
            self.create_subscription(Contacts, '/bumper', self._on_bumper,
                                     qos_profile_sensor_data)

        self.create_timer(0.05, self._drain_pending)

    # -- profile-driven subscriptions -----------------------------------------
    def attach_profile(self, profile: dict) -> None:
        """Queue the profile's subscriptions for creation on the spin thread."""
        with self._pending_lock:
            self._pending.append(('attach', profile))

    def detach_profile(self) -> None:
        with self._pending_lock:
            self._pending.append(('detach', None))

    def _drain_pending(self) -> None:
        with self._pending_lock:
            pending, self._pending = self._pending, []
        for action, payload in pending:
            try:
                if action == 'attach':
                    self._attach(payload)
                else:
                    self._detach()
            except Exception:                           # noqa: BLE001 - reported
                traceback.print_exc()

    def _attach(self, profile: dict) -> None:
        self._detach()
        self.profile = profile
        self._bringup_t0 = time.monotonic()
        self.bringup_seen = {entry['topic']: False for entry in profile['bringup_topics']}
        self.bringup_at_wall = {entry['topic']: None for entry in profile['bringup_topics']}

        estimate = profile['estimate']
        # qos_profile_sensor_data (BEST_EFFORT) matches both a reliable and a
        # best-effort publisher. The car's MAVROS odometry is BEST_EFFORT, so a
        # RELIABLE subscriber would receive nothing at all and the estimate
        # would simply never appear.
        self._dynamic.append(self.create_subscription(
            get_message(estimate['type']), estimate['topic'],
            lambda msg: self.out.publish(
                'est', encode_estimate(msg, estimate['label'], estimate['frame'])),
            qos_profile_sensor_data))

        for entry in profile['bringup_topics']:
            topic = entry['topic']
            self._dynamic.append(self.create_subscription(
                get_message(entry['type']), topic,
                lambda _msg, topic=topic: self._on_bringup(topic),
                qos_profile_sensor_data))

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        event_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)

        if profile['command']['kind'] == 'velocity_twist':
            from nav2_msgs.action import NavigateToPose
            self._dynamic.append(self.create_subscription(
                Twist, profile['command']['topic'],
                lambda msg: self.out.publish('cmd', encode_cmd(msg, self.sim_now())), 10))
            self._dynamic.append(self.create_subscription(
                Path, '/plan', lambda msg: self.out.publish('plan', encode_plan(msg)), 10))
            self._dynamic.append(self.create_subscription(
                NavigateToPose.Impl.FeedbackMessage, '/navigate_to_pose/_action/feedback',
                lambda msg: self.out.publish('fb', encode_feedback(msg)), 10))
            self._dynamic.append(self.create_subscription(
                OccupancyGrid, '/map',
                lambda msg: self.out.publish('map', encode_grid(msg)), latched))
            self._dynamic.append(self.create_subscription(
                OccupancyGrid, '/global_costmap/costmap',
                lambda msg: self.out.publish('costmap', encode_grid(msg)), latched))

        if profile['mission_driver'] == 'mavros_guided':
            from mavros_msgs.msg import NavControllerOutput, State
            self._dynamic.append(self.create_subscription(
                State, '/mavros/state',
                lambda msg: self.out.publish(
                    'mavros_state', encode_mavros_state(msg, self.sim_now())), event_qos))
            self._dynamic.append(self.create_subscription(
                NavControllerOutput, '/mavros/nav_controller_output/output',
                lambda msg: self.out.publish('wp', encode_wp(msg, self.sim_now())), 10))
            self._dynamic.append(self.create_subscription(
                FrameAlignment, '/frame_alignment',
                lambda msg: self.out.publish('align', encode_alignment(msg)), latched))

    def _detach(self) -> None:
        for subscription in self._dynamic:
            self.destroy_subscription(subscription)
        self._dynamic = []
        self.profile = None
        self.bringup_seen = {}
        self.bringup_at_wall = {}
        self._bringup_t0 = None

    # -- callbacks ------------------------------------------------------------
    def _on_bringup(self, topic: str) -> None:
        if not self.bringup_seen.get(topic):
            self.bringup_seen[topic] = True
            if self._bringup_t0 is not None:
                self.bringup_at_wall[topic] = time.monotonic() - self._bringup_t0

    def _on_clock(self, msg) -> None:
        self.out.publish('clock', {'sim_s': _stamp_s(msg.clock)})

    def _on_truth(self, msg) -> None:
        self.out.publish('truth', encode_truth(msg))

    def _on_scan(self, msg) -> None:
        self.out.publish('scan', encode_scan(msg))

    def _on_scan_raw(self, msg) -> None:
        self.out.publish('scan_raw', encode_scan(msg))

    def _on_mission(self, msg) -> None:
        self.out.publish('mission', encode_mission(msg))

    def _on_fault_event(self, msg) -> None:
        self.out.publish('fault_event', encode_fault_event(msg))

    def _on_active(self, msg) -> None:
        self.out.publish('active', encode_active(msg))

    def _on_bumper(self, msg) -> None:
        self.out.publish('bumper', encode_bumper(msg, self.sim_now()))

    def sim_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


# -- runs ---------------------------------------------------------------------

def _read_json(path: str):
    with open(path) as handle:
        return json.load(handle)


def _tail(path: str, limit: int = 4096) -> 'str | None':
    try:
        size = os.path.getsize(path)
        with open(path, 'rb') as handle:
            handle.seek(max(0, size - limit))
            return handle.read().decode('utf-8', 'replace')
    except OSError:
        return None


def _bag_manifest(bag_dir: str):
    """(topics, duration_s) from the recorder's own metadata.yaml, or (None, None)."""
    path = os.path.join(bag_dir, 'metadata.yaml')
    try:
        with open(path) as handle:
            info = (yaml.safe_load(handle) or {})['rosbag2_bagfile_information']
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
        return None, None
    topics = []
    for entry in info.get('topics_with_message_count', []):
        meta = entry['topic_metadata']
        topics.append([meta['name'], meta.get('type', ''), int(entry['message_count'])])
    duration = info.get('duration', {})
    nanoseconds = duration.get('nanoseconds') if isinstance(duration, dict) else None
    return topics, (nanoseconds * 1e-9 if nanoseconds is not None else None)


PROV_KEYS = ('image_ref', 'git_sha', 'git_dirty', 'git_diff_sha256', 'map_sha256',
             'scenario_sha256', 'nav2_params_sha256', 'autopilot_parm_sha256',
             'firmware_sha', 'ros_domain_id', 'launch_command')

PARTIAL_NOTE = ('run.json, metrics.json and bag/ absent: scenario invalid or run '
                'directory already existed')


def build_run_entry(run_dir: str) -> dict:
    """
    One runs/ directory as the index entry the inspector lists.

    Never raises on a malformed directory: a run that failed before it could
    write run.json is itself a result, and hiding it would make the index a
    filtered view of the evidence. `partial` says which, `note` says why.
    """
    run_id = os.path.basename(run_dir.rstrip('/'))
    entry = {
        'id': run_id,
        'scenario': run_id.rsplit('_', 1)[0],
        'robot': None, 'world': None, 'seed': None, 'version': None,
        'exit': None, 'state': '', 'outcome': '', 'detail': '',
        'sim': [None, None], 'wall': [None, None],
        'prov': {key: None for key in PROV_KEYS},
        'partial': False, 'note': '', 'scorable': False,
    }
    try:
        scenario_path = os.path.join(run_dir, 'scenario.yaml')
        if os.path.isfile(scenario_path):
            with open(scenario_path) as handle:
                scenario = yaml.safe_load(handle) or {}
            entry['scenario'] = scenario.get('experiment_id', entry['scenario'])
            entry['robot'] = scenario.get('robot')
            entry['world'] = scenario.get('world')
            entry['seed'] = scenario.get('seed')

        env_path = os.path.join(run_dir, 'env.json')
        if os.path.isfile(env_path):
            env = _read_json(env_path)
            entry['robot'] = env.get('robot', entry['robot'])
            entry['world'] = env.get('world', entry['world'])
            entry['seed'] = env.get('seed', entry['seed'])
            for key in PROV_KEYS:
                entry['prov'][key] = env.get(key)
            entry['wall'][0] = env.get('created_utc')

        run_path = os.path.join(run_dir, 'run.json')
        if not os.path.isfile(run_path):
            entry['partial'] = True
            entry['note'] = PARTIAL_NOTE
            return entry
        record = _read_json(run_path)
        entry['exit'] = record.get('exit_code')
        entry['state'] = record.get('mission_state') or ''
        entry['outcome'] = record.get('outcome') or ''
        entry['detail'] = record.get('detail') or ''
        entry['sim'] = [record.get('t_accept_sim'), record.get('t_terminal_sim')]
        entry['wall'] = [record.get('started_utc'), record.get('finished_utc')]
        entry['prov']['world_to_map'] = record.get('world_to_map')
        entry['prov']['world_to_ekf_local'] = record.get('world_to_ekf_local')

        metrics_path = os.path.join(run_dir, 'metrics.json')
        if os.path.isfile(metrics_path):
            meta = (_read_json(metrics_path).get('meta') or {})
            entry['version'] = meta.get('metrics_version')
        # Version 5 is the first that carries the `runner:` outcome prefix and
        # run.json's mission_driver. Earlier runs KEEP their data and are shown;
        # they are simply not scorable against today's vocabulary (R-P10.9).
        entry['scorable'] = bool(entry['version'] is not None and entry['version'] >= 5)
    except Exception as exc:                            # noqa: BLE001 - reported, not hidden
        entry['partial'] = True
        entry['note'] = f'{type(exc).__name__}: {exc}'
    return entry


def _started_key(entry: dict) -> str:
    """
    When a run STARTED, for ordering only.

    run.json's `started_utc` where it exists, and the `_<UTC>Z` suffix the
    runner builds the directory name from otherwise - a run that died before
    writing run.json still has to appear in the right place, or the newest
    evidence of a failure sorts to the bottom.
    """
    started = (entry.get('wall') or [None])[0]
    if started:
        return str(started)
    suffix = entry['id'].rsplit('_', 1)[-1]
    if len(suffix) == 16 and suffix.endswith('Z'):
        return (f'{suffix[0:4]}-{suffix[4:6]}-{suffix[6:8]}T'
                f'{suffix[9:11]}:{suffix[11:13]}:{suffix[13:15]}Z')
    return ''


def build_run_index(runs_dir: str) -> list:
    """Every runs/ subdirectory, newest first."""
    if not os.path.isdir(runs_dir):
        return []
    names = sorted(name for name in os.listdir(runs_dir)
                   if os.path.isdir(os.path.join(runs_dir, name)))
    entries = [build_run_entry(os.path.join(runs_dir, name)) for name in names]
    entries.sort(key=lambda entry: (_started_key(entry), entry['id']), reverse=True)
    return entries


def build_run_detail(runs_dir: str, run_id: str) -> 'dict | None':
    """One run's full payload: the artifacts verbatim, nothing recomputed."""
    run_dir = os.path.join(runs_dir, run_id)
    if not os.path.isdir(run_dir):
        return None
    detail = build_run_entry(run_dir)
    detail['metrics'] = None
    detail['meta'] = None
    detail['interventions'] = []
    detail['preflight'] = None
    detail['preflight_initial'] = None
    detail['bumper_liveness'] = None
    detail['bag'] = None
    detail['bag_duration_s'] = None
    detail['scenario_yaml'] = None
    detail['log_tail'] = _tail(os.path.join(run_dir, 'launch.log'))
    detail['bag_record_log_tail'] = _tail(os.path.join(run_dir, 'bag_record.log'))
    detail['has_plot'] = os.path.isfile(os.path.join(run_dir, 'plot.png'))
    try:
        scenario_path = os.path.join(run_dir, 'scenario.yaml')
        if os.path.isfile(scenario_path):
            with open(scenario_path) as handle:
                detail['scenario_yaml'] = handle.read()
        metrics_path = os.path.join(run_dir, 'metrics.json')
        if os.path.isfile(metrics_path):
            document = _read_json(metrics_path)
            # The WHOLE metric object: value, unit, source, definition and
            # reason. The inspector shows the real source and definition
            # strings, which is the difference between a number and evidence.
            detail['metrics'] = document.get('metrics')
            detail['meta'] = document.get('meta')
        run_path = os.path.join(run_dir, 'run.json')
        if os.path.isfile(run_path):
            record = _read_json(run_path)
            detail['interventions'] = record.get('interventions') or []
            detail['preflight'] = record.get('preflight')
            detail['preflight_initial'] = record.get('preflight_initial')
            detail['bumper_liveness'] = record.get('bumper_liveness')
            detail['rtf_samples'] = record.get('rtf_samples')
            detail['goal_map'] = record.get('goal_map')
            detail['goal_world'] = record.get('goal_world')
            detail['initial_map'] = record.get('initial_map')
            detail['initial_world'] = record.get('initial_world')
        topics, duration = _bag_manifest(os.path.join(run_dir, 'bag'))
        detail['bag'] = topics
        detail['bag_duration_s'] = duration
    except Exception as exc:                            # noqa: BLE001 - reported
        detail['partial'] = True
        detail['note'] = f'{type(exc).__name__}: {exc}'
    return detail


def world_geometry(world: str) -> dict:
    """
    Read the static box models of the world SDF, and the doorways they leave.

    Read from ral_description's installed world rather than copied into the
    browser: the console draws these boxes as WORLD GEOMETRY beside the
    robot's own occupancy grid, and a copy would go stale the first time a
    wall moved, silently drawing an apartment the robot is not in.

    A doorway is DERIVED, not declared: two models named `<stem>_lower` and
    `<stem>_upper` at the same x leave the gap between their facing edges.
    The width is therefore the one the simulation actually has.
    """
    path = os.path.join(get_package_share_directory('ral_description'), 'worlds',
                        f'{world}.sdf')
    tree = ElementTree.parse(path)
    boxes = []
    for model in tree.getroot().iter('model'):
        name = model.get('name')
        pose = model.findtext('pose')
        size = None
        for box in model.iter('box'):
            size = box.findtext('size')
            break
        if not name or not pose or not size:
            continue
        pose_values = [float(value) for value in pose.split()]
        size_values = [float(value) for value in size.split()]
        if len(pose_values) < 6 or len(size_values) < 3:
            continue
        boxes.append({'name': name,
                      'x': pose_values[0], 'y': pose_values[1], 'z': pose_values[2],
                      'yaw': pose_values[5],
                      'sx': size_values[0], 'sy': size_values[1], 'sz': size_values[2]})

    by_name = {box['name']: box for box in boxes}
    doorways = []
    for name, lower in sorted(by_name.items()):
        if not name.endswith('_lower'):
            continue
        upper = by_name.get(f'{name[:-len("_lower")]}_upper')
        if upper is None or abs(upper['x'] - lower['x']) > 1e-9:
            continue
        gap = (upper['y'] - upper['sy'] / 2.0) - (lower['y'] + lower['sy'] / 2.0)
        stem = name[:-len('_lower')]
        doorways.append({
            'id': stem,
            'label': f'doorway {stem.split("_")[-1].upper()}',
            'x': lower['x'],
            'y': (lower['y'] + lower['sy'] / 2.0 + upper['y'] - upper['sy'] / 2.0) / 2.0,
            'width_m': gap,
            'between': [lower['name'], upper['name']],
        })
    return {'source': path, 'boxes': boxes, 'doorways': doorways}


def installed_scenarios() -> list:
    """
    Read the committed scenarios, so the console's presets are the real poses.

    The dashboard does not RUN a scenario - a browser mission writes no run
    directory - but every pose it offers as a preset (the baseline goal, the
    committed kidnap destination, the doorway obstacle) has to be a pose a
    measured run actually used, or the operator is comparing a live mission
    against a scenario it never matched.
    """
    share = os.path.join(get_package_share_directory('ral_eval'), 'scenarios')
    scenarios = []
    for name in sorted(os.listdir(share)):
        if not name.endswith('.yaml'):
            continue
        try:
            with open(os.path.join(share, name)) as handle:
                document = yaml.safe_load(handle) or {}
        except (OSError, yaml.YAMLError):
            continue
        scenarios.append({
            'file': name,
            'experiment_id': document.get('experiment_id'),
            'robot': document.get('robot'),
            'world': document.get('world'),
            'seed': document.get('seed'),
            'timeout_sec': document.get('timeout_sec'),
            'initial_pose': document.get('initial_pose'),
            'goal': document.get('goal'),
            'interventions': document.get('interventions') or [],
        })
    return scenarios


# -- reference ----------------------------------------------------------------

def build_reference(map_yaml: str) -> dict:
    """Everything the UI must not hardcode, read from the installed artifacts."""
    faults_share = get_package_share_directory('ral_faults')
    bringup_share = get_package_share_directory('ral_bringup')
    with open(os.path.join(faults_share, 'config', 'fault_catalogue.yaml')) as handle:
        catalogue = yaml.safe_load(handle)

    profiles = {}
    robots_dir = os.path.join(bringup_share, 'config', 'robots')
    for name in sorted(os.listdir(robots_dir)):
        if not name.endswith('.yaml'):
            continue
        robot = name[:-len('.yaml')]
        profiles[robot] = load_profile(profile_path(bringup_share, robot))

    _, info = grid_from_pgm(map_yaml)
    with open(map_yaml) as handle:
        map_meta = yaml.safe_load(handle) or {}
    wtm = frames.load_alignment_for_map(map_yaml)
    return {
        'world': WORLD,
        'catalogue': catalogue,
        'profiles': profiles,
        'map': {
            'yaml_path': map_yaml,
            'resolution': info['resolution'],
            'width': info['width'],
            'height': info['height'],
            'origin': list(info['origin']),
            'occupied_thresh': map_meta.get('occupied_thresh'),
            'free_thresh': map_meta.get('free_thresh'),
            'mode': map_meta.get('mode'),
            'map_sha256': frames.grid_sha256(map_yaml),
        },
        'world_to_map': {'x': wtm[0], 'y': wtm[1], 'yaw': wtm[2]},
        'alignment_path': frames.frames_path_for_map(map_yaml),
        'world_geometry': world_geometry(WORLD),
        'scenarios': installed_scenarios(),
        'frames': FRAMES,
    }


def committed_grid_payload(map_yaml: str) -> dict:
    """Encode the committed grid the way /map arrives, for the same drawing."""
    grid = occupancy_grid_from_pgm(map_yaml)
    grid.header.frame_id = 'map'
    payload = encode_grid(grid)
    payload['source'] = map_yaml
    payload['map_sha256'] = frames.grid_sha256(map_yaml)
    return payload


# -- the gateway --------------------------------------------------------------

class Gateway:
    """Owns the stack, the mission and the fault clients; one at a time each."""

    def __init__(self, repo_root: str, map_yaml: str) -> None:
        self.repo_root = repo_root
        self.runs_dir = os.path.join(repo_root, 'runs')
        self.map_yaml = map_yaml
        self.out = Broadcaster()
        self.telemetry = TelemetryNode(self.out)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.telemetry)

        self.lock = threading.RLock()
        self.state = 'idle'
        self.robot = None
        self.seed = 1
        self.spawn = None
        self.spawn_map = None
        self.started_wall = None
        self.started_wall_utc = None
        self.wall_limit_s = None
        self.detail = ''
        self.log_path = None
        self.prepared = False
        self.prepare_record = {}
        self.launch = None
        self.launch_log = None
        self.profile = None
        self.mission_node = None
        self.driver = None
        self.worker = None
        self.bringup_expected = []
        self._worker_stop = threading.Event()

        self.mission = None
        self.pending_mission = None
        self.cancel_requested = False

        self._clients = {}
        self._client_lock = threading.Lock()
        self._stop = threading.Event()

    # -- lifecycle ------------------------------------------------------------
    def start_threads(self) -> None:
        threading.Thread(target=self._spin_telemetry, name='telemetry',
                         daemon=True).start()
        threading.Thread(target=self._rtf_loop, name='rtf', daemon=True).start()

    def _spin_telemetry(self) -> None:
        while not self._stop.is_set() and rclpy.ok():
            try:
                self.executor.spin_once(timeout_sec=0.1)
            except Exception:                           # noqa: BLE001 - reported
                traceback.print_exc()
                time.sleep(0.1)

    def _rtf_loop(self) -> None:
        while not self._stop.wait(10.0):
            if self.state not in ('ready', 'starting'):
                continue
            samples = measure_rtf(WORLD)
            self.out.publish('rtf', {'world': WORLD, 'samples': samples,
                                     'source': f'gz topic -e -t /world/{WORLD}/stats'})

    def shutdown(self) -> None:
        self._stop.set()
        if self.state in ('ready', 'failed', 'starting'):
            try:
                self.stop_stack()
            except Exception:                           # noqa: BLE001 - reported
                traceback.print_exc()

    # -- stack state ----------------------------------------------------------
    def stack_payload(self) -> dict:
        # The expected set comes from the PROFILE, not from the telemetry
        # node's flag dict: that dict is created on the telemetry thread when
        # attach_profile is applied, and reading it before then yields an
        # EMPTY set of expectations - a bring-up gate that passes before the
        # stack has published anything at all.
        bringup = []
        for topic in self.bringup_expected:
            bringup.append({'topic': topic,
                            'seen': bool(self.telemetry.bringup_seen.get(topic)),
                            'at_wall_s': self.telemetry.bringup_at_wall.get(topic)})
        elapsed = None
        if self.started_wall is not None:
            elapsed = time.monotonic() - self.started_wall
        return {
            'state': self.state,
            'robot': self.robot,
            'world': WORLD if self.robot else None,
            'seed': self.seed,
            'spawn': self.spawn,
            'started_wall_utc': self.started_wall_utc,
            'wall_elapsed_s': elapsed,
            'wall_limit_s': self.wall_limit_s,
            'bringup': bringup,
            'detail': self.detail,
            'log_path': self.log_path,
            'prepared': self.prepared,
            'prepare_record': self.prepare_record,
            'mission_active': self.mission is not None,
            'bumper_available': self.telemetry.bumper_available,
            'bumper_reason': self.telemetry.bumper_reason,
        }

    def _set_state(self, state: str, detail: str = None) -> None:
        self.state = state
        if detail is not None:
            self.detail = detail
        self.out.publish('stack', self.stack_payload())

    def broadcast_stack(self) -> None:
        self.out.publish('stack', self.stack_payload())

    # -- start ----------------------------------------------------------------
    def start_stack(self, body: dict) -> dict:
        with self.lock:
            if self.state != 'idle':
                return {'accepted': False,
                        'message': f'the stack is {self.state}; stop it before starting '
                                   'another'}
            robot = body.get('robot') or 'roomba'
            seed = int(body.get('seed', 1))
            bringup_share = get_package_share_directory('ral_bringup')
            try:
                profile = load_profile(profile_path(bringup_share, robot))
            except (ValueError, OSError) as exc:
                return {'accepted': False, 'message': f'robot profile rejected: {exc}'}

            spawn = body.get('spawn') or {}
            x = float(spawn.get('x', profile['spawn']['x']))
            y = float(spawn.get('y', profile['spawn']['y']))
            yaw = float(spawn.get('yaw', profile['spawn']['yaw']))

            # A second stack over somebody else's /clock would drive the same
            # robot from two processes. The graph is the only honest check.
            publishers = self.telemetry.count_publishers('/clock')
            if publishers > 0:
                domain = os.environ.get('ROS_DOMAIN_ID', '0')
                return {'accepted': False,
                        'message': f'a /clock publisher already exists in ROS_DOMAIN_ID '
                                   f'{domain}; the dashboard refuses to start a second '
                                   'stack over it'}

            # The spawn pose gets the runner's own footprint check, before
            # anything is launched: a body standing in a wall produces an
            # estimate error with no explanation (C8).
            try:
                wtm = frames.load_alignment_for_map(self.map_yaml)
                occupied_thresh = read_occupied_thresh(self.map_yaml)
            except Exception as exc:                    # noqa: BLE001 - reported
                return {'accepted': False, 'message': f'map/alignment rejected: {exc}'}
            spawn_map = frames.world_pose_to_map(x, y, yaw, wtm)
            grid = occupancy_grid_from_pgm(self.map_yaml)
            preflight = preflight_goal(grid, spawn_map,
                                       float(profile['footprint']['radius_m']),
                                       occupied_thresh)
            if not preflight['ok']:
                return {'accepted': False,
                        'outcome': 'runner:not_sent:initial_pose_occupied',
                        'message': preflight['detail']}

            logs_dir = os.path.join(self.repo_root, 'logs')
            os.makedirs(logs_dir, exist_ok=True)
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
            self.log_path = os.path.join(logs_dir, f'stack_{robot}_{stamp}.log')
            # The runner's argv, verbatim: the dashboard must launch the same
            # stack a measured run launches, or what the operator watches is
            # not what the harness measures.
            command = ['ros2', 'launch', 'ral_bringup', profile['stack_launch'],
                       f'world:={WORLD}', f'robot:={robot}', f'seed:={seed}',
                       f'spawn_x:={x}', f'spawn_y:={y}', f'spawn_yaw:={yaw}']
            self.launch_log = open(self.log_path, 'w')
            self.launch = subprocess.Popen(command, stdout=self.launch_log,
                                           stderr=subprocess.STDOUT,
                                           start_new_session=True)
            self.profile = profile
            self.robot = robot
            self.seed = seed
            self.spawn = {'x': x, 'y': y, 'yaw': yaw}
            self.spawn_map = list(spawn_map)
            self.prepared = False
            self.prepare_record = {}
            self.started_wall = time.monotonic()
            self.started_wall_utc = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
            self.wall_limit_s = float(profile['bringup_wall_limit_s'])
            self.bringup_expected = [entry['topic'] for entry in profile['bringup_topics']]
            self.telemetry.attach_profile(profile)
            self._set_state('starting',
                            f'launched {" ".join(command)}; see {self.log_path}')
            self._worker_stop.clear()
            self.pending_mission = None
            self.worker = threading.Thread(target=self._bringup_worker, name='worker',
                                           daemon=True)
            self.worker.start()
            return {'accepted': True, 'message': 'stack starting',
                    'launch_command': command, 'log_path': self.log_path,
                    'preflight': preflight}

    def _bringup_worker(self) -> None:
        try:
            profile = self.profile
            limit = float(profile['bringup_wall_limit_s'])
            deadline = time.monotonic() + limit
            missing = list(self.bringup_expected)
            while time.monotonic() < deadline:
                time.sleep(0.25)
                self.broadcast_stack()
                missing = [topic for topic in self.bringup_expected
                           if not (self.telemetry.bringup_seen.get(topic)
                                   or (topic == '/clock'
                                       and self.telemetry.sim_now() > 0.0))]
                if not missing:
                    break
            if missing:
                self._set_state('failed',
                                f'no messages on {", ".join(missing)} within {limit:.0f} s '
                                f'wall; see {self.log_path}')
                return

            # The same node and the same driver a measured run uses.
            self.mission_node = MissionNode('-', profile, frames.load_alignment_for_map(
                self.map_yaml), node_name='ral_dashboard_mission')
            self.prepare_record = {}
            self.driver = build_driver(self.mission_node, profile, self.prepare_record)
            not_ready = self.driver.wait_ready(STACK_READY_WALL_LIMIT)
            if not_ready:
                self._set_state('failed', not_ready)
                return

            if profile['mission_driver'] == 'nav2':
                # Nav2Driver.prepare republishes /initialpose and warms AMCL up
                # with an in-place rotation. Running it before a SECOND mission
                # would reset the filter to the spawn pose after the robot has
                # moved, so it runs ONCE per stack. The autopilot's prepare is
                # mode+arm and runs before every mission instead, because its
                # stop() disarms.
                refusal = self.driver.prepare(self.spawn_map, self.prepare_record)
                if refusal:
                    outcome, detail = refusal
                    self._set_state('failed', f'{outcome}: {detail}')
                    return
            self.prepared = True
            self._set_state('ready', 'stack up and prepared to accept a goal')
            self._service_loop()
        except Exception:                               # noqa: BLE001 - reported
            traceback.print_exc()
            self._set_state('failed', traceback.format_exc())

    def _service_loop(self) -> None:
        """
        Spin MissionNode for as long as the stack is up, and run missions on it.

        This loop exists because a node that is NOT spun stops receiving
        /clock: its sim clock freezes at the last message it processed and
        then jumps forward on the next spin. Every sim-time deadline in the
        drivers - the goal-acceptance wait, the cancel wait, spin_sim itself -
        measures `sim_now() - start`, so one jump expires all of them at once
        and a perfectly healthy goal is reported as "no response". MEASURED:
        a goal sent 60 s after prepare was refused in 1 s wall with
        runner:not_accepted.

        It is also the reason a mission runs HERE rather than on a thread of
        its own: this is the one thread allowed to spin MissionNode, and two
        spinners on one node is the bug this whole structure exists to avoid.
        """
        while not self._worker_stop.is_set() and rclpy.ok():
            pending = self.pending_mission
            if pending is None:
                rclpy.spin_once(self.mission_node, timeout_sec=0.05)
                continue
            self.pending_mission = None
            try:
                self._run_mission(**pending)
            except Exception:                           # noqa: BLE001 - reported
                traceback.print_exc()
            finally:
                self.mission = None
                self.cancel_requested = False
                self.broadcast_stack()

    # -- stop -----------------------------------------------------------------
    def stop_stack(self) -> dict:
        with self.lock:
            if self.state == 'idle':
                return {'accepted': False, 'message': 'no stack is running'}
            if self.mission is not None:
                return {'accepted': False,
                        'message': 'a mission is running; cancel it before stopping '
                                   'the stack'}
            self._set_state('stopping', 'stopping the stack')
        try:
            # The service loop owns MissionNode. It has to be out of the node
            # before the node is destroyed, or spin_once runs against a
            # destroyed handle.
            self._worker_stop.set()
            if self.worker is not None and self.worker.is_alive():
                self.worker.join(timeout=15)
            if self.driver is not None:
                try:
                    self.driver.stop()
                    self.driver.teardown()
                except Exception:                       # noqa: BLE001 - reported
                    traceback.print_exc()
            if self.mission_node is not None:
                self.mission_node.destroy_node()
            self.telemetry.detach_profile()
            if self.launch is not None:
                try:
                    os.killpg(os.getpgid(self.launch.pid), signal.SIGINT)
                    self.launch.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(self.launch.pid), signal.SIGKILL)
                    self.launch.wait(timeout=10)
                except ProcessLookupError:
                    pass
            if self.launch_log is not None:
                self.launch_log.close()
        finally:
            with self.lock:
                self.driver = None
                self.mission_node = None
                self.launch = None
                self.launch_log = None
                self.profile = None
                self.robot = None
                self.spawn = None
                self.started_wall = None
                self.started_wall_utc = None
                self.wall_limit_s = None
                self.prepared = False
                self.bringup_expected = []
                self.out.reset()
                self._set_state('idle', 'no stack running')
        return {'accepted': True, 'message': 'stack stopped'}

    # -- mission --------------------------------------------------------------
    def start_mission(self, body: dict) -> dict:
        with self.lock:
            if self.state != 'ready':
                return {'accepted': False,
                        'message': f'the stack is {self.state}; a goal can only be sent '
                                   'to a stack that is ready'}
            if self.mission is not None:
                return {'accepted': False, 'message': 'a mission is already running'}
            goal = body.get('goal') or {}
            try:
                x = float(goal['x'])
                y = float(goal['y'])
                yaw = float(goal.get('yaw', 0.0))
            except (KeyError, TypeError, ValueError) as exc:
                return {'accepted': False,
                        'message': f'goal must carry finite x and y in the gz world '
                                   f'frame ({exc})'}
            timeout_sec = float(body.get('timeout_sec', DEFAULT_TIMEOUT_SEC))
            experiment_id = (f'ui_{self.robot}_'
                             f'{datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")}')
            self.cancel_requested = False
            self.mission = {'experiment_id': experiment_id,
                            'goal_world': [x, y, yaw], 'timeout_sec': timeout_sec}
            # Handed to the thread that is already spinning MissionNode; see
            # _service_loop for why no second thread is started here.
            self.pending_mission = {'x': x, 'y': y, 'yaw': yaw,
                                    'timeout_sec': timeout_sec,
                                    'experiment_id': experiment_id}
            self.broadcast_stack()
            return {'accepted': True, 'experiment_id': experiment_id,
                    'message': 'goal pre-flighted and sent through the mission driver; '
                               'watch /mission_state for what the stack did with it'}

    def cancel_mission(self) -> dict:
        if self.mission is None:
            return {'accepted': False, 'message': 'no mission is running'}
        self.cancel_requested = True
        return {'accepted': True,
                'message': 'cancel requested; a cancel is terminal only when the stack '
                           'reports one'}

    def _run_mission(self, x, y, yaw, timeout_sec, experiment_id) -> None:
        """
        One operator mission, on the thread that already spins MissionNode.

        The body below is the runner's mission loop minus the interventions
        (an operator injects faults by hand) and plus an operator cancel. The
        terminal resolution, the wordings and the wall deadlock guard are the
        runner's, so a mission driven from the browser ends the same way the
        same mission ends under run_scenario.
        """
        node = self.mission_node
        driver = self.driver
        profile = self.profile
        wtm = node.wtm
        goal_map = frames.world_pose_to_map(x, y, yaw, wtm)
        node.experiment_id = experiment_id
        goal_pose = goal_pose_stamped(node, goal_map)

        node.publish_mission_state(
            MissionState.STATE_IDLE, goal_pose,
            detail='stack up, goal not yet sent; `goal` is the goal that will be '
                   'sent; this mission is driven by the dashboard and is NOT '
                   'recorded as a run')

        grid = node.grid if node.grid is not None else occupancy_grid_from_pgm(
            self.map_yaml)
        occupied_thresh = read_occupied_thresh(self.map_yaml)
        preflight = preflight_goal(grid, goal_map,
                                   float(profile['footprint']['radius_m']),
                                   occupied_thresh)
        if not preflight['ok']:
            # The ONLY path to `unreachable`: a pre-flight refusal, never an
            # inference from a stack abort (C8).
            node.publish_mission_state(
                MissionState.STATE_UNREACHABLE, goal_pose,
                outcome=f'runner:{preflight["outcome"]}', detail=preflight['detail'])
            return

        if profile['mission_driver'] == 'mavros_guided':
            # stop() disarms this platform, so GUIDED + arm is per mission.
            refusal = driver.prepare(self.spawn_map, self.prepare_record)
            if refusal:
                outcome, detail = refusal
                node.publish_mission_state(MissionState.STATE_REJECTED, goal_pose,
                                           outcome=outcome, detail=detail)
                return

        record = dict(self.prepare_record)
        accepted, outcome, detail = driver.send_goal(goal_pose, record)
        if not accepted:
            node.publish_mission_state(MissionState.STATE_REJECTED, goal_pose,
                                       outcome=outcome, detail=detail)
            return

        t_accept = node.sim_now()
        node.publish_mission_state(MissionState.STATE_GOAL_ACCEPTED, goal_pose,
                                   detail=detail)
        navigating_published = False
        canceled = False
        cancel_deadline = None
        stalled = False
        # Wall deadlock guard ONLY, as in the runner: ten times the sim budget
        # plus two minutes, so it can fire only when /clock stopped advancing.
        wall_deadline = time.monotonic() + timeout_sec * 10.0 + 120.0
        last_sim = node.sim_now()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            driver.spin_once()
            if driver.terminal() is not None:
                break
            if not navigating_published and driver.navigating_since is not None:
                navigating_published = True
                node.publish_mission_state(
                    MissionState.STATE_NAVIGATING, goal_pose,
                    detail='the stack was observed executing the goal at sim '
                           f'{driver.navigating_since:.2f} s')
            elapsed = node.sim_now() - t_accept
            last_sim = node.sim_now()
            if not canceled and (elapsed >= timeout_sec or self.cancel_requested):
                reason = 'operator' if self.cancel_requested else 'timeout'
                driver.request_cancel(reason)
                canceled = True
                cancel_deadline = node.sim_now() + CANCEL_WAIT_SIM
            if canceled and node.sim_now() > cancel_deadline:
                break
            if time.monotonic() > wall_deadline:
                stalled = True
                break

        terminal = driver.terminal()
        if terminal is not None:
            state, outcome, detail = terminal
        elif stalled:
            state = MissionState.STATE_UNKNOWN
            outcome = 'runner:unknown:clock_stalled'
            detail = (f'{driver.no_terminal_detail()}; /clock stopped advancing: last '
                      f'sim time seen {last_sim:.2f} s after '
                      f'{timeout_sec * 10.0 + 120.0:.0f} s wall, and the stack was '
                      f'{"asked to stop" if canceled else "not asked to stop"}')
        else:
            state = MissionState.STATE_UNKNOWN
            outcome = 'runner:unknown:no_terminal_status'
            detail = (f'{driver.no_terminal_detail()}; no terminal result arrived '
                      + (f'within {CANCEL_WAIT_SIM:.0f} s sim of the cancel'
                         if canceled else 'and the mission loop ended'))
        node.publish_mission_state(state, goal_pose, outcome=outcome, detail=detail)
        node.spin_sim(POST_TERMINAL_SIM)
        driver.stop()

    # -- faults ---------------------------------------------------------------
    def _client_for(self, service_name: str, service_type):
        with self._client_lock:
            client = self._clients.get(service_name)
            if client is None:
                client = self.telemetry.create_client(service_type, service_name)
                self._clients[service_name] = client
            return client

    def call_fault(self, kind: str, params: dict) -> dict:
        """
        Proxy one operator fault request to the fault manager, verbatim.

        The request object is built by the RUNNER's own builders, so the
        dashboard cannot ask for something a scenario could not. `fault_id` is
        always empty: the manager mints `<kind>_ui_<n>` and answers with it.
        No capability gate, no bounds check and no refusal wording lives here -
        all three belong to the manager, which is the thing that would have to
        apply the fault.
        """
        if kind == 'clear_all':
            service_name, service_type = '/faults/clear_all', Trigger
            request = Trigger.Request()
            target = ''
        else:
            if kind not in DISPATCH:
                return {'fault_id': '', 'success': False,
                        'message': f'{kind!r} is not a fault kind; known kinds are '
                                   f'{sorted(DISPATCH)}'}
            service_name, service_type, build_request = DISPATCH[kind]
            try:
                request = build_request(_floats(params), '')
            except (AssertionError, KeyError, TypeError, ValueError) as exc:
                return {'fault_id': '', 'success': False,
                        'message': f'{kind} request could not be built from the given '
                                   f'parameters ({type(exc).__name__}: {exc}); the field '
                                   'names are the scenario schema\'s'}
            model = self.profile['model'] if self.profile else ''
            try:
                target = _target_of(kind, params, model)
            except KeyError:
                target = model

        client = self._client_for(service_name, service_type)
        deadline = time.monotonic() + SERVICE_WAIT_WALL
        while not client.service_is_ready() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not client.service_is_ready():
            message = (f'{service_name} not available after {SERVICE_WAIT_WALL:.1f} s; '
                       'the fault manager is not running, so the fault was NOT applied')
            # The one FaultEvent the non-manager side is entitled to publish:
            # the manager cannot report a service that was never there.
            if self.mission_node is not None and kind != 'clear_all':
                self.mission_node.publish_fault_rejected('', kind, WORLD, target, message)
            return {'fault_id': '', 'success': False, 'message': message}

        future = client.call_async(request)
        call_deadline = time.monotonic() + SERVICE_CALL_WALL
        while not future.done() and time.monotonic() < call_deadline:
            time.sleep(0.01)
        if not future.done():
            message = (f'{service_name} did not answer within {SERVICE_CALL_WALL:.1f} s; '
                       'the fault cannot be shown to have been applied')
            if self.mission_node is not None and kind != 'clear_all':
                self.mission_node.publish_fault_rejected('', kind, WORLD, target, message)
            return {'fault_id': '', 'success': False, 'message': message}

        response = future.result()
        return {'fault_id': getattr(response, 'fault_id', ''),
                'success': bool(response.success),
                'message': response.message}


# -- HTTP ---------------------------------------------------------------------

ALLOWED_ORIGINS = ('http://localhost:5173', 'http://127.0.0.1:5173')

CONTENT_TYPES = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript',
                 '.css': 'text/css', '.json': 'application/json', '.svg': 'image/svg+xml',
                 '.png': 'image/png', '.ico': 'image/x-icon', '.woff': 'font/woff',
                 '.woff2': 'font/woff2', '.map': 'application/json'}

NO_BUILD = ('the dashboard has not been built: run `npm install && npm run build` in '
            'dashboard/, or use the Vite dev server on http://localhost:5173')


class Handler(BaseHTTPRequestHandler):
    """Routes. Every answer is JSON except the static bundle and plot.png."""

    protocol_version = 'HTTP/1.1'
    gateway = None
    dist_dir = None

    def log_message(self, fmt, *args):                  # noqa: A003 - BaseHTTPRequestHandler
        pass

    # -- helpers --------------------------------------------------------------
    def _cors(self) -> None:
        origin = self.headers.get('Origin')
        if origin in ALLOWED_ORIGINS:
            self.send_header('Access-Control-Allow-Origin', origin)
            self.send_header('Access-Control-Allow-Headers', 'Content-Type')
            self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')

    def _json(self, payload, status: int = 200) -> None:
        body = json.dumps(payload, allow_nan=False, default=str).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def _bytes(self, body: bytes, content_type: str, status: int = 200) -> None:
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def _body(self) -> dict:
        length = int(self.headers.get('Content-Length') or 0)
        if not length:
            return {}
        raw = self.rfile.read(length)
        try:
            return json.loads(raw.decode())
        except ValueError as exc:
            raise ValueError(f'request body is not JSON: {exc}') from exc

    # -- verbs ----------------------------------------------------------------
    def do_OPTIONS(self):                               # noqa: N802 - http.server API
        self.send_response(204)
        self._cors()
        self.send_header('Content-Length', '0')
        self.end_headers()

    def do_GET(self):                                   # noqa: N802 - http.server API
        path = urlparse(self.path).path
        gateway = self.gateway
        try:
            if path == '/api/stream':
                return self._stream()
            if path == '/api/reference':
                return self._json(build_reference(gateway.map_yaml))
            if path == '/api/map':
                return self._json(committed_grid_payload(gateway.map_yaml))
            if path == '/api/stack':
                return self._json(gateway.stack_payload())
            if path == '/api/runs':
                return self._json(build_run_index(gateway.runs_dir))
            if path.startswith('/api/runs/'):
                return self._run_route(path)
            if path.startswith('/api/'):
                return self._json({'message': f'no route {path}'}, 404)
            return self._static(path)
        except BrokenPipeError:
            return None
        except Exception as exc:                        # noqa: BLE001 - reported
            traceback.print_exc()
            return self._json({'message': f'{type(exc).__name__}: {exc}'}, 500)

    def do_POST(self):                                  # noqa: N802 - http.server API
        path = urlparse(self.path).path
        gateway = self.gateway
        try:
            body = self._body()
            if path == '/api/stack/start':
                return self._json(gateway.start_stack(body))
            if path == '/api/stack/stop':
                return self._json(gateway.stop_stack())
            if path == '/api/mission/start':
                return self._json(gateway.start_mission(body))
            if path == '/api/mission/cancel':
                return self._json(gateway.cancel_mission())
            if path.startswith('/api/faults/'):
                kind = path[len('/api/faults/'):]
                return self._json(gateway.call_fault(kind, body))
            return self._json({'message': f'no route {path}'}, 404)
        except ValueError as exc:
            return self._json({'message': str(exc)}, 400)
        except BrokenPipeError:
            return None
        except Exception as exc:                        # noqa: BLE001 - reported
            traceback.print_exc()
            return self._json({'message': f'{type(exc).__name__}: {exc}'}, 500)

    # -- routes ---------------------------------------------------------------
    def _run_route(self, path: str):
        rest = path[len('/api/runs/'):]
        if rest.endswith('/plot.png'):
            run_id = rest[:-len('/plot.png')]
            plot = os.path.join(self.gateway.runs_dir, run_id, 'plot.png')
            if '/' in run_id or not os.path.isfile(plot):
                return self._json(
                    {'message': f'no plot.png in runs/{run_id}; the figure is written '
                                'after the metrics, so a run that failed before that '
                                'has none'}, 404)
            with open(plot, 'rb') as handle:
                return self._bytes(handle.read(), 'image/png')
        run_id = rest.strip('/')
        if '/' in run_id:
            return self._json({'message': f'no route {path}'}, 404)
        detail = build_run_detail(self.gateway.runs_dir, run_id)
        if detail is None:
            return self._json({'message': f'no run directory runs/{run_id}'}, 404)
        return self._json(detail)

    def _static(self, path: str):
        if self.dist_dir is None or not os.path.isdir(self.dist_dir):
            return self._bytes(NO_BUILD.encode(), 'text/plain; charset=utf-8', 503)
        rel = posixpath.normpath(path).lstrip('/')
        if rel in ('', '.'):
            rel = 'index.html'
        target = os.path.join(self.dist_dir, rel)
        if not os.path.abspath(target).startswith(os.path.abspath(self.dist_dir)):
            return self._json({'message': 'refused'}, 403)
        if not os.path.isfile(target):
            # A single-page app: unknown paths are routes, not files.
            target = os.path.join(self.dist_dir, 'index.html')
            if not os.path.isfile(target):
                return self._bytes(NO_BUILD.encode(), 'text/plain; charset=utf-8', 503)
        extension = os.path.splitext(target)[1]
        with open(target, 'rb') as handle:
            return self._bytes(handle.read(),
                               CONTENT_TYPES.get(extension, 'application/octet-stream'))

    def _stream(self):
        gateway = self.gateway
        client = gateway.out.register()
        # No Content-Length is possible for a stream, so the framing is
        # read-to-EOF: `Connection: close` is what tells the browser that.
        self.close_connection = True
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Accel-Buffering', 'no')
        self.send_header('Connection', 'close')
        self._cors()
        self.end_headers()
        try:
            hello = {'topic': 'hello', 'payload': {
                'world': WORLD, 'repo_root': gateway.repo_root,
                'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
                'source': 'live'}}
            self.wfile.write(f'data: {json.dumps(hello)}\n\n'.encode())
            # Everything already known, so a console connecting mid-run is not
            # blank until the next message: the latched topics and the last
            # sample of each stream.
            stack = {'topic': 'stack', 'payload': gateway.stack_payload()}
            self.wfile.write(f'data: {json.dumps(stack, default=str)}\n\n'.encode())
            for topic, frame in list(gateway.out.latest.items()):
                if topic == 'stack':
                    continue
                self.wfile.write(f'data: {frame}\n\n'.encode())
            self.wfile.flush()
            last_keepalive = time.monotonic()
            while not gateway._stop.is_set():
                frames_out, dropped = client.drain(1.0)
                if dropped:
                    frames_out.append(json.dumps(
                        {'topic': 'dropped', 'payload': {'count': dropped}}))
                for frame in frames_out:
                    self.wfile.write(f'data: {frame}\n\n'.encode())
                now = time.monotonic()
                if frames_out:
                    self.wfile.flush()
                    last_keepalive = now
                elif now - last_keepalive >= 15.0:
                    self.wfile.write(b': keepalive\n\n')
                    self.wfile.flush()
                    last_keepalive = now
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            gateway.out.unregister(client)
        return None


# -- selftest -----------------------------------------------------------------

def _selftest() -> int:
    """Encode one synthetic message per topic and print the result."""
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import PoseStamped

    def stamp(value):
        return Time(sec=int(value), nanosec=int((value % 1) * 1e9))

    truth = Odometry()
    truth.header.stamp = stamp(12.5)
    truth.header.frame_id = 'world'
    truth.pose.pose.position.x = 1.0
    truth.pose.pose.orientation.w = 1.0
    truth.twist.covariance[0] = -1.0

    estimate = PoseWithCovarianceStamped()
    estimate.header.stamp = stamp(12.4)
    estimate.pose.pose.orientation.w = 1.0
    estimate.pose.covariance[0] = 0.05
    estimate.pose.covariance[7] = 0.04
    estimate.pose.covariance[35] = 0.01

    scan = LaserScan()
    scan.header.stamp = stamp(12.3)
    scan.range_min = 0.15
    scan.range_max = 12.0
    scan.ranges = [1.0, float('inf'), float('nan'), 0.01, 13.0]

    plan = Path()
    plan.header.stamp = stamp(12.0)
    plan.header.frame_id = 'map'
    pose = PoseStamped()
    pose.pose.position.x = 2.0
    plan.poses = [pose]

    mission = MissionState()
    mission.stamp = stamp(12.1)
    mission.experiment_id = 'selftest'
    mission.state = MissionState.STATE_IDLE
    mission.goal.header.frame_id = 'map'
    mission.goal.pose.orientation.w = 1.0

    event = FaultEvent()
    event.stamp = stamp(12.2)
    event.id = 'spawn_obstacle_ui_1'
    event.kind = FaultEvent.KIND_SPAWN_OBSTACLE
    event.injection_point = injection_point(FaultEvent.KIND_SPAWN_OBSTACLE, WORLD)
    event.target = 'blocker_1'
    event.state = FaultEvent.STATE_REQUESTED
    event.detail = 'selftest'

    active = ActiveFaults()
    active.stamp = stamp(12.2)
    active.active = [event]

    alignment = FrameAlignment()
    alignment.stamp = stamp(5.0)
    alignment.parent_frame = 'world'
    alignment.child_frame = 'ekf_local'
    alignment.source = 'selftest'

    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = 0.05
    grid.info.width = 2
    grid.info.height = 1
    grid.info.origin.orientation.w = 1.0
    grid.data = [0, -1]

    samples = {
        'truth': encode_truth(truth),
        'est': encode_estimate(estimate, 'AMCL (nav2_amcl) over /scan', 'map'),
        'scan': encode_scan(scan),
        'plan': encode_plan(plan),
        'cmd': encode_cmd(Twist(), 12.0),
        'mission': encode_mission(mission),
        'fault_event': encode_fault_event(event),
        'active': encode_active(active),
        'align': encode_alignment(alignment),
        'map': encode_grid(grid),
    }
    for topic, payload in samples.items():
        print(f'{topic}: {json.dumps(payload, allow_nan=False)}', flush=True)
    return 0


# -- entry point --------------------------------------------------------------

def run(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8771)
    parser.add_argument('--repo-root', default='/ws')
    parser.add_argument('--map', default=None,
                        help='map yaml; defaults to ral_bringup share maps/apartment.yaml')
    parser.add_argument('--selftest', action='store_true',
                        help='encode one synthetic message per topic and exit')
    args = parser.parse_args(argv)

    if args.selftest:
        return _selftest()

    bringup_share = get_package_share_directory('ral_bringup')
    map_yaml = args.map or os.path.join(bringup_share, 'maps', 'apartment.yaml')

    rclpy.init()
    gateway = Gateway(args.repo_root, map_yaml)
    gateway.start_threads()

    Handler.gateway = gateway
    Handler.dist_dir = os.path.join(args.repo_root, 'dashboard', 'dist')
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.daemon_threads = True

    def _signal(_signum, _frame):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, _signal)
    signal.signal(signal.SIGTERM, _signal)

    print(f'--- dashboard gateway on http://{args.host}:{args.port} '
          f'(repo root {args.repo_root}, map {map_yaml})', flush=True)
    print(f'--- static bundle: {Handler.dist_dir}'
          f'{"" if os.path.isdir(Handler.dist_dir) else " (absent; " + NO_BUILD + ")"}',
          flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        print('--- shutting down; stopping the stack if this gateway owns one', flush=True)
        gateway.shutdown()
        server.server_close()
        try:
            rclpy.shutdown()
        except Exception:                               # noqa: BLE001 - reported
            traceback.print_exc()
    return 0


def main(args=None) -> None:
    sys.exit(run(args))


if __name__ == '__main__':
    main()
