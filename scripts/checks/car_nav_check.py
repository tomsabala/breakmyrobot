#!/usr/bin/env python3
"""
End-to-end proof of the ArduRover SITL, MAVROS and EKF3 closed loop.

Brings the whole car stack up with `ros2 launch ral_bringup car_nav.launch.py`,
puts the autopilot into GUIDED, arms it, drives one real mission to
`baseline_car.yaml`'s goal and asserts thirteen properties against the running
system. Writes docs/car-closed-loop.md and exits non-zero if any check fails.

This is the Roomba harness's sibling and is numbered the same way, but the
stack underneath it shares almost nothing with Nav2: there is no action server,
no map_server, no AMCL and no costmap. ArduRover plans and controls inside the
autopilot, EKF3 estimates the pose from the IMU and SITL's synthesised GPS, and
MAVROS is the only link between that autopilot and ROS. So this harness drives
MAVROS directly — set_mode, arming, a republished position setpoint — exactly
as `ral_eval`'s MavrosGuidedDriver does for a scenario run, and it decides the
mission ended by the same rule the driver uses: the FCU's own
NAV_CONTROLLER_OUTPUT.wp_dist inside the profile's goal_tolerance_m on three
consecutive messages, while the mode is still GUIDED and the vehicle armed.

Every duration is measured in SIM time off /clock, never wall time: a slow
real-time factor must not change a result, only how long the run takes. Wall
limits exist solely as deadlock guards. The setpoint republication period is
the one exception and is deliberately wall-clock: ArduPilot stops a guided
vehicle after three REAL seconds of setpoint silence.

Run through scripts/checks/car_nav_check.sh, which resolves the container image
reference on the host and runs this inside ral:dev.

What this harness reads, and what it does not import
----------------------------------------------------
The robot profile (ral_bringup/config/robots/car.yaml), the mission
(ral_eval/scenarios/baseline_car.yaml) and the autopilot parameters
(ral_bringup/config/rover_ackermann.parm) are read as DATA, from the installed
share directories — the same files the running stack was configured from.
Nothing a number here depends on is hardcoded in this file: the goal tolerance,
the footprint radius, the plant maximum and the geometric minimum turning
radius all come out of the profile at runtime.

The planar geometry below (yaw from a quaternion, SE(2) composition) is
re-implemented here rather than imported from ral_eval.frames, deliberately and
for the same reason the Roomba harness re-implements its own: a verifier that
imports the code under test shares its bugs.

Frames
------
The mission is stated in the gz WORLD frame, which is the frame the world SDF,
the spawn pose and ground truth are all expressed in, so checks 8, 9 and 12
compare truth against the goal without converting anything.

The autopilot works in `ekf_local`, an ENU frame whose origin ArduPilot sets at
its first GPS fix. The offset between the two is a per-run MEASUREMENT made by
ral_eval's ekf_local_tf node and published latched on /frame_alignment; this
harness reads that message, checks its residual (check 6) and uses it to
convert the goal into the autopilot's frame and the autopilot's estimate back
into the world frame (check 11). Without it there is no honest way to relate
the two frames, so its absence fails the bring-up gate rather than being
papered over with an assumed identity.

Ground truth
------------
/ground_truth/odom is simulator truth and is used here as an EVALUATION SIGNAL
only: to confirm the body actually arrived (check 8), to measure the turning
radius the body actually achieved (check 9), to measure how far the robot's own
estimate was from the truth (check 11) and to measure the achieved speed
(check 12). Nothing the robot acts on is derived from it, and check 4 is what
asserts that no other node reads it either.

Negative controls
-----------------
NONE HAVE BEEN RUN FOR THIS HARNESS YET. Green checks prove the stack works;
they do not prove the checks can fail, and the Roomba harness only earned its
confidence by having each mutation applied and each named check observed going
red. The equivalent mutations here, to be applied and recorded in this
docstring in the Roomba harness's format once the toolchain image can run the
stack:

* Point the setpoint publisher at the `world` frame instead of composing
  through /frame_alignment — check 7 or 8 must go red (the vehicle drives to
  the wrong place), and the size of the error must equal the measured
  world->ekf_local offset.
* Replace the estimate subscription's QoS with the default RELIABLE profile.
  MAVROS publishes local_position/* as BEST_EFFORT sensor data, so the
  subscription must then receive NOTHING: the bring-up gate (check 0) must
  fail on the estimate topic rather than the run proceeding with no estimate.
* Give ral_car's two steering joints opposite ArduPilotPlugin channels, or
  swap SERVO1_FUNCTION and SERVO3_FUNCTION in rover_ackermann.parm — check 7
  and check 9 must both go red.
* Bridge /model/ral_car/pose into /tf — check 5 must report the simulator's
  own `ral_apartment` and `ral_car` frames as unexpected, as it does for the
  Roomba.
"""

import argparse
import hashlib
import math
import os
import sys
import time
from datetime import datetime, timezone

from ament_index_python.packages import get_package_share_directory
# Sibling module, imported by path: this script is executed as a file, so its
# own directory is sys.path[0] and `common` resolves with no path handling.
# These assertions are shared with roomba_nav_check.py and have one
# implementation there; everything car-specific stays in this file.
from common import (bumper_liveness_check, launch_stack, measure_rtf,
                    scan_publisher_check, teardown, tf_topology_check,
                    truth_subscriber_check, write_doc)
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import NavControllerOutput, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from ral_interfaces.msg import FrameAlignment
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import LaserScan
import tf2_ros
import yaml

WORLD = 'ral_apartment'
ROBOT = 'car'
SCENARIO = 'baseline_car'
LAUNCH_FILE = 'car_nav.launch.py'
# The autopilot parameter file is NOT a constant here: the profile names it
# (`autopilot_parm`), the launch file starts ardurover with that file, and
# this harness hashes the same one, so the document cannot cite a
# configuration the stack did not use. Read in main() as `parm_file`.

# Every TF frame this stack may publish, and nothing else. The set is closed on
# purpose: simulator truth bridged onto /tf arrives under the gz
# PosePublisher's own names (ral_apartment -> ral_car), which an absence test
# on a known truth frame name would never see.
#
# `map` and `world` are the two frames no robot link corresponds to: `map` is
# the frame every metric is scored in, `world` hangs off it through the
# display-only static transform car_nav.launch.py publishes from the committed
# alignment, and neither carries a robot pose. `ekf_local` is the autopilot's
# own frame, linked to `map` by ekf_local_tf's measurement. The remaining
# frames are ral_car's links, published by robot_state_publisher from
# /joint_states: one lidar, two steering knuckles and four wheels. This
# platform has no wheel-odometry integrator of its own, so nothing it owns
# publishes `odom`: its estimate chain is map -> ekf_local -> base_link.
#
# MAVROS adds four frames of its own, and they are listed because they are
# MEASURED to be there, not because this project wants them: `map_ned`,
# `odom`, `odom_ned` and `base_link_frd` are the aerospace NED/FRD aliases
# mavros publishes as static transforms for every vehicle. None of them is in
# the estimate chain this project scores - the topology below pins base_link's
# parent as `ekf_local`, so an alias quietly becoming base_link's parent still
# fails this check - and none of them is fed by simulator truth.
EXPECTED_TF_FRAMES = {
    'map',
    'world',
    'ekf_local',
    'base_link',
    'rplidar_link',
    'steer_left',
    'steer_right',
    'front_left_wheel',
    'front_right_wheel',
    'rear_left_wheel',
    'rear_right_wheel',
    'map_ned',
    'odom',
    'odom_ned',
    'base_link_frd',
}

# child -> required parent.
TF_TOPOLOGY = {
    'ekf_local': 'map',
    'base_link': 'ekf_local',
    'rplidar_link': 'base_link',
    'world': 'map',
}

# The ONLY nodes allowed to subscribe to simulator truth. `car_nav_check` is
# this harness, which measures the residual between truth and the estimate;
# `fault_manager` is the fault INJECTOR, which reads truth to project a
# body-frame wrench into the world frame gz requires and tells the robot
# nothing. `ekf_local_tf` is the third, and it is the one that needs stating:
# it reads truth ONCE, while the vehicle is still standing at its spawn pose,
# to MEASURE where ArduPilot put its EKF origin - an offset, not a pose. Every
# transform it broadcasts afterwards is the robot's own EKF3 estimate composed
# with that fixed offset, so nothing downstream can recover truth from it. That
# exception is asserted here rather than assumed, and anything outside this
# list is a new truth consumer nobody declared.
TRUTH_SUBSCRIBER_ALLOWLIST = ('car_nav_check', 'fault_manager', 'ekf_local_tf')

# The bring-up limit comes from the profile's `bringup_wall_limit_s`, as the
# scenario runner's does: an autopilot cannot report a position until EKF3 has
# an origin, and that waits on a simulated GPS fix. MEASURED on the three
# committed baseline_car runs, relative to run.json's started_utc: "EKF3 IMU0
# origin set" at +28.7, +29.8 and +33.9 s wall, and /frame_alignment published
# at +49.4, +49.7 and +55.1 s. The profile's 240 s is about 4x the slowest of
# those rather than a measurement in itself.
SERVICE_WALL_LIMIT = 60.0       # s wall for /mavros/set_mode and /mavros/cmd/arming
SERVICE_WAIT_SIM = 10.0         # s sim for one service call to answer
MODE_WAIT_SIM = 10.0            # s sim for the FCU to report GUIDED
ARM_WAIT_SIM = 10.0             # s sim for the FCU to report armed
# ArduPilot stops a guided vehicle after 3 s of setpoint silence. That timeout
# is real time, so this period is too - it has to hold whatever the RTF is.
SETPOINT_PERIOD_WALL = 0.1      # s wall
WP_DIST_HITS = 3                # consecutive in-tolerance wp_dist messages
# NAV_CONTROLLER_OUTPUT.wp_dist is a uint16 in METRES and truncates, so a
# reported value of N means the FCU believes it is somewhere in [N, N+1).
# That quantum is still half of the arrival rule - this harness mirrors the
# driver's test, which requires BOTH an in-tolerance wp_dist and an
# estimate-side gap inside arrival_estimate_tolerance_m - but it is no longer
# part of check 8's budget: the estimate-side gate is a real distance, and the
# budget is built from that plus the footprint and this run's measured
# estimate error.
WP_DIST_QUANTUM_M = 1.0
ALIGNMENT_RESIDUAL_LIMIT = 0.15  # m, the limit ekf_local_tf itself enforces
# How far the harness's OWN re-derivation of world -> ekf_local may differ
# from the published one before check 6 fails. Not the publisher's limit: a
# check that asserts the publisher's own threshold cannot fail.
ALIGNMENT_AGREEMENT_M = 0.05
ALIGNMENT_AGREEMENT_RAD = 0.02
ESTIMATE_ERROR_MIN = 0.005      # m, below this the "estimate" is truth
ESTIMATE_ERROR_MAX = 1.0        # m, above this the vehicle is not localized
TURN_MIN_SPEED = 0.3            # m/s, below which a turning radius is meaningless
TURN_RADIUS_FACTOR = 2.0        # measured vs geometric radius, either way
SPEED_ENVELOPE_FACTOR = 1.2     # achieved speed allowed above the configured cruise
STAMP_MATCH_S = 0.05            # s, nearest-stamp pairing window
# gz reports a contact every physics step while two bodies touch, so raw
# reports are collapsed per colliding pair. Same rule and same window as
# ral_eval/metrics.py's debounce_events, re-implemented here rather than
# imported for the reason given in the module docstring.
COLLISION_DEBOUNCE_S = 0.5


# ral_eval.frames and ral_eval.ekf_local_tf have these; see the module
# docstring for why they are not imported. Standard planar geometry.
def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(parent, child):
    """Pose `child`, given in a frame whose own pose is `parent`, in that parent."""
    cos_p = math.cos(parent[2])
    sin_p = math.sin(parent[2])
    return (parent[0] + child[0] * cos_p - child[1] * sin_p,
            parent[1] + child[0] * sin_p + child[1] * cos_p,
            wrap_to_pi(parent[2] + child[2]))


def into_frame(pose, frame):
    """Express `pose`, given in the parent frame, in a child frame at `frame`."""
    dx = pose[0] - frame[0]
    dy = pose[1] - frame[1]
    cos_a = math.cos(frame[2])
    sin_a = math.sin(frame[2])
    return (dx * cos_a + dy * sin_a,
            -dx * sin_a + dy * cos_a,
            wrap_to_pi(pose[2] - frame[2]))


def frame_from_pair(truth, estimate):
    """
    The child frame's pose in the parent, from one pose expressed in both.

    `truth` is the body in `world`, `estimate` the same body in ekf_local at
    the same (stationary) instant, so the frame offset is whatever makes
    compose(frame, estimate) == truth. This is the harness's OWN derivation of
    what ekf_local_tf publishes, computed from this harness's own samples with
    this harness's own geometry - which is the only way check 6 can disagree
    with the publisher rather than echo it.
    """
    yaw = wrap_to_pi(truth[2] - estimate[2])
    cos_a = math.cos(yaw)
    sin_a = math.sin(yaw)
    return (truth[0] - (estimate[0] * cos_a - estimate[1] * sin_a),
            truth[1] - (estimate[0] * sin_a + estimate[1] * cos_a),
            yaw)


def stamp_seconds(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def nearest(track, stamp: float):
    """Nearest sample in `track` to `stamp`, or None beyond STAMP_MATCH_S."""
    best = None
    best_delta = STAMP_MATCH_S
    for sample in track:
        delta = abs(sample[0] - stamp)
        if delta <= best_delta:
            best = sample
            best_delta = delta
    return best


def motion_series(track):
    """
    Speed and yaw rate by central difference of the truth pose track.

    `track` is [(sim stamp, x, y, yaw)] in the world frame; the result is
    [(sim stamp, speed_m_s, yaw_rate_rad_s)]. A central difference, not a
    consecutive one, so a single sample's stamp quantisation does not dominate
    the derivative. These are differentiated SIMULATOR TRUTH: they are the
    evaluation signal for checks 9 and 12 and are not available to the robot.
    """
    series = []
    for before, sample, after in zip(track, track[1:], track[2:]):
        dt = after[0] - before[0]
        if dt <= 0.0:
            continue
        speed = math.hypot(after[1] - before[1], after[2] - before[2]) / dt
        yaw_rate = wrap_to_pi(after[3] - before[3]) / dt
        series.append((sample[0], speed, yaw_rate))
    return series


def turning_radii(series, min_speed: float):
    """
    Instantaneous turning radius `speed / |yaw_rate|` over the driving samples.

    Samples slower than `min_speed` are dropped: at a standstill the ratio is
    noise, and a vehicle that is barely moving can change heading for reasons
    (a nudge, a kerb) that say nothing about its steering geometry.
    """
    radii = []
    for _stamp, speed, yaw_rate in series:
        if speed <= min_speed or abs(yaw_rate) < 1e-6:
            continue
        radii.append(speed / abs(yaw_rate))
    return radii


def debounce(events, window: float) -> int:
    """Count events, collapsing repeats of the same key within `window`."""
    last = {}
    count = 0
    for stamp, key in events:
        previous = last.get(key)
        if previous is None or stamp - previous > window:
            count += 1
        last[key] = stamp
    return count


def parm_value(text: str, name: str):
    """One parameter's value as the autopilot would parse it, or None."""
    for line in text.splitlines():
        line = line.split('#', 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if parts and parts[0] == name and len(parts) > 1:
            return parts[1]
    return None


class CarNavCheck(Node):

    def __init__(self, profile: dict) -> None:
        super().__init__(
            'car_nav_check',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.profile = profile
        self.estimate_topic = profile['estimate']['topic']
        self.estimate_frame = profile['estimate']['frame']

        self.truth = None          # latest Odometry in WORLD frame
        self.scan = None
        self.scan_raw = None
        self.state = None          # latest mavros_msgs/State
        self.estimate = None       # latest (x, y, yaw) in ekf_local
        self.alignment = None      # FrameAlignment, world -> ekf_local
        self.wp_dist = None        # metres, uint16, the FCU's own distance to go
        self.nav_output_count = 0
        # The first pose each source reported, both taken long before the
        # vehicle is armed. Check 6 re-derives the frame offset from them.
        self.first_truth = None    # (x, y, yaw) in WORLD frame
        self.first_estimate = None  # (x, y, yaw) in EKF_LOCAL frame

        # Mission recordings. Collected whenever `recording` is set, so
        # bring-up traffic cannot contribute to a mission statistic.
        self.recording = False
        self.truth_track = []      # (sim stamp, x, y, yaw) in WORLD frame
        self.estimate_track = []   # (sim stamp, x, y, yaw) in EKF_LOCAL frame
        self.collisions = []       # (sim stamp, 'collision1 / collision2')

        # Check 3's running comparison. Every /scan is compared against the
        # /scan_raw of the same header stamp, for the WHOLE run rather than at
        # one sampled instant: degradation is refused for this robot, so the
        # relay's pass-through must hold from bring-up to teardown.
        self.scan_raw_by_stamp = {}
        self.scan_pairs = 0
        self.scan_mismatches = 0
        self.scan_unmatched = 0
        self.scan_first_mismatch = None

        self.setpoint = None       # PoseStamped in ekf_local
        self._last_setpoint_wall = 0.0

        # QoS is stated for every subscription because a mismatch here receives
        # NOTHING while the topic lists and the node looks healthy. These are
        # the profiles the car stack's publishers use: MAVROS publishes
        # local_position/* as BEST_EFFORT sensor data, /mavros/state latched
        # RELIABLE, and ekf_local_tf publishes its single measurement latched.
        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        alignment_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(Odometry, '/ground_truth/odom', self._on_truth, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/scan_raw', self._on_scan_raw, qos_profile_sensor_data)
        self.create_subscription(Contacts, '/bumper', self._on_bumper, 10)
        self.create_subscription(State, '/mavros/state', self._on_state, latched)
        self.create_subscription(Odometry, self.estimate_topic, self._on_estimate,
                                 qos_profile_sensor_data)
        self.create_subscription(NavControllerOutput,
                                 '/mavros/nav_controller_output/output',
                                 self._on_nav_output, 10)
        self.create_subscription(FrameAlignment, '/frame_alignment', self._on_alignment,
                                 alignment_qos)

        # The bring-up gate is the PROFILE's list, not a list this file keeps:
        # a topic added to car.yaml must gate a run here too. Topics the typed
        # subscriptions above already track are read through them (that is also
        # how the latched ones are received at all - a VOLATILE subscription
        # joining after ekf_local_tf's single publication gets nothing). Any
        # other declared topic gets a generic "a message arrived" subscription.
        self.bringup_seen = {}
        self._bringup_ready = {
            '/clock': lambda: self.sim_now() > 0.0,
            '/ground_truth/odom': lambda: self.truth is not None,
            '/scan': lambda: self.scan is not None,
            '/scan_raw': lambda: self.scan_raw is not None,
            '/mavros/state': lambda: self.state is not None,
            self.estimate_topic: lambda: self.estimate is not None,
            '/frame_alignment': lambda: self.alignment is not None,
        }
        for entry in profile['bringup_topics']:
            topic = entry['topic']
            if topic in self._bringup_ready:
                continue
            self.bringup_seen[topic] = False
            self.create_subscription(
                get_message(entry['type']), topic,
                lambda _msg, topic=topic: self.bringup_seen.__setitem__(topic, True),
                qos_profile_sensor_data)

        self.setpoint_pub = self.create_publisher(
            PoseStamped, profile['command']['topic'], 10)
        self.set_mode = self.create_client(SetMode, '/mavros/set_mode')
        self.arming = self.create_client(CommandBool, '/mavros/cmd/arming')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)

    # -- subscriptions ---------------------------------------------------------
    def _on_truth(self, msg):
        self.truth = msg
        pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                yaw_of_quaternion(msg.pose.pose.orientation))
        # The first sample arrives long before the vehicle is armed, so it is
        # a STATIONARY pose. Check 6 re-derives the world -> ekf_local offset
        # from it and the first estimate, which is how a harness can disagree
        # with the publisher's own self-report.
        if self.first_truth is None:
            self.first_truth = pose
        if self.recording:
            self.truth_track.append((stamp_seconds(msg.header), *pose))

    def _on_scan(self, msg):
        self.scan = msg
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        raw = self.scan_raw_by_stamp.pop(key, None)
        if raw is None:
            # A /scan whose /scan_raw this harness never saw. Expected once or
            # twice at start-up, when the two subscriptions join between the
            # relay's input and its output; counted and reported, never fatal.
            self.scan_unmatched += 1
            return
        self.scan_pairs += 1
        identical = (list(msg.ranges) == list(raw.ranges)
                     and msg.header.frame_id == raw.header.frame_id
                     and msg.angle_min == raw.angle_min
                     and msg.range_max == raw.range_max)
        if not identical:
            self.scan_mismatches += 1
            if self.scan_first_mismatch is None:
                self.scan_first_mismatch = key

    def _on_scan_raw(self, msg):
        self.scan_raw = msg
        self.scan_raw_by_stamp[(msg.header.stamp.sec, msg.header.stamp.nanosec)] = msg

    def _on_state(self, msg):
        self.state = msg

    def _on_estimate(self, msg):
        pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                yaw_of_quaternion(msg.pose.pose.orientation))
        self.estimate = pose
        if self.first_estimate is None:
            self.first_estimate = pose
        if self.recording:
            self.estimate_track.append((stamp_seconds(msg.header), *pose))

    def _on_nav_output(self, msg):
        self.wp_dist = float(msg.wp_dist)
        self.nav_output_count += 1

    def _on_alignment(self, msg):
        self.alignment = msg

    def _on_bumper(self, msg):
        if self.recording and msg.contacts:
            stamp = stamp_seconds(msg.header)
            for contact in msg.contacts:
                self.collisions.append(
                    (stamp, f'{contact.collision1.name} / {contact.collision2.name}'))

    # -- time ------------------------------------------------------------------
    def sim_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_wall(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_wall(self, predicate, wall_limit: float) -> bool:
        """Spin until `predicate()` holds, bounded in WALL time.

        Service discovery is a wall-clock property of the ROS graph, not
        something that happens in sim time, so the one place this is used -
        waiting for MAVROS's services to appear - is bounded that way. Every
        measurement in this harness is bounded in sim time instead.
        """
        deadline = time.monotonic() + wall_limit
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.02)
        return predicate()

    def wait_until(self, predicate, sim_limit: float) -> bool:
        """Spin until `predicate()` holds or `sim_limit` SIM seconds pass."""
        deadline = self.sim_now() + sim_limit
        wall_deadline = time.monotonic() + sim_limit * 10.0 + 30.0
        while rclpy.ok():
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.sim_now() > deadline or time.monotonic() > wall_deadline:
                return False
        return False

    def bringup_missing(self):
        """Declared bring-up topics with no message yet, in profile order."""
        missing = []
        for entry in self.profile['bringup_topics']:
            topic = entry['topic']
            ready = self._bringup_ready.get(
                topic, lambda topic=topic: self.bringup_seen.get(topic, False))
            if not ready():
                missing.append(topic)
        return missing

    # -- commanding the autopilot ----------------------------------------------
    def call(self, client, request, name: str):
        """Call one MAVROS service and report its own answer, never a guess."""
        future = client.call_async(request)
        if not self.wait_until(future.done, SERVICE_WAIT_SIM):
            return False, f'{name} did not answer within {SERVICE_WAIT_SIM:.0f} s sim'
        response = future.result()
        accepted = getattr(response, 'mode_sent', None)
        if accepted is None:
            accepted = getattr(response, 'success', False)
        return bool(accepted), f'{name} answered {response}'

    def command_mode(self, mode: str):
        request = SetMode.Request()
        request.custom_mode = mode
        return self.call(self.set_mode, request, '/mavros/set_mode')

    def command_arming(self, value: bool):
        request = CommandBool.Request()
        request.value = value
        return self.call(self.arming, request, '/mavros/cmd/arming')

    def publish_setpoint(self) -> None:
        self.setpoint.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_pub.publish(self.setpoint)
        self._last_setpoint_wall = time.monotonic()

    def set_goal(self, goal_world, alignment) -> 'tuple[float, float, float]':
        """Convert the world-frame goal into the autopilot's frame and arm it."""
        local = into_frame(goal_world, alignment)
        setpoint = PoseStamped()
        setpoint.header.frame_id = self.estimate_frame
        setpoint.pose.position.x = local[0]
        setpoint.pose.position.y = local[1]
        setpoint.pose.orientation.z = math.sin(local[2] / 2.0)
        setpoint.pose.orientation.w = math.cos(local[2] / 2.0)
        self.setpoint = setpoint
        return local

    def estimate_gap(self) -> 'float | None':
        """
        |estimate - commanded setpoint| in ekf_local, from this harness's own data.

        Both terms are the robot's own: the estimate it published and the
        setpoint this harness built. Truth is not read here. It mirrors
        MavrosGuidedDriver._estimate_gap deliberately - the harness checks the
        driver's rule by re-implementing it, never by importing it.
        """
        if self.estimate is None or self.setpoint is None:
            return None
        return math.hypot(self.estimate[0] - self.setpoint.pose.position.x,
                          self.estimate[1] - self.setpoint.pose.position.y)

    def fly(self, goal_tolerance: float, arrival_estimate_tol: float, limit_sim: float):
        """
        Run the mission and return (outcome, detail, sim duration).

        The terminal rule is MavrosGuidedDriver's, deliberately: the FCU must
        first report a distance BEYOND the profile's goal_tolerance_m - its own
        way of saying it has a target and the target is over there - and then
        three consecutive NAV_CONTROLLER_OUTPUT.wp_dist messages inside that
        tolerance while the mode is GUIDED and the vehicle armed, AND the
        robot's own estimate within arrival_estimate_tolerance_m of the
        commanded setpoint. MEASURED failure without the first half: wp_dist is
        0 whenever the autopilot has no active target, so a run was declared
        reached at 0.0 s sim with the vehicle standing on its spawn pose.
        MEASURED failure without the last: wp_dist truncates to whole metres,
        so three runs reported wp_dist 2 against WP_RADIUS 2.0 and came to rest
        2.87-2.96 m from the goal. A mode change or a disarm the autopilot
        decided by itself is terminal too, and is reported as what it is.

        The driver's no-progress supervisor is NOT reproduced here: it is a
        deterministic policy belonging to the scenario runner, and a harness
        that stopped the vehicle on its own schedule would be measuring its own
        policy. This loop simply runs to the scenario's own timeout.
        """
        start = self.sim_now()
        self.recording = True
        self.publish_setpoint()
        hits = 0
        acquired = False
        wall_deadline = time.monotonic() + limit_sim * 10.0 + 120.0
        outcome = None
        detail = 'the ROS context shut down before any terminal condition'
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            if time.monotonic() - self._last_setpoint_wall >= SETPOINT_PERIOD_WALL:
                self.publish_setpoint()

            armed = self.state.armed if self.state is not None else False
            mode = self.state.mode if self.state is not None else None

            if (self.wp_dist is not None and mode == 'GUIDED' and armed
                    and self.wp_dist > goal_tolerance):
                acquired = True

            gap = self.estimate_gap()
            if (acquired and self.wp_dist is not None and mode == 'GUIDED' and armed
                    and self.wp_dist <= goal_tolerance
                    and gap is not None and gap <= arrival_estimate_tol):
                hits += 1
                if hits >= WP_DIST_HITS:
                    outcome = 'mavros:reached_wp_radius'
                    detail = (
                        f'/mavros/nav_controller_output/output reported wp_dist '
                        f'{self.wp_dist:.0f} m within the profile\'s goal_tolerance_m '
                        f'{goal_tolerance:.0f} m on {WP_DIST_HITS} consecutive messages '
                        'while /mavros/state reported mode GUIDED and armed, having '
                        'first reported a distance beyond that tolerance, AND the '
                        f'robot\'s own estimate was {gap:.2f} m from the commanded '
                        'setpoint, within the profile\'s arrival_estimate_tolerance_m '
                        f'{arrival_estimate_tol:.2f}')
                    break
            elif self.wp_dist is not None:
                hits = 0

            if mode is not None and mode != 'GUIDED':
                outcome = f'mavros:mode_changed:{mode}'
                detail = (f'/mavros/state reports mode {mode!r}: the autopilot left '
                          f'GUIDED by itself, so the mission ended for a reason the FCU '
                          f'decided. Last wp_dist {self.wp_dist}')
                break
            if self.state is not None and not armed:
                outcome = 'mavros:disarmed'
                detail = ('/mavros/state reports armed=false mid-mission; a disarmed '
                          f'rover cannot move. Last wp_dist {self.wp_dist}, mode {mode!r}')
                break

            if self.sim_now() - start >= limit_sim:
                outcome = None
                detail = (f'no terminal condition within {limit_sim:.0f} s sim: last '
                          f'wp_dist {self.wp_dist}, mode {mode!r}, armed {armed}, '
                          f'{self.nav_output_count} NAV_CONTROLLER_OUTPUT message(s) '
                          'seen in total')
                break
            if time.monotonic() > wall_deadline:
                outcome = None
                detail = (f'the mission loop hit its WALL guard before {limit_sim:.0f} s '
                          'sim elapsed; the simulation is not advancing (see the launch '
                          f'log). Last wp_dist {self.wp_dist}, mode {mode!r}')
                break

        duration = self.sim_now() - start
        self.spin_wall(1.0)
        self.recording = False
        return outcome, detail, duration

    def stop(self) -> None:
        """HOLD, then disarm. Best effort; each answer is printed as received."""
        ok, detail = self.command_mode('HOLD')
        print(f'--- HOLD: {"ok" if ok else "FAILED"} ({detail})', flush=True)
        ok, detail = self.command_arming(False)
        print(f'--- disarm: {"ok" if ok else "FAILED"} ({detail})', flush=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--image-ref', default='unknown',
                        help='ral:dev digest or image id, collected on the host')
    parser.add_argument('--repo-root', default='/ws')
    args = parser.parse_args()

    log_path = os.path.join(args.repo_root, 'scripts', 'checks', 'car_nav_launch.log')
    results = []

    def record(number, name, passed, detail):
        results.append((number, name, passed, detail))
        print(f'{"PASS" if passed else "FAIL"}  {number}. {name}: {detail}', flush=True)

    def not_measured(number, name, reason):
        record(number, name, False, f'not measured: {reason}')

    bringup_share = get_package_share_directory('ral_bringup')
    eval_share = get_package_share_directory('ral_eval')
    # Read as data, from the installed copies the running stack is configured
    # from - not from the repository, and not through the loader the stack
    # uses. Every key accessed below is required by the profile schema, so a
    # missing one raises here rather than defaulting to something plausible.
    profile_path = os.path.join(bringup_share, 'config', 'robots', f'{ROBOT}.yaml')
    scenario_path = os.path.join(eval_share, 'scenarios', f'{SCENARIO}.yaml')
    with open(profile_path) as handle:
        profile = yaml.safe_load(handle)
    with open(scenario_path) as handle:
        scenario = yaml.safe_load(handle)
    parm_file = profile['autopilot_parm']
    parm_path = os.path.join(bringup_share, 'config', parm_file)
    with open(parm_path, 'rb') as handle:
        parm_bytes = handle.read()
    parm_sha = hashlib.sha256(parm_bytes).hexdigest()
    parm_text = parm_bytes.decode('utf-8')
    arming_check = parm_value(parm_text, 'ARMING_CHECK')

    goal_world = (float(scenario['goal']['x']), float(scenario['goal']['y']),
                  float(scenario['goal']['yaw']))
    initial_world = (float(scenario['initial_pose']['x']),
                     float(scenario['initial_pose']['y']),
                     float(scenario['initial_pose']['yaw']))
    mission_limit_sim = float(scenario['timeout_sec'])
    goal_tolerance = float(profile['goal_tolerance_m'])
    arrival_estimate_tol = float(profile['arrival_estimate_tolerance_m'])
    footprint_radius = float(profile['footprint']['radius_m'])
    plant_max_linear = float(profile['plant']['max_linear_m_s'])
    geometric_turn_radius = float(profile['plant']['min_turn_radius_m'])
    # Check 8's budget is NOT built here: its third term is this run's own
    # measured maximum |estimate - truth|, so it is assembled beside that
    # measurement, below.

    print(f'--- profile {profile_path}', flush=True)
    print(f'--- scenario {SCENARIO}: initial pose world {initial_world}, goal world '
          f'{goal_world}, timeout {mission_limit_sim:.0f} s sim', flush=True)
    print(f'--- autopilot {parm_path} sha256 {parm_sha[:16]}, ARMING_CHECK '
          f'{arming_check}', flush=True)

    rclpy.init()
    node = CarNavCheck(profile)

    # The scenario's own initial pose is the spawn pose: the launch takes it so
    # the pose the mission was planned from and the pose the body starts at
    # cannot disagree.
    launch, launch_log = launch_stack(LAUNCH_FILE, log_path, extra_args=(
        f'world:={WORLD}', f'robot:={ROBOT}', f'seed:={scenario["seed"]}',
        f'spawn_x:={initial_world[0]}', f'spawn_y:={initial_world[1]}',
        f'spawn_yaw:={initial_world[2]}'))
    with launch_log:

        sim_start = None
        sim_end = 0.0
        rtf = 'not measured'
        alignment = None
        mission_outcome = None
        mission_sim = 0.0
        try:
            # -- 0. bring-up ---------------------------------------------------
            bringup_limit = float(profile['bringup_wall_limit_s'])
            deadline = time.monotonic() + bringup_limit
            # The full gated list, NOT None: a zero-iteration loop - a deadline
            # already passed, a clock that never advanced - would otherwise
            # leave `missing` falsy and record a PASS nobody observed.
            missing = [entry['topic'] for entry in profile['bringup_topics']]
            while time.monotonic() < deadline:
                node.spin_wall(0.5)
                missing = node.bringup_missing()
                if not missing:
                    break
            if missing:
                record(0, 'bring-up', False,
                       f'no messages on {", ".join(missing)} within '
                       f'{bringup_limit:.0f} s wall (the gate is '
                       f'{os.path.basename(profile_path)}\'s bringup_topics)')
                raise SystemExit(1)
            record(0, 'bring-up', True,
                   'one message on each of '
                   f'{", ".join(entry["topic"] for entry in profile["bringup_topics"])} '
                   f'within {bringup_limit:.0f} s wall')

            sim_start = node.sim_now()
            print(f'--- stack up at sim t={sim_start:.2f} s', flush=True)

            # The observational checks run BEFORE the vehicle is armed, and the
            # link check (1) is run last, immediately before the mission: an
            # autopilot armed while a slow check runs could be disarmed again
            # by its own inactivity timer, which would fail the mission for a
            # reason that is not the experiment. The results are sorted by
            # number before the document is written; the console prints them in
            # the order they were actually measured.

            # -- 2. One /scan publisher, and it is the relay -------------------
            passed_2, detail_2 = scan_publisher_check(node)
            record(2, '/scan has exactly one publisher: scan_fault_relay',
                   passed_2, detail_2)

            # -- 4. Truth is consumed by the harness, the injector and the
            #       frame measurer only ----------------------------------------
            passed_4, detail_4 = truth_subscriber_check(
                node, TRUTH_SUBSCRIBER_ALLOWLIST)
            record(4, 'only the harness, the fault injector and the frame measurer '
                   'subscribe to truth', passed_4, detail_4)

            # -- 5. TF topology -------------------------------------------------
            passed_5, detail_5 = tf_topology_check(node, EXPECTED_TF_FRAMES, TF_TOPOLOGY)
            record(5, 'TF topology', passed_5, detail_5)

            # -- bumper liveness, measured BEFORE the mission ------------------
            # A dead contact path must fail fast rather than after a mission
            # that then reports "0 collisions". Check 10 reads the result.
            bumper_alive, bumper_detail = bumper_liveness_check(
                node, WORLD, profile['model'])
            print(f'--- bumper liveness: {bumper_alive} ({bumper_detail})', flush=True)

            # -- 6. The EKF origin was measured, and the harness agrees --------
            # NOT a re-assertion of the publisher's own threshold: ekf_local_tf
            # refuses to publish at all above its residual limit, so checking
            # that limit here could never fail. Instead the harness re-derives
            # the offset from ITS OWN first stationary truth sample and first
            # estimate sample, with its own geometry, and requires the
            # published measurement to agree with that.
            alignment = node.alignment
            if alignment is None:
                not_measured(6, 'EKF origin alignment',
                             'no /frame_alignment message, so ekf_local_tf never '
                             'published a measurement')
            elif node.first_truth is None or node.first_estimate is None:
                not_measured(6, 'EKF origin alignment',
                             'the harness never saw both a truth sample and an '
                             'estimate sample, so it has no independent derivation to '
                             'compare the published alignment against')
            else:
                derived = frame_from_pair(node.first_truth, node.first_estimate)
                published = (alignment.x, alignment.y, alignment.yaw)
                delta_xy = math.hypot(derived[0] - published[0],
                                      derived[1] - published[1])
                delta_yaw = abs(wrap_to_pi(derived[2] - published[2]))
                frames_ok = (alignment.parent_frame == 'world'
                             and alignment.child_frame == node.estimate_frame)
                record(6, 'EKF origin alignment',
                       frames_ok and delta_xy <= ALIGNMENT_AGREEMENT_M
                       and delta_yaw <= ALIGNMENT_AGREEMENT_RAD,
                       f'{alignment.parent_frame} -> {alignment.child_frame} published '
                       f'at ({published[0]:.4f}, {published[1]:.4f}, '
                       f'{published[2]:.4f}); this harness derived '
                       f'({derived[0]:.4f}, {derived[1]:.4f}, {derived[2]:.4f}) from '
                       'its own first stationary /ground_truth/odom sample '
                       f'({node.first_truth[0]:.4f}, {node.first_truth[1]:.4f}, '
                       f'{node.first_truth[2]:.4f}) and its own first '
                       f'{node.estimate_topic} sample ({node.first_estimate[0]:.4f}, '
                       f'{node.first_estimate[1]:.4f}, {node.first_estimate[2]:.4f}); '
                       f'they differ by {delta_xy:.4f} m and {delta_yaw:.4f} rad '
                       f'(limits {ALIGNMENT_AGREEMENT_M} m, '
                       f'{ALIGNMENT_AGREEMENT_RAD} rad). The comparison is this '
                       'harness\'s own measurement, not the publisher\'s self-report; '
                       f'for reference the publisher reported residuals '
                       f'{alignment.residual_m:.4f} m / '
                       f'{alignment.residual_yaw_rad:.4f} rad against the spawn pose '
                       f'it predicts, and its source was: {alignment.source}')

            # -- 1. The autopilot link, the mode and the arming ----------------
            services_ready = node.wait_wall(
                lambda: node.set_mode.service_is_ready() and node.arming.service_is_ready(),
                SERVICE_WALL_LIMIT)
            connected = node.state is not None and node.state.connected
            mode_ok = armed_ok = False
            mode_detail = arm_detail = 'not attempted'
            if services_ready and connected:
                ok, mode_detail = node.command_mode('GUIDED')
                mode_ok = ok and node.wait_until(
                    lambda: node.state is not None and node.state.mode == 'GUIDED',
                    MODE_WAIT_SIM)
            if mode_ok:
                ok, arm_detail = node.command_arming(True)
                armed_ok = ok and node.wait_until(
                    lambda: node.state is not None and node.state.armed, ARM_WAIT_SIM)
            state = node.state
            # /mavros/state carries no firmware VERSION field - the autopilot
            # identity it does carry is the mode string vocabulary (GUIDED and
            # HOLD here are ArduPilot rover modes) and the MAV_STATE enum. The
            # autopilot build is not claimed from this topic; the configuration
            # provenance in the document is the .parm sha256.
            record(1, 'autopilot link: connected, GUIDED, armed',
                   bool(services_ready and connected and mode_ok and armed_ok),
                   f'/mavros/state connected={getattr(state, "connected", None)}, '
                   f'mode={getattr(state, "mode", None)!r}, '
                   f'armed={getattr(state, "armed", None)}, '
                   f'guided={getattr(state, "guided", None)}, '
                   f'manual_input={getattr(state, "manual_input", None)}, '
                   f'system_status={getattr(state, "system_status", None)} '
                   '(MAV_STATE); services ready = '
                   f'{services_ready}; set_mode: {mode_detail}; arming: {arm_detail}')

            # -- 7. The mission -------------------------------------------------
            if not armed_ok:
                not_measured(7, 'mission succeeds',
                             'the vehicle never reached GUIDED and armed, so no '
                             'setpoint was sent and no mission was flown')
            elif alignment is None:
                not_measured(7, 'mission succeeds',
                             'the world -> ekf_local offset was never measured, so the '
                             'goal cannot be expressed in the autopilot\'s frame')
            else:
                local = node.set_goal(goal_world, (alignment.x, alignment.y,
                                                   alignment.yaw))
                print(f'--- goal world {goal_world} -> {node.estimate_frame} '
                      f'({local[0]:.3f}, {local[1]:.3f}, {local[2]:.3f})', flush=True)
                mission_outcome, mission_detail, mission_sim = node.fly(
                    goal_tolerance, arrival_estimate_tol, mission_limit_sim)
                if mission_outcome is None:
                    not_measured(7, 'mission succeeds',
                                 f'{mission_detail}. SET_POSITION_TARGET_LOCAL_NED is '
                                 'not acknowledged, so there is no result to read: the '
                                 'terminal state is whatever the FCU reported, and it '
                                 'reported none')
                else:
                    record(7, 'mission succeeds',
                           mission_outcome == 'mavros:reached_wp_radius',
                           f'terminal outcome {mission_outcome} after '
                           f'{mission_sim:.1f} s sim (limit {mission_limit_sim:.0f}): '
                           f'{mission_detail}')

            # -- the estimate/truth error, measured ONCE --------------------
            # Check 8's budget and check 11 both need it, and computing it
            # twice would let the two disagree.
            errors = []
            if alignment is not None:
                frame = (alignment.x, alignment.y, alignment.yaw)
                for stamp, ex, ey, _eyaw in node.estimate_track:
                    match = nearest(node.truth_track, stamp)
                    if match is None:
                        continue
                    # The estimate is reported in ekf_local; compose it through
                    # the MEASURED offset to compare it with truth in `world`.
                    wx, wy, _ = compose(frame, (ex, ey, 0.0))
                    errors.append(math.hypot(wx - match[1], wy - match[2]))
            est_error_max = max(errors) if errors else None

            # -- 8. Goal reached in ground truth --------------------------------
            if node.truth is None:
                not_measured(8, 'goal reached in ground truth',
                             'no /ground_truth/odom message to compare against')
            elif est_error_max is None:
                not_measured(8, 'goal reached in ground truth',
                             'no estimate/truth sample pair, so this run\'s own '
                             'estimate error - the third term of the budget - was '
                             'never measured and the limit cannot be built from '
                             'measurements alone')
            else:
                final = (node.truth.pose.pose.position.x, node.truth.pose.pose.position.y)
                goal_error = math.hypot(final[0] - goal_world[0], final[1] - goal_world[1])
                # Where the BODY may honestly come to rest, in truth, given an
                # arrival rule that closes on the robot's own estimate. Three
                # terms, each a real property of this run:
                #   arrival_estimate_tolerance_m  the runner's own gate, a real
                #                                 distance in the estimate frame
                #   footprint radius              the goal is a point; the body
                #                                 has extent
                #   est_error_max                 how far this run's estimate
                #                                 actually was from truth, so
                #                                 the budget is not a constant
                # The wp_dist quantum is NOT here: the FCU's truncated metre is
                # the coarse half of the rule, and the estimate-side gate is
                # what bounds the resting place.
                goal_truth_tolerance = (arrival_estimate_tol + footprint_radius
                                        + est_error_max)
                record(8, 'goal reached in ground truth',
                       goal_error < goal_truth_tolerance,
                       f'|final truth - goal| in world = {goal_error:.3f} m (limit '
                       f'{goal_truth_tolerance:.3f} = arrival_estimate_tolerance_m '
                       f'{arrival_estimate_tol} + footprint radius {footprint_radius} '
                       f'+ this run\'s measured max |estimate - truth| '
                       f'{est_error_max:.3f}); truth world = '
                       f'({final[0]:.3f}, {final[1]:.3f}), goal world = '
                       f'({goal_world[0]:.3f}, {goal_world[1]:.3f})')

            # -- 9. The minimum turning radius this body actually achieved ------
            series = motion_series(node.truth_track)
            radii = turning_radii(series, TURN_MIN_SPEED)
            if not radii:
                not_measured(9, 'measured minimum turning radius',
                             f'no truth sample faster than {TURN_MIN_SPEED} m/s with a '
                             f'measurable yaw rate during the mission '
                             f'({len(node.truth_track)} truth sample(s) recorded)')
            else:
                measured_radius = min(radii)
                low = geometric_turn_radius / TURN_RADIUS_FACTOR
                high = geometric_turn_radius * TURN_RADIUS_FACTOR
                # The property this check exists to establish is that the body
                # cannot turn tighter than its steering geometry allows - the
                # fact narrow_doorway_car's expected failure rests on. So only
                # a radius BELOW the band fails: it would mean the tyres
                # skidded or the vehicle is not behaving as an Ackermann
                # platform at all. A radius above the band is not a defect in
                # the plant, it is a mission that never asked for full lock,
                # and the detail says so rather than letting the number read
                # as a measured plant limit.
                informative = measured_radius <= high
                record(9, 'measured minimum turning radius',
                       measured_radius >= low,
                       f'min(speed / |yaw rate|) over truth = {measured_radius:.3f} m '
                       f'from {len(radii)} sample(s) faster than {TURN_MIN_SPEED} m/s, '
                       f'against the profile\'s geometric {geometric_turn_radius} m '
                       f'(wheelbase / tan(steer limit)); the floor this check '
                       f'enforces is {low:.3f} m. '
                       + ('The measurement is inside a factor of '
                          f'{TURN_RADIUS_FACTOR} of the geometric radius, so this '
                          'mission did demand something near full steering lock.'
                          if informative else
                          'This mission never demanded anything near full steering '
                          'lock, so the number is a LOWER BOUND on what this body '
                          'did here and NOT a measurement of the plant limit. '
                          'Below the floor would mean the body turned tighter than '
                          'its geometry allows - skidding, or not behaving as an '
                          'Ackermann vehicle - and that is the only way this check '
                          'fails.'))

            # -- 10. No collisions during the mission ---------------------------
            # `events == 0` is only evidence when the path that would have
            # carried a report is shown to exist: a contact sensor is silent
            # until something touches it, so a dead bridge and a clean run
            # count the same. The liveness proof was made before the mission.
            events = debounce(node.collisions, COLLISION_DEBOUNCE_S)
            record(10, 'no collisions during the mission',
                   bool(bumper_alive) and events == 0,
                   f'{events} debounced collision event(s) from '
                   f'{len(node.collisions)} raw /bumper contact report(s), collapsed at '
                   f'{COLLISION_DEBOUNCE_S} s sim per colliding pair'
                   + (f': {sorted({key for _stamp, key in node.collisions})}'
                      if node.collisions else '')
                   + f'. Contact path liveness: {bumper_detail}')

            # -- 11. The estimate is an estimate, not truth ---------------------
            if not errors:
                reason = ('the world -> ekf_local offset was never measured'
                          if alignment is None else
                          f'no estimate/truth sample pair within '
                          f'{STAMP_MATCH_S * 1000:.0f} ms during the mission '
                          f'({len(node.estimate_track)} estimate, '
                          f'{len(node.truth_track)} truth sample(s))')
                not_measured(11, 'estimate is an estimate, not truth', reason)
            else:
                mean_error = sum(errors) / len(errors)
                max_error = max(errors)
                record(11, 'estimate is an estimate, not truth',
                       ESTIMATE_ERROR_MIN < max_error < ESTIMATE_ERROR_MAX,
                       f'max |EKF3 estimate - truth| = {max_error:.4f} m, mean = '
                       f'{mean_error:.4f} m over {len(errors)} paired samples '
                       f'(required: > {ESTIMATE_ERROR_MIN} so it is not truth, < '
                       f'{ESTIMATE_ERROR_MAX} so the vehicle is localized). The '
                       f'estimate is {profile["estimate"]["label"]}, read from '
                       f'{node.estimate_topic}')

            # -- 12. Achieved speed inside the configured envelope --------------
            if not series:
                not_measured(12, 'achieved speed inside the configured envelope',
                             'fewer than three truth samples during the mission, so no '
                             'central difference exists')
            else:
                max_speed = max(speed for _stamp, speed, _rate in series)
                ceiling = plant_max_linear * SPEED_ENVELOPE_FACTOR
                record(12, 'achieved speed inside the configured envelope',
                       max_speed <= ceiling,
                       f'max achieved speed (central difference of truth) = '
                       f'{max_speed:.3f} m/s over {len(series)} sample(s), against '
                       f'{plant_max_linear} m/s configured (CRUISE_SPEED/WP_SPEED in '
                       f'{parm_file}, carried in the profile as plant.max_linear_m_s) '
                       f'x {SPEED_ENVELOPE_FACTOR} = {ceiling:.3f} m/s. This is '
                       'achieved speed, not a command: ArduRover is commanded by '
                       'position setpoint and its throttle output is PWM')

            # -- 3. The relay is a faithful pass-through, for the WHOLE run -----
            # Compared on identical header stamps, every message: the relay must
            # republish the object unchanged, so a restamp, a reorder or a
            # resynthesis of ranges shows up here. Degradation is REFUSED for
            # this robot (car.yaml's `unsupported`), so nothing in this run may
            # ever alter the stream - which is why this is a whole-run
            # comparison rather than the Roomba's single sampled stamp.
            record(3, 'relay output equals relay input',
                   node.scan_pairs > 0 and node.scan_mismatches == 0,
                   f'{node.scan_pairs} /scan and /scan_raw message(s) shared a header '
                   f'stamp over the whole run; {node.scan_mismatches} differed'
                   + (f' (first at {node.scan_first_mismatch})'
                      if node.scan_first_mismatch else '')
                   + f'; {len(node.scan_raw_by_stamp)} raw message(s) were never '
                   f'republished and {node.scan_unmatched} /scan message(s) had no '
                   'matching raw (start-up artefacts of this harness joining between '
                   'the relay\'s input and its output)')

            rtf = measure_rtf(WORLD)

        except SystemExit:
            pass
        finally:
            if node.state is not None:
                node.stop()
            sim_end = node.sim_now()
            node.destroy_node()
            rclpy.shutdown()
            teardown(launch)

    sim_duration = (sim_end - sim_start) if sim_start is not None else 0.0
    # Sorted by number: the checks are executed in the order the stack allows
    # (observational first, armed last), and numbered by subject.
    results.sort(key=lambda result: result[0])
    passed = all(result[2] for result in results) and len(results) == 13

    alignment_cell = 'not measured'
    if alignment is not None:
        alignment_cell = (f'x = {alignment.x:.4f}, y = {alignment.y:.4f}, '
                          f'yaw = {alignment.yaw:.4f}, residual = '
                          f'{alignment.residual_m:.4f} m')

    doc = [
        '# ArduRover SITL, MAVROS and the car closed loop — measured proof',
        '',
        f'Measured {datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")} by',
        '`scripts/checks/car_nav_check.sh`, which launches',
        '`ral_bringup/car_nav.launch.py`, puts the autopilot into `GUIDED`, arms it',
        'and drives one real mission to `baseline_car.yaml`\'s goal. Every duration is',
        'sim time off `/clock`.',
        '',
        'There is no Nav2, no `map_server` and no AMCL on this platform: ArduRover',
        'plans and controls inside the autopilot, EKF3 estimates the pose from the IMU',
        'and SITL\'s synthesised GPS, and MAVROS is the only link between that',
        'autopilot and ROS. The mission therefore ends on a CONJUNCTION of two',
        'things the robot itself reports: the FCU\'s own',
        '`NAV_CONTROLLER_OUTPUT.wp_dist` inside the profile\'s `goal_tolerance_m`',
        'on three consecutive messages while the mode is still `GUIDED` and the',
        'vehicle armed, AND the robot\'s own estimate within',
        '`arrival_estimate_tolerance_m` of the setpoint this run published —',
        'the same rule `ral_eval`\'s `MavrosGuidedDriver` applies in a scenario',
        'run, re-implemented here rather than imported.',
        '',
        f'## Result: {"all checks passed" if passed else "FAILURES PRESENT"}',
        '',
        '| # | Check | Result | Measured |',
        '| --- | --- | --- | --- |',
    ]
    for number, name, ok, detail in results:
        cell = detail.replace('|', r'\|')
        doc.append(f'| {number} | {name} | {"PASS" if ok else "**FAIL**"} | {cell} |')
    doc += [
        '',
        '## Run',
        '',
        '| Item | Value |',
        '| --- | --- |',
        f'| World | `{WORLD}` |',
        f'| Robot | `{profile["model"]}` (profile '
        f'`ral_bringup/config/robots/{ROBOT}.yaml`) |',
        f'| Scenario | `{SCENARIO}` |',
        f'| Initial pose (world) | {initial_world} |',
        f'| Goal (world) | {goal_world} |',
        f'| Goal tolerance | `goal_tolerance_m` {goal_tolerance} m '
        f'(`WP_RADIUS`) AND `arrival_estimate_tolerance_m` '
        f'{arrival_estimate_tol} m, footprint radius {footprint_radius} m |',
        f'| Autopilot | `ardurover` SITL, `ral_bringup/config/{parm_file}` |',
        f'| `sha256({parm_file})` | `{parm_sha}` |',
        f'| `ARMING_CHECK` in force | `{arming_check}` '
        '(read from the `.parm` this run was started with; `0` disables the '
        'pre-arm checks that can only fail for simulation artefacts — an RC '
        'transmitter and a battery monitor SITL does not have) |',
        f'| Estimate | {profile["estimate"]["label"]}, `{profile["estimate"]["topic"]}` |',
        f'| `world_to_ekf_local` (measured this run) | {alignment_cell} |',
        '| Alignment source | '
        + (alignment.source if alignment is not None else 'not measured') + ' |',
        f'| Terminal outcome | `{mission_outcome or "none reported"}` |',
        f'| Mission duration | {mission_sim:.2f} s sim |',
        f'| Sim time elapsed | {sim_duration:.2f} s |',
        f'| Real-time factor under lockstep (`/world/{WORLD}/stats`) | {rtf} |',
        f'| Image | `{args.image_ref}` |',
        '',
        '## What these checks establish',
        '',
        '- The autopilot is a real one: MAVROS completed the MAVLink handshake,',
        '  the FCU accepted `GUIDED` and armed, and it is the FCU\'s own',
        '  `wp_dist` — not this harness\'s arithmetic and not simulator truth —',
        '  that decided the mission ended (checks 1, 7). Ground truth confirms',
        '  arrival independently of what the autopilot believed (check 8).',
        '- The estimate is an estimate: EKF3\'s pose differs from truth at every',
        '  sample but stays localized (check 11). Equality would mean truth had',
        '  leaked into localization.',
        '- The frame the autopilot works in was MEASURED, not assumed (check 6).',
        '  ArduPilot sets its EKF origin at its first GPS fix, so the offset',
        '  between the gz `world` frame and `ekf_local` is a per-run',
        '  measurement; `ekf_local_tf` makes it, publishes it on',
        '  `/frame_alignment` with the residual against the predicted origin,',
        '  and refuses rather than fudging it.',
        '- The vehicle is non-holonomic in fact and not only in its SDF: the',
        '  tightest turn its body actually made is measured from truth and',
        '  compared with the geometry the model and `TURN_RADIUS` declare',
        '  (check 9).',
        '- Truth reaches the harness, the fault injector and the frame measurer',
        '  and nobody else (check 4), and the complete TF frame set is asserted',
        '  (check 5), so truth bridged onto `/tf` under any name — including the',
        '  simulator\'s own `<world> -> <model>` names — fails as well. There is',
        '  no `odom` frame here at all: this platform has no wheel-odometry',
        '  integrator, and its estimate chain is `map -> ekf_local -> base_link`.',
        '- `/scan` has exactly one publisher, `scan_fault_relay`, and its output',
        '  was byte-identical to `/scan_raw` on every message pair of the whole',
        '  run (checks 2, 3).',
        '',
        '## Known limitations',
        '',
        '- **The steering is parallel, not true Ackermann.** Both knuckles take',
        '  `ArduPilotPlugin` channel 0 with identical gains, so the inner and',
        '  outer wheels steer to the same angle instead of to the two angles a',
        '  common turn centre requires. The tyres therefore scrub in tight',
        '  turns. What the MVP relies on survives: the vehicle is non-holonomic,',
        '  cannot rotate in place, and has a finite minimum turning radius —',
        '  check 9 measures what that radius actually was.',
        '- **Check 9 is a measurement over this run\'s samples and nothing more.**',
        '  It is `min(speed / |yaw rate|)` over the truth samples of ONE mission.',
        '  If this mission never demanded full steering lock, the number is a',
        '  lower bound for this run and not the plant\'s limit; it is evidence',
        '  about what the body did, not a specification of what it can do. The',
        '  `narrow_doorway_car` expectation rests on it together with the',
        '  geometric radius in the profile, not on it alone.',
        '- **`wp_dist` is a `uint16` in METRES, so arrival has two halves.** The',
        '  FCU\'s own distance to its target has 1 m resolution and TRUNCATES, so',
        f'  with `WP_RADIUS` {goal_tolerance} a reported '
        f'{goal_tolerance:.0f} means anywhere in '
        f'[{goal_tolerance:.0f}, {goal_tolerance + WP_DIST_QUANTUM_M:.0f}). That',
        '  half alone cannot say where the vehicle came to rest, so the mission',
        '  ends only when it is satisfied AND the robot\'s own estimate is within',
        f'  `arrival_estimate_tolerance_m` ({arrival_estimate_tol} m) of the',
        '  commanded setpoint — both terms the robot\'s own, truth never read.',
        '  Check 8 is scored against that gate plus the footprint radius plus',
        '  THIS run\'s measured maximum |estimate − truth|, so its limit is a sum',
        '  of measurements rather than a constant.',
        '- **The lidar is carried for evidence and consumed by nothing.**',
        '  ArduRover navigates on EKF3 position alone; no costmap and no',
        '  obstacle avoidance runs for this robot. `/scan` is bridged, relayed',
        '  and recorded so the sensor gate and the evidence cover both robots',
        '  identically — and that is exactly why `scan_degradation` is REFUSED',
        '  for this platform (`car.yaml`\'s `unsupported`) instead of being',
        '  offered as a fault with no effect.',
        '- **The no-progress stop is a deterministic policy, not autonomy.** It',
        '  lives in `ral_eval/mavros_driver.py` and belongs to the scenario',
        '  runner: ArduRover carries no obstacle avoidance and will push against',
        '  a wall until the scenario times out, and the policy turns that into a',
        '  safe stop with a named cause. It closes on the robot\'s own estimate,',
        '  never on truth. This harness does NOT run it — a harness that stopped',
        '  the vehicle on its own schedule would be measuring its own policy —',
        '  so an unfinished mission here is reported as "no terminal condition",',
        '  not as a stall.',
        '- **This platform publishes no numeric pose uncertainty.** ArduPilot',
        '  does not send `LOCAL_POSITION_NED_COV`, so `/mavros/local_position/*`',
        '  carries no covariance, and `mavros_msgs/msg/EstimatorStatus` in this',
        '  image carries the twelve EKF health FLAGS only — no',
        '  `pos_horiz_accuracy`. There is therefore no equivalent of the',
        '  Roomba\'s covariance trace, and no convergence instant to time: EKF3',
        '  sets its origin at the autopilot\'s first GPS fix with no',
        '  operator-triggered initialization.',
        f'- **Real-time factor under lockstep was {rtf}.**',
        '  `ArduPilotPlugin`\'s `<lock_step>1</lock_step>` ties the autopilot\'s',
        '  clock to gz\'s, so the two cannot drift apart, but the pair runs only',
        '  as fast as the slower of them. Every duration above is sim time and',
        '  is unaffected; the wall-clock cost of a run is not.',
        '- **One mission, no fault.** This harness runs a fault-free baseline on',
        '  purpose: it is the control the car\'s fault runs are read against.',
        '  The fault evidence is the scenario runs (`mobility_fault_car`,',
        '  `kidnap_car`, `narrow_doorway_car`), each with its own `run.json`,',
        '  `metrics.json` and `plot.png` under `runs/`.',
        '',
    ]
    doc_path = os.path.join(args.repo_root, 'docs', 'car-closed-loop.md')
    write_doc(doc_path, doc)
    print(f'--- launch log: {log_path}', flush=True)

    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
