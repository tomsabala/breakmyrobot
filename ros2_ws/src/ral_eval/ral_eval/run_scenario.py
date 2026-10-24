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
Scenario in, run directory out.

Reads one scenario file, brings the whole stack up, records a bag, localizes,
drives one mission, dispatches the scenario's interventions on SIM time, and
leaves behind runs/<experiment_id>_<UTC>/ containing scenario.yaml, env.json,
run.json, bag/, metrics.json, plot.png and launch.log.

Time
----
Every duration in a scenario and in the artifacts is SIM time off /clock.
Wall deadlines appear only as deadlock guards, on things that happen before
or outside the simulation (process start-up, discovery), and are named as
such. A slow real-time factor must change how long a run takes, never what it
measures.

Exit codes
----------
  0  the experiment RAN and was MEASURED. A mission that aborted, timed out or
     was refused before launch is data, not an error: the artifacts are
     complete and metrics.json says what happened.
  1  the map/alignment pair was refused (stale, unsealed or missing).
  2  the scenario is invalid, or the run directory already exists. Nothing is
     created under runs/.
  3  the stack never came up, or nav2_roomba.yaml / the map yaml does not
     carry the footprint limits the pre-flight checks against. run.json
     records what was missing.
  4  the bag is missing, unfinalised, empty, or short of the latched topics
     the QoS override pins; the run cannot be measured.
  5  metrics or the plot raised. The bag is kept so it can be recomputed.

Bag size
--------
RECORD_TOPICS is deliberately wide: a baseline run writes ~21 MB over ~75,000
messages, of which /joint_states contributes 29,170 and /scan plus /scan_raw
most of the remainder. rosbag2's Humble CLI has no per-topic throttle, so each
topic is all-or-nothing; /joint_states stays because wheel motion against
truth motion is how slip is observed, and that is what Phase 4.3's mobility
fault will be read from.

Concurrency
-----------
ONE run at a time per ROS_DOMAIN_ID. The stack owns fixed topic names and a
single gz server, so two runs in one domain would interleave commands into one
robot. The runner does not police this - there is no reliable way to tell a
second runner from a stale node - so scripts/run_scenario.sh takes an flock on
/tmp/ral-run-<domain>.lock and refuses the second caller.

Who publishes a FaultEvent
--------------------------
The fault manager (Phase 4) owns `requested`, `activated` and `cleared`,
because only it observes the underlying gz or relay call. This runner
publishes exactly one kind of FaultEvent: `rejected`, and only for the case
the manager cannot report - its service was not there to be called. In Phase 3
that is every intervention, by construction.

Frames
------
Scenario poses are in the gz WORLD frame. /initialpose and the NavigateToPose
goal are converted to `map` with ral_eval.frames and the alignment sealed to
the map in use; gz entity poses (spawn_obstacle, kidnap) are passed through
UNCONVERTED, because `world` is the frame gz defines them in. run.json records
both forms for the mission poses and `pose_frame: world` for the gz ones.

Unlike the Phase 2 proof harness in scripts/checks/, this module DOES import
ral_eval.frames and ral_eval.scenario. It is production code, not an
independent verifier: there must be exactly one conversion and one schema in
the thing being run.

Negative controls
-----------------
Green runs prove the runner works; they do not prove it can fail. Each
mutation below was applied and the named failure observed. Repeat them after
changing the corresponding path.

2026-10-23, image sha256:ce16e283:

* Set `world: ral_kitchen` in a copy of baseline_roomba.yaml. Observed: exit 2
  before anything was launched, `SCENARIO REJECTED: wrong_world.yaml: `world`
  is 'ral_kitchen'; the only authored world is 'ral_apartment' ...`, naming the
  bridge config, map and alignment a second world would need - and NOTHING was
  created under runs/, so a rejected scenario leaves no artifact that could be
  mistaken for a result.
* Set `goal: {x: 8.0, y: 0.5, yaw: 0.0}`, i.e. inside wall B. Observed:
  MissionState `unreachable`, outcome `not_sent:goal_cell_occupied`, detail
  naming 21 occupied cells of the 41 sampled within robot_radius of grid cell
  (160, 11); `run.json.goal_uuid` is null because no goal was ever sent; the
  run still exits 0 with a complete bag, metrics.json and plot.png, and every
  windowed metric is null carrying `no mission window: final state
  'unreachable'`. localization_converged_sim_s is still MEASURED (2.758 s sim),
  because it does not depend on the mission window.
* Delete both entries from config/bag_qos_overrides.yaml and rerun. Observed:
  the control FAILED TO FAIL - the bag still contained 1 /map and 2 /tf_static
  messages. rosbag2 adapts its subscription to the QoS the existing publishers
  offer, so the override is a pin rather than a repair; see the file's own
  comment. The claim that the bag would contain no map at all was wrong and has
  been corrected everywhere it appeared.
* An earlier version waited a fixed 3 s wall after starting the recorder.
  Observed: /initialpose recorded 0 messages and the entire localization
  warm-up was missing, because the recorder's last subscription landed 5.6 s
  after its process started; localization_converged_sim_s and the start-pose
  residual (then named est_error_floor_m, now est_error_at_start_pose_m)
  were both null. /mission_state survived only because it is published
  transient-local. The runner now waits for the recorder's own subscriptions to
  appear in the graph (wait_for_recorder) and records any that never did.
"""

import argparse
import hashlib
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import time
import traceback
from datetime import datetime, timezone

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from ral_eval.frames import (
    frames_path_for_map,
    grid_sha256,
    load_alignment_for_map,
    wrap_to_pi,
)
from ral_eval.scenario import ScenarioError, load_scenario, mission_poses
from ral_interfaces.msg import FaultEvent, MissionState
from ral_interfaces.srv import (
    ClearObstacle,
    Kidnap,
    SetMobilityFault,
    SetScanDegradation,
    SpawnObstacle,
)
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

import yaml

EXIT_OK = 0
EXIT_ALIGNMENT = 1
EXIT_SCENARIO = 2
EXIT_BRINGUP = 3
EXIT_BAG = 4
EXIT_ARTIFACTS = 5

MODEL = 'ral_roomba'

BRINGUP_WALL_LIMIT = 90.0        # s wall, guards a stack that never comes up
ACTION_SERVER_WALL_LIMIT = 60.0  # s wall for navigate_to_pose to appear
RECORDER_WALL_LIMIT = 60.0       # s wall for the recorder to subscribe to the gate topics
RECORDER_SETTLE_WALL = 1.0       # s wall after it subscribed, for publisher-side matching
AMCL_CONVERGE_SIM = 30.0         # s sim allowed for the filter to converge
AMCL_WARMUP_RATE = 0.6           # rad/s in-place rotation; AMCL updates on motion only
# AMCL publishes on filter UPDATE, and its filter updates on motion. The
# instant it converges the warm-up would otherwise stop, and the robot then
# stands still publishing nothing - measured: exactly ONE converged /amcl_pose
# before the goal, so est_error_at_start_pose_m was an average over a single
# sample. Keeping the in-place rotation going for a few more sim seconds costs
# nothing (xy is unchanged, so the samples are still taken at the start pose)
# and makes the residual a mean rather than one reading.
AMCL_FLOOR_SAMPLE_SIM = 4.0      # s sim of extra in-place rotation after convergence
AMCL_SETTLE_SIM = 2.0            # s sim stopped before reading the residual
COVARIANCE_LIMIT = 0.25          # trace(x, y, yaw) below which AMCL has converged
YAW_ALIGN_SIM = 20.0             # s sim allowed to turn back to the scenario's yaw
# > amcl update_min_a (0.2 rad, nav2_roomba.yaml:79). AMCL publishes on filter
# update and its filter updates on motion, so a tolerance below that quantum
# asks the loop to close on a signal finer than the estimate it reads.
YAW_ALIGN_TOLERANCE = 0.25       # rad (~14 deg), the heading the mission starts from
YAW_ALIGN_MIN_RATE = 0.15        # rad/s floor, so the last few degrees do not stall
GOAL_ACCEPT_WAIT_SIM = 30.0      # s sim for the action server to accept or refuse
SERVICE_WAIT_SIM = 5.0           # s sim an intervention waits for its service
POST_TERMINAL_SIM = 2.0          # s sim recorded after the terminal result
CANCEL_WAIT_SIM = 10.0           # s sim allowed for a cancel to produce a terminal status
# robot_radius and occupied_thresh are NOT constants here: they are read from
# the nav2 params and the map yaml this run actually used, and recorded in
# run.json. A footprint check against a guessed radius is not a check.

# The gz contact topic the /bumper bridge carries, per
# ral_bringup/config/bridge_roomba_<world>.yaml.
BUMPER_GZ_TOPIC = ('/world/{world}/model/ral_roomba/link/base_link/sensor/bumper/'
                   'contact')

# The mission states this runner can publish. test_mission_vocabulary asserts
# these are exactly MissionState's constants - a typo'd state string would
# otherwise surface only as a dashboard that silently never matches.
MISSION_STATES = (
    MissionState.STATE_IDLE,
    MissionState.STATE_GOAL_ACCEPTED,
    MissionState.STATE_NAVIGATING,
    MissionState.STATE_SUCCEEDED,
    MissionState.STATE_ABORTED,
    MissionState.STATE_CANCELED,
    MissionState.STATE_UNREACHABLE,
    MissionState.STATE_REJECTED,
    MissionState.STATE_UNKNOWN,
)

# Terminal action status -> mission state. Only these three are terminal for a
# goal this runner sent; anything else is reported as no terminal status
# rather than mapped to a guess.
TERMINAL_STATES = {
    GoalStatus.STATUS_SUCCEEDED: (MissionState.STATE_SUCCEEDED, 'STATUS_SUCCEEDED'),
    GoalStatus.STATUS_CANCELED: (MissionState.STATE_CANCELED, 'STATUS_CANCELED'),
    GoalStatus.STATUS_ABORTED: (MissionState.STATE_ABORTED, 'STATUS_ABORTED'),
}

# Topics recorded for every run, each because a metric or a plot panel needs
# it. The bag is the source of record; run.json is the fallback only for the
# action status.
RECORD_TOPICS = (
    '/clock',                               # the sim time every other stamp is in
    '/tf',                                  # map->odom->base_link, for inspection
    '/tf_static',                           # sensor mounting; latched, see the QoS file
    '/map',                                 # plot panel A's grid; latched, see the QoS file
    '/initialpose',                         # localization_converged_sim_s start
    '/amcl_pose',                           # est_error_*, est_cov_trace_max
    '/odom',                                # wheel odometry, for inspection
    '/ground_truth/odom',                   # path_length_m, goal_pose_error_m, achieved v
    '/scan',                                # the stream AMCL and the costmaps consume
    '/scan_raw',                            # Phase 4.2 proves degradation between the two
    '/cmd_vel',                             # stopped_time_s, cmd_vs_achieved
    '/plan',                                # plan_publications, fault_to_response_s
    '/bumper',                              # collision_count
    '/joint_states',                        # wheel motion; truth-vs-wheel divergence is
                                            # how slip is observed (Phase 4.3's mobility
                                            # fault, roomba_teleop_check 7)
    '/mission_state',                       # the mission window itself
    '/fault_events',                        # fault_to_response_s anchors
    '/rosout',                              # node-side errors beside the run
    '/navigate_to_pose/_action/status',     # nav2_result, verbatim
    '/navigate_to_pose/_action/feedback',   # recoveries, distance_remaining
)

# Topics whose PRE-MISSION history is evidence, so nothing may be published on
# them until the recorder is actually subscribed. `ros2 bag record` subscribes
# topic by topic after its own start-up; measured on this machine, the last
# subscription landed 5.6 s after the process started, and a fixed 3 s wait
# silently lost every /initialpose and the whole localization warm-up. The
# only reliable signal is the recorder's own subscription appearing in the
# graph, so that is what the runner waits for.
RECORDER_GATE_TOPICS = (
    '/initialpose',          # localization_converged_sim_s starts here
    '/cmd_vel',              # the warm-up rotation, and the start-pose residual window
    '/mission_state',        # the mission window
    '/fault_events',         # a fault rejected before the mission must be recorded
    '/amcl_pose',            # convergence, and the stationary residual
    '/ground_truth/odom',    # the truth half of that residual
)
RECORDER_NODE_PREFIX = 'rosbag2'


def wait_for_recorder(node, wall_limit: float = RECORDER_WALL_LIMIT) -> list:
    """
    Block until the recorder has subscribed to every gate topic.

    Returns the topics it never subscribed to, which the caller records in
    run.json. Those topics' pre-mission messages would be missing from the
    bag, and a metric computed from them would be null with a reason - never
    silently zero.
    """
    deadline = time.monotonic() + wall_limit
    missing = list(RECORDER_GATE_TOPICS)
    while time.monotonic() < deadline and missing:
        node.spin_wall(0.25)
        missing = [
            topic for topic in RECORDER_GATE_TOPICS
            if not any(info.node_name.startswith(RECORDER_NODE_PREFIX)
                       for info in node.get_subscriptions_info_by_topic(topic))]
    # The subscription is visible in the graph; give the local publishers one
    # second of wall time to finish matching it before anything is published.
    node.spin_wall(RECORDER_SETTLE_WALL)
    return missing


def bumper_liveness(node, world: str) -> dict:
    """
    Prove the contact path is wired without waiting for a contact.

    A gz contact sensor publishes only while two bodies touch, so a silent
    /bumper is the normal case for a clean run. The only way `collision_count:
    0` can be a measurement rather than an absence is to show, in this run,
    that the sensor exists in gz and the bridge is publishing the ROS topic.
    """
    publishers = sorted(info.node_name
                        for info in node.get_publishers_info_by_topic('/bumper'))
    gz_topic = BUMPER_GZ_TOPIC.format(world=world)
    try:
        listed = subprocess.run(['gz', 'topic', '-l'], capture_output=True,
                                text=True, timeout=30).stdout
        matches = [line.strip() for line in listed.splitlines()
                   if line.strip().endswith('/sensor/bumper/contact')]
        gz_present = gz_topic in listed
        if gz_present:
            gz_detail = 'listed by gz topic -l'
        elif matches:
            # The bridge config names the full path; if gz lists it under a
            # different one, the full listing line is the honest evidence.
            gz_detail = f'NOT listed under that name; gz topic -l has {matches}'
        else:
            gz_detail = 'NOT listed by gz topic -l'
    except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
        gz_present = None
        gz_detail = f'unavailable ({type(exc).__name__})'
    return {'ros_publishers': publishers, 'gz_topic': gz_topic,
            'gz_topic_present': gz_present, 'gz_detail': gz_detail,
            'alive': bool(publishers) and gz_present is True}


def injection_point(kind: str, world: str) -> str:
    """Return the literal gz service/topic or ROS node a fault kind acts on."""
    return {
        FaultEvent.KIND_SPAWN_OBSTACLE: f'/world/{world}/create (gz UserCommands)',
        FaultEvent.KIND_CLEAR_OBSTACLE: f'/world/{world}/remove (gz UserCommands)',
        FaultEvent.KIND_SCAN_DEGRADATION: 'scan_fault_relay (driver/transport level)',
        FaultEvent.KIND_MOBILITY_FAULT: f'/world/{world}/wrench/persistent (gz physics)',
        FaultEvent.KIND_KIDNAP: f'/world/{world}/set_pose (gz entity pose)',
    }[kind]


def twist(linear_x: float = 0.0, angular_z: float = 0.0) -> Twist:
    msg = Twist()
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    return msg


def quaternion_of_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def yaw_of_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _sha256_file(path: str) -> str:
    with open(path, 'rb') as handle:
        return hashlib.sha256(handle.read()).hexdigest()


def _git_sha(cwd: str) -> str:
    """HEAD of the repository holding the run, or a stated unknown."""
    try:
        result = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=cwd,
                                capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired):
        return 'unknown (git unavailable)'
    if result.returncode != 0:
        return 'unknown (git unavailable)'
    return result.stdout.strip() or 'unknown (git unavailable)'


def _git_dirty(cwd: str) -> 'bool | None':
    """
    Report whether the tree has uncommitted changes; None when git is absent.

    A commit SHA recorded beside uncommitted changes is a false provenance
    claim: the code that produced the run is not the code at that commit.
    """
    try:
        result = subprocess.run(['git', 'status', '--porcelain'], cwd=cwd,
                                capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired):
        return None
    if result.returncode != 0:
        return None
    return bool(result.stdout.strip())


def _git_diff_sha256(cwd: str) -> 'str | None':
    """
    Fingerprint the uncommitted delta, so a dirty run is still identifiable.

    `git_sha` alone names a commit the code did not come from. This hashes
    `git diff HEAD` concatenated with `git status --porcelain`, which together
    cover tracked modifications and untracked paths. None when git is absent
    or either command fails - stated, never guessed.
    """
    parts = []
    for command in (['git', 'diff', 'HEAD'], ['git', 'status', '--porcelain']):
        try:
            result = subprocess.run(command, cwd=cwd, capture_output=True,
                                    text=True, timeout=30)
        except (OSError, subprocess.TimeoutExpired):
            return None
        if result.returncode != 0:
            return None
        parts.append(result.stdout)
    return hashlib.sha256(''.join(parts).encode()).hexdigest()


def write_json(path: str, payload: dict) -> None:
    """Atomic write: a reader never sees half an artifact."""
    temporary = path + '.partial'
    with open(temporary, 'w') as handle:
        json.dump(payload, handle, indent=2, sort_keys=False)
        handle.write('\n')
    os.replace(temporary, path)


def measure_rtf(world: str) -> str:
    """Read the real-time factor gz reports for this world."""
    try:
        out = subprocess.run(
            ['gz', 'topic', '-e', '-t', f'/world/{world}/stats', '-n', '3'],
            capture_output=True, text=True, timeout=30).stdout
    except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
        return f'unavailable ({type(exc).__name__})'
    values = re.findall(r'real_time_factor:\s*([0-9.]+)', out)
    if not values:
        return 'unavailable (no real_time_factor in /world stats)'
    return ', '.join(values)


class RunAbort(Exception):
    """Stop the run with a specific exit code; artifacts are still written."""

    def __init__(self, code: int, message: str) -> None:
        super().__init__(message)
        self.code = code


# -- intervention requests ----------------------------------------------------
# gz entity poses (spawn_obstacle, kidnap) are WORLD frame by definition and
# are passed through unconverted. Only /initialpose and the Nav2 goal go
# through world_pose_to_map. Converting an obstacle pose would place the box
# somewhere else entirely, so the rule is written here, beside the code that
# would be the obvious wrong place to "fix".

def _spawn_request(params: dict) -> SpawnObstacle.Request:
    request = SpawnObstacle.Request()
    request.name = params['name']
    request.pose.position.x = params['pose']['x']
    request.pose.position.y = params['pose']['y']
    qx, qy, qz, qw = quaternion_of_yaw(params['pose']['yaw'])
    request.pose.orientation.x = qx
    request.pose.orientation.y = qy
    request.pose.orientation.z = qz
    request.pose.orientation.w = qw
    request.size.x = params['size']['x']
    request.size.y = params['size']['y']
    request.size.z = params['size']['z']
    return request


def _clear_request(params: dict) -> ClearObstacle.Request:
    request = ClearObstacle.Request()
    request.name = params['name']
    return request


def _scan_request(params: dict) -> SetScanDegradation.Request:
    request = SetScanDegradation.Request()
    request.enabled = params['enabled']
    request.range_noise_stddev_m = params['range_noise_stddev_m']
    request.max_range_m = params['max_range_m']
    request.blind_sector_start_deg = params['blind_sector_start_deg']
    request.blind_sector_end_deg = params['blind_sector_end_deg']
    request.dropout_probability = params['dropout_probability']
    return request


def _mobility_request(params: dict) -> SetMobilityFault.Request:
    request = SetMobilityFault.Request()
    request.enabled = params['enabled']
    request.force_x_n = params['force_x_n']
    request.force_y_n = params['force_y_n']
    request.torque_z_nm = params['torque_z_nm']
    return request


def _kidnap_request(params: dict) -> Kidnap.Request:
    request = Kidnap.Request()
    request.new_pose.position.x = params['pose']['x']
    request.new_pose.position.y = params['pose']['y']
    qx, qy, qz, qw = quaternion_of_yaw(params['pose']['yaw'])
    request.new_pose.orientation.x = qx
    request.new_pose.orientation.y = qy
    request.new_pose.orientation.z = qz
    request.new_pose.orientation.w = qw
    return request


def _target_of(kind: str, params: dict) -> str:
    """Return the entity or stream a fault acts on, for FaultEvent.target."""
    if kind in (FaultEvent.KIND_SPAWN_OBSTACLE, FaultEvent.KIND_CLEAR_OBSTACLE):
        return params['name']
    if kind == FaultEvent.KIND_SCAN_DEGRADATION:
        return '/scan'
    return MODEL


# The single place an intervention type maps to the service that implements
# it. Phase 4's fault_manager must bind exactly these names.
DISPATCH = {
    FaultEvent.KIND_SPAWN_OBSTACLE: ('/faults/spawn_obstacle', SpawnObstacle, _spawn_request),
    FaultEvent.KIND_CLEAR_OBSTACLE: ('/faults/clear_obstacle', ClearObstacle, _clear_request),
    FaultEvent.KIND_SCAN_DEGRADATION: ('/faults/set_scan_degradation', SetScanDegradation,
                                       _scan_request),
    FaultEvent.KIND_MOBILITY_FAULT: ('/faults/set_mobility_fault', SetMobilityFault,
                                     _mobility_request),
    FaultEvent.KIND_KIDNAP: ('/faults/kidnap', Kidnap, _kidnap_request),
}


class ScenarioRunner(Node):
    """Drives one scenario against a running stack. Observes; never fakes."""

    def __init__(self, experiment_id: str) -> None:
        super().__init__(
            'run_scenario',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.experiment_id = experiment_id

        self.odom = None
        self.truth = None
        self.scan = None
        self.scan_raw = None
        self.grid = None
        self.amcl = None
        # The sim instant the filter's covariance trace FIRST fell below
        # COVARIANCE_LIMIT, observed in the callback. Sampling it later, after
        # the extra rotation and the yaw alignment, would measure the whole
        # warm-up and publish it under the convergence metric's name -
        # measured 17.01 s against metrics.json's 2.786 s for the same run.
        self.converged_sim = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Depth 10, transient local: a dashboard or a recorder that subscribes
        # after a transition still receives it. The bag gets them anyway - the
        # recorder is started before the first publication - but a late
        # /mission_state subscriber seeing nothing would look like a stalled
        # run rather than a missed message.
        event_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self._on_truth, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_raw', self._on_scan_raw,
                                 qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        # Used ONLY for the pre-mission localization warm-up. Nav2's
        # velocity_smoother owns /cmd_vel once a goal is active.
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mission_pub = self.create_publisher(MissionState, '/mission_state', event_qos)
        self.fault_pub = self.create_publisher(FaultEvent, '/fault_events', event_qos)

    # -- subscriptions --------------------------------------------------------
    def _on_odom(self, msg):
        self.odom = msg

    def _on_truth(self, msg):
        self.truth = msg

    def _on_scan(self, msg):
        self.scan = msg

    def _on_scan_raw(self, msg):
        self.scan_raw = msg

    def _on_map(self, msg):
        self.grid = msg

    def _on_amcl(self, msg):
        self.amcl = msg
        if self.converged_sim is None:
            cov = msg.pose.covariance
            if cov[0] + cov[7] + cov[35] < COVARIANCE_LIMIT:
                self.converged_sim = self.sim_now()

    # -- time -----------------------------------------------------------------
    def sim_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_wall(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def spin_sim(self, sim_seconds: float, until=None, cmd: Twist = None) -> bool:
        """
        Spin until `sim_seconds` of SIM time pass, or `until()` fires.

        Republishes `cmd` at 20 Hz while spinning, for the localization
        warm-up only. The wall guard exists so a stopped /clock cannot hang
        the run for ever; it is deliberately ten times the sim budget.
        """
        start = self.sim_now()
        wall_deadline = time.monotonic() + sim_seconds * 10.0 + 60.0
        last_cmd = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.monotonic()
            if cmd is not None and now - last_cmd >= 0.05:
                self.cmd_pub.publish(cmd)
                last_cmd = now
            if until is not None and until():
                return True
            if self.sim_now() - start >= sim_seconds or now > wall_deadline:
                return False
        return False

    def stop(self) -> None:
        for _ in range(5):
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    # -- publications ---------------------------------------------------------
    def publish_initial_pose(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        qx, qy, qz, qw = quaternion_of_yaw(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # Nav2's own RViz tool uses these; they state a believable initial
        # uncertainty rather than claiming the pose is exact.
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942
        self.initial_pose_pub.publish(msg)

    def publish_mission_state(self, state: str, goal: PoseStamped,
                              outcome: str = '', detail: str = '') -> None:
        msg = MissionState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.experiment_id = self.experiment_id
        msg.state = state
        msg.goal = goal
        msg.outcome = outcome
        msg.detail = detail
        self.mission_pub.publish(msg)
        print(f'--- mission_state {state} outcome={outcome!r} {detail}', flush=True)

    def publish_fault_rejected(self, fault_id: str, kind: str, world: str,
                               target: str, detail: str) -> None:
        """
        Publish the ONE FaultEvent this runner is entitled to publish.

        The fault manager owns requested/activated/cleared because only it
        observes the underlying call. This covers the case it cannot report:
        its service was not there to be called at all.
        """
        msg = FaultEvent()
        msg.stamp = self.get_clock().now().to_msg()
        msg.id = fault_id
        msg.kind = kind
        msg.injection_point = injection_point(kind, world)
        msg.target = target
        msg.state = FaultEvent.STATE_REJECTED
        msg.detail = detail
        self.fault_pub.publish(msg)
        print(f'--- fault_event rejected {fault_id}: {detail}', flush=True)


def align_to_initial_yaw(node: 'ScenarioRunner',
                         target_yaw: float) -> 'tuple[bool, float | None]':
    """
    Turn in place until the ESTIMATED yaw matches the scenario's initial yaw.

    Returns (aligned, estimated yaw error in rad). The error is None when AMCL
    has published no pose at all, which is reported rather than treated as
    zero.

    The loop closes on /amcl_pose - the robot's own estimate - and never on
    /ground_truth/odom. Steering the body with truth would make the start pose
    exact in a way no real robot's would be, and would be the localization
    shortcut ENGINEERING.md forbids. The consequence is honest: if AMCL is
    wrong about the heading, the mission starts from the heading AMCL believes,
    and run.json records the residual it believed it had.
    """
    deadline = node.sim_now() + YAW_ALIGN_SIM
    wall_deadline = time.monotonic() + YAW_ALIGN_SIM * 10.0 + 60.0
    error = None
    last_cmd = 0.0
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
        if node.amcl is not None:
            error = wrap_to_pi(target_yaw - yaw_of_quaternion(node.amcl.pose.pose.orientation))
            if abs(error) <= YAW_ALIGN_TOLERANCE:
                node.stop()
                node.spin_sim(AMCL_SETTLE_SIM)
                return True, error
        now = time.monotonic()
        if now - last_cmd >= 0.05:
            rate = 0.0 if error is None else math.copysign(
                min(AMCL_WARMUP_RATE, max(YAW_ALIGN_MIN_RATE, abs(error))), error)
            node.cmd_pub.publish(twist(angular_z=rate))
            last_cmd = now
        if node.sim_now() > deadline or now > wall_deadline:
            break
    node.stop()
    node.spin_sim(AMCL_SETTLE_SIM)
    return False, error


def goal_pose_stamped(node: ScenarioRunner, goal_map) -> PoseStamped:
    """Build the goal in `map`, stamped now: used by the action and the state."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = goal_map[0]
    pose.pose.position.y = goal_map[1]
    qx, qy, qz, qw = quaternion_of_yaw(goal_map[2])
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def preflight_goal(grid: OccupancyGrid, goal_map, robot_radius_m: float,
                   occupied_thresh: int) -> dict:
    """
    Decide whether a pose is worth sending, from the static map alone.

    Samples every cell whose centre lies within `robot_radius_m` of the pose.
    Occupied (>= `occupied_thresh`) or outside the grid means the pose cannot
    be occupied by this body and the goal is NOT sent - the mission state is
    `unreachable`, which is a pre-flight refusal and never an inference from a
    Nav2 abort.

    Both limits are ARGUMENTS, read by the caller from the nav2 params and the
    map yaml this run actually used. A footprint check against a guessed
    radius is not a check.

    Unknown (-1) cells are ALLOWED: planner_server's GridBased planner runs
    with allow_unknown: true, so refusing them would make this verdict
    disagree with the planner that would actually be asked to plan. The count
    is reported so a reader can see how much of the footprint was unknown.

    Returns {'ok': bool, 'outcome': str, 'detail': str}.
    """
    info = grid.info
    res = info.resolution
    ox = info.origin.position.x
    oy = info.origin.position.y
    oyaw = yaw_of_quaternion(info.origin.orientation)
    cos_o = math.cos(-oyaw)
    sin_o = math.sin(-oyaw)

    radius_cells = int(math.ceil(robot_radius_m / res))
    dx = goal_map[0] - ox
    dy = goal_map[1] - oy
    # Into grid-aligned coordinates (origin.yaw is 0 for this map, but a
    # rotated origin is legal in nav_msgs/OccupancyGrid and must not silently
    # index the wrong cell).
    gx = dx * cos_o - dy * sin_o
    gy = dx * sin_o + dy * cos_o
    centre_col = int(math.floor(gx / res))
    centre_row = int(math.floor(gy / res))

    occupied = []
    unknown = 0
    outside = 0
    sampled = 0
    for row in range(centre_row - radius_cells, centre_row + radius_cells + 1):
        for col in range(centre_col - radius_cells, centre_col + radius_cells + 1):
            cell_x = (col + 0.5) * res
            cell_y = (row + 0.5) * res
            if math.hypot(cell_x - gx, cell_y - gy) > robot_radius_m:
                continue
            sampled += 1
            if not (0 <= col < info.width and 0 <= row < info.height):
                outside += 1
                continue
            value = grid.data[row * info.width + col]
            if value < 0:
                unknown += 1
            elif value >= occupied_thresh:
                occupied.append((col, row, int(value)))

    # The centre's own status, because `outside` counts ANY sampled footprint
    # cell: a pose 0.10 m inside the border is refused for its footprint while
    # its own cell is well inside the grid, and the two must not read alike.
    if 0 <= centre_col < info.width and 0 <= centre_row < info.height:
        centre_value = int(grid.data[centre_row * info.width + centre_col])
        centre_status = f'the goal centre cell is inside the grid with value {centre_value}'
    else:
        centre_status = 'the goal centre cell is itself outside the grid'

    footprint = (f'{sampled} cell(s) within {robot_radius_m} m of the goal at '
                 f'map ({goal_map[0]:.3f}, {goal_map[1]:.3f}), grid cell '
                 f'({centre_col}, {centre_row}) of {info.width}x{info.height} at '
                 f'{res} m: {unknown} unknown (allowed, planner allow_unknown is true), '
                 f'{len(occupied)} occupied (>= {occupied_thresh}), {outside} outside; '
                 f'{centre_status}')

    if outside:
        return {'ok': False, 'outcome': 'not_sent:footprint_outside_map',
                'detail': footprint}
    if occupied:
        values = ', '.join(f'({c},{r})={v}' for c, r, v in occupied[:8])
        return {'ok': False, 'outcome': 'not_sent:goal_cell_occupied',
                'detail': f'{footprint}; occupied cells: {values}'}
    return {'ok': True, 'outcome': '', 'detail': footprint}


def read_robot_radius(nav2_params_path: str) -> 'tuple[float, dict]':
    """
    Read the footprint radius the costmaps this run used were configured with.

    Returns (radius, {section: value}). When the two costmaps disagree the
    LARGER is used: a footprint check that passes for one costmap and fails
    for the other is not a check. A missing key raises, because a guessed
    radius would turn the pre-flight refusal into a coin toss.
    """
    with open(nav2_params_path) as handle:
        doc = yaml.safe_load(handle) or {}
    found = {}
    for section in ('global_costmap', 'local_costmap'):
        params = (doc.get(section) or {}).get(section, {}).get('ros__parameters', {})
        if isinstance(params, dict) and 'robot_radius' in params:
            found[section] = float(params['robot_radius'])
    if 'global_costmap' not in found:
        raise RunAbort(
            EXIT_BRINGUP,
            f'{nav2_params_path} has no '
            'global_costmap.global_costmap.ros__parameters.robot_radius; the goal '
            'pre-flight has no footprint to check and will not guess one')
    return max(found.values()), found


def read_occupied_thresh(map_yaml_path: str) -> int:
    """`occupied_thresh` from the map this run used, on the grid's 0..100 scale."""
    with open(map_yaml_path) as handle:
        meta = yaml.safe_load(handle) or {}
    if 'occupied_thresh' not in meta:
        raise RunAbort(
            EXIT_BRINGUP,
            f'{map_yaml_path} has no occupied_thresh; the goal pre-flight has no '
            'occupancy limit to check against and will not guess one')
    return int(round(float(meta['occupied_thresh']) * 100.0))


def record_undispatched(record: dict, pending: list, world: str, reason: str,
                        detail: str) -> None:
    """
    Record every intervention that never fired, with why.

    The schema only rejects at_sec >= timeout_sec, so an intervention whose
    at_sec exceeds the ACTUAL mission duration is legal and simply never
    reached - and without this it leaves no trace anywhere, which reads as a
    baseline run. No FaultEvent is published for these: nothing was requested,
    so there is nothing to reject.
    """
    for intervention in pending:
        record['interventions'].append({
            'id': intervention['id'],
            'type': intervention['type'],
            'at_sec': intervention['at_sec'],
            'dispatched_sim_s': None,
            'service': DISPATCH[intervention['type']][0],
            'injection_point': injection_point(intervention['type'], world),
            'target': _target_of(intervention['type'], intervention['params']),
            'params': intervention['params'],
            'pose_frame': 'world' if 'pose' in intervention['params'] else None,
            'available': False,
            'success': False,
            'message': detail,
            'result': f'not_dispatched:{reason}',
        })


def resolve_scenario_path(given: str) -> str:
    """Resolve a path as given, or a bare name in the installed scenarios."""
    candidates = [given, os.path.abspath(given)]
    share = os.path.join(get_package_share_directory('ral_eval'), 'scenarios')
    candidates += [os.path.join(share, given), os.path.join(share, given + '.yaml')]
    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate
    raise ScenarioError(
        f'no scenario at {given!r}. Tried: {candidates}. Installed scenarios: '
        f'{sorted(os.listdir(share)) if os.path.isdir(share) else "none"}')


def dispatch_intervention(node: ScenarioRunner, intervention: dict, world: str,
                          dispatched_sim: float) -> dict:
    """
    Call the service that implements one intervention, and report honestly.

    Returns the run.json record. The runner never claims an effect: `applied`
    is true only when the service itself returned success, and when the
    service does not exist - which is every intervention until Phase 4 - the
    record says `not_applied` and one FaultEvent{state: rejected} is published.
    """
    kind = intervention['type']
    service_name, service_type, build_request = DISPATCH[kind]
    target = _target_of(kind, intervention['params'])
    record = {
        'id': intervention['id'],
        'type': kind,
        'at_sec': intervention['at_sec'],
        'dispatched_sim_s': dispatched_sim,
        'service': service_name,
        'injection_point': injection_point(kind, world),
        'target': target,
        'params': intervention['params'],
        # gz entity poses stay in `world`; see the module docstring.
        'pose_frame': 'world' if 'pose' in intervention['params'] else None,
        'available': False,
        'success': False,
        'message': '',
        'result': 'not_applied',
    }

    client = node.create_client(service_type, service_name)
    deadline = node.sim_now() + SERVICE_WAIT_SIM
    wall_deadline = time.monotonic() + SERVICE_WAIT_SIM * 10.0 + 30.0
    while not client.service_is_ready():
        rclpy.spin_once(node, timeout_sec=0.02)
        if node.sim_now() > deadline or time.monotonic() > wall_deadline:
            break

    if not client.service_is_ready():
        detail = (f'service {service_name} unavailable after {SERVICE_WAIT_SIM:.1f} s sim '
                  '(ral_faults/fault_manager.py implements it in Phase 4)')
        record['message'] = detail
        node.publish_fault_rejected(intervention['id'], kind, world, target, detail)
        node.destroy_client(client)
        return record

    record['available'] = True
    future = client.call_async(build_request(intervention['params']))
    deadline = node.sim_now() + SERVICE_WAIT_SIM
    wall_deadline = time.monotonic() + SERVICE_WAIT_SIM * 10.0 + 30.0
    while not future.done():
        rclpy.spin_once(node, timeout_sec=0.02)
        if node.sim_now() > deadline or time.monotonic() > wall_deadline:
            break

    if not future.done():
        detail = (f'{service_name} did not answer within {SERVICE_WAIT_SIM:.1f} s sim; '
                  'the fault cannot be shown to have been applied')
        record['message'] = detail
        node.publish_fault_rejected(intervention['id'], kind, world, target, detail)
        node.destroy_client(client)
        return record

    response = future.result()
    record['success'] = bool(response.success)
    record['message'] = response.message
    if response.success:
        # The fault manager publishes requested/activated/cleared; it observed
        # the gz call, this runner did not.
        record['result'] = 'applied'
    else:
        node.publish_fault_rejected(
            intervention['id'], kind, world, target,
            f'{service_name} returned success=false: {response.message}')
    node.destroy_client(client)
    return record


def run(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--scenario', required=True,
                        help='scenario path, or a bare name installed in '
                             'share/ral_eval/scenarios/')
    parser.add_argument('--runs-dir', default='/ws/runs')
    parser.add_argument('--image-ref', default='unknown',
                        help='ral:dev digest or image id, collected on the host')
    parser.add_argument('--repo-root', default='/ws')
    parser.add_argument('--map', default=None,
                        help='map yaml; defaults to ral_bringup share maps/apartment.yaml')
    args = parser.parse_args(argv)

    bringup_share = get_package_share_directory('ral_bringup')
    eval_share = get_package_share_directory('ral_eval')
    map_yaml = args.map or os.path.join(bringup_share, 'maps', 'apartment.yaml')
    nav2_params = os.path.join(bringup_share, 'config', 'nav2_roomba.yaml')
    qos_overrides = os.path.join(eval_share, 'config', 'bag_qos_overrides.yaml')

    # 1. The scenario. Invalid -> nothing is created under runs/.
    try:
        scenario_path = resolve_scenario_path(args.scenario)
        scenario = load_scenario(scenario_path)
    except ScenarioError as exc:
        print(f'SCENARIO REJECTED: {exc}', file=sys.stderr, flush=True)
        return EXIT_SCENARIO

    # 2. The alignment that belongs to this exact grid. A stale or unsealed
    # pair raises, naming the file and the command that fixes it.
    try:
        wtm = load_alignment_for_map(map_yaml)
    except (RuntimeError, OSError) as exc:
        print(f'MAP/ALIGNMENT REJECTED: {exc}', file=sys.stderr, flush=True)
        return EXIT_ALIGNMENT
    poses = mission_poses(scenario, wtm)
    goal_map = poses['goal']['map']
    initial_map = poses['initial']['map']

    # 2b. The footprint limits the pre-flight will check against, read from the
    # files this run actually uses rather than duplicated as constants.
    try:
        robot_radius_m, radius_sources = read_robot_radius(nav2_params)
        occupied_thresh = read_occupied_thresh(map_yaml)
    except RunAbort as abort:
        print(f'PARAMS REJECTED: {abort}', file=sys.stderr, flush=True)
        return abort.code

    # 3. The run directory and the provenance that must exist even if the
    # stack never comes up.
    started = datetime.now(timezone.utc)
    run_dir = os.path.join(args.runs_dir,
                           f'{scenario["experiment_id"]}_'
                           f'{started.strftime("%Y%m%dT%H%M%SZ")}')
    try:
        # NOT exist_ok: the directory name resolves to one second, and a second
        # run in the same second would leave the previous run's metrics.json
        # and plot.png beside the new run.json.
        os.makedirs(run_dir)
    except FileExistsError:
        print(f'RUN DIRECTORY EXISTS: {run_dir}; a second run in the same UTC second '
              'would mix two runs\' artifacts', file=sys.stderr, flush=True)
        return EXIT_SCENARIO
    shutil.copyfile(scenario_path, os.path.join(run_dir, 'scenario.yaml'))

    # The alignment that belongs to THIS map, never a hardcoded filename: the
    # copy recorded here is the one metrics.py scores every truth-vs-estimate
    # number against, so recording a different file's contents than the one
    # load_alignment_for_map sealed would put a constant frame offset into
    # every one of them.
    alignment_path = frames_path_for_map(map_yaml)
    with open(alignment_path) as handle:
        alignment_text = handle.read()
    git_dirty = _git_dirty(args.repo_root)
    git_sha = _git_sha(args.repo_root)
    launch_command = ['ros2', 'launch', 'ral_bringup', 'roomba_nav.launch.py']
    write_json(os.path.join(run_dir, 'env.json'), {
        'image_ref': args.image_ref,
        'git_sha': f'{git_sha}+dirty' if git_dirty else git_sha,
        'git_dirty': git_dirty,
        'git_diff_sha256': _git_diff_sha256(args.repo_root) if git_dirty else None,
        'ros_distro': os.environ.get('ROS_DISTRO', 'unknown'),
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', 'unset'),
        'world': scenario['world'],
        'robot': scenario['robot'],
        'seed': scenario['seed'],
        'map_yaml': map_yaml,
        'map_sha256': grid_sha256(map_yaml),
        'alignment': alignment_text,
        'alignment_path': alignment_path,
        'nav2_params_path': nav2_params,
        'nav2_params_sha256': _sha256_file(nav2_params),
        'scenario_path': scenario_path,
        'scenario_sha256': _sha256_file(scenario_path),
        'launch_command': launch_command,
        'record_topics': list(RECORD_TOPICS),
        'bag_qos_overrides_path': qos_overrides,
        'bag_qos_overrides_sha256': _sha256_file(qos_overrides),
        'gate_reference': 'docs/gate.md',
        'created_utc': started.strftime('%Y-%m-%dT%H:%M:%SZ'),
    })
    print(f'--- run directory {run_dir}', flush=True)
    print(f'--- initial pose world {poses["initial"]["world"]} -> map '
          f'({initial_map[0]:.3f}, {initial_map[1]:.3f}, {initial_map[2]:.3f})', flush=True)
    print(f'--- goal world {poses["goal"]["world"]} -> map '
          f'({goal_map[0]:.3f}, {goal_map[1]:.3f}, {goal_map[2]:.3f})', flush=True)

    record = {
        'experiment_id': scenario['experiment_id'],
        'run_dir': run_dir,
        'scenario_path': scenario_path,
        'started_utc': started.strftime('%Y-%m-%dT%H:%M:%SZ'),
        'finished_utc': None,
        'sim_start': None,
        'sim_end': None,
        't_accept_sim': None,
        't_terminal_sim': None,
        'goal_uuid': None,
        'goal_map': list(goal_map),
        'goal_world': list(poses['goal']['world']),
        'initial_map': list(initial_map),
        'initial_world': list(poses['initial']['world']),
        'world_to_map': list(wtm),
        'recorder_unsubscribed_topics': None,
        'bumper_liveness': None,
        'initial_yaw_aligned': None,
        'initial_yaw_error_rad': None,
        'localization_converged': None,
        'localization_cov_trace': None,
        'localization_converged_sim_s': None,
        'warmup_total_sim_s': None,
        'preflight_initial': None,
        'preflight': None,
        'mission_state': None,
        'outcome': None,
        'nav2_status_code': None,
        'nav2_status_name': None,
        'detail': None,
        'interventions': [],
        'rtf_samples': 'not measured',
        'bag_dir': os.path.join(run_dir, 'bag'),
        'exit_code': None,
    }

    rclpy.init()
    node = ScenarioRunner(scenario['experiment_id'])
    launch_log = open(os.path.join(run_dir, 'launch.log'), 'w')
    launch = subprocess.Popen(launch_command, stdout=launch_log,
                              stderr=subprocess.STDOUT, start_new_session=True)
    recorder = None
    recorder_log = None
    exit_code = EXIT_OK

    try:
        # 5. Bring-up gate. A stack that never comes up is a failed run with
        # artifacts, not a traceback.
        needed = {
            '/clock': lambda: node.sim_now() > 0.0,
            '/odom': lambda: node.odom is not None,
            '/ground_truth/odom': lambda: node.truth is not None,
            '/scan': lambda: node.scan is not None,
            '/map': lambda: node.grid is not None,
        }
        deadline = time.monotonic() + BRINGUP_WALL_LIMIT
        missing = list(needed)
        while time.monotonic() < deadline:
            node.spin_wall(0.5)
            missing = [topic for topic, ready in needed.items() if not ready()]
            if not missing:
                break
        if missing:
            record['outcome'] = 'bringup_failed'
            record['mission_state'] = 'none'
            record['detail'] = (f'no messages on {", ".join(missing)} within '
                                f'{BRINGUP_WALL_LIMIT:.0f} s wall; see launch.log')
            raise RunAbort(EXIT_BRINGUP, record['detail'])

        nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        server_deadline = time.monotonic() + ACTION_SERVER_WALL_LIMIT
        while time.monotonic() < server_deadline and not nav_client.server_is_ready():
            node.spin_wall(0.5)
        if not nav_client.server_is_ready():
            record['outcome'] = 'bringup_failed'
            record['mission_state'] = 'none'
            record['detail'] = (
                f'navigate_to_pose never appeared within {ACTION_SERVER_WALL_LIMIT:.0f} s '
                'wall; Nav2 did not activate (see launch.log)')
            raise RunAbort(EXIT_BRINGUP, record['detail'])

        record['sim_start'] = node.sim_now()
        print(f'--- stack up at sim t={record["sim_start"]:.2f} s', flush=True)

        # 5b. Prove the contact path exists NOW, while the world is up. A gz
        # contact sensor publishes only on contact, so without this
        # `collision_count: 0` cannot be told from a dead bridge.
        record['bumper_liveness'] = bumper_liveness(node, scenario['world'])
        print(f'--- bumper liveness {record["bumper_liveness"]}', flush=True)

        # 6. Record BEFORE localizing: the warm-up rotation, /initialpose and
        # AMCL's convergence are evidence, and metrics.py windows them out by
        # the mission window rather than by never having recorded them.
        bag_dir = os.path.join(run_dir, 'bag')
        recorder_log = open(os.path.join(run_dir, 'bag_record.log'), 'w')
        recorder = subprocess.Popen(
            ['ros2', 'bag', 'record', '-s', 'mcap', '-o', bag_dir, '--use-sim-time',
             '--include-hidden-topics', '--qos-profile-overrides-path', qos_overrides,
             *RECORD_TOPICS],
            stdout=recorder_log, stderr=subprocess.STDOUT, start_new_session=True)
        unrecorded = wait_for_recorder(node)
        record['recorder_unsubscribed_topics'] = unrecorded
        if unrecorded:
            print(f'--- WARNING: the recorder never subscribed to {unrecorded} within '
                  f'{RECORDER_WALL_LIMIT:.0f} s wall; their pre-mission messages are not '
                  'in the bag and the metrics built on them will be null', flush=True)

        goal_pose = goal_pose_stamped(node, goal_map)

        # 7. Idle: the stack is up and nothing has been asked of it yet.
        node.publish_mission_state(
            MissionState.STATE_IDLE, goal_pose,
            detail='stack up, goal not yet sent; `goal` is the goal that will be sent')

        # 8. Localize. AMCL updates on MOTION (update_min_d 0.1, update_min_a
        # 0.2), so a robot standing still never resamples and its covariance
        # stays at whatever /initialpose declared - measured: trace 0.4474
        # after 30 s sim. The in-place rotation gives the filter angular
        # motion and returns the body to the same xy.
        #
        # The scenario's START pose gets the same footprint check as the goal,
        # BEFORE /initialpose is published: a start inside a wall produces a
        # run whose estimate error has no explanation, and the body is already
        # standing there by the time anything else could notice.
        preflight_initial = preflight_goal(node.grid, initial_map, robot_radius_m,
                                           occupied_thresh)
        preflight_initial['robot_radius_m'] = robot_radius_m
        preflight_initial['robot_radius_sources'] = radius_sources
        preflight_initial['occupied_thresh'] = occupied_thresh
        record['preflight_initial'] = preflight_initial
        if not preflight_initial['ok']:
            node.publish_mission_state(
                MissionState.STATE_UNREACHABLE, goal_pose,
                outcome='not_sent:initial_pose_occupied',
                detail=preflight_initial['detail'])
            record['mission_state'] = MissionState.STATE_UNREACHABLE
            record['outcome'] = 'not_sent:initial_pose_occupied'
            record_undispatched(
                record, scenario['interventions'], scenario['world'],
                'start_pose_refused',
                'the scenario start pose was refused pre-flight, so no mission ran')
            record['rtf_samples'] = measure_rtf(scenario['world'])
            raise RunAbort(EXIT_OK, 'start pose refused pre-flight; no goal sent')

        node.amcl = None
        initialpose_sim = node.sim_now()
        for _ in range(3):
            node.publish_initial_pose(*initial_map)
            node.spin_wall(0.5)

        def converged():
            if node.amcl is None:
                return False
            cov = node.amcl.pose.covariance
            return cov[0] + cov[7] + cov[35] < COVARIANCE_LIMIT

        did_converge = node.spin_sim(AMCL_CONVERGE_SIM,
                                     cmd=twist(angular_z=AMCL_WARMUP_RATE),
                                     until=converged)
        if did_converge:
            # Keep rotating so the converged filter publishes more than the one
            # sample est_error_at_start_pose_m would otherwise be averaged over.
            node.spin_sim(AMCL_FLOOR_SAMPLE_SIM, cmd=twist(angular_z=AMCL_WARMUP_RATE))
        node.stop()
        node.spin_sim(AMCL_SETTLE_SIM)
        # The warm-up rotation ends wherever convergence happened to land, so
        # without this the body faces an ARBITRARY direction when the goal is
        # sent - while the scenario declares an initial yaw. Measured: a run
        # that ended facing away from the goal had DWB reverse at 0.1 m/s for
        # 40 s, taking 76 s sim against 37 s for the same path driven forward.
        # That is real Nav2 behaviour, but it is a response to a heading no
        # scenario asked for, and it makes two runs of one scenario
        # incomparable. Aligning uses the robot's OWN estimate (/amcl_pose),
        # never ground truth: closing this loop on truth would be exactly the
        # localization shortcut this project forbids.
        aligned, yaw_error = align_to_initial_yaw(node, initial_map[2])
        record['initial_yaw_aligned'] = aligned
        record['initial_yaw_error_rad'] = yaw_error
        if not aligned:
            print(f'--- WARNING: could not align to the scenario initial yaw within '
                  f'{YAW_ALIGN_SIM:.0f} s sim; estimated yaw error {yaw_error} rad. The '
                  'mission starts from a different heading than the scenario declares',
                  flush=True)
        record['localization_converged'] = bool(did_converge)
        # Two different quantities, under two different names. The convergence
        # time is observed in the AMCL callback at the instant the trace first
        # fell below the limit; the warm-up total also covers the extra
        # rotation, the settle and the yaw alignment that follow it.
        record['localization_converged_sim_s'] = (
            node.converged_sim - initialpose_sim if node.converged_sim is not None
            else None)
        record['warmup_total_sim_s'] = node.sim_now() - initialpose_sim
        if node.amcl is not None:
            cov = node.amcl.pose.covariance
            record['localization_cov_trace'] = cov[0] + cov[7] + cov[35]
        # Non-convergence is DATA. A run that navigates badly because the
        # filter never converged is a result; raising COVARIANCE_LIMIT to make
        # it look clean would not be.
        if not did_converge:
            print(f'--- AMCL did not converge within {AMCL_CONVERGE_SIM:.0f} s sim '
                  f'(trace {record["localization_cov_trace"]}); continuing', flush=True)

        # 9. Pre-flight. The only path to `unreachable`.
        preflight = preflight_goal(node.grid, goal_map, robot_radius_m, occupied_thresh)
        preflight['robot_radius_m'] = robot_radius_m
        preflight['robot_radius_sources'] = radius_sources
        preflight['occupied_thresh'] = occupied_thresh
        record['preflight'] = preflight
        if not preflight['ok']:
            node.publish_mission_state(MissionState.STATE_UNREACHABLE, goal_pose,
                                       outcome=preflight['outcome'],
                                       detail=preflight['detail'])
            record['mission_state'] = MissionState.STATE_UNREACHABLE
            record['outcome'] = preflight['outcome']
            record_undispatched(
                record, scenario['interventions'], scenario['world'], 'goal_unreachable',
                'the goal was refused pre-flight, so no mission ran')
            record['rtf_samples'] = measure_rtf(scenario['world'])
            raise RunAbort(EXIT_OK, 'goal refused pre-flight; no goal sent')

        # 10. Send the goal.
        feedback = {'count': 0, 'distance_remaining': None, 'number_of_recoveries': None}

        def on_feedback(message):
            feedback['count'] += 1
            feedback['distance_remaining'] = message.feedback.distance_remaining
            feedback['number_of_recoveries'] = message.feedback.number_of_recoveries
            if feedback['count'] == 1:
                node.publish_mission_state(
                    MissionState.STATE_NAVIGATING, goal_pose,
                    detail='first action feedback received; the server is executing')

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        send_future = nav_client.send_goal_async(goal, feedback_callback=on_feedback)
        node.spin_sim(GOAL_ACCEPT_WAIT_SIM, until=send_future.done)
        handle = send_future.result() if send_future.done() else None
        if handle is None or not handle.accepted:
            # NOT `aborted`: no GoalStatus was ever produced for a goal the
            # server refused, and `aborted` names STATUS_ABORTED (6).
            detail = ('no response to the goal request' if handle is None
                      else 'the action server refused the goal')
            node.publish_mission_state(MissionState.STATE_REJECTED, goal_pose,
                                       outcome='not_accepted', detail=detail)
            record['mission_state'] = MissionState.STATE_REJECTED
            record['outcome'] = 'not_accepted'
            record_undispatched(
                record, scenario['interventions'], scenario['world'], 'goal_not_accepted',
                'the action server did not accept the goal, so no mission ran')
            record['rtf_samples'] = measure_rtf(scenario['world'])
            raise RunAbort(EXIT_OK, detail)

        t_accept = node.sim_now()
        record['t_accept_sim'] = t_accept
        record['goal_uuid'] = bytes(handle.goal_id.uuid).hex()
        node.publish_mission_state(MissionState.STATE_GOAL_ACCEPTED, goal_pose,
                                   detail=f'goal_id {record["goal_uuid"]}')
        result_future = handle.get_result_async()

        # 11. Mission loop. A single spin_sim(until=...) would not do: the
        # interventions have to fire on SIM time while the goal is running.
        pending = list(scenario['interventions'])
        timeout_sec = scenario['timeout_sec']
        canceled = False
        cancel_deadline = None
        stalled = False
        # Wall deadlock guard ONLY. Ten times the sim budget plus two minutes: a
        # slow real-time factor must change how long a run takes, never what it
        # measures, so this can only fire when /clock has stopped advancing.
        # Without it, a gz server that dies mid-mission freezes sim_now(), and
        # both loop exits (result_future.done() and the cancel deadline) depend
        # on it - the runner would spin for ever with the recorder still up.
        mission_wall_deadline = time.monotonic() + timeout_sec * 10.0 + 120.0
        last_sim = node.sim_now()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if result_future.done():
                break
            elapsed = node.sim_now() - t_accept
            last_sim = node.sim_now()
            if pending and elapsed >= pending[0]['at_sec']:
                intervention = pending.pop(0)
                print(f'--- intervention {intervention["id"]} at {elapsed:.2f} s sim '
                      f'(scheduled {intervention["at_sec"]:.2f})', flush=True)
                record['interventions'].append(
                    dispatch_intervention(node, intervention, scenario['world'], elapsed))
                continue
            if not canceled and elapsed >= timeout_sec:
                print(f'--- timeout at {elapsed:.1f} s sim; cancelling the goal',
                      flush=True)
                handle.cancel_goal_async()
                canceled = True
                cancel_deadline = node.sim_now() + CANCEL_WAIT_SIM
            if canceled and node.sim_now() > cancel_deadline:
                break
            if time.monotonic() > mission_wall_deadline:
                stalled = True
                break

        # Interventions the mission never reached. The schema only rejects
        # at_sec >= timeout_sec, so one scheduled past the ACTUAL mission
        # duration is legal and would otherwise leave no trace at all.
        record_undispatched(
            record, pending, scenario['world'], 'mission_ended_first',
            f'the mission ended at {node.sim_now() - t_accept:.2f} s sim, before this '
            'intervention\'s at_sec')

        # 12/13. Terminal handling. The outcome is the literal GoalStatus
        # name, never a phrase, and never inferred when it is absent.
        detail = (f'distance_remaining={feedback["distance_remaining"]}, '
                  f'number_of_recoveries={feedback["number_of_recoveries"]}'
                  if feedback['count'] else 'no action feedback was received')
        if result_future.done():
            status = result_future.result().status
            record['nav2_status_code'] = int(status)
            # A non-terminal status as the FINAL one is not an abort: it is a
            # status whose meaning for a finished mission is unknown.
            state, outcome = TERMINAL_STATES.get(
                status, (MissionState.STATE_UNKNOWN, f'unknown:status_{int(status)}'))
            record['nav2_status_name'] = outcome
        elif stalled:
            status = None
            state = MissionState.STATE_UNKNOWN
            outcome = 'unknown:clock_stalled'
            detail = (f'{detail}; /clock stopped advancing: last sim time seen '
                      f'{last_sim:.2f} s after {timeout_sec * 10.0 + 120.0:.0f} s wall, '
                      f'and the goal was {"cancelled" if canceled else "not cancelled"}')
        else:
            status = None
            # NOT `canceled`: STATUS_CANCELED is a status the server sends, and
            # none arrived. The cancel is reported in `detail` instead.
            state = MissionState.STATE_UNKNOWN
            outcome = 'unknown:no_terminal_status'
            detail = (f'{detail}; no terminal status arrived '
                      + (f'within {CANCEL_WAIT_SIM:.0f} s sim of the cancel'
                         if canceled else 'and the mission loop ended'))
        record['t_terminal_sim'] = node.sim_now()
        record['mission_state'] = state
        record['outcome'] = outcome
        node.publish_mission_state(state, goal_pose, outcome=outcome, detail=detail)
        node.spin_sim(POST_TERMINAL_SIM)
        record['rtf_samples'] = measure_rtf(scenario['world'])

    except RunAbort as abort:
        exit_code = abort.code
        print(f'--- run stopped: {abort}', flush=True)
    except Exception:                                   # noqa: BLE001 - reported, not hidden
        exit_code = EXIT_ARTIFACTS
        record['outcome'] = record['outcome'] or 'runner_exception'
        record['detail'] = traceback.format_exc()
        traceback.print_exc()
    finally:
        record['sim_end'] = node.sim_now()
        node.destroy_node()
        rclpy.shutdown()

        # 14. The recorder FIRST: killing the stack first costs the trailing
        # messages, and SIGKILL truncates the mcap.
        if recorder is not None:
            try:
                os.killpg(os.getpgid(recorder.pid), signal.SIGINT)
                recorder.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(recorder.pid), signal.SIGKILL)
                recorder.wait(timeout=10)
            except ProcessLookupError:
                pass
        if recorder_log is not None:
            recorder_log.close()

        # 15. Then the stack.
        try:
            os.killpg(os.getpgid(launch.pid), signal.SIGINT)
            launch.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(launch.pid), signal.SIGKILL)
            launch.wait(timeout=10)
        except ProcessLookupError:
            pass
        launch_log.close()

    # 16. The bag must be present AND finalised AND carry the pinned latched
    # topics. `metadata.yaml` is written by the recorder on shutdown, so its
    # absence means SIGKILL truncated the run; a parsed metadata with zero
    # messages, or without the single /map and the two /tf_static messages the
    # QoS override pins, means the recorder ran but the evidence is not there.
    # Converting that into a failed run is the point: the alternative is an
    # empty panel A that looks like a plotting bug.
    bag_dir = os.path.join(run_dir, 'bag')
    has_metadata = os.path.isfile(os.path.join(bag_dir, 'metadata.yaml'))
    has_mcap = os.path.isdir(bag_dir) and any(
        name.endswith('.mcap') for name in os.listdir(bag_dir))
    bag_detail = (f'{bag_dir}: metadata.yaml {"present" if has_metadata else "MISSING"}, '
                  f'*.mcap {"present" if has_mcap else "MISSING"}')
    bag_ok = has_metadata and has_mcap
    if bag_ok:
        try:
            with open(os.path.join(bag_dir, 'metadata.yaml')) as handle:
                meta = (yaml.safe_load(handle) or {})['rosbag2_bagfile_information']
            counts = {entry['topic_metadata']['name']: int(entry['message_count'])
                      for entry in meta.get('topics_with_message_count', [])}
            total = int(meta.get('message_count', 0))
            required = {'/map': 1, '/tf_static': 2}
            short = {name: counts.get(name, 0) for name, least in required.items()
                     if counts.get(name, 0) < least}
            recorded = {name: counts.get(name, 0) for name in required}
            if total <= 0 or short:
                bag_ok = False
                bag_detail = (f'{bag_detail}; metadata reports {total} message(s) in '
                              f'total, pinned minimum {required}, recorded {recorded}, '
                              f'short {short or "none"}')
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            bag_ok = False
            bag_detail = (f'{bag_detail}; metadata.yaml could not be read '
                          f'({type(exc).__name__}: {exc}), so the bag is not finalised')
    if exit_code == EXIT_OK and not bag_ok:
        exit_code = EXIT_BAG
        record['outcome'] = record['outcome'] or 'bag_missing'
        record['detail'] = f'{bag_detail}; the run cannot be measured'
        print(f'BAG UNUSABLE: {record["detail"]}', file=sys.stderr, flush=True)

    record['finished_utc'] = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
    record['exit_code'] = exit_code
    write_json(os.path.join(run_dir, 'run.json'), record)

    if exit_code in (EXIT_BRINGUP, EXIT_BAG):
        return exit_code

    # 17. Artifacts, in-process. Imported here so a runner that fails at
    # bring-up does not also need matplotlib to report that.
    try:
        from ral_eval.metrics import write_metrics
        from ral_eval.plot_run import write_plot
        write_metrics(run_dir)
        write_plot(run_dir)
    except Exception:                                   # noqa: BLE001 - the bag is kept
        traceback.print_exc()
        print('ARTIFACTS FAILED: the bag is kept; recompute with '
              f'`ros2 run ral_eval compute_metrics --run-dir {run_dir}`',
              file=sys.stderr, flush=True)
        record['exit_code'] = EXIT_ARTIFACTS
        write_json(os.path.join(run_dir, 'run.json'), record)
        return EXIT_ARTIFACTS

    print(f'--- run complete: {run_dir} (exit {exit_code})', flush=True)
    return exit_code


def main(args=None) -> None:
    sys.exit(run(args))


if __name__ == '__main__':
    main()
