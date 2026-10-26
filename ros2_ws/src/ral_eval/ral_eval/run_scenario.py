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

Reads one scenario file, brings that robot's whole stack up, records a bag,
prepares the platform for a goal, drives one mission, dispatches the
scenario's interventions on SIM time, and leaves behind
runs/<experiment_id>_<UTC>/ containing scenario.yaml, env.json, run.json,
bag/, metrics.json, plot.png and launch.log.

What is PER ROBOT and what is not
---------------------------------
Everything platform-specific is read from the run's robot profile
(ral_bringup/config/robots/<robot>.yaml): the stack launch file, the gz model
a fault addresses, the estimate topic and its type, the footprint radius, the
bring-up gate, the recorded topics and - through `mission_driver` - HOW a goal
is sent and monitored. That last part is a MissionDriver
(ral_eval/mission_driver.py); this module owns /mission_state, the bag, the
intervention schedule, the timeout and the artifacts, and nothing else knows
whether the robot underneath is driven by Nav2 or by an autopilot.

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
  3  the stack never came up, the autonomy stack never became ready, or the
     map yaml does not carry the occupancy limit the pre-flight checks
     against. run.json records what was missing.
  4  the bag is missing, unfinalised, empty, or short of the latched topics
     the QoS override pins; the run cannot be measured.
  5  metrics or the plot raised. The bag is kept so it can be recomputed.

Bag size
--------
RECORD_TOPICS_COMMON plus the profile's `record_topics` is deliberately wide:
a baseline Roomba run writes ~21 MB over ~75,000 messages, of which
/joint_states contributes 29,170 and /scan plus /scan_raw most of the
remainder. rosbag2's Humble CLI has no per-topic throttle, so each topic is
all-or-nothing; /joint_states stays because wheel motion against truth motion
is how slip is observed, and that is what the mobility fault is read from.

Concurrency
-----------
ONE run at a time per ROS_DOMAIN_ID. The stack owns fixed topic names and a
single gz server, so two runs in one domain would interleave commands into one
robot. The runner does not police this - there is no reliable way to tell a
second runner from a stale node - so scripts/run_scenario.sh takes an flock on
/tmp/ral-run-<domain>.lock and refuses the second caller.

Who publishes a FaultEvent
--------------------------
The fault manager (ral_faults/fault_manager.py) owns `requested`,
`activated`, `rejected` and `cleared`, because only it observes the
underlying gz or relay call. This runner publishes exactly one kind of
FaultEvent: `rejected`, and only for the two cases the manager cannot report
- its service was not there to be called, or it never answered. A service
that answered success=false has already published its own `rejected` with the
same id, and this runner adds nothing to it.

Frames
------
Scenario poses are in the gz WORLD frame. The mission poses are converted to
`map` with ral_eval.frames and the alignment sealed to the map in use, and a
driver converts them again if its platform works in another frame - the car's
setpoints are in the autopilot's EKF-local frame, through the offset measured
for that run. The SPAWN pose is the scenario's declared initial pose, passed
to the launch file, so the declared and the actual pose cannot disagree. gz
entity poses (spawn_obstacle, kidnap) are passed through UNCONVERTED, because
`world` is the frame gz defines them in. run.json records every form.

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

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from ral_eval.frames import (
    frames_path_for_map,
    grid_sha256,
    load_alignment_for_map,
    quaternion_of_yaw,
    yaw_of_quaternion,
)
from ral_eval.grid import grid_from_pgm
from ral_eval.mission_driver import build_driver, outcome_is_known
from ral_eval.mission_node import MissionNode, goal_pose_stamped, injection_point
from ral_eval.robot_profile import load_profile, profile_path
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

import yaml

EXIT_OK = 0
EXIT_ALIGNMENT = 1
EXIT_SCENARIO = 2
EXIT_BRINGUP = 3
EXIT_BAG = 4
EXIT_ARTIFACTS = 5

# The bring-up limit is the PROFILE's `bringup_wall_limit_s`, not a constant:
# what counts as "too long to come up" is a property of the stack. Nav2 is
# ready as soon as its lifecycle manager activates; an autopilot cannot report
# a position until EKF3 has an origin, which waits on a simulated GPS fix.
STACK_READY_WALL_LIMIT = 60.0    # s wall for the autonomy stack to accept a goal
RECORDER_WALL_LIMIT = 60.0       # s wall for the recorder to subscribe to the gate topics
RECORDER_SETTLE_WALL = 1.0       # s wall after it subscribed, for publisher-side matching
SERVICE_WAIT_SIM = 5.0           # s sim an intervention waits for its service
POST_TERMINAL_SIM = 2.0          # s sim recorded after the terminal result
CANCEL_WAIT_SIM = 10.0           # s sim allowed for a cancel to produce a terminal status
# robot_radius, occupied_thresh, the gz model name, the estimate topic and the
# localization warm-up are NOT constants here. The first two are read from the
# robot profile and the map yaml this run actually used and recorded in
# run.json; the model and the estimate come from the profile; the warm-up
# belongs to whichever MissionDriver this platform uses, because an Ackermann
# vehicle has no in-place rotation to warm a filter up with. A check against a
# guessed value is not a check.

# The gz contact topic the /bumper bridge carries, per
# ral_bringup/config/bridge_<robot>_<world>.yaml. The model name is the
# profile's, so the liveness proof addresses the body this run spawned.
BUMPER_GZ_TOPIC = ('/world/{world}/model/{model}/link/base_link/sensor/bumper/'
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

# Topics recorded for EVERY run, whatever the robot, each because a metric or
# a plot panel needs it. The bag is the source of record; run.json is the
# fallback only for the terminal status. A platform's own topics - the
# estimate, the command, the autonomy stack's internals - are listed in its
# profile's `record_topics`, and the two are concatenated for a run.
RECORD_TOPICS_COMMON = (
    '/clock',                               # the sim time every other stamp is in
    '/tf',                                  # the estimate chain, for inspection
    '/tf_static',                           # sensor mounting; latched, see the QoS file
    '/ground_truth/odom',                   # path_length_m, goal_pose_error_m, achieved v
    '/scan',                                # the stream the relay publishes
    '/scan_raw',                            # Phase 4.2 proves degradation between the two
    '/bumper',                              # collision_count
    '/joint_states',                        # wheel motion; truth-vs-wheel divergence is
                                            # how slip is observed (Phase 4.3's mobility
                                            # fault, roomba_teleop_check 7)
    '/mission_state',                       # the mission window itself
    '/fault_events',                        # fault_to_response_s anchors
    '/faults/active',                       # the manager's authoritative snapshot of
                                            # what was in force, latched
    '/rosout',                              # node-side errors beside the run
)

# Topics whose PRE-MISSION history is evidence, so nothing may be published on
# them until the recorder is actually subscribed. `ros2 bag record` subscribes
# topic by topic after its own start-up; measured on this machine, the last
# subscription landed 5.6 s after the process started, and a fixed 3 s wait
# silently lost every /initialpose and the whole localization warm-up. The
# only reliable signal is the recorder's own subscription appearing in the
# graph, so that is what the runner waits for. A platform adds its own in the
# profile's `recorder_gate_topics`.
RECORDER_GATE_TOPICS_COMMON = (
    '/mission_state',        # the mission window
    '/fault_events',         # a fault rejected before the mission must be recorded
    '/ground_truth/odom',    # the truth half of every residual
)
RECORDER_NODE_PREFIX = 'rosbag2'


def wait_for_recorder(node, gate_topics,
                      wall_limit: float = RECORDER_WALL_LIMIT) -> list:
    """
    Block until the recorder has subscribed to every gate topic.

    Returns the topics it never subscribed to, which the caller records in
    run.json. Those topics' pre-mission messages would be missing from the
    bag, and a metric computed from them would be null with a reason - never
    silently zero.
    """
    deadline = time.monotonic() + wall_limit
    missing = list(gate_topics)
    while time.monotonic() < deadline and missing:
        node.spin_wall(0.25)
        missing = [
            topic for topic in gate_topics
            if not any(info.node_name.startswith(RECORDER_NODE_PREFIX)
                       for info in node.get_subscriptions_info_by_topic(topic))]
    # The subscription is visible in the graph; give the local publishers one
    # second of wall time to finish matching it before anything is published.
    node.spin_wall(RECORDER_SETTLE_WALL)
    return missing


def bumper_liveness(node, world: str, model: str) -> dict:
    """
    Prove the contact path is wired without waiting for a contact.

    A gz contact sensor publishes only while two bodies touch, so a silent
    /bumper is the normal case for a clean run. The only way `collision_count:
    0` can be a measurement rather than an absence is to show, in this run,
    that the sensor exists in gz and the bridge is publishing the ROS topic.
    """
    publishers = sorted(info.node_name
                        for info in node.get_publishers_info_by_topic('/bumper'))
    gz_topic = BUMPER_GZ_TOPIC.format(world=world, model=model)
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


def twist(linear_x: float = 0.0, angular_z: float = 0.0) -> Twist:
    msg = Twist()
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    return msg


def _sha256_file(path: str) -> str:
    with open(path, 'rb') as handle:
        return hashlib.sha256(handle.read()).hexdigest()


# Written by docker/build_overlay.sh at image build time: the ArduPilot commit
# the SITL binary in this image was built from. docker/ardupilot.repos pins a
# BRANCH, so without this file a run can say which branch it meant but not
# which code it flew.
FIRMWARE_VERSION_PATH = '/opt/ardupilot/VERSION.txt'


def _firmware_sha() -> 'str | None':
    """
    Read the ArduPilot commit this image was built from, or None when absent.

    None is not a failure: an image built without the autopilot overlay has no
    firmware, and env.json carries the source path beside this value so a
    reader can tell "no autopilot here" from "the file was not written".
    """
    try:
        with open(FIRMWARE_VERSION_PATH) as handle:
            return handle.read().strip() or None
    except OSError:
        return None


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
#
# Every builder takes the intervention's `fault_id` and sets it FIRST. That id
# is the join key: the manager echoes it on every FaultEvent it publishes for
# this instance, so run.json's intervention record and the bag's event stream
# can be joined without matching on timing or parameters.

def _spawn_request(params: dict, fault_id: str) -> SpawnObstacle.Request:
    request = SpawnObstacle.Request()
    request.fault_id = fault_id
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


def _clear_request(params: dict, fault_id: str) -> ClearObstacle.Request:
    request = ClearObstacle.Request()
    request.fault_id = fault_id
    request.name = params['name']
    return request


def _scan_request(params: dict, fault_id: str) -> SetScanDegradation.Request:
    request = SetScanDegradation.Request()
    request.fault_id = fault_id
    request.enabled = params['enabled']
    request.range_noise_stddev_m = params['range_noise_stddev_m']
    request.max_range_m = params['max_range_m']
    request.blind_sector_start_deg = params['blind_sector_start_deg']
    request.blind_sector_end_deg = params['blind_sector_end_deg']
    request.dropout_probability = params['dropout_probability']
    return request


def _mobility_request(params: dict, fault_id: str) -> SetMobilityFault.Request:
    request = SetMobilityFault.Request()
    request.fault_id = fault_id
    request.enabled = params['enabled']
    request.force_x_n = params['force_x_n']
    request.force_y_n = params['force_y_n']
    request.torque_z_nm = params['torque_z_nm']
    return request


def _kidnap_request(params: dict, fault_id: str) -> Kidnap.Request:
    request = Kidnap.Request()
    request.fault_id = fault_id
    request.new_pose.position.x = params['pose']['x']
    request.new_pose.position.y = params['pose']['y']
    qx, qy, qz, qw = quaternion_of_yaw(params['pose']['yaw'])
    request.new_pose.orientation.x = qx
    request.new_pose.orientation.y = qy
    request.new_pose.orientation.z = qz
    request.new_pose.orientation.w = qw
    return request


def _target_of(kind: str, params: dict, model: str) -> str:
    """
    Return the entity or stream a fault acts on, for FaultEvent.target.

    `model` is the gz entity from the run's robot profile, never a constant:
    the manager labels its own events with the same name, and the two reading
    differently would put one fault under two targets in one bag.
    """
    if kind in (FaultEvent.KIND_SPAWN_OBSTACLE, FaultEvent.KIND_CLEAR_OBSTACLE):
        return params['name']
    if kind == FaultEvent.KIND_SCAN_DEGRADATION:
        return '/scan'
    return model


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


def occupancy_grid_from_pgm(map_yaml_path: str) -> OccupancyGrid:
    """
    Read the COMMITTED map as the message a stack with a map_server publishes.

    The pre-flight takes a nav_msgs/OccupancyGrid because that is what it gets
    on a platform running map_server. The car runs none - its autopilot
    localizes against GPS and an IMU, not a grid - but its scenarios were
    authored against the same committed map, and a goal inside a wall must be
    refused for it too. ral_eval.grid is the one reader of the .pgm; this
    wraps its output so the pre-flight has exactly one implementation.
    """
    occupancy, info = grid_from_pgm(map_yaml_path)
    grid = OccupancyGrid()
    grid.info.resolution = float(info['resolution'])
    grid.info.width = int(info['width'])
    grid.info.height = int(info['height'])
    grid.info.origin.position.x = float(info['origin'][0])
    grid.info.origin.position.y = float(info['origin'][1])
    # The reader refuses a rotated origin, so the orientation is identity and
    # the pre-flight's own rotation handling is exercised by a /map instead.
    grid.info.origin.orientation.w = 1.0
    grid.data = [int(value) for value in occupancy.reshape(-1)]
    return grid


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


def record_undispatched(record: dict, pending: list, world: str, model: str,
                        reason: str, detail: str) -> None:
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
            'target': _target_of(intervention['type'], intervention['params'], model),
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


def dispatch_intervention(node: MissionNode, intervention: dict, world: str,
                          model: str, dispatched_sim: float) -> dict:
    """
    Call the service that implements one intervention, and report honestly.

    Returns the run.json record. The runner never claims an effect: `applied`
    is true only when the service itself returned success. The request carries
    the intervention's id, so every FaultEvent the manager publishes for it
    joins back to this record.

    This function publishes a FaultEvent only for the two cases the MANAGER
    cannot report - its service was not there to call, or it never answered.
    A service that answered success=false has already published its own
    `rejected` carrying the same id and the same reason; a second one here
    would double-count in fault_to_response_s and in any timeline built from
    the event stream.

    `manager_fault_id` is the id the MANAGER answered with - the one it
    actually joined its FaultEvents on. It should equal `id`, because the
    runner always sends a non-empty one; recording it makes a mismatch
    visible in run.json instead of invisible.
    """
    kind = intervention['type']
    service_name, service_type, build_request = DISPATCH[kind]
    target = _target_of(kind, intervention['params'], model)
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
        'manager_fault_id': '',
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
                  '(fault_manager is not running or did not advertise it)')
        record['message'] = detail
        node.publish_fault_rejected(intervention['id'], kind, world, target, detail)
        node.destroy_client(client)
        return record

    record['available'] = True
    future = client.call_async(build_request(intervention['params'], intervention['id']))
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
    record['manager_fault_id'] = response.fault_id
    if response.success:
        # The fault manager publishes requested/activated/cleared; it observed
        # the gz call, this runner did not.
        record['result'] = 'applied'
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

    # 2b. The robot profile. Resolved BEFORE the run directory is made, so a
    # scenario naming a robot with no profile aborts without leaving an
    # artifact directory that looks like a result. Everything platform-
    # specific below comes from it: the gz model, the stack launched, the
    # estimate, the footprint, the recorded topics and the mission driver.
    try:
        profile = load_profile(profile_path(bringup_share, scenario['robot']))
    except (ValueError, OSError) as exc:
        print(f'ROBOT PROFILE REJECTED: {exc}', file=sys.stderr, flush=True)
        return EXIT_SCENARIO
    model = profile['model']
    # Nav2's parameters are one platform's, not every platform's: a car run
    # records null rather than the hash of a file nothing in it read.
    nav2_params = (os.path.join(bringup_share, 'config', 'nav2_roomba.yaml')
                   if profile['mission_driver'] == 'nav2' else None)
    # The autopilot's parameter file, for the same reason and from the same
    # place the launch file reads it: ardurover is started with
    # `--defaults <this file> --wipe`, so it IS the autopilot configuration
    # for the run and nothing carries over from a previous one. A profile
    # without the key is a platform with no autopilot, and records null.
    autopilot_parm = (os.path.join(bringup_share, 'config', profile['autopilot_parm'])
                      if profile.get('autopilot_parm') else None)

    # 2c. The footprint limits the pre-flight will check against, read from
    # the files this run actually uses rather than duplicated as constants.
    robot_radius_m = float(profile['footprint']['radius_m'])
    radius_sources = {'robot_profile': profile['footprint']['source']}
    try:
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
    with open(profile['profile_path']) as handle:
        profile_text = handle.read()
    # The common set plus this platform's own. A topic listed twice would make
    # `ros2 bag record` subscribe twice and double every message in the bag.
    record_topics = tuple(RECORD_TOPICS_COMMON) + tuple(
        topic for topic in profile['record_topics']
        if topic not in RECORD_TOPICS_COMMON)
    recorder_gate_topics = tuple(RECORDER_GATE_TOPICS_COMMON) + tuple(
        topic for topic in profile['recorder_gate_topics']
        if topic not in RECORDER_GATE_TOPICS_COMMON)
    git_dirty = _git_dirty(args.repo_root)
    git_sha = _git_sha(args.repo_root)
    # The scenario's own values, not defaults: `world` selects the SDF, the
    # bridge config and the truth frame; `robot` selects the profile, and with
    # it the gz model spawned and the faults that are real for it; `seed` is
    # the scan relay's RNG seed. The spawn pose is the scenario's DECLARED
    # initial pose, so the pose the scenario claims and the pose the body is
    # standing at cannot disagree. env.json copies this list verbatim, so the
    # artifacts record exactly what was launched.
    initial_world = poses['initial']['world']
    launch_command = ['ros2', 'launch', 'ral_bringup', profile['stack_launch'],
                      f'world:={scenario["world"]}',
                      f'robot:={scenario["robot"]}',
                      f'seed:={scenario["seed"]}',
                      f'spawn_x:={initial_world[0]}',
                      f'spawn_y:={initial_world[1]}',
                      f'spawn_yaw:={initial_world[2]}']
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
        # The profile's own TEXT, as `alignment` carries the alignment file's:
        # metrics.py and plot_run.py read the robot's description out of the
        # RUN, never out of the repository, so a profile edited after a run
        # cannot change what that run's numbers claim to be.
        'robot_profile': profile_text,
        'robot_profile_path': profile['profile_path'],
        'nav2_params_path': nav2_params,
        'nav2_params_sha256': _sha256_file(nav2_params) if nav2_params else None,
        # The autopilot half of the same provenance: which parameter file was
        # fed to ardurover, and which ArduPilot source the firmware in this
        # image was built from. docker/ardupilot.repos tracks a floating
        # branch on purpose, so the SHA recorded here is what makes a run
        # traceable to a build rather than to a branch name.
        'autopilot_parm_path': autopilot_parm,
        'autopilot_parm_sha256': _sha256_file(autopilot_parm) if autopilot_parm else None,
        'firmware_sha': _firmware_sha(),
        'firmware_sha_source': (f'{FIRMWARE_VERSION_PATH}, written by '
                                'docker/build_overlay.sh'),
        'scenario_path': scenario_path,
        'scenario_sha256': _sha256_file(scenario_path),
        'launch_command': launch_command,
        'record_topics': list(record_topics),
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
        # Measured per run by the car's ekf_local_tf and reported by the
        # MAVROS driver; null for a platform whose estimate is already in
        # `map`, with no claim either way.
        'world_to_ekf_local': None,
        'recorder_unsubscribed_topics': None,
        'bumper_liveness': None,
        'initial_yaw_aligned': None,
        'initial_yaw_error_rad': None,
        'initial_yaw_detail': None,
        'localization_converged': None,
        'localization_cov_trace': None,
        'localization_converged_sim_s': None,
        'localization_detail': None,
        'warmup_total_sim_s': None,
        'preflight_initial': None,
        'preflight': None,
        'mission_state': None,
        'outcome': None,
        'mission_driver': profile['mission_driver'],
        'nav2_status_code': None,
        'nav2_status_name': None,
        'detail': None,
        'interventions': [],
        'rtf_samples': 'not measured',
        'bag_dir': os.path.join(run_dir, 'bag'),
        'exit_code': None,
    }

    rclpy.init()
    node = MissionNode(scenario['experiment_id'], profile, wtm)
    driver = build_driver(node, profile, record)
    launch_log = open(os.path.join(run_dir, 'launch.log'), 'w')
    launch = subprocess.Popen(launch_command, stdout=launch_log,
                              stderr=subprocess.STDOUT, start_new_session=True)
    recorder = None
    recorder_log = None
    exit_code = EXIT_OK

    try:
        # 5. Bring-up gate, from the profile: every topic it declares must
        # have delivered a message. A stack that never comes up is a failed
        # run with artifacts, not a traceback.
        needed = dict.fromkeys(node.bringup_seen, False)
        bringup_limit = float(profile['bringup_wall_limit_s'])
        deadline = time.monotonic() + bringup_limit
        missing = list(needed)
        while time.monotonic() < deadline:
            node.spin_wall(0.5)
            missing = [topic for topic in needed
                       if not (node.bringup_seen[topic]
                               or (topic == '/clock' and node.sim_now() > 0.0))]
            if not missing:
                break
        if missing:
            record['outcome'] = 'runner:bringup_failed'
            record['mission_state'] = 'none'
            record['detail'] = (f'no messages on {", ".join(missing)} within '
                                f'{bringup_limit:.0f} s wall; see launch.log')
            raise RunAbort(EXIT_BRINGUP, record['detail'])

        # The autonomy stack's own readiness, asked of the driver: a Nav2
        # action server for one platform, a live MAVLink link and the mode and
        # arming services for the other. The refusal names which was missing.
        not_ready = driver.wait_ready(STACK_READY_WALL_LIMIT)
        if not_ready:
            record['outcome'] = 'runner:bringup_failed'
            record['mission_state'] = 'none'
            record['detail'] = not_ready
            raise RunAbort(EXIT_BRINGUP, not_ready)

        record['sim_start'] = node.sim_now()
        print(f'--- stack up at sim t={record["sim_start"]:.2f} s', flush=True)

        # 5b. Prove the contact path exists NOW, while the world is up. A gz
        # contact sensor publishes only on contact, so without this
        # `collision_count: 0` cannot be told from a dead bridge.
        record['bumper_liveness'] = bumper_liveness(node, scenario['world'], model)
        print(f'--- bumper liveness {record["bumper_liveness"]}', flush=True)

        # 6. Record BEFORE the driver prepares: a localization warm-up, an
        # /initialpose, a mode change and an arming are all evidence, and
        # metrics.py windows them out by the mission window rather than by
        # never having recorded them.
        bag_dir = os.path.join(run_dir, 'bag')
        recorder_log = open(os.path.join(run_dir, 'bag_record.log'), 'w')
        recorder = subprocess.Popen(
            ['ros2', 'bag', 'record', '-s', 'mcap', '-o', bag_dir, '--use-sim-time',
             '--include-hidden-topics', '--qos-profile-overrides-path', qos_overrides,
             *record_topics],
            stdout=recorder_log, stderr=subprocess.STDOUT, start_new_session=True)
        unrecorded = wait_for_recorder(node, recorder_gate_topics)
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

        # 8. Pre-flight, then whatever this platform does before a goal.
        #
        # The scenario's START pose gets the same footprint check as the goal,
        # BEFORE anything is published: a start inside a wall produces a run
        # whose estimate error has no explanation, and the body is already
        # standing there by the time anything else could notice.
        #
        # The grid is the stack's own /map where the platform publishes one,
        # and the COMMITTED map otherwise: the car runs no map_server, but the
        # same static grid is what its scenarios were authored against and
        # what every metric is scored in.
        grid = node.grid if node.grid is not None else occupancy_grid_from_pgm(map_yaml)
        preflight_initial = preflight_goal(grid, initial_map, robot_radius_m,
                                           occupied_thresh)
        preflight_initial['robot_radius_m'] = robot_radius_m
        preflight_initial['robot_radius_sources'] = radius_sources
        preflight_initial['occupied_thresh'] = occupied_thresh
        record['preflight_initial'] = preflight_initial
        if not preflight_initial['ok']:
            node.publish_mission_state(
                MissionState.STATE_UNREACHABLE, goal_pose,
                outcome='runner:not_sent:initial_pose_occupied',
                detail=preflight_initial['detail'])
            record['mission_state'] = MissionState.STATE_UNREACHABLE
            record['outcome'] = 'runner:not_sent:initial_pose_occupied'
            record_undispatched(
                record, scenario['interventions'], scenario['world'], model,
                'start_pose_refused',
                'the scenario start pose was refused pre-flight, so no mission ran')
            record['rtf_samples'] = measure_rtf(scenario['world'])
            raise RunAbort(EXIT_OK, 'start pose refused pre-flight; no goal sent')

        # 8b. The driver's own pre-goal work: a localization warm-up for the
        # Nav2 platform, a mode change and arming for the autopilot. A refusal
        # here ends the run before any goal is sent, with the driver's reason.
        refusal = driver.prepare(initial_map, record)
        if refusal:
            outcome, detail = refusal
            node.publish_mission_state(MissionState.STATE_REJECTED, goal_pose,
                                       outcome=outcome, detail=detail)
            record['mission_state'] = MissionState.STATE_REJECTED
            record['outcome'] = outcome
            record['detail'] = detail
            record_undispatched(
                record, scenario['interventions'], scenario['world'], model,
                'stack_not_ready',
                'the stack could not be prepared to accept a goal, so no mission ran')
            record['rtf_samples'] = measure_rtf(scenario['world'])
            raise RunAbort(EXIT_OK, detail)

        # 9. Pre-flight. The only path to `unreachable`.
        preflight = preflight_goal(grid, goal_map, robot_radius_m, occupied_thresh)
        preflight['robot_radius_m'] = robot_radius_m
        preflight['robot_radius_sources'] = radius_sources
        preflight['occupied_thresh'] = occupied_thresh
        record['preflight'] = preflight
        if not preflight['ok']:
            # `runner:` is the SOURCE prefix every outcome carries: the gate
            # that refused is this runner's, not either stack's. It is added
            # here as well as in the record so the published value and the
            # recorded one are the same member of the closed set.
            outcome = f'runner:{preflight["outcome"]}'
            node.publish_mission_state(MissionState.STATE_UNREACHABLE, goal_pose,
                                       outcome=outcome,
                                       detail=preflight['detail'])
            record['mission_state'] = MissionState.STATE_UNREACHABLE
            record['outcome'] = outcome
            record_undispatched(
                record, scenario['interventions'], scenario['world'], model,
                'goal_unreachable',
                'the goal was refused pre-flight, so no mission ran')
            record['rtf_samples'] = measure_rtf(scenario['world'])
            raise RunAbort(EXIT_OK, 'goal refused pre-flight; no goal sent')

        # 10. Send the goal, through the driver. What "sent" means differs:
        # an accepted action goal for Nav2, a published setpoint for the
        # autopilot, which acknowledges nothing. The driver's detail says
        # which, so run.json cannot read as a stronger claim than was made.
        accepted, outcome, detail = driver.send_goal(goal_pose, record)
        if not accepted:
            # The driver NAMES the outcome; the runner does not infer one by
            # inspecting the detail string, which is prose written for an
            # operator and is not a protocol.
            node.publish_mission_state(MissionState.STATE_REJECTED, goal_pose,
                                       outcome=outcome, detail=detail)
            record['mission_state'] = MissionState.STATE_REJECTED
            record['outcome'] = outcome
            record_undispatched(
                record, scenario['interventions'], scenario['world'], model,
                'goal_not_accepted',
                'the stack did not accept the goal, so no mission ran')
            record['rtf_samples'] = measure_rtf(scenario['world'])
            raise RunAbort(EXIT_OK, detail)

        t_accept = node.sim_now()
        record['t_accept_sim'] = t_accept
        node.publish_mission_state(MissionState.STATE_GOAL_ACCEPTED, goal_pose,
                                   detail=detail)
        navigating_published = False

        # 11. Mission loop. A single spin_sim(until=...) would not do: the
        # interventions have to fire on SIM time while the goal is running,
        # and the driver has to be spun for its own supervision and - for a
        # platform commanded by setpoint - its republication.
        pending = list(scenario['interventions'])
        timeout_sec = scenario['timeout_sec']
        canceled = False
        cancel_deadline = None
        stalled = False
        # Wall deadlock guard ONLY. Ten times the sim budget plus two minutes: a
        # slow real-time factor must change how long a run takes, never what it
        # measures, so this can only fire when /clock has stopped advancing.
        # Without it, a gz server that dies mid-mission freezes sim_now(), and
        # both loop exits (the driver's terminal and the cancel deadline)
        # depend on it - the runner would spin for ever with the recorder up.
        mission_wall_deadline = time.monotonic() + timeout_sec * 10.0 + 120.0
        last_sim = node.sim_now()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            driver.spin_once()
            if driver.terminal() is not None:
                break
            # The runner is the only publisher of /mission_state, so the
            # transition the driver OBSERVED is published here, once.
            if not navigating_published and driver.navigating_since is not None:
                navigating_published = True
                node.publish_mission_state(
                    MissionState.STATE_NAVIGATING, goal_pose,
                    detail='the stack was observed executing the goal at sim '
                           f'{driver.navigating_since:.2f} s')
            elapsed = node.sim_now() - t_accept
            last_sim = node.sim_now()
            if pending and elapsed >= pending[0]['at_sec']:
                intervention = pending.pop(0)
                print(f'--- intervention {intervention["id"]} at {elapsed:.2f} s sim '
                      f'(scheduled {intervention["at_sec"]:.2f})', flush=True)
                record['interventions'].append(
                    dispatch_intervention(node, intervention, scenario['world'],
                                          model, elapsed))
                continue
            if not canceled and elapsed >= timeout_sec:
                print(f'--- timeout at {elapsed:.1f} s sim; asking the stack to stop',
                      flush=True)
                driver.request_cancel('timeout')
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
            record, pending, scenario['world'], model, 'mission_ended_first',
            f'the mission ended at {node.sim_now() - t_accept:.2f} s sim, before this '
            'intervention\'s at_sec')

        # 12/13. Terminal handling. The outcome is whatever the stack itself
        # reported, carrying the prefix of the system that reported it, and is
        # never inferred when it is absent.
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
            # NOT `canceled`: a cancel is terminal only when the stack says so,
            # and none did. The cancel is reported in `detail` instead.
            state = MissionState.STATE_UNKNOWN
            outcome = 'runner:unknown:no_terminal_status'
            detail = (f'{driver.no_terminal_detail()}; no terminal result arrived '
                      + (f'within {CANCEL_WAIT_SIM:.0f} s sim of the cancel'
                         if canceled else 'and the mission loop ended'))
        record['t_terminal_sim'] = node.sim_now()
        record['mission_state'] = state
        record['outcome'] = outcome
        record['detail'] = detail
        node.publish_mission_state(state, goal_pose, outcome=outcome, detail=detail)
        node.spin_sim(POST_TERMINAL_SIM)
        record['rtf_samples'] = measure_rtf(scenario['world'])

    except RunAbort as abort:
        exit_code = abort.code
        print(f'--- run stopped: {abort}', flush=True)
    except Exception:                                   # noqa: BLE001 - reported, not hidden
        exit_code = EXIT_ARTIFACTS
        record['outcome'] = record['outcome'] or 'runner:exception'
        record['detail'] = traceback.format_exc()
        traceback.print_exc()
    finally:
        record['sim_end'] = node.sim_now()
        # The plant first, while the node and the stack are both still alive:
        # a driver that has armed an autopilot or left a velocity standing has
        # to undo it, and it cannot once rclpy is down.
        try:
            driver.stop()
            driver.teardown()
        except Exception:                               # noqa: BLE001 - reported
            traceback.print_exc()
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
    # messages, or short of the latched messages the QoS override pins, means
    # the recorder ran but the evidence is not there. Converting that into a
    # failed run is the point: the alternative is an empty panel A that looks
    # like a plotting bug.
    #
    # WHICH latched topics are pinned is per robot: /tf_static exists for
    # every platform, /map only for one that runs a map_server. The car
    # localizes against GPS and an IMU and records no /map at all, and pinning
    # one would fail every car run for a message its stack never publishes.
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
            required = {'/tf_static': 2}
            if '/map' in record_topics:
                required['/map'] = 1
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
        record['outcome'] = record['outcome'] or 'runner:bag_missing'
        record['detail'] = f'{bag_detail}; the run cannot be measured'
        print(f'BAG UNUSABLE: {record["detail"]}', file=sys.stderr, flush=True)

    record['finished_utc'] = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
    record['exit_code'] = exit_code
    _warn_unknown_outcome(record)
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
        _warn_unknown_outcome(record)
        write_json(os.path.join(run_dir, 'run.json'), record)
        return EXIT_ARTIFACTS

    print(f'--- run complete: {run_dir} (exit {exit_code})', flush=True)
    return exit_code


def _warn_unknown_outcome(record: dict) -> None:
    """
    Shout when run.json is about to carry an outcome outside the closed set.

    A warning rather than a refusal: an unknown outcome is a defect in the
    vocabulary, and losing the run's artifacts to it would destroy the
    evidence needed to fix it. publish_mission_state is the strict gate; this
    covers the outcomes the runner writes without publishing a MissionState.
    """
    outcome = record.get('outcome') or ''
    if outcome and not outcome_is_known(outcome):
        print(f'--- WARNING: outcome {outcome} is not in mission_driver.OUTCOMES',
              file=sys.stderr, flush=True)


def main(args=None) -> None:
    sys.exit(run(args))


if __name__ == '__main__':
    main()
