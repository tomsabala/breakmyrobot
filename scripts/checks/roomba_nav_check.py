#!/usr/bin/env python3
"""
End-to-end proof of the map, AMCL and Nav2 closed loop.

Brings the whole stack up with `ros2 launch ral_bringup roomba_nav.launch.py`,
localizes the robot against the committed static map, sends one real
NavigateToPose goal that can only be reached through wall B's 0.70 m doorway,
and asserts twelve properties against the running system. Writes
docs/nav-closed-loop.md and exits non-zero if any check fails.

Every duration is measured in SIM time off /clock, never wall time: a slow
real-time factor must not change a result, only how long the run takes. Wall
limits exist solely as deadlock guards.

Run through scripts/checks/roomba_nav_check.sh, which resolves the container
image reference on the host and runs this inside ral:dev.

Frames
------
The goal and the initial pose below are WORLD-frame coordinates, converted to
`map` here with the measured alignment in maps/apartment_frames.yaml. The
four lines of conversion are re-implemented in this file rather than imported
from ral_eval.frames, deliberately: a verifier that imports the code under
test shares its bugs, and frames.py's own unit tests cannot catch a wrong
alignment FILE, only wrong arithmetic.

Negative controls
-----------------
Green checks prove the stack works; they do not prove the checks can fail.
Each mutation below was applied to a working stack and the named check was
observed to go red. Repeat them after changing a check - a check that cannot
fail is decoration.

2026-10-22, image sha256:ce16e283:

* Point `amcl.scan_topic` and both costmaps' `obstacle_layer.scan.topic` at
  /scan_raw and remove scan_fault_relay from roomba_nav.launch.py, i.e. route
  every lidar consumer around the fault boundary. Observed: the bring-up gate
  fails first - `FAIL 0. bring-up: no messages on /scan within 90 s wall` -
  because nothing publishes /scan at all, and ral_bringup's
  test_nav2_contract fails one case without starting a simulator
  (`test_every_lidar_consumer_reads_the_relay_output`). Check 2 is the
  runtime backstop for the subtler version of this, a SECOND publisher on
  /scan, which brings the stack up normally.
* Add a second /tf row to the bridge config fed from /model/ral_roomba/pose
  (simulator truth). Observed: check 5 FAIL, `unexpected = ['ral_apartment',
  'ral_roomba']`, and ral_bringup's test_bridge_contract fails two cases
  without a simulator. Note check 4 still PASSED under this mutation: truth
  reaches the stack through /tf, which no node has to subscribe to by name,
  so the subscriber test alone does not see it. That is precisely why
  check 5 asserts the whole frame set.
* Delete maps/apartment_frames.yaml. Observed: roomba_nav.launch.py aborts
  with `[ERROR] [launch]: Caught exception in launch ... cannot read the
  world->map alignment at .../apartment_frames.yaml ... regenerate it with:
  ros2 launch ral_bringup map_build.launch.py` - rather than silently
  publishing an identity alignment that every later metric would report as
  localization error.

2026-10-23, image sha256:ce16e283:

* Overwrite `map_sha256` in maps/apartment_frames.yaml with a different
  digest, i.e. the stale pair: a regenerated map beside the alignment of the
  one before it. Observed: `scripts/checks/map_sanity.py` FAIL `alignment
  belongs to this grid ... records map_sha256 0..., apartment.pgm hashes to
  47d1e2ad01a41a52...`, MAP REJECTED 6/7, and nothing re-sealed; and
  roomba_nav.launch.py aborts before gz is spawned with `... was measured
  against a different grid ... Rebuild both together`. Check 1 is the runtime
  backstop, re-derived from the bytes the running stack loaded.
* That control's first run exposed a bug in the guard itself: an all-digit
  digest parses as a YAML integer, so the comparison raised `'int' object is
  not subscriptable` from inside the error message. Both readers now coerce
  to str, and map_sanity.py quotes the value it writes.
* Set `velocity_smoother.max_velocity[0]` back to Nav2's shipped TurtleBot3
  `0.26`, i.e. the silent clamp every override in nav2_roomba.yaml exists to
  prevent. Observed: check 12 FAIL, `max |cmd_vel.linear.x| = 0.260 m/s
  (configured 0.4, required 0.380-0.4, plant 0.46)` - and check 7 still
  PASSED, the mission simply took 54.1 s sim instead of 37.6 s. That is the
  whole argument for the LOWER bound: the clamp changes no outcome, only a
  duration, and an upper bound at the plant's 0.46 m/s cannot see it.
  test_nav2_contract catches this particular mutation in the YAML without a
  simulator; the runtime bound is what shows the YAML is in force.

Harness bugs these controls and the first green run exposed, fixed here:

* The TF topology check originally ran BEFORE /initialpose was published.
  AMCL publishes map -> odom only once it has a pose, so `odom` was correctly
  reported as a root frame and the check failed for a reason unrelated to the
  tree. Localization now happens first.
* The alignment residual check originally waited for AMCL to converge while
  the robot stood still. AMCL updates its filter on motion (update_min_d 0.1,
  update_min_a 0.2), so the covariance trace sat at 0.4474 after 30 s sim and
  never dropped. The harness now rotates the robot in place while waiting,
  which leaves xy unchanged, then stops before measuring.
"""

import argparse
import hashlib
import math
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan
import tf2_ros
import yaml

WORLD = 'ral_apartment'
MODEL = 'ral_roomba'

# World-frame mission. The start is the spawn pose; the goal is in room 3,
# which is reachable from room 2 only through wall B's 0.70 m doorway at
# (8.0, 3.0). Checks 7-9 together are what make that a navigation result
# rather than a coincidence.
INITIAL_POSE_WORLD = (0.5, 0.5, 0.0)
GOAL_WORLD = (10.5, 6.5, 1.57)

# Wall B, from ral_description/worlds/ral_apartment.sdf: the gap spans
# y 2.65..3.35 at x = 8.0, i.e. 0.70 m centred on y = 3.0.
DOORWAY_X = 8.0
DOORWAY_Y = 3.0
DOORWAY_HALF_GAP = 0.35

# Plant limits from ral_description/models/ral_roomba/model.sdf's DiffDrive.
# Asserted against, never derived from, the configuration they verify.
PLANT_MAX_LINEAR = 0.46
PLANT_MAX_ANGULAR = 1.9

# Every TF frame this stack may publish, and nothing else. `world` is present
# because roomba_nav.launch.py publishes a static, display-only map -> world
# transform built from the measured alignment; it carries no robot pose.
# Asserting the WHOLE set is what makes this a truth-leak detector: simulator
# truth bridged onto /tf arrives under the gz PosePublisher's own names
# (ral_apartment -> ral_roomba), which an absence test on `base_link_truth`
# would not see (measured in Phase 1).
EXPECTED_TF_FRAMES = {
    'map',
    'odom',
    'base_link',
    'rplidar_link',
    'left_wheel',
    'right_wheel',
    'caster_front',
    'caster_rear',
    'world',
}

# Nodes that must never subscribe to simulator truth (ENGINEERING.md section
# 4). Matched as substrings against live node names.
NAV_NODE_PATTERNS = ('amcl', 'controller_server', 'planner_server', 'bt_navigator',
                     'costmap', 'velocity_smoother', 'slam_toolbox', 'behavior_server',
                     'smoother_server', 'waypoint_follower')

BRINGUP_WALL_LIMIT = 90.0      # s wall, guards a stack that never comes up
AMCL_CONVERGE_SIM = 30.0       # s sim allowed for the filter to converge
# AMCL updates its filter on motion, not on time. The harness therefore rotates
# the robot in place while waiting for convergence; rotation leaves xy
# unchanged, so the residual is still measured at the start pose.
AMCL_WARMUP_RATE = 0.6         # rad/s commanded during convergence
AMCL_SETTLE_SIM = 2.0          # s sim stopped before reading the residual
MISSION_LIMIT_SIM = 180.0      # s sim allowed for the whole mission
COVARIANCE_LIMIT = 0.25        # trace(x, y, yaw) below which AMCL has converged
RESIDUAL_LIMIT = 0.15          # m, stationary estimate-vs-truth floor
GOAL_TRUTH_TOLERANCE = 0.25    # m, truth distance to the goal in `map`
ESTIMATE_ERROR_MAX = 0.50      # m, mean |AMCL - truth| ceiling
ESTIMATE_ERROR_MIN = 0.005     # m, below this the "estimate" is truth
STAMP_MATCH_S = 0.05           # s, nearest-stamp pairing window


def twist(linear_x: float = 0.0, angular_z: float = 0.0) -> Twist:
    msg = Twist()
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    return msg


# ral_eval.frames has these; see the module docstring for why they are not
# imported. Four lines of standard planar geometry.
def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def world_to_map_pose(x: float, y: float, yaw: float, wtm):
    wx, wy, wyaw = wtm
    dx, dy = x - wx, y - wy
    return (dx * math.cos(wyaw) + dy * math.sin(wyaw),
            -dx * math.sin(wyaw) + dy * math.cos(wyaw),
            wrap_to_pi(yaw - wyaw))


def yaw_of_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_of_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def stamp_seconds(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def load_alignment(maps_dir: str):
    """Read the measured world->map alignment that belongs to the map in use."""
    path = os.path.join(maps_dir, 'apartment_frames.yaml')
    with open(path, 'r') as handle:
        data = yaml.safe_load(handle)
    wtm = data['world_to_map']
    return (float(wtm['x']), float(wtm['y']), float(wtm['yaw'])), data, path


class Phase2Check(Node):

    def __init__(self) -> None:
        super().__init__(
            'roomba_nav_check',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.odom = None
        self.truth = None
        self.scan = None
        self.scan_raw = None
        self.grid = None
        self.amcl = None

        # Mission recordings. Collected whenever `recording` is set, so
        # bring-up traffic cannot contribute to a mission statistic.
        self.recording = False
        self.truth_track = []      # (sim stamp, x, y) in WORLD frame
        self.amcl_track = []       # (sim stamp, x, y) in MAP frame
        self.cmd_max_linear = 0.0
        self.cmd_max_angular = 0.0
        self.plan_count = 0
        self.collisions = []       # collision name pairs seen on /bumper
        # /scan vs /scan_raw pairs, keyed by header stamp, for check 3.
        self.scan_by_stamp = {}
        self.scan_raw_by_stamp = {}

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self._on_truth, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/scan_raw', self._on_scan_raw, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(Path, '/plan', self._on_plan, 10)
        self.create_subscription(Contacts, '/bumper', self._on_bumper, 10)

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        # Used ONLY for the pre-mission localization warm-up, never during the
        # mission: Nav2's velocity_smoother owns /cmd_vel while a goal is
        # active, and `recording` is off here so these commands cannot enter
        # check 12's envelope measurement.
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)

    # -- subscriptions ---------------------------------------------------------
    def _on_odom(self, msg):
        self.odom = msg

    def _on_truth(self, msg):
        self.truth = msg
        if self.recording:
            self.truth_track.append((stamp_seconds(msg.header),
                                     msg.pose.pose.position.x,
                                     msg.pose.pose.position.y))

    def _on_scan(self, msg):
        self.scan = msg
        self.scan_by_stamp[(msg.header.stamp.sec, msg.header.stamp.nanosec)] = msg

    def _on_scan_raw(self, msg):
        self.scan_raw = msg
        self.scan_raw_by_stamp[(msg.header.stamp.sec, msg.header.stamp.nanosec)] = msg

    def _on_map(self, msg):
        self.grid = msg

    def _on_amcl(self, msg):
        self.amcl = msg
        if self.recording:
            self.amcl_track.append((stamp_seconds(msg.header),
                                    msg.pose.pose.position.x,
                                    msg.pose.pose.position.y))

    def _on_cmd(self, msg):
        if self.recording:
            self.cmd_max_linear = max(self.cmd_max_linear, abs(msg.linear.x))
            self.cmd_max_angular = max(self.cmd_max_angular, abs(msg.angular.z))

    def _on_plan(self, msg):
        if self.recording:
            self.plan_count += 1

    def _on_bumper(self, msg):
        if self.recording and msg.contacts:
            for contact in msg.contacts:
                self.collisions.append(f'{contact.collision1.name} / {contact.collision2.name}')

    # -- time ------------------------------------------------------------------
    def sim_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_wall(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def spin_sim(self, sim_seconds: float, until=None, cmd: Twist = None) -> bool:
        """Spin until `sim_seconds` of SIM time pass, or `until()` fires.

        Republishes `cmd` at 20 Hz while spinning, for the localization
        warm-up only. Returns True if `until` fired.
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


def measure_rtf() -> str:
    """Read the real-time factor gz reports for this world."""
    try:
        out = subprocess.run(
            ['gz', 'topic', '-e', '-t', f'/world/{WORLD}/stats', '-n', '3'],
            capture_output=True, text=True, timeout=30).stdout
    except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
        return f'unavailable ({type(exc).__name__})'
    values = re.findall(r'real_time_factor:\s*([0-9.]+)', out)
    if not values:
        return 'unavailable (no real_time_factor in /world stats)'
    return ', '.join(values)


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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--image-ref', default='unknown',
                        help='ral:dev digest or image id, collected on the host')
    parser.add_argument('--repo-root', default='/ws')
    args = parser.parse_args()

    log_path = os.path.join(args.repo_root, 'scripts', 'checks', 'roomba_nav_launch.log')
    results = []

    def record(number, name, passed, detail):
        results.append((number, name, passed, detail))
        print(f'{"PASS" if passed else "FAIL"}  {number}. {name}: {detail}', flush=True)

    maps_dir = os.path.join(get_package_share_directory('ral_bringup'), 'maps')
    wtm, alignment_doc, alignment_path = load_alignment(maps_dir)
    pgm_sha = hashlib.sha256(
        open(os.path.join(maps_dir, 'apartment.pgm'), 'rb').read()).hexdigest()
    nav2_params = os.path.join(
        get_package_share_directory('ral_bringup'), 'config', 'nav2_roomba.yaml')
    params = yaml.safe_load(open(nav2_params))
    # The envelope Nav2 is CONFIGURED to stay inside, which is what the
    # observed commands are judged against; PLANT_MAX_* above is the wider
    # bound the hardware itself imposes.
    configured_max_linear = params[
        'controller_server']['ros__parameters']['FollowPath']['max_vel_x']
    inflation = {
        name: params[name][name]['ros__parameters']['inflation_layer']['inflation_radius']
        for name in ('local_costmap', 'global_costmap')}

    initial_map = world_to_map_pose(*INITIAL_POSE_WORLD, wtm)
    goal_map = world_to_map_pose(*GOAL_WORLD, wtm)
    print(f'--- world->map alignment {wtm} from {alignment_path}', flush=True)
    print(f'--- initial pose world {INITIAL_POSE_WORLD} -> map '
          f'({initial_map[0]:.3f}, {initial_map[1]:.3f}, {initial_map[2]:.3f})', flush=True)
    print(f'--- goal world {GOAL_WORLD} -> map '
          f'({goal_map[0]:.3f}, {goal_map[1]:.3f}, {goal_map[2]:.3f})', flush=True)

    rclpy.init()
    node = Phase2Check()

    with open(log_path, 'w') as log:
        launch = subprocess.Popen(
            ['ros2', 'launch', 'ral_bringup', 'roomba_nav.launch.py'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

        sim_start = None
        sim_end = 0.0
        rtf = 'not measured'
        try:
            # -- bring-up ------------------------------------------------------
            needed = {
                '/clock': lambda: node.sim_now() > 0.0,
                '/odom': lambda: node.odom is not None,
                '/ground_truth/odom': lambda: node.truth is not None,
                '/scan': lambda: node.scan is not None,
                '/scan_raw': lambda: node.scan_raw is not None,
                '/map': lambda: node.grid is not None,
            }
            deadline = time.monotonic() + BRINGUP_WALL_LIMIT
            missing = None
            while time.monotonic() < deadline:
                node.spin_wall(0.5)
                missing = [topic for topic, ready in needed.items() if not ready()]
                if not missing:
                    break
            if missing:
                record(0, 'bring-up', False,
                       f'no messages on {", ".join(missing)} within '
                       f'{BRINGUP_WALL_LIMIT:.0f} s wall')
                raise SystemExit(1)

            nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
            server_up = False
            server_deadline = time.monotonic() + 60.0
            while time.monotonic() < server_deadline:
                node.spin_wall(0.5)
                if nav_client.server_is_ready():
                    server_up = True
                    break
            if not server_up:
                record(0, 'bring-up', False,
                       'navigate_to_pose action server never appeared within 60 s wall; '
                       'Nav2 did not activate (see the launch log)')
                raise SystemExit(1)

            sim_start = node.sim_now()
            print(f'--- stack up at sim t={sim_start:.2f} s', flush=True)

            # -- 1. Map loaded, and the alignment belongs to it ------------------
            # The alignment is only meaningful for the grid it was measured
            # against. ral_eval.frames enforces that on the launch path; this
            # re-derives it from the bytes the stack actually loaded, because
            # a verifier that trusts the thing it verifies proves nothing.
            info = node.grid.info
            span_x = info.width * info.resolution
            span_y = info.height * info.resolution
            sealed_to = alignment_doc.get('map_sha256')
            record(1, 'static map loaded and matched to its alignment',
                   abs(info.resolution - 0.05) < 1e-6 and span_x >= 12.0
                   and span_y >= 8.0 and sealed_to == pgm_sha,
                   f'resolution {info.resolution} m, {info.width} x {info.height} cells = '
                   f'{span_x:.2f} x {span_y:.2f} m (>= 12.00 x 8.00 required for the '
                   f'apartment), sha256(apartment.pgm) = {pgm_sha[:16]}, alignment '
                   f'sealed to {str(sealed_to)[:16]}')

            # -- 2. One /scan publisher, and it is the relay ---------------------
            scan_pubs = node.get_publishers_info_by_topic('/scan')
            raw_pubs = node.get_publishers_info_by_topic('/scan_raw')
            scan_names = sorted(pub.node_name for pub in scan_pubs)
            raw_names = sorted(pub.node_name for pub in raw_pubs)
            record(2, '/scan has exactly one publisher: scan_fault_relay',
                   scan_names == ['scan_fault_relay'] and len(raw_names) == 1,
                   f'/scan publishers = {scan_names}, /scan_raw publishers = {raw_names}; '
                   'a second publisher or a different node means lidar faults would not '
                   'reach every consumer')

            # -- 3. The relay is a faithful pass-through -------------------------
            # Compared on identical header stamps: the relay must republish the
            # message object unchanged, so a restamp, a reorder or a resynthesis
            # of ranges shows up here. A corrupted pass-through silently
            # invalidates every non-fault run in the project.
            node.spin_wall(2.0)
            shared = set(node.scan_by_stamp) & set(node.scan_raw_by_stamp)
            if not shared:
                record(3, 'relay output equals relay input', False,
                       'no /scan and /scan_raw message shared a header stamp')
            else:
                stamp = sorted(shared)[-1]
                relayed = node.scan_by_stamp[stamp]
                raw = node.scan_raw_by_stamp[stamp]
                identical = (list(relayed.ranges) == list(raw.ranges)
                             and relayed.header.frame_id == raw.header.frame_id
                             and relayed.angle_min == raw.angle_min
                             and relayed.range_max == raw.range_max)
                record(3, 'relay output equals relay input', identical,
                       f'at stamp {stamp[0]}.{stamp[1]:09d}: {len(raw.ranges)} ranges, '
                       f"frame_id '{raw.header.frame_id}' -> '{relayed.header.frame_id}', "
                       f'element-for-element identical = {identical}')

            # -- 4. Truth is consumed by nobody in the nav path ------------------
            truth_subs = node.get_subscriptions_info_by_topic('/ground_truth/odom')
            truth_sub_names = sorted(sub.node_name for sub in truth_subs)
            offenders = [name for name in truth_sub_names
                         if any(pattern in name for pattern in NAV_NODE_PATTERNS)]
            record(4, 'no navigation component subscribes to truth', not offenders,
                   f'/ground_truth/odom subscribers = {truth_sub_names}; navigation '
                   f'offenders = {offenders}')

            # -- localize, then assert TF and the residual -----------------------
            # Ordering matters twice over:
            #   * AMCL publishes map -> odom only once it HAS a pose. With
            #     set_initial_pose false there is none until /initialpose is
            #     published, so a TF topology check run before this point
            #     correctly reports `odom` as a root frame and fails for a
            #     reason that has nothing to do with the tree being wrong.
            #   * AMCL's filter updates on MOTION (update_min_d 0.1,
            #     update_min_a 0.2). A robot standing still never resamples, so
            #     its covariance stays at whatever /initialpose declared no
            #     matter how long it waits - measured: trace 0.4474 after 30 s
            #     sim of standing still. The in-place rotation below gives the
            #     filter the angular motion it needs and returns the robot to
            #     the same xy, so the residual is still measured at the start
            #     pose with the robot stationary.
            node.amcl = None
            for _ in range(3):
                node.publish_initial_pose(*initial_map)
                node.spin_wall(0.5)

            def converged():
                if node.amcl is None:
                    return False
                cov = node.amcl.pose.covariance
                return cov[0] + cov[7] + cov[35] < COVARIANCE_LIMIT

            ok = node.spin_sim(AMCL_CONVERGE_SIM, cmd=twist(angular_z=AMCL_WARMUP_RATE),
                               until=converged)
            node.stop()
            node.spin_sim(AMCL_SETTLE_SIM)

            # -- 5. TF topology ---------------------------------------------------
            frames_yaml = yaml.safe_load(node.tf_buffer.all_frames_as_yaml()) or {}
            parents = {name: info.get('parent') for name, info in frames_yaml.items()}
            present = set(parents) | {p for p in parents.values() if p}
            unexpected = present - EXPECTED_TF_FRAMES
            missing_frames = EXPECTED_TF_FRAMES - present
            topology = {
                'odom': 'map',
                'base_link': 'odom',
                'rplidar_link': 'base_link',
                'world': 'map',
            }
            wrong = {child: (parents.get(child), parent)
                     for child, parent in topology.items()
                     if parents.get(child) != parent}
            record(5, 'TF topology', not unexpected and not missing_frames and not wrong,
                   f'frames = {sorted(present)}; unexpected = {sorted(unexpected)}, '
                   f'missing = {sorted(missing_frames)}, wrong parents (got, want) = '
                   f'{wrong or "none"}')

            # -- 6. Alignment residual with the robot stationary -----------------
            if not ok or node.amcl is None:
                trace_text = ('no /amcl_pose' if node.amcl is None else
                              f'{sum(node.amcl.pose.covariance[i] for i in (0, 7, 35)):.4f}')
                record(6, 'alignment residual', False,
                       f'AMCL did not converge within {AMCL_CONVERGE_SIM:.0f} s sim '
                       f'(covariance trace {trace_text}, limit {COVARIANCE_LIMIT})')
            else:
                cov = node.amcl.pose.covariance
                trace = cov[0] + cov[7] + cov[35]
                truth_in_map = world_to_map_pose(node.truth.pose.pose.position.x,
                                                 node.truth.pose.pose.position.y,
                                                 0.0, wtm)
                residual = math.hypot(node.amcl.pose.pose.position.x - truth_in_map[0],
                                      node.amcl.pose.pose.position.y - truth_in_map[1])
                record(6, 'alignment residual', residual < RESIDUAL_LIMIT,
                       f'stationary |AMCL - truth| = {residual:.4f} m (limit '
                       f'{RESIDUAL_LIMIT}), covariance trace {trace:.4f} after an '
                       f'in-place rotation at {AMCL_WARMUP_RATE} rad/s. This is the '
                       f'floor below which estimate error cannot be interpreted: it is '
                       f'map error plus alignment error, not localization error')

            # -- 7. Mission ------------------------------------------------------
            node.recording = True
            mission_start = node.sim_now()
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x = goal_map[0]
            goal.pose.pose.position.y = goal_map[1]
            qx, qy, qz, qw = quaternion_of_yaw(goal_map[2])
            goal.pose.pose.orientation.x = qx
            goal.pose.pose.orientation.y = qy
            goal.pose.pose.orientation.z = qz
            goal.pose.pose.orientation.w = qw

            send_future = nav_client.send_goal_async(goal)
            node.spin_sim(30.0, until=send_future.done)
            handle = send_future.result() if send_future.done() else None
            if handle is None or not handle.accepted:
                record(7, 'mission succeeds', False,
                       'NavigateToPose goal was not accepted'
                       if handle is not None else 'no response to the goal request')
                status = None
            else:
                result_future = handle.get_result_async()
                finished = node.spin_sim(MISSION_LIMIT_SIM, until=result_future.done)
                status = result_future.result().status if finished else None
                mission_sim = node.sim_now() - mission_start
                verdict = ('SUCCEEDED' if status == GoalStatus.STATUS_SUCCEEDED
                           else 'not SUCCEEDED')
                record(7, 'mission succeeds', status == GoalStatus.STATUS_SUCCEEDED,
                       f'NavigateToPose result status = {status} ({verdict}) '
                       f'after {mission_sim:.1f} s sim (limit {MISSION_LIMIT_SIM:.0f})')
            node.spin_wall(1.0)
            node.recording = False

            # -- 8. Goal reached in truth -----------------------------------------
            final_truth = world_to_map_pose(node.truth.pose.pose.position.x,
                                            node.truth.pose.pose.position.y, 0.0, wtm)
            goal_error = math.hypot(final_truth[0] - goal_map[0],
                                    final_truth[1] - goal_map[1])
            record(8, 'goal reached in ground truth', goal_error < GOAL_TRUTH_TOLERANCE,
                   f'|final truth - goal| in map = {goal_error:.3f} m (limit '
                   f'{GOAL_TRUTH_TOLERANCE}); truth world = '
                   f'({node.truth.pose.pose.position.x:.3f}, '
                   f'{node.truth.pose.pose.position.y:.3f})')

            # -- 9. Doorway threaded ----------------------------------------------
            crossings = []
            for before, after in zip(node.truth_track, node.truth_track[1:]):
                if (before[1] - DOORWAY_X) * (after[1] - DOORWAY_X) <= 0.0 \
                        and before[1] != after[1]:
                    y_at = (before[2] + after[2]) / 2.0
                    crossings.append(y_at)
            threaded = [y for y in crossings if abs(y - DOORWAY_Y) <= DOORWAY_HALF_GAP]
            record(9, f'threaded the {2 * DOORWAY_HALF_GAP:.2f} m doorway',
                   bool(threaded),
                   f'{len(crossings)} crossing(s) of x = {DOORWAY_X} at y = '
                   f'{[round(y, 3) for y in crossings] or "none"}; inside the gap '
                   f'(|y - {DOORWAY_Y}| <= {DOORWAY_HALF_GAP}): {len(threaded)}')

            # -- 10. No collisions -------------------------------------------------
            record(10, 'no collisions during the mission', not node.collisions,
                   f'{len(node.collisions)} contact report(s) on /bumper'
                   + (f': {sorted(set(node.collisions))}' if node.collisions else ''))

            # -- 11. The estimate is an estimate, not truth ------------------------
            errors = []
            for stamp, ax, ay in node.amcl_track:
                match = nearest(node.truth_track, stamp)
                if match is None:
                    continue
                tx, ty, _ = world_to_map_pose(match[1], match[2], 0.0, wtm)
                errors.append(math.hypot(ax - tx, ay - ty))
            if not errors:
                record(11, 'estimate is an estimate, not truth', False,
                       'no AMCL/truth sample pair within '
                       f'{STAMP_MATCH_S * 1000:.0f} ms during the mission')
                mean_error = max_error = float('nan')
            else:
                mean_error = sum(errors) / len(errors)
                max_error = max(errors)
                record(11, 'estimate is an estimate, not truth',
                       ESTIMATE_ERROR_MIN < mean_error < ESTIMATE_ERROR_MAX,
                       f'mean |AMCL - truth| = {mean_error:.4f} m, max = {max_error:.4f} m '
                       f'over {len(errors)} paired samples (required: > '
                       f'{ESTIMATE_ERROR_MIN} so it is not truth, < {ESTIMATE_ERROR_MAX} '
                       f'so it is localized)')

            # -- 12. Commands match the envelope Nav2 is configured for ------------
            # Two-sided on linear speed, and the lower bound is the point. An
            # upper bound at the PLANT limit (0.46 m/s) passes for any clamp
            # below it - including velocity_smoother left at its shipped
            # TurtleBot3 0.26 m/s, the exact failure the config overrides
            # guard against. What proves the configured envelope is in force
            # at runtime is that the robot actually reached it.
            #
            # Angular has no lower bound: this mission's geometry does not
            # require the full 1.5 rad/s, so demanding it would fail for a
            # reason that is not a fault.
            linear_floor = 0.95 * configured_max_linear
            record(12, 'commands match the configured envelope',
                   linear_floor <= node.cmd_max_linear <= configured_max_linear + 1e-3
                   and node.cmd_max_angular <= PLANT_MAX_ANGULAR
                   and node.plan_count > 0,
                   f'max |cmd_vel.linear.x| = {node.cmd_max_linear:.3f} m/s (configured '
                   f'{configured_max_linear}, required {linear_floor:.3f}-'
                   f'{configured_max_linear}, plant {PLANT_MAX_LINEAR}), max '
                   f'|cmd_vel.angular.z| = {node.cmd_max_angular:.3f} rad/s (plant '
                   f'{PLANT_MAX_ANGULAR}); {node.plan_count} /plan publication(s)')

            rtf = measure_rtf()

        except SystemExit:
            pass
        finally:
            sim_end = node.sim_now()
            node.destroy_node()
            rclpy.shutdown()
            try:
                os.killpg(os.getpgid(launch.pid), signal.SIGINT)
                launch.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(launch.pid), signal.SIGKILL)
                launch.wait(timeout=10)
            except ProcessLookupError:
                pass

    sim_duration = (sim_end - sim_start) if sim_start is not None else 0.0
    passed = all(result[2] for result in results) and len(results) == 12

    doc = [
        '# Map, AMCL and Nav2 closed loop — measured proof',
        '',
        f'Measured {datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")} by',
        '`scripts/checks/roomba_nav_check.sh`, which launches',
        '`ral_bringup/roomba_nav.launch.py`, localizes the robot against the',
        'committed static map and sends one `NavigateToPose` goal in room 3 that',
        'is reachable only through wall B\'s 0.70 m doorway. Every duration is sim',
        'time off `/clock`. This is the MVP plan\'s Phase 2 scope: no fault can be',
        'injected yet, and no scenario runner or dashboard exists.',
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
        f'| Robot | `{MODEL}` |',
        '| Map | `ral_bringup/maps/apartment.yaml` |',
        f'| `sha256(apartment.pgm)` | `{pgm_sha}` |',
        f'| `world_to_map` | x = {wtm[0]}, y = {wtm[1]}, yaw = {wtm[2]} |',
        f'| Alignment captured | {alignment_doc.get("captured_utc", "unknown")} |',
        f'| Alignment `git_sha` | `{alignment_doc.get("git_sha", "unknown")}` |',
        '| `inflation_radius` in force | '
        + ', '.join(f'{name} {value} m' for name, value in inflation.items()) + ' |',
        f'| Initial pose (world -> map) | {INITIAL_POSE_WORLD} -> '
        f'({initial_map[0]:.3f}, {initial_map[1]:.3f}, {initial_map[2]:.3f}) |',
        f'| Goal (world -> map) | {GOAL_WORLD} -> '
        f'({goal_map[0]:.3f}, {goal_map[1]:.3f}, {goal_map[2]:.3f}) |',
        f'| Sim time elapsed | {sim_duration:.2f} s |',
        f'| Real-time factor (`/world/{WORLD}/stats`) | {rtf} |',
        f'| Image | `{args.image_ref}` |',
        '',
        '## What these checks establish',
        '',
        '- The robot localizes against a map built by `slam_toolbox` from its own',
        '  `/scan`, not against world geometry, and against the alignment measured',
        '  for that exact grid — check 1 compares the loaded `.pgm`\'s sha256 with',
        '  the one sealed into `apartment_frames.yaml`, so a new map beside an old',
        '  alignment fails rather than reporting a frame offset as drift. Check 6',
        '  measures the stationary residual, which is map error plus alignment',
        '  error — the floor under every estimate-error number in this project.',
        '- Nav2 planned and drove a real mission: the goal was accepted and',
        '  reported `SUCCEEDED` (check 7), ground truth confirms arrival',
        '  independently of what AMCL believed (check 8), and truth shows the body',
        '  physically passing through the 0.70 m doorway (check 9).',
        '- The estimate is an estimate: AMCL\'s pose differs from truth at every',
        '  sample, but stays localized (check 11). Equality would mean truth had',
        '  leaked into localization.',
        '- `/scan` has exactly one publisher, `scan_fault_relay`, and its output is',
        '  byte-identical to `/scan_raw` (checks 2, 3). Phase 4.2 injects sensor',
        '  degradation at that node, so every lidar consumer — AMCL and both',
        '  costmaps — is downstream of the fault by construction.',
        '- No navigation component subscribes to `/ground_truth/odom` (check 4),',
        '  and the complete TF frame set is asserted (check 5), so truth bridged',
        '  onto `/tf` under any name — including the simulator\'s own',
        '  `<world> -> <model>` names — fails the check.',
        '- The commands the plant received reached the envelope Nav2 is',
        '  configured for and did not exceed it (check 12, two-sided on linear',
        '  speed). The lower bound is the load-bearing half: `velocity_smoother`',
        '  publishes `/cmd_vel`, and left at its shipped TurtleBot3 `0.26 m/s` it',
        '  would clamp DWB while still sitting inside the plant\'s `0.46 m/s`.',
        '  Reaching the configured maximum is what shows the override is in force',
        '  at runtime, not merely present in the YAML.',
        '- The configuration is additionally enforced without a simulator by',
        '  `ral_bringup/test/test_nav2_contract.py`, which runs in every',
        '  `colcon test`. It also covers `behavior_server`, whose recovery',
        '  rotations are published on `cmd_vel` **unremapped** by Nav2\'s',
        '  `navigation_launch.py` and therefore bypass the velocity smoother:',
        '  its shipped `3.2 rad/s²` exceeds this plant\'s `2.5 rad/s²`, so it is',
        '  overridden too.',
        '- The map itself was accepted by `scripts/checks/map_sanity.py`, which',
        '  checks it against the world SDF rather than by eye.',
        '',
        '## Known limitations',
        '',
        '- **The map carries real SLAM error by design.** It was built from one',
        '  mapping drive, not rasterised from `ral_apartment.sdf`. Walls are',
        '  slightly thick and slightly skewed; that error is part of every result',
        '  here and is intentional. The drive was scripted',
        '  (`scripts/map_drive.py`) and steered from ground truth as the',
        '  *operator\'s* view; `slam_toolbox` consumed only `/scan` and',
        '  `odom -> base_link`.',
        '- **The first mapping run of this world produced an unusable map** — a',
        '  ghost copy of room 1 offset by −7.95 m, from loop closure aliasing',
        '  across three near-identical 4 m rooms. `config/slam_roomba.yaml`',
        '  narrows the loop search window below the room pitch; see',
        '  `ral_bringup/maps/README.md`.',
        '- **Check 6 is a floor, not a localization score.** The stationary',
        '  residual mixes map error, the `world -> map` alignment measurement and',
        '  AMCL\'s own error, and nothing in this phase separates them.',
        '- **AMCL\'s particle cloud is not displayed.** It is a',
        '  `nav2_msgs/msg/ParticleCloud` and RViz runs on the host, which has no',
        '  `nav2_msgs`. `/amcl_pose` with its covariance is shown instead.',
        '- **The `map -> world` transform is display only.** It carries a fixed,',
        '  measured frame alignment so RViz can draw truth on the map; no',
        '  navigation component reads it, and Phase 3\'s metrics must read',
        '  `maps/apartment_frames.yaml` directly rather than listening to it.',
        '- **No fault is exercised.** Scan degradation, obstacle spawning,',
        '  communication loss and kidnapping are Phase 4. Nothing here shows how',
        '  the stack behaves when a subsystem is broken.',
        '- **One robot, one world, one goal.** The car-like platform is Phase 5,',
        '  and the MVP\'s "Roomba passes, car does not" scenario depends on the',
        '  same 0.70 m doorway measured in check 9.',
        '',
    ]
    doc_path = os.path.join(args.repo_root, 'docs', 'nav-closed-loop.md')
    with open(doc_path, 'w') as handle:
        handle.write('\n'.join(doc))
    print(f'--- wrote {doc_path}', flush=True)
    print(f'--- launch log: {log_path}', flush=True)

    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
