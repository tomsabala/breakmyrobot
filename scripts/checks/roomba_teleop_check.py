#!/usr/bin/env python3
"""
End-to-end proof of the Roomba platform and its teleop closed loop.

Brings the whole stack up with `ros2 launch ral_bringup robot_sim.launch.py`,
drives the robot from ROS, and asserts nine properties against the real
simulated subsystems. Writes docs/roomba-teleop-loop.md and exits non-zero if
any check fails.

Every duration below is measured in SIM time off /clock, never wall time: a
slow real-time factor must not change a result, only how long the run takes.
Wall-clock limits exist solely as deadlock guards.

Run through scripts/checks/roomba_teleop_check.sh (which sources the workspace
and supplies the container image reference).

Negative controls
-----------------
Green checks prove the stack works; they do not prove the checks can fail. Each
mutation below was applied to a working stack and the named check was observed
to go red. Repeat them after changing a check - a check that cannot fail is
decoration.

2026-10-22, image sha256:ce16e283:

* Add a second /tf row to the bridge config fed from /model/ral_roomba/pose
  (simulator truth). Observed: check 3 FAIL, `unexpected = ['ral_apartment',
  'ral_roomba']`, harness exit 1, and ral_bringup's test_bridge_contract fails
  two cases without starting a simulator. The PREVIOUS form of check 3, an
  absence test on `world`/`base_link_truth`, passed this mutation - truth
  arrives under the simulator's own frame names, not this node's output names.
* Add gz-sim-odometry-publisher-system to the model (odometry computed from the
  model's true world pose) and point the bridge's /odom row at it, i.e. feed the
  estimate from truth. Observed: check 7 FAIL, `advanced 0.0007 m while held`.
  The divergence term alone did NOT catch this (0.5 m of spawn offset reads as
  divergence); the requirement that dead reckoning keep integrating while the
  body is held is what fails.
"""

import argparse
import math
import os
import re
import sys
import time
from datetime import datetime, timezone
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
# Launching, tearing down and measuring the real-time factor are the same
# operations the two nav harnesses perform, with the same SIGINT-then-SIGKILL
# grace: one implementation in common.py rather than three that can drift.
from common import launch_stack, measure_rtf, teardown
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time as RclTime
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import Imu, JointState, LaserScan
import tf2_ros

WORLD = 'ral_apartment'
MODEL = 'ral_roomba'

# Geometry the checks assert against. The wall face is PARSED from the installed
# world SDF rather than hardcoded: a hardcoded copy silently re-aims check 5 at
# empty space when the wall moves, which is the failure mode the check exists to
# catch. The robot radius and lidar height come from the model SDF and are
# asserted against, not derived from, the same source they verify.
ROBOT_RADIUS = 0.1675         # base_link cylinder radius, models/ral_roomba/model.sdf
SPAWN_X = 0.5                 # <include> pose in the world; odom starts here
LIDAR_HEIGHT = 0.08           # rplidar_link z offset from base_link
EXPECTED_COLLISION = f'{MODEL}::base_link::base_collision'

# Every TF frame this stack may publish, and nothing else. Check 3 asserts the
# whole set: simulator truth entering /tf arrives under the gz PosePublisher's
# own names (world name -> model name), so an absence test on the truth node's
# output frames (`world`, `base_link_truth`) would not see it. Measured: with
# /model/ral_roomba/pose bridged to /tf the tree gains ral_apartment ->
# ral_roomba and an absence test still passes.
EXPECTED_TF_FRAMES = {
    'odom',
    'base_link',
    'rplidar_link',
    'left_wheel',
    'right_wheel',
    'caster_front',
    'caster_rear',
}

DRIVE_SPEED = 0.3             # m/s commanded into the wall
DRIVE_LIMIT_SIM = 25.0        # sim seconds allowed to reach the wall
PUSH_SIM = 2.0                # sim seconds of continued push after contact
SETTLE_SIM = 1.0              # sim seconds of zero command before rotating
ROTATE_RATE = 1.0             # rad/s commanded for the in-place rotation
ROTATE_SIM = 6.0              # sim seconds of rotation


# ral_eval.ground_truth_node has equivalent private helpers. The duplication is
# deliberate: a verifier that imports the code under test shares its bugs, and
# these two are four lines of standard quaternion math.
def yaw_of(odom: Odometry) -> float:
    q = odom.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def wall_a_face_x() -> float:
    """West face of wall_a_lower, read from the installed world SDF.

    The wall is a box centred on its model <pose>; its west face is
    centre_x - size_x / 2. Raises if the world stops containing that model, so
    moving the wall fails loudly instead of leaving check 5 aimed at a surface
    that is no longer there.
    """
    world_sdf = os.path.join(
        get_package_share_directory('ral_description'), 'worlds', f'{WORLD}.sdf')
    tree = ElementTree.parse(world_sdf)
    for model in tree.getroot().find('world').findall('model'):
        if model.get('name') != 'wall_a_lower':
            continue
        centre_x = float(model.find('pose').text.split()[0])
        size_x = float(model.find('link/collision/geometry/box/size').text.split()[0])
        return centre_x - size_x / 2.0
    raise RuntimeError(f'no model named wall_a_lower in {world_sdf}')


class Phase1Check(Node):

    def __init__(self) -> None:
        super().__init__(
            'roomba_teleop_check',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )

        self.odom = None
        self.truth = None
        self.scan = None
        self.imu = None
        self.joints = None
        self.contact_names = []      # collision names from the last non-empty /bumper
        self.bumper_seen = False
        # Peak |angular velocity z| the IMU reported while `record_imu` was set,
        # i.e. during the commanded in-place rotation only.
        self.record_imu = False
        self.imu_wz_peak = 0.0

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self._on_truth, 10)
        self.create_subscription(LaserScan, '/scan_raw', self._on_scan, 10)
        self.create_subscription(Contacts, '/bumper', self._on_bumper, 10)
        self.create_subscription(Imu, '/imu', self._on_imu, 10)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)

    # -- subscriptions ---------------------------------------------------------
    def _on_odom(self, msg):
        self.odom = msg

    def _on_truth(self, msg):
        self.truth = msg

    def _on_scan(self, msg):
        self.scan = msg

    def _on_imu(self, msg):
        self.imu = msg
        if self.record_imu:
            self.imu_wz_peak = max(self.imu_wz_peak, abs(msg.angular_velocity.z))

    def _on_joints(self, msg):
        self.joints = msg

    def _on_bumper(self, msg):
        self.bumper_seen = True
        if msg.contacts:
            names = []
            for contact in msg.contacts:
                names.extend([contact.collision1.name, contact.collision2.name])
            self.contact_names = names

    # -- time ------------------------------------------------------------------
    def sim_now(self) -> float:
        """Sim seconds off /clock. 0.0 until the bridge publishes."""
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_wall(self, seconds: float) -> None:
        """Spin for a wall-clock budget. Used only while waiting for bring-up."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def spin_sim(self, sim_seconds: float, cmd: Twist = None, until=None,
                 on_tick=None) -> bool:
        """Spin until `sim_seconds` of SIM time pass, or `until()` fires.

        Commands `cmd` at 20 Hz while spinning and calls `on_tick` once per
        iteration for callers that have to sample between messages. Returns
        True if `until` fired, else False. The wall-clock deadline (10x the sim
        budget plus 30 s) only guards against a stalled simulator.
        """
        start = self.sim_now()
        wall_deadline = time.monotonic() + sim_seconds * 10.0 + 30.0
        last_cmd = 0.0
        while True:
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.monotonic()
            if cmd is not None and now - last_cmd >= 0.05:
                self.cmd_pub.publish(cmd)
                last_cmd = now
            if on_tick is not None:
                on_tick()
            if until is not None and until():
                return True
            if self.sim_now() - start >= sim_seconds or now > wall_deadline:
                return False

    def stop(self) -> None:
        for _ in range(5):
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)


def twist(linear_x: float = 0.0, angular_z: float = 0.0) -> Twist:
    msg = Twist()
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    return msg


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--image-ref', default='unknown',
                        help='ral:dev digest or image id, collected on the host')
    parser.add_argument('--repo-root', default='/ws')
    args = parser.parse_args()

    log_path = os.path.join(args.repo_root, 'scripts', 'checks', 'roomba_teleop_launch.log')
    results = []   # (number, name, passed, detail)

    def record(number, name, passed, detail):
        results.append((number, name, passed, detail))
        print(f'{"PASS" if passed else "FAIL"}  {number}. {name}: {detail}', flush=True)

    rclpy.init()
    node = Phase1Check()

    launch, launch_log = launch_stack('robot_sim.launch.py', log_path,
                                      ('robot:=roomba',))
    with launch_log:

        sim_start = None
        rtf = 'not measured'
        try:
            # -- bring-up ------------------------------------------------------
            needed = {
                '/clock': lambda: node.sim_now() > 0.0,
                '/odom': lambda: node.odom is not None,
                '/ground_truth/odom': lambda: node.truth is not None,
                '/scan_raw': lambda: node.scan is not None,
                '/imu': lambda: node.imu is not None,
                '/joint_states': lambda: node.joints is not None,
            }
            deadline = time.monotonic() + 60.0
            # The full gated list, NOT None: a zero-iteration loop would
            # otherwise leave `missing` falsy and record a PASS nobody
            # observed.
            missing = list(needed)
            while time.monotonic() < deadline:
                node.spin_wall(0.5)
                missing = [t for t, ready in needed.items() if not ready()]
                if not missing:
                    break
            if missing:
                record(0, 'bring-up', False,
                       f'no messages on {", ".join(missing)} within 60 s wall')
                raise SystemExit(1)
            sim_start = node.sim_now()
            wheels_at_start = dict(zip(node.joints.name, node.joints.position))
            print(f'--- stack up at sim t={sim_start:.2f} s', flush=True)

            # -- 1. TF chain ---------------------------------------------------
            # Both hops are asserted, not just the composed result: the lidar
            # hop is static, the odom->base_link hop is the moving one DiffDrive
            # publishes, and a tree missing the second still answers the
            # composed lookup with whatever /tf_static alone can reach.
            node.spin_wall(3.0)   # let /tf_static and the first /tf arrive
            try:
                drive_tf = node.tf_buffer.lookup_transform('odom', 'base_link', RclTime())
                lidar_tf = node.tf_buffer.lookup_transform('base_link', 'rplidar_link', RclTime())
                offset = lidar_tf.transform.translation
                mounted = (abs(offset.z - LIDAR_HEIGHT) <= 0.005
                           and abs(offset.x) <= 0.005 and abs(offset.y) <= 0.005)
                # Both lookups above must have succeeded to get here; the
                # remaining question is where the lidar sits on the body.
                record(1, 'TF chain odom->base_link->rplidar_link', mounted,
                       f'odom->base_link x = {drive_tf.transform.translation.x:.4f} m, '
                       f'base_link->rplidar_link = ({offset.x:.4f}, {offset.y:.4f}, '
                       f'{offset.z:.4f}) m (expected (0, 0, {LIDAR_HEIGHT}) +/- 0.005)')
            except tf2_ros.TransformException as exc:
                record(1, 'TF chain odom->base_link->rplidar_link', False,
                       f'lookup failed: {type(exc).__name__}: {exc}')

            # -- 2. Scan frame and content -------------------------------------
            # A single valid beam would pass a presence test while 359 beams are
            # garbage, so require the overwhelming majority of the ring to be
            # finite and inside the datasheet range.
            scan = node.scan
            in_range = [r for r in scan.ranges if math.isfinite(r) and 0.15 <= r <= 12.0]
            ratio = len(in_range) / len(scan.ranges) if scan.ranges else 0.0
            record(2, 'scan frame and content',
                   scan.header.frame_id == 'rplidar_link' and len(scan.ranges) == 360
                   and ratio >= 0.9,
                   f"frame_id='{scan.header.frame_id}', {len(in_range)}/{len(scan.ranges)} "
                   f'finite ranges in [0.15, 12.0] (>= 90 % required), '
                   f'min={min(in_range):.3f} m, max={max(in_range):.3f} m'
                   if in_range else f"frame_id='{scan.header.frame_id}', no finite in-range beams")

            # -- 3. Truth is off the TF tree -----------------------------------
            # Asserts the WHOLE frame set. Truth bridged onto /tf arrives under
            # the gz PosePublisher's names (ral_apartment -> ral_roomba), so an
            # absence test on `world`/`base_link_truth` - this node's output
            # frames - passes while truth is in the tree. Measured.
            frames = node.tf_buffer.all_frames_as_string()
            children = set(re.findall(r'Frame ([^ ]+) exists', frames))
            parents = set(re.findall(r'exists with parent ([^.]+)\.', frames))
            present = children | parents
            unexpected = present - EXPECTED_TF_FRAMES
            missing_frames = EXPECTED_TF_FRAMES - present
            truth_resolves = True
            try:
                node.tf_buffer.lookup_transform('world', 'base_link_truth', RclTime())
            except tf2_ros.TransformException:
                # LookupException is the expected one; any other transform error
                # also means the frames are not usable, which is the same answer.
                truth_resolves = False
            verdict = ('RESOLVES (truth is in the tree)' if truth_resolves
                       else 'does not resolve')
            record(3, 'truth off the TF tree',
                   not unexpected and not missing_frames and not truth_resolves,
                   f'TF frames = {sorted(present)}; unexpected = {sorted(unexpected)}, '
                   f'missing = {sorted(missing_frames)}; '
                   f'world->base_link_truth {verdict}')

            # -- 4. Forward drive into the wall --------------------------------
            contact_truth_x = None
            contact_odom_x = None
            hit = node.spin_sim(
                DRIVE_LIMIT_SIM, cmd=twist(linear_x=DRIVE_SPEED),
                until=lambda: bool(node.contact_names))
            if hit:
                contact_truth_x = node.truth.pose.pose.position.x
                contact_odom_x = node.odom.pose.pose.position.x
                named = EXPECTED_COLLISION in node.contact_names
                record(4, 'bumper contact while driving', named,
                       f'contact after {node.sim_now() - sim_start:.2f} s sim, '
                       f'collisions = {node.contact_names}')
            else:
                record(4, 'bumper contact while driving', False,
                       f'no non-empty /bumper message within {DRIVE_LIMIT_SIM} s sim '
                       f'(topic seen: {node.bumper_seen}); truth x = '
                       f'{node.truth.pose.pose.position.x:.3f}')

            # -- 5. Contact at the wall surface, no penetration -----------------
            # Two-sided: the upper bound catches a robot that tunnelled into the
            # wall, the lower bound catches a "contact" reported somewhere the
            # wall is not.
            wall_face = wall_a_face_x()
            limit = wall_face - ROBOT_RADIUS + 0.02
            floor = wall_face - ROBOT_RADIUS - 0.05
            if contact_truth_x is None:
                record(5, 'contact at the wall surface', False, 'no contact to evaluate')
            else:
                record(5, 'contact at the wall surface', floor <= contact_truth_x <= limit,
                       f'truth x at contact = {contact_truth_x:.4f} m, expected in '
                       f'[{floor:.4f}, {limit:.4f}] (wall face {wall_face:.4f} parsed from '
                       f'the world SDF, robot radius {ROBOT_RADIUS})')

            # -- 6/7. Keep pushing: robot held, dead reckoning runs on ----------
            if contact_truth_x is None:
                record(6, 'robot stopped by the wall', False, 'no contact to evaluate')
                record(7, 'dead reckoning diverges from truth', False, 'no contact to evaluate')
            else:
                node.spin_sim(PUSH_SIM, cmd=twist(linear_x=DRIVE_SPEED))
                pushed_truth_x = node.truth.pose.pose.position.x
                pushed_odom_x = node.odom.pose.pose.position.x
                moved = abs(pushed_truth_x - contact_truth_x)
                record(6, 'robot stopped by the wall', moved < 0.01,
                       f'truth x moved {moved:.4f} m over {PUSH_SIM} s sim while still '
                       f'commanding {DRIVE_SPEED} m/s')
                # Divergence alone is satisfied by a frozen or dead /odom, so
                # also require the estimate to have kept integrating forward
                # while truth was held: that is what makes it dead reckoning
                # rather than an absent signal.
                divergence = abs(pushed_odom_x - (pushed_truth_x - SPAWN_X))
                odom_advanced = pushed_odom_x - contact_odom_x
                record(7, 'dead reckoning diverges from truth',
                       divergence > 0.02 and odom_advanced > 0.05,
                       f'odom x = {pushed_odom_x:.4f} (advanced {odom_advanced:.4f} m while '
                       f'held), truth x - spawn = {pushed_truth_x - SPAWN_X:.4f}, '
                       f'divergence = {divergence:.4f} m (wheels slip while the body is held)')

            # -- 8. Rotate in place ---------------------------------------------
            node.stop()
            node.spin_sim(SETTLE_SIM, cmd=twist())
            start_truth = node.truth
            x0 = start_truth.pose.pose.position.x
            y0 = start_truth.pose.pose.position.y
            yaw_prev = yaw_of(start_truth)
            yaw_total = 0.0

            def accumulate_yaw():
                # Summing wrapped increments, not end minus start: the robot
                # turns more than one revolution, which a single difference
                # cannot express.
                nonlocal yaw_prev, yaw_total
                yaw_now = yaw_of(node.truth)
                yaw_total += wrap_to_pi(yaw_now - yaw_prev)
                yaw_prev = yaw_now

            node.record_imu = True   # IMU evidence for check 9 comes from here
            node.spin_sim(ROTATE_SIM, cmd=twist(angular_z=ROTATE_RATE),
                          on_tick=accumulate_yaw)
            node.record_imu = False
            node.stop()

            drift = math.hypot(node.truth.pose.pose.position.x - x0,
                               node.truth.pose.pose.position.y - y0)
            record(8, 'rotate in place', drift < 0.02 and abs(yaw_total) > 3.0,
                   f'truth translation {drift:.4f} m (< 0.02) while accumulating '
                   f'{yaw_total:.3f} rad of yaw (> 3.0) over {ROTATE_SIM} s sim')

            # -- 9. IMU and joint states reach ROS with real content -----------
            # Presence alone proves nothing: the IMU must have measured the
            # rotation the robot actually performed, and the wheel joints must
            # have turned by the amount the drive required.
            imu_frame = node.imu.header.frame_id
            wheels_now = dict(zip(node.joints.name, node.joints.position))
            wheel_names = ['left_wheel_joint', 'right_wheel_joint']
            have_wheels = all(name in wheels_now and name in wheels_at_start
                              for name in wheel_names)
            turned = ({name: wheels_now[name] - wheels_at_start[name] for name in wheel_names}
                      if have_wheels else {})
            # 3.28 m of approach at wheel radius 0.036 m is ~91 rad of rotation;
            # require at least 10 rad so a stuck or synthetic stream fails.
            wheels_turned = have_wheels and all(abs(delta) > 10.0 for delta in turned.values())
            imu_ok = (imu_frame == 'base_link'
                      and abs(node.imu_wz_peak - ROTATE_RATE) < 0.15)
            record(9, 'IMU and joint states carry real content', imu_ok and wheels_turned,
                   f"imu frame_id='{imu_frame}', peak angular rate about z during "
                   f'the commanded '
                   f'{ROTATE_RATE} rad/s rotation = {node.imu_wz_peak:.3f} rad/s; wheel joint '
                   + ', '.join(f'{name} turned {turned.get(name, float("nan")):.1f} rad'
                               for name in wheel_names))

            rtf = measure_rtf(WORLD)

        except SystemExit:
            # A bring-up failure already recorded itself; fall through so the
            # report is still written and the teardown below still runs.
            pass
        finally:
            sim_end = node.sim_now()
            node.destroy_node()
            rclpy.shutdown()
            teardown(launch)

    sim_duration = (sim_end - sim_start) if sim_start is not None else 0.0
    # The count matters as much as the verdicts: a run that recorded only
    # three checks and passed all three is not a pass. 10 is the number of
    # distinct check numbers this file records (0 through 9), the same guard
    # both nav harnesses carry.
    passed = all(result[2] for result in results) and len(results) == 10

    doc = [
        '# Roomba platform and teleop closed loop — measured proof',
        '',
        f'Measured {datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")} by',
        '`scripts/checks/roomba_teleop_check.sh`, which launches',
        '`ral_bringup/robot_sim.launch.py` and drives the robot from ROS. Every',
        'duration is sim time off `/clock`. This is the MVP plan\'s Phase 1 scope:',
        'the robot, the world, the bridge and the teleop loop, with no map,',
        'localization, planner or fault injection yet.',
        '',
        f'## Result: {"all checks passed" if passed else "FAILURES PRESENT"}',
        '',
        '| # | Check | Result | Measured |',
        '| --- | --- | --- | --- |',
    ]
    for number, name, ok, detail in results:
        # A pipe inside a cell would split the markdown table.
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
        f'| Sim time elapsed | {sim_duration:.2f} s |',
        f'| Real-time factor (`/world/{WORLD}/stats`) | {rtf} |',
        f'| Image | `{args.image_ref}` |',
        '',
        '## What these checks establish',
        '',
        '- The robot is driven only through `/cmd_vel` -> `DiffDrive`; nothing',
        '  teleports it and no path is scripted.',
        '- `/odom` is integrated from wheel-joint rotation, so it diverges from',
        '  truth exactly when the wheels slip (check 7). Equal values would mean',
        '  simulator truth had leaked into the estimate.',
        '- Ground truth reaches ROS on `/ground_truth/odom` only. Check 3',
        '  asserts the complete TF frame set, so truth bridged onto `/tf` under',
        '  any frame name - including the simulator\'s own `<world> -> <model>`',
        '  names - shows up as an unexpected frame and fails the check.',
        '- The bumper is a real contact sensor on `base_collision` (check 4), and',
        '  the wall stops the robot through physics, not a script (checks 5, 6).',
        '- The IMU measured the rotation the robot actually performed and the',
        '  wheel joints turned by the amount the drive required (check 9), so',
        '  those streams carry sensor content, not just a live topic name.',
        '- The checks themselves have been exercised against deliberately',
        '  broken stacks; the mutations, the dates and the checks that went red',
        '  are recorded in the "Negative controls" section of',
        '  `scripts/checks/roomba_teleop_check.py`. The bridge rule is also',
        '  enforced without a simulator by `ral_bringup`\'s',
        '  `test/test_bridge_contract.py`, which runs in every `colcon test`.',
        '',
        '## Known plant properties and limits',
        '',
        '- `DiffDrive` has no command timeout ON THIS PLATFORM: the last',
        '  `/cmd_vel` is held indefinitely. A Phase 4 communication fault',
        '  therefore produces a robot that keeps driving on its last command,',
        '  not one that stops. That is the modelled behavior, and no watchdog',
        '  exists to change it. The car is different: ArduRover carries its own',
        '  failsafes and stops a guided vehicle after 3 s without a setpoint.',
        '- `/ground_truth/odom.twist` is a finite difference of truth poses, not',
        '  a simulator-reported velocity: forward and lateral components are',
        '  resolved on the previous heading, and a sample with no valid',
        '  difference is marked `twist.covariance[0] = -1`, never published as a',
        '  zero velocity.',
        '- The IMU is modelled without noise or bias, so its stream is cleaner',
        '  than real hardware. The lidar carries sigma = 0.015 m.',
        '',
        '## Not covered here',
        '',
        'This harness runs the bare platform: no map, localization, planner or',
        'controller is started here, and no fault is injected here. Both exist',
        'now and are exercised elsewhere - the navigation stack by',
        '`scripts/checks/roomba_nav_check.py`, the five faults this platform',
        'supports by the',
        '`ral_eval` scenario runs under `runs/`. Nothing in this stack consumes',
        '`/imu` or `/joint_states`: check 9 reads them directly, and no state',
        'estimator fuses them.',
        '',
    ]
    doc_path = os.path.join(args.repo_root, 'docs', 'roomba-teleop-loop.md')
    with open(doc_path, 'w') as handle:
        handle.write('\n'.join(doc))
    print(f'--- wrote {doc_path}', flush=True)
    print(f'--- launch log: {log_path}', flush=True)

    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
