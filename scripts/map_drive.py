#!/usr/bin/env python3
"""
Scripted OPERATOR drive for the mapping session. Not autonomy.

ral_bringup/map_build.launch.py expects a human to drive the robot with
`ros2 run teleop_twist_keyboard teleop_twist_keyboard` while slam_toolbox
builds the map. This script is the reproducible stand-in for that human: it
drives the same `/cmd_vel` topic a keyboard would, along a fixed route that
satisfies the coverage requirements in ral_bringup/maps/README.md (all three
rooms, both doorways twice in each direction, a circuit of each furniture
item, and a return to the start so the loop closes).

What it uses, and why that is honest
------------------------------------
It steers from `/ground_truth/odom`. That is the OPERATOR's view - the
equivalent of a human watching the robot in RViz and the simulator - and it
is used for exactly one thing: deciding which way to push the joystick. It
is not localization and it is not part of the robot's stack:

  * slam_toolbox builds the map from `/scan` and `odom -> base_link` only.
    Nothing it consumes comes from here.
  * This script publishes `/cmd_vel` and nothing else. It broadcasts no TF,
    publishes no pose, and no node in the mapped stack subscribes to it.
  * The resulting map therefore carries the SLAM error a real mapping drive
    produces. A perfectly-driven route does not make the map perfect.

Driving a mapping run by hand with perfect knowledge of where the robot is
is what a human operator does too; the alternative - a robot that navigates
itself while mapping - would require the localization this phase is building
the map FOR.

Speeds are held at or below 0.18 m/s and 0.6 rad/s. slam_toolbox is
configured with minimum_travel_distance 0.2, so scan registration happens
every 0.2 m; dragging a wall faster than that smears it.

Usage (with map_build.launch.py already running, after it has logged
`world_to_map captured`):

    scripts/dev.sh bash -lc 'source /ws/ros2_ws/install/setup.bash && \
      python3 /ws/scripts/map_drive.py'
"""

import math
import os
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

# The alignment file capture_world_to_map writes. Driving before it exists
# invalidates the capture (the node exits 2), so the drive waits for it.
FRAMES_PATH = '/ws/ros2_ws/src/ral_bringup/maps/apartment_frames.yaml'

MAX_LINEAR = 0.18        # m/s, well under the plant's 0.46
MAX_ANGULAR = 0.6        # rad/s, well under the plant's 1.9
DOORWAY_LINEAR = 0.12    # m/s while inside a 0.70/0.90 m gap
HEADING_GAIN = 1.6
TURN_FIRST = 0.35        # rad of heading error above which we rotate in place
ARRIVE_TOL = 0.14        # m
ARRIVE_TOL_TIGHT = 0.08  # m, for the colinear doorway approach points

# World-frame route. Geometry from ral_description/worlds/ral_apartment.sdf:
#   rooms split by wall A at x = 4.0 (gap y 3.55..4.45) and wall B at x = 8.0
#   (gap y 2.65..3.35); furniture_a (2.0, 5.5), furniture_b (6.0, 1.5),
#   furniture_c (9.5, 2.0); outer walls at x = 0/12, y = 0/8.
# Doorway crossings are colinear triples so the robot enters a 0.70 m gap
# already aligned instead of turning inside it.
ROUTE = [
    # -- room 1, south and east edges ------------------------------------
    (3.2, 0.7), (3.2, 2.0),
    # -- doorway A, pass 1 eastbound -------------------------------------
    (3.2, 4.0), (3.5, 4.0), (4.5, 4.0), (4.8, 4.0),
    # -- doorway A, pass 2 westbound -------------------------------------
    (4.5, 4.0), (3.5, 4.0), (3.2, 4.0),
    # -- room 1, north, and a circuit of furniture_a ---------------------
    (3.2, 7.2), (0.8, 7.2), (0.8, 6.3),
    (1.1, 4.7), (2.9, 4.7), (2.9, 6.3), (1.1, 6.3), (1.1, 4.7),
    (0.8, 3.0), (0.8, 1.0),
    # -- back to doorway A, pass 3 eastbound -----------------------------
    (3.2, 2.0), (3.2, 4.0), (3.5, 4.0), (4.5, 4.0), (4.8, 4.0),
    # -- room 2, north and east ------------------------------------------
    (4.8, 6.5), (7.2, 6.5), (7.2, 4.5),
    # -- circuit of furniture_b -------------------------------------------
    (7.0, 0.9), (5.0, 0.9), (5.0, 2.2), (7.0, 2.2),
    # -- doorway B, pass 1 eastbound --------------------------------------
    (7.0, 3.0), (7.6, 3.0), (8.4, 3.0), (9.0, 3.0),
    # -- doorway B, pass 2 westbound --------------------------------------
    (8.4, 3.0), (7.6, 3.0), (7.0, 3.0),
    # -- doorway B, pass 3 eastbound --------------------------------------
    (7.6, 3.0), (8.4, 3.0), (9.0, 3.0),
    # -- room 3 perimeter and a circuit of furniture_c ---------------------
    (8.7, 6.0), (8.7, 7.2), (11.2, 7.2), (11.2, 1.0), (8.7, 1.0),
    (8.6, 1.3), (10.7, 1.3), (10.7, 2.7), (8.6, 2.7),
    # -- doorway B, pass 4 westbound ---------------------------------------
    (8.6, 3.0), (8.4, 3.0), (7.6, 3.0), (7.0, 3.0),
    # -- back across room 2 and doorway A, pass 4 westbound ----------------
    (6.0, 4.0), (4.8, 4.0), (4.5, 4.0), (3.5, 4.0), (3.2, 4.0),
    # -- home --------------------------------------------------------------
    (3.2, 1.5), (0.8, 0.8), (0.5, 0.5),
]

# x windows in which the robot is inside a doorway and must go slowly.
DOORWAYS = ((4.0, 0.9), (8.0, 0.9))


def _yaw_of(odom: Odometry) -> float:
    q = odom.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class OperatorDrive(Node):
    """Publishes /cmd_vel along a fixed route, steering off truth."""

    def __init__(self) -> None:
        super().__init__(
            'map_drive_operator',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.truth = None
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self._on_truth, 10)

    def _on_truth(self, msg: Odometry) -> None:
        self.truth = msg

    def sim_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def pose(self):
        return (self.truth.pose.pose.position.x,
                self.truth.pose.pose.position.y,
                _yaw_of(self.truth))

    def publish(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def stop(self) -> None:
        for _ in range(10):
            self.publish(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)


def in_doorway(x: float) -> bool:
    return any(abs(x - centre) <= half for centre, half in DOORWAYS)


def drive_to(node: OperatorDrive, target, tol: float, sim_budget: float) -> bool:
    """Steer to one waypoint. Returns True on arrival, False on timeout."""
    tx, ty = target
    start_sim = node.sim_now()
    wall_deadline = time.monotonic() + sim_budget * 10.0 + 30.0
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
        # Checked before the truth guard, not after it: if /ground_truth/odom
        # stops - gz gone, bridge restarted - `continue` would otherwise skip
        # the deadline below and spin here silently for the rest of the
        # session instead of reporting the waypoint as missed.
        if node.sim_now() - start_sim > sim_budget or time.monotonic() > wall_deadline:
            return False
        if node.truth is None:
            continue
        x, y, yaw = node.pose()
        dx, dy = tx - x, ty - y
        distance = math.hypot(dx, dy)
        if distance <= tol:
            return True
        heading_error = _wrap_to_pi(math.atan2(dy, dx) - yaw)
        angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, HEADING_GAIN * heading_error))
        if abs(heading_error) > TURN_FIRST:
            linear = 0.0          # rotate in place first; a wide arc clips walls
        else:
            cap = DOORWAY_LINEAR if in_doorway(x) else MAX_LINEAR
            # Slow down near the waypoint so the arrival test is not overshot
            # between two 20 Hz control ticks.
            linear = min(cap, max(0.05, 0.8 * distance))
        node.publish(linear, angular)
    return False


def main() -> int:
    rclpy.init()
    node = OperatorDrive()
    try:
        # The alignment capture samples truth at slam_toolbox's first /map and
        # requires the robot to have stayed still until then.
        deadline = time.monotonic() + 180.0
        while not os.path.exists(FRAMES_PATH) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not os.path.exists(FRAMES_PATH):
            print(f'FAIL: {FRAMES_PATH} never appeared; capture_world_to_map did not '
                  'complete, so the map would have no alignment. Not driving.', flush=True)
            return 2
        print(f'--- alignment captured, starting the drive at sim t={node.sim_now():.1f}',
              flush=True)

        failures = 0
        for index, target in enumerate(ROUTE):
            tol = ARRIVE_TOL_TIGHT if in_doorway(target[0]) else ARRIVE_TOL
            reached = drive_to(node, target, tol, sim_budget=90.0)
            x, y, _ = node.pose()
            status = 'ok ' if reached else 'MISS'
            print(f'{status} waypoint {index + 1}/{len(ROUTE)} {target} '
                  f'-> truth ({x:.2f}, {y:.2f}) at sim t={node.sim_now():.1f}', flush=True)
            if not reached:
                failures += 1
            # A brief stop at each waypoint: slam_toolbox registers a scan from
            # a stationary pose far more cleanly than from one taken mid-turn.
            node.stop()
        print(f'--- drive complete, {failures} waypoint(s) not reached', flush=True)
        return 1 if failures else 0
    finally:
        try:
            node.stop()
            node.destroy_node()
        except (KeyboardInterrupt, RuntimeError):
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
