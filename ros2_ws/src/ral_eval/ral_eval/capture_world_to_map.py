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
Measure the gz `world` -> SLAM `map` alignment for one specific map.

slam_toolbox registers its first scan at that scan's ODOMETRIC pose, so
`map` -> `odom` starts as identity: the `map` frame lands on the ODOM FRAME
ORIGIN, not on wherever the robot happens to be. The two coincide only while
the robot still sits where it spawned, which is also the only instant the
pose is knowable from simulator truth. This node captures there, and checks
that both halves of the coincidence actually hold.

This is evaluation-side use of truth, not navigation use: the number it
records is a fixed frame alignment, never a robot pose fed back into the
stack. ral_bringup's roomba_nav.launch.py inverts it once at launch for a
display-only TF; Phase 3's metrics apply it to map truth into `map` so that
truth and estimate can be differenced at all.

Three guards exist because all three failure modes are silent otherwise:

  * stationarity - if the robot is driven before the first /map arrives, the
    truth pose sampled here is no longer where `map` was defined, and every
    later estimate-error number inherits that offset as a constant bias. The
    node refuses to write and exits 2.
  * odom at its origin - stationarity alone does not establish that `map`
    and truth coincide, only that truth stopped changing. A robot parked
    away from the odom origin (slam_toolbox restarted on its own after a
    drive, or a staged bring-up) is perfectly stationary and still produces
    an alignment offset by the whole drive. /odom is the direct observation
    of that invariant, so it is checked directly. Exits 4.
  * timeout - no /map within `timeout_sim_s` of SIM time means slam_toolbox
    never produced a map (typically an unresolved TF chain). Exits 3 rather
    than waiting forever inside an operator session.

Exit codes: 0 captured, 2 robot moved before capture, 3 no map in time,
4 odom was not at its origin, so `map` is not where truth says it is.
"""

import math
import os
import subprocess
import sys
from datetime import datetime, timezone

from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import yaml

EXIT_OK = 0
EXIT_MOVED = 2
EXIT_NO_MAP = 3
EXIT_ODOM_OFF_ORIGIN = 4

HEADER_TEMPLATE = """\
# world -> map alignment for the `{map_name}` map. Measured, never assumed.
#
# world_to_map is the pose of the `map` frame ORIGIN expressed in the gz
# `world` frame. slam_toolbox places `map` on the ODOM frame origin, which
# for a robot still sitting where it spawned is also its true pose - both
# halves verified here before writing, from /ground_truth/odom and /odom.
#
# This file belongs to one specific {map_name}.pgm, and `map_sha256` below
# records which one. That field is added by scripts/checks/map_sanity.py
# when it accepts the saved grid; until then this pair is unsealed and
# ral_bringup/roomba_nav.launch.py will refuse to start.
# Procedure: ros2 launch ral_bringup map_build.launch.py
"""


def _yaw_from_quaternion(q) -> float:
    """Yaw about +z, in [-pi, pi]."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _git_sha(cwd: str) -> str:
    """HEAD of the repository holding the output file, or a stated unknown."""
    try:
        result = subprocess.run(
            ['git', 'rev-parse', 'HEAD'], cwd=cwd,
            capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired):
        return 'unknown (git unavailable)'
    if result.returncode != 0:
        return 'unknown (git unavailable)'
    return result.stdout.strip() or 'unknown (git unavailable)'


class CaptureWorldToMap(Node):
    """Samples truth at the first /map and writes the alignment file."""

    def __init__(self) -> None:
        super().__init__('capture_world_to_map')

        self.declare_parameter(
            'output_path', '/ws/ros2_ws/src/ral_bringup/maps/apartment_frames.yaml')
        self.declare_parameter('map_name', 'apartment')
        self.declare_parameter('world_name', 'ral_apartment')
        self.declare_parameter('stationary_xy_tol', 0.01)
        self.declare_parameter('stationary_yaw_tol', 0.01)
        self.declare_parameter('timeout_sim_s', 120.0)

        self._output_path = self.get_parameter('output_path').get_parameter_value().string_value
        self._map_name = self.get_parameter('map_name').get_parameter_value().string_value
        self._world_name = self.get_parameter('world_name').get_parameter_value().string_value
        self._xy_tol = self.get_parameter('stationary_xy_tol').get_parameter_value().double_value
        self._yaw_tol = self.get_parameter('stationary_yaw_tol').get_parameter_value().double_value
        self._timeout = self.get_parameter('timeout_sim_s').get_parameter_value().double_value

        self._first_pose = None       # (x, y, yaw) of the first truth sample
        self._odom = None             # latest dead-reckoned pose, (x, y, yaw)
        self._max_xy = 0.0            # largest deviation from it, metres
        self._max_yaw = 0.0           # largest deviation from it, radians
        self._sim_start = None        # sim time of the first non-zero /clock
        self.exit_code = None         # set once; main() stops spinning on it

        self.create_subscription(Odometry, '/ground_truth/odom', self._on_truth, 10)
        # `map` lands on the ODOM frame origin, not on the robot. /odom is
        # the robot's pose relative to that origin, so it is the direct
        # observation of whether the two coincide.
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        # slam_toolbox latches its map: a late subscriber gets the last one
        # only with TRANSIENT_LOCAL durability. Without it this node can miss
        # the first publication entirely and time out on a healthy stack.
        self.create_subscription(
            OccupancyGrid, '/map', self._on_map,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(1.0, self._on_timer)

        self.get_logger().info(
            f'waiting for slam_toolbox\'s first /map to capture world->map for '
            f'`{self._map_name}`; the robot must stay still until then '
            f'-> {self._output_path}')

    def _on_truth(self, msg: Odometry) -> None:
        pose = (msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                _yaw_from_quaternion(msg.pose.pose.orientation))
        if self._first_pose is None:
            self._first_pose = pose
            return
        x0, y0, yaw0 = self._first_pose
        self._max_xy = max(self._max_xy, math.hypot(pose[0] - x0, pose[1] - y0))
        self._max_yaw = max(self._max_yaw, abs(_wrap_to_pi(pose[2] - yaw0)))

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = (msg.pose.pose.position.x,
                      msg.pose.pose.position.y,
                      _yaw_from_quaternion(msg.pose.pose.orientation))

    def _on_timer(self) -> None:
        if self.exit_code is not None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0:
            return
        if self._sim_start is None:
            self._sim_start = now
            return
        if now - self._sim_start >= self._timeout:
            self.get_logger().error(
                f'no /map within {self._timeout:.0f} s of sim time. slam_toolbox '
                'produced no map - check that its TF chain (map <- odom <- base_link '
                '<- rplidar_link) resolves and that /scan is being published. '
                'Nothing written.')
            self.exit_code = EXIT_NO_MAP

    def _on_map(self, msg: OccupancyGrid) -> None:
        if self.exit_code is not None:
            return
        if self._first_pose is None:
            # No truth yet: /map cannot be interpreted, keep waiting for the
            # next one rather than recording a pose we never observed.
            self.get_logger().warn('/map arrived before any /ground_truth/odom; waiting')
            return
        if self._max_xy > self._xy_tol or self._max_yaw > self._yaw_tol:
            self.get_logger().error(
                f'robot moved before the alignment could be captured: '
                f'{self._max_xy:.4f} m / {self._max_yaw:.4f} rad of deviation exceeds '
                f'the {self._xy_tol} m / {self._yaw_tol} rad tolerance. The pose where '
                '`map` was defined is no longer observable, so nothing was written. '
                'Restart map_build.launch.py and do not drive until this node logs '
                'the capture.')
            self.exit_code = EXIT_MOVED
            return
        if self._odom is None:
            self.get_logger().warn('/map arrived before any /odom; waiting')
            return
        odom_xy = math.hypot(self._odom[0], self._odom[1])
        odom_yaw = abs(_wrap_to_pi(self._odom[2]))
        if odom_xy > self._xy_tol or odom_yaw > self._yaw_tol:
            self.get_logger().error(
                f'the robot is {odom_xy:.4f} m / {odom_yaw:.4f} rad from the odom '
                f'frame origin, outside the {self._xy_tol} m / {self._yaw_tol} rad '
                'tolerance. slam_toolbox places `map` on the ODOM origin, so the truth '
                'pose sampled here is not where `map` is - it is offset by everything '
                'the robot drove since odometry started. Nothing was written. Restart '
                'the whole stack, not just slam_toolbox, so odometry and the map frame '
                'start together.')
            self.exit_code = EXIT_ODOM_OFF_ORIGIN
            return

        x, y, yaw = self._first_pose
        self._write(x, y, yaw, msg)
        self.get_logger().info(
            f'world_to_map captured: x={x:.4f} y={y:.4f} yaw={yaw:.4f} '
            f'-> {self._output_path}')
        self.exit_code = EXIT_OK

    def _write(self, x: float, y: float, yaw: float, grid: OccupancyGrid) -> None:
        directory = os.path.dirname(self._output_path)
        os.makedirs(directory, exist_ok=True)
        data = {
            'world_to_map': {'x': round(x, 6), 'y': round(y, 6), 'yaw': round(yaw, 6)},
            'map': self._map_name,
            'world': self._world_name,
            'captured_utc': datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
            'git_sha': _git_sha(directory),
            'source': '/ground_truth/odom, held stationary and at the odom origin '
                      "through slam_toolbox's first /map publication",
            'map_resolution_m': round(float(grid.info.resolution), 6),
            'stationary_max_xy_m': round(self._max_xy, 6),
            'stationary_max_yaw_rad': round(self._max_yaw, 6),
            'odom_offset_m': round(math.hypot(self._odom[0], self._odom[1]), 6),
            'odom_offset_rad': round(abs(_wrap_to_pi(self._odom[2])), 6),
        }
        header = HEADER_TEMPLATE.format(map_name=self._map_name)
        # Written through a temporary file: a half-written alignment that
        # still parses is the one form of this file that fails quietly.
        temporary = self._output_path + '.partial'
        with open(temporary, 'w') as handle:
            handle.write(header)
            yaml.safe_dump(data, handle, sort_keys=False, default_flow_style=False)
        os.replace(temporary, self._output_path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CaptureWorldToMap()
    code = EXIT_NO_MAP
    try:
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.exit_code is not None:
            code = node.exit_code
    except (KeyboardInterrupt, ExternalShutdownException):
        # Operator ended the mapping session before a map existed. Report it
        # as the no-map case rather than as a successful capture.
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
