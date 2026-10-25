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
The car's estimate-side frame authority, and the only measurer of the EKF origin.

ArduPilot sets its EKF origin at the autopilot's first GPS fix, so the offset
between the gz `world` frame and the ENU frame /mavros/local_position/* reports
in is a PER-RUN measurement, not a constant anyone can write down. This node
measures it once, while the vehicle is still standing at its spawn pose,
publishes it latched on /frame_alignment, and from then on broadcasts the
robot's own estimate as TF:

    map -> ekf_local     static, measured once (this node)
    ekf_local -> base_link   the estimate itself, on every estimate message

Nothing it publishes carries simulator truth. The node DOES subscribe to
/ground_truth/odom - that is how a frame offset is measured at all - and so it
superficially resembles the localization shortcut this project forbids. It is
not one: what is broadcast afterwards is the robot's own EKF3 estimate composed
with a fixed measured offset, exactly as roomba_nav.launch.py's display-only
map -> world transform carries a fixed alignment and no robot pose. Nothing
downstream can recover truth from either.

The measurement is REFUSED rather than fudged in four cases, each with its own
exit code, because scoring a run against an unmeasured frame offset would
publish that offset as localization error:

  2  the robot moved before the measurement could be made
  3  the POSITION residual against the predicted origin exceeds
     residual_limit_m
  4  no estimate message arrived at all within timeout_sim_s
  5  the YAW residual against the predicted origin exceeds
     residual_limit_rad

The prediction the residual is measured against: ArduPilotPlugin sends the
vehicle's position as NED relative to the gz world origin, SITL synthesises GPS
from it against --home, and EKF3 sets its origin at the first fix - so
world_to_ekf_local should equal the spawn POSE, heading included. That is a
prediction this node CHECKS, not an assumption it relies on; the measurement is
used whatever the prediction said, as long as both residuals are inside their
limits.

Both halves are checked because an unchecked yaw offset is not a small error:
it rotates the entire estimate track about the origin, so a position error
that grows with distance from it reads downstream as estimator drift. This
platform's known failure mode - an inverted magnetometer - is exactly a yaw
error, and it would leave the position residual untouched. MEASURED on the
committed baseline_car runs: 0.045 rad (2.6 deg), which is why the limit is
0.10 rad rather than something tighter.
"""

import math
import sys

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from ral_eval.frames import world_pose_to_map, wrap_to_pi
from ral_eval.robot_profile import load_profile
from ral_interfaces.msg import FrameAlignment
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

EXIT_OK = 0
EXIT_MOVED = 2
EXIT_RESIDUAL = 3
EXIT_NO_ESTIMATE = 4
EXIT_RESIDUAL_YAW = 5


def _yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def world_to_child(truth: 'tuple[float, float, float]',
                   estimate: 'tuple[float, float, float]') -> 'tuple[float, float, float]':
    """
    SE(2) pose of the estimate frame's origin in `world`, from one sample pair.

    The robot is at `truth` in `world` and reports being at `estimate` in its
    own frame, so the frame's origin sits where `truth` would be if the
    estimate were measured from it: compose truth with the inverse estimate.
    Pure, so test_frames-style arithmetic can exercise it without a simulator.
    """
    yaw = wrap_to_pi(truth[2] - estimate[2])
    cos_a = math.cos(yaw)
    sin_a = math.sin(yaw)
    return (truth[0] - (estimate[0] * cos_a - estimate[1] * sin_a),
            truth[1] - (estimate[0] * sin_a + estimate[1] * cos_a),
            yaw)


def compose(parent: 'tuple[float, float, float]',
            child: 'tuple[float, float, float]') -> 'tuple[float, float, float]':
    """Pose `child`, given in a frame whose own pose is `parent`, in that parent."""
    cos_p = math.cos(parent[2])
    sin_p = math.sin(parent[2])
    return (parent[0] + child[0] * cos_p - child[1] * sin_p,
            parent[1] + child[0] * sin_p + child[1] * cos_p,
            wrap_to_pi(parent[2] + child[2]))


class EkfLocalTf(Node):
    """Measures world -> ekf_local once, then broadcasts the estimate as TF."""

    def __init__(self) -> None:
        super().__init__('ekf_local_tf')

        self.declare_parameter('robot', 'car')
        self.declare_parameter('profile_path', '')
        # The committed world -> map alignment, resolved by the launch file
        # from the map this run uses. Passed in rather than read here so there
        # is exactly one place that refuses a stale map/alignment pair.
        self.declare_parameter('world_to_map_x', 0.0)
        self.declare_parameter('world_to_map_y', 0.0)
        self.declare_parameter('world_to_map_yaw', 0.0)
        self.declare_parameter('stationary_xy_tol', 0.01)
        self.declare_parameter('stationary_yaw_tol', 0.01)
        # The EKF origin is predicted to be the pose the body was SPAWNED at,
        # which is the scenario's declared initial pose and only defaults to
        # the profile's when a scenario gave none. Empty strings rather than
        # doubles because that is how the launch layer carries "not given" for
        # a pose; comparing against the profile default while the body stood
        # somewhere else refuses a healthy run (MEASURED: a 2.996 m residual
        # for a vehicle spawned on its scenario's start pose).
        self.declare_parameter('spawn_x', '')
        self.declare_parameter('spawn_y', '')
        self.declare_parameter('spawn_yaw', '')
        # 0.15 m is the limit beyond which the prediction is wrong in a way
        # that would show up in every est_error_* number as a constant bias.
        self.declare_parameter('residual_limit_m', 0.15)
        # The heading half of the same prediction. An unchecked yaw offset
        # rotates the whole estimate track about the origin, so it reads
        # downstream as drift that grows with distance rather than as the
        # frame error it is. 0.10 rad is roughly twice the 0.045 rad the
        # committed baseline_car runs measured.
        self.declare_parameter('residual_limit_rad', 0.10)
        self.declare_parameter('timeout_sim_s', 120.0)

        profile_path = self._string_param('profile_path')
        if not profile_path:
            raise ValueError(
                'profile_path is empty: the robot profile lives in '
                'ral_bringup/config/robots/<robot>.yaml and its path must be supplied '
                'by the launch file, because ral_eval does not depend on ral_bringup')
        self.profile = load_profile(profile_path)
        self.estimate_topic = self.profile['estimate']['topic']
        self.estimate_frame = self.profile['estimate']['frame']
        self.spawn = dict(self.profile['spawn'])
        for axis in ('x', 'y', 'yaw'):
            given = self._string_param(f'spawn_{axis}').strip()
            if given:
                self.spawn[axis] = float(given)

        self._xy_tol = self._double_param('stationary_xy_tol')
        self._yaw_tol = self._double_param('stationary_yaw_tol')
        self._residual_limit = self._double_param('residual_limit_m')
        self._residual_limit_rad = self._double_param('residual_limit_rad')
        self._timeout = self._double_param('timeout_sim_s')
        self._wtm = (self._double_param('world_to_map_x'),
                     self._double_param('world_to_map_y'),
                     self._double_param('world_to_map_yaw'))

        self._truth = None            # latest (x, y, yaw) in `world`
        self._first_truth = None      # the first one seen
        self._max_xy = 0.0
        self._max_yaw = 0.0
        self._alignment = None        # (x, y, yaw) of ekf_local's origin in `world`
        self._sim_start = None
        self.exit_code = None         # set once; main() stops spinning on it

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.alignment_pub = self.create_publisher(
            FrameAlignment, '/frame_alignment', latched)
        self.static_tf = StaticTransformBroadcaster(self)
        self.tf = TransformBroadcaster(self)

        self.create_subscription(Odometry, '/ground_truth/odom', self._on_truth, 10)
        # SensorDataQoS, not the default: MAVROS publishes local_position/* as
        # BEST_EFFORT sensor data, and a RELIABLE subscription to it matches no
        # publisher and receives NOTHING - silently, with the topic listed and
        # the node apparently healthy.
        self.create_subscription(Odometry, self.estimate_topic, self._on_estimate,
                                 qos_profile_sensor_data)
        self.create_timer(1.0, self._on_timer)

        self.get_logger().info(
            f'ekf_local_tf: measuring world -> {self.estimate_frame} from '
            f'/ground_truth/odom and {self.estimate_topic}, predicted at the spawn '
            f'pose ({self.spawn["x"]}, {self.spawn["y"]}, {self.spawn["yaw"]}) within '
            f'{self._residual_limit} m and {self._residual_limit_rad} rad')

    def _string_param(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _double_param(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    # -- subscriptions --------------------------------------------------------
    def _on_truth(self, msg: Odometry) -> None:
        pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                _yaw_of(msg.pose.pose.orientation))
        self._truth = pose
        if self._first_truth is None:
            self._first_truth = pose
            return
        x0, y0, yaw0 = self._first_truth
        self._max_xy = max(self._max_xy, math.hypot(pose[0] - x0, pose[1] - y0))
        self._max_yaw = max(self._max_yaw, abs(wrap_to_pi(pose[2] - yaw0)))

    def _on_estimate(self, msg: Odometry) -> None:
        if self.exit_code is not None:
            return
        estimate = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                    _yaw_of(msg.pose.pose.orientation))
        if self._alignment is None:
            if not self._measure(estimate):
                return
        self._broadcast_estimate(msg, estimate)

    def _measure(self, estimate: 'tuple[float, float, float]') -> bool:
        if self._truth is None:
            # No truth yet: an estimate alone says nothing about where the EKF
            # origin is. Wait rather than record an offset never observed.
            return False
        if self._max_xy > self._xy_tol or self._max_yaw > self._yaw_tol:
            self.get_logger().error(
                f'the vehicle moved {self._max_xy:.4f} m / {self._max_yaw:.4f} rad '
                f'before the EKF origin could be measured, outside the '
                f'{self._xy_tol} m / {self._yaw_tol} rad tolerance. The pose pair this '
                'measurement needs is no longer simultaneous, and an offset measured '
                'from a moving body would be published as localization error for the '
                'whole run. Nothing was published.')
            self.exit_code = EXIT_MOVED
            return False

        alignment = world_to_child(self._truth, estimate)
        residual = math.hypot(alignment[0] - float(self.spawn['x']),
                              alignment[1] - float(self.spawn['y']))
        if residual > self._residual_limit:
            self.get_logger().error(
                f'the measured {self.estimate_frame} origin is at '
                f'({alignment[0]:.4f}, {alignment[1]:.4f}) in `world`, '
                f'{residual:.4f} m from the spawn pose '
                f'({self.spawn["x"]}, {self.spawn["y"]}) it was predicted at - outside '
                f'the {self._residual_limit} m limit. Scoring a run against a frame '
                'offset this far from its prediction would publish that offset as '
                'localization error. Nothing was published.')
            self.exit_code = EXIT_RESIDUAL
            return False

        yaw_residual = abs(wrap_to_pi(alignment[2] - float(self.spawn['yaw'])))
        if yaw_residual > self._residual_limit_rad:
            self.get_logger().error(
                f'the measured {self.estimate_frame} origin has yaw '
                f'{alignment[2]:.4f} rad in `world`, {yaw_residual:.4f} rad from the '
                f'spawn yaw {self.spawn["yaw"]} it was predicted at - outside the '
                f'{self._residual_limit_rad} rad limit. An unchecked yaw offset '
                'rotates the whole estimate track about this origin, so it reads '
                'downstream as drift that grows with distance from it rather than as '
                'the frame error it is. Nothing was published.')
            self.exit_code = EXIT_RESIDUAL_YAW
            return False

        self._alignment = alignment
        self._publish_alignment(residual, yaw_residual)
        self._broadcast_static(alignment)
        self.get_logger().info(
            f'world -> {self.estimate_frame} measured: x={alignment[0]:.4f} '
            f'y={alignment[1]:.4f} yaw={alignment[2]:.4f}, residual {residual:.4f} m '
            f'/ {yaw_residual:.4f} rad against the spawn pose')
        return True

    # -- publication ----------------------------------------------------------
    def _publish_alignment(self, residual: float, yaw_residual: float) -> None:
        msg = FrameAlignment()
        msg.stamp = self.get_clock().now().to_msg()
        msg.parent_frame = 'world'
        msg.child_frame = self.estimate_frame
        msg.x, msg.y, msg.yaw = self._alignment
        msg.residual_m = residual
        msg.residual_yaw_rad = yaw_residual
        msg.source = (f'/ground_truth/odom and {self.estimate_topic}, both stationary '
                      'at the spawn pose')
        self.alignment_pub.publish(msg)

    def _broadcast_static(self, alignment: 'tuple[float, float, float]') -> None:
        """
        Broadcast map -> ekf_local: the measurement in the frame RViz uses.

        The measurement is in `world`; the committed world -> map alignment
        carries it the rest of the way, through the same conversion every truth
        sample takes, so the estimate chain and the map agree by construction.
        """
        x, y, yaw = world_pose_to_map(alignment[0], alignment[1], alignment[2],
                                      self._wtm)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = self.estimate_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        self.static_tf.sendTransform(transform)

    def _broadcast_estimate(self, msg: Odometry,
                            estimate: 'tuple[float, float, float]') -> None:
        """ekf_local -> base_link: the robot's own estimate, and nothing else."""
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.estimate_frame
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = estimate[0]
        transform.transform.translation.y = estimate[1]
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(transform)

    # -- the deadline ---------------------------------------------------------
    def _on_timer(self) -> None:
        if self.exit_code is not None or self._alignment is not None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0:
            return
        if self._sim_start is None:
            self._sim_start = now
            return
        if now - self._sim_start >= self._timeout:
            self.get_logger().error(
                f'no message on {self.estimate_topic} within {self._timeout:.0f} s of '
                'sim time, so the EKF origin was never measured. Check that MAVROS is '
                'connected to the autopilot (/mavros/state.connected) and that EKF3 '
                'has an origin. Nothing was published.')
            self.exit_code = EXIT_NO_ESTIMATE


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EkfLocalTf()
    code = EXIT_OK
    try:
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.exit_code is not None:
            code = node.exit_code
    except (KeyboardInterrupt, ExternalShutdownException):
        # The run ended; the alignment, if it was measured, is already latched.
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
