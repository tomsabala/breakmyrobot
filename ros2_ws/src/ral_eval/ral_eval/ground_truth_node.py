"""Republish simulator ground truth as an evaluation-only Odometry stream.

This topic exists for **evaluation and display only**. It is simulator ground
truth: it is never an input to localization, planning or control, and this node
broadcasts **no TF**. There is no TransformBroadcaster anywhere in this file,
and that absence is the mechanism - a component that wants truth has to ask for
it by name on /ground_truth/odom, which is auditable, rather than silently
receiving it through the TF tree.

Source: the gz PosePublisher on ral_roomba, bridged to /ground_truth/tf as a
tf2_msgs/msg/TFMessage (gz.msgs.Pose_V). The gz world name arrives as
header.frame_id and the model name as child_frame_id.
"""

import math

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

# Output frames. Deliberately NOT map/base_link:
#   `world` - the `map` frame does not exist yet and, once Phase 2.1a measures
#       the SLAM-map-to-gz-world alignment, will not coincide with the gz world
#       origin. Labelling truth `map` would fabricate that alignment.
#   `base_link_truth` - exists in no TF tree by design, so a TF listener pointed
#       at it fails loudly instead of quietly resolving to the robot's own
#       estimated `base_link`.
TRUTH_FRAME = 'world'
TRUTH_CHILD_FRAME = 'base_link_truth'


def _yaw_from_quaternion(q: Quaternion) -> float:
    """Yaw about +z, in [-pi, pi]."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def truth_odometry(msg: TFMessage, world_frame: str, model_frame: str,
                   prev: 'tuple[float, float, float, float] | None'
                   ) -> 'tuple[Odometry, tuple[float, float, float, float]] | None':
    """Convert one ground-truth TFMessage into an Odometry sample.

    `prev` is (x, y, yaw, t_sec) from the previously accepted message, or None
    for the first one. Returns (odometry, new_prev), or None when the message
    carries no transform matching world_frame -> model_frame. A TFMessage may
    carry several transforms, so the match is by name; taking transforms[0]
    would publish whatever happened to be first.

    Kept free of node state so it is directly testable without a ROS graph.
    """
    selected = None
    for transform in msg.transforms:
        if (transform.header.frame_id == world_frame
                and transform.child_frame_id == model_frame):
            selected = transform
            break

    if selected is None:
        return None

    odom = Odometry()
    # Stamp copied verbatim: this sample describes the simulator's state at the
    # instant the simulator stamped it, not at the instant this node ran.
    odom.header.stamp = selected.header.stamp
    odom.header.frame_id = TRUTH_FRAME
    odom.child_frame_id = TRUTH_CHILD_FRAME

    odom.pose.pose.position.x = selected.transform.translation.x
    odom.pose.pose.position.y = selected.transform.translation.y
    odom.pose.pose.position.z = selected.transform.translation.z
    odom.pose.pose.orientation = selected.transform.rotation

    x = selected.transform.translation.x
    y = selected.transform.translation.y
    yaw = _yaw_from_quaternion(selected.transform.rotation)
    t_sec = selected.header.stamp.sec + selected.header.stamp.nanosec * 1e-9

    # Twist is DERIVED here by finite difference between consecutive truth
    # poses. The simulator does not report a velocity on this path, so this is
    # the only honest way to produce one - and it is differentiated truth, not a
    # simulator-reported measurement. Phase 4.3 compares commanded /cmd_vel
    # against this achieved velocity, so it must exist and must not be invented.
    if prev is not None:
        prev_x, prev_y, prev_yaw, prev_t = prev
        dt = t_sec - prev_t
        if dt > 0.0:
            dx = x - prev_x
            dy = y - prev_y
            # Signed body-frame speed: project the displacement on the previous
            # heading so reversing reads negative instead of positive.
            projection = math.cos(prev_yaw) * dx + math.sin(prev_yaw) * dy
            speed = math.hypot(dx, dy) / dt
            odom.twist.twist.linear.x = math.copysign(speed, projection) if projection != 0.0 else 0.0
            odom.twist.twist.angular.z = _wrap_to_pi(yaw - prev_yaw) / dt

    # Covariances stay all-zero. Ground truth carries no uncertainty, so there is
    # no covariance to report; these zeros must never be rendered as a
    # confidence figure (ENGINEERING.md section 4) - a zero-covariance display
    # would read as "perfectly certain estimate", which is a different claim.

    return odom, (x, y, yaw, t_sec)


class GroundTruthNode(Node):
    """Subscribes bridged truth poses, publishes truth odometry. No TF."""

    def __init__(self) -> None:
        super().__init__('ground_truth_node')

        # PosePublisher stamps the gz world name as frame_id and the model name
        # as child_frame_id; both must match the world SDF and the spawned model.
        self.declare_parameter('world_frame', 'ral_apartment')
        self.declare_parameter('model_frame', 'ral_roomba')

        self._world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self._model_frame = self.get_parameter('model_frame').get_parameter_value().string_value
        self._prev: 'tuple[float, float, float, float] | None' = None

        self._pub = self.create_publisher(Odometry, '/ground_truth/odom', 10)
        self._sub = self.create_subscription(
            TFMessage, '/ground_truth/tf', self._on_truth_tf, 10)

        self.get_logger().info(
            f'ground truth: /ground_truth/tf [{self._world_frame} -> {self._model_frame}] '
            f'-> /ground_truth/odom [{TRUTH_FRAME} -> {TRUTH_CHILD_FRAME}], no TF broadcast')

    def _on_truth_tf(self, msg: TFMessage) -> None:
        result = truth_odometry(msg, self._world_frame, self._model_frame, self._prev)
        if result is None:
            return
        odom, self._prev = result
        self._pub.publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
