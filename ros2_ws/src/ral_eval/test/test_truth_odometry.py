"""Unit tests for ral_eval.ground_truth_node.truth_odometry.

Plain pytest: TFMessage objects are constructed directly, no ROS graph, no
node. A failure here means the truth stream is wrong regardless of how healthy
/ground_truth/odom looks on the wire.
"""

import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
import pytest
from tf2_msgs.msg import TFMessage

from ral_eval.ground_truth_node import (
    TRUTH_CHILD_FRAME,
    TRUTH_FRAME,
    truth_odometry,
)

WORLD = 'ral_apartment'
MODEL = 'ral_roomba'


def _stamped(frame_id, child_frame_id, x=0.0, y=0.0, yaw=0.0, t=0.0):
    transform = TransformStamped()
    transform.header.frame_id = frame_id
    transform.header.stamp = Time(sec=int(t), nanosec=int(round((t % 1.0) * 1e9)))
    transform.child_frame_id = child_frame_id
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.rotation.z = math.sin(yaw / 2.0)
    transform.transform.rotation.w = math.cos(yaw / 2.0)
    return transform


def _message(*transforms):
    msg = TFMessage()
    msg.transforms = list(transforms)
    return msg


def test_selects_matching_transform_not_the_first_one():
    """A Pose_V may carry several models; selection is by frame name."""
    msg = _message(
        _stamped(WORLD, 'other_model', x=9.0, y=9.0),
        _stamped(WORLD, MODEL, x=1.25, y=-0.5),
    )

    odom, _ = truth_odometry(msg, WORLD, MODEL, None)

    assert odom.pose.pose.position.x == pytest.approx(1.25)
    assert odom.pose.pose.position.y == pytest.approx(-0.5)


def test_no_matching_transform_returns_none():
    msg = _message(_stamped(WORLD, 'other_model', x=9.0))

    assert truth_odometry(msg, WORLD, MODEL, None) is None


def test_output_frames_and_stamp():
    """Truth never claims the map frame and never claims base_link."""
    msg = _message(_stamped(WORLD, MODEL, x=0.5, t=12.25))

    odom, _ = truth_odometry(msg, WORLD, MODEL, None)

    assert odom.header.frame_id == TRUTH_FRAME == 'world'
    assert odom.child_frame_id == TRUTH_CHILD_FRAME == 'base_link_truth'
    assert odom.header.stamp.sec == 12
    assert odom.header.stamp.nanosec == 250000000


def test_first_sample_has_zero_twist():
    msg = _message(_stamped(WORLD, MODEL, x=0.5, t=1.0))

    odom, prev = truth_odometry(msg, WORLD, MODEL, None)

    assert odom.twist.twist.linear.x == 0.0
    assert odom.twist.twist.angular.z == 0.0
    assert prev == pytest.approx((0.5, 0.0, 0.0, 1.0))


def test_twist_is_finite_difference_of_consecutive_truth_poses():
    first = _message(_stamped(WORLD, MODEL, x=0.0, y=0.0, yaw=0.0, t=1.0))
    _, prev = truth_odometry(first, WORLD, MODEL, None)

    second = _message(_stamped(WORLD, MODEL, x=0.02, y=0.0, yaw=0.05, t=1.1))
    odom, _ = truth_odometry(second, WORLD, MODEL, prev)

    assert odom.twist.twist.linear.x == pytest.approx(0.2, abs=1e-3)
    assert odom.twist.twist.angular.z == pytest.approx(0.5, abs=1e-3)


def test_reversing_gives_negative_linear_x():
    first = _message(_stamped(WORLD, MODEL, x=0.0, y=0.0, yaw=0.0, t=1.0))
    _, prev = truth_odometry(first, WORLD, MODEL, None)

    second = _message(_stamped(WORLD, MODEL, x=-0.02, y=0.0, yaw=0.0, t=1.1))
    odom, _ = truth_odometry(second, WORLD, MODEL, prev)

    assert odom.twist.twist.linear.x == pytest.approx(-0.2, abs=1e-3)


def test_yaw_wrap_does_not_produce_a_spurious_spin():
    """+3.10 rad to -3.10 rad is +0.0832 rad of rotation, not -6.2."""
    first = _message(_stamped(WORLD, MODEL, yaw=3.10, t=1.0))
    _, prev = truth_odometry(first, WORLD, MODEL, None)

    second = _message(_stamped(WORLD, MODEL, yaw=-3.10, t=1.1))
    odom, _ = truth_odometry(second, WORLD, MODEL, prev)

    assert odom.twist.twist.angular.z == pytest.approx(0.832, abs=1e-3)


def test_covariances_are_zero():
    """Ground truth carries no uncertainty; zeros must not be read as confidence."""
    msg = _message(_stamped(WORLD, MODEL, x=0.5))

    odom, _ = truth_odometry(msg, WORLD, MODEL, None)

    assert list(odom.pose.covariance) == [0.0] * 36
    assert list(odom.twist.covariance) == [0.0] * 36
