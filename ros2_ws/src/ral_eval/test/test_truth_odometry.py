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
Unit tests for ral_eval.ground_truth_node.truth_odometry.

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
    TWIST_UNKNOWN,
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


def test_first_sample_marks_the_twist_unknown_instead_of_zero():
    """No difference is available yet; 0.0 would assert a measured standstill."""
    msg = _message(_stamped(WORLD, MODEL, x=0.5, t=1.0))

    odom, prev = truth_odometry(msg, WORLD, MODEL, None)

    assert odom.twist.covariance[0] == TWIST_UNKNOWN
    assert odom.pose.pose.position.x == pytest.approx(0.5)
    assert prev == pytest.approx((0.5, 0.0, 0.0, 1.0))


def test_duplicate_stamp_marks_the_twist_unknown():
    """Two poses sharing a stamp: dt = 0, so the velocity is not computable."""
    first = _message(_stamped(WORLD, MODEL, x=0.0, t=1.0))
    _, prev = truth_odometry(first, WORLD, MODEL, None)

    second = _message(_stamped(WORLD, MODEL, x=0.5, t=1.0))
    odom, _ = truth_odometry(second, WORLD, MODEL, prev)

    assert odom.twist.covariance[0] == TWIST_UNKNOWN
    assert odom.twist.twist.linear.x == 0.0


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


def test_valid_twist_is_not_marked_unknown():
    first = _message(_stamped(WORLD, MODEL, x=0.0, t=1.0))
    _, prev = truth_odometry(first, WORLD, MODEL, None)

    second = _message(_stamped(WORLD, MODEL, x=0.02, t=1.1))
    odom, _ = truth_odometry(second, WORLD, MODEL, prev)

    assert odom.twist.covariance[0] == 0.0


def test_skid_is_not_counted_as_forward_speed():
    """
    A 45-degree skid is 1.0 m/s forward + 1.0 m/s sideways, not 1.414 forward.

    Signing the displacement magnitude would report 1.414 and hide half of an
    actuation fault from the Phase 4.3 commanded-vs-achieved comparison.
    """
    first = _message(_stamped(WORLD, MODEL, x=0.0, y=0.0, yaw=0.0, t=1.0))
    _, prev = truth_odometry(first, WORLD, MODEL, None)

    second = _message(_stamped(WORLD, MODEL, x=0.1, y=0.1, yaw=0.0, t=1.1))
    odom, _ = truth_odometry(second, WORLD, MODEL, prev)

    assert odom.twist.twist.linear.x == pytest.approx(1.0, abs=1e-3)
    assert odom.twist.twist.linear.y == pytest.approx(1.0, abs=1e-3)


def test_sideways_motion_reports_lateral_not_forward():
    """A diff-drive robot cannot do this; non-zero linear.y is the evidence."""
    first = _message(_stamped(WORLD, MODEL, x=0.0, y=0.0, yaw=0.0, t=1.0))
    _, prev = truth_odometry(first, WORLD, MODEL, None)

    second = _message(_stamped(WORLD, MODEL, x=0.0, y=0.05, yaw=0.0, t=1.1))
    odom, _ = truth_odometry(second, WORLD, MODEL, prev)

    assert odom.twist.twist.linear.x == pytest.approx(0.0, abs=1e-6)
    assert odom.twist.twist.linear.y == pytest.approx(0.5, abs=1e-3)


def test_forward_component_is_measured_along_the_previous_heading():
    """Heading +90 deg: a +y displacement is forward, a +x displacement is lateral."""
    first = _message(_stamped(WORLD, MODEL, x=0.0, y=0.0, yaw=math.pi / 2.0, t=1.0))
    _, prev = truth_odometry(first, WORLD, MODEL, None)

    second = _message(_stamped(WORLD, MODEL, x=0.0, y=0.03, yaw=math.pi / 2.0, t=1.1))
    odom, _ = truth_odometry(second, WORLD, MODEL, prev)

    assert odom.twist.twist.linear.x == pytest.approx(0.3, abs=1e-3)
    assert odom.twist.twist.linear.y == pytest.approx(0.0, abs=1e-6)


def test_pose_covariance_is_zero():
    """Ground truth carries no uncertainty; zeros must not be read as confidence."""
    msg = _message(_stamped(WORLD, MODEL, x=0.5))

    odom, _ = truth_odometry(msg, WORLD, MODEL, None)

    assert list(odom.pose.covariance) == [0.0] * 36
