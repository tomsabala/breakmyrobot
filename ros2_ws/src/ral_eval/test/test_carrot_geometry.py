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
Which carrots a non-holonomic body may be commanded onto, and which it may not.

The decision this pins is a REFUSAL: when carrot_unreachable is true the
mavros driver publishes NO setpoint and asks the planner for a fresh route
instead. Getting it wrong in either direction is a measured failure mode -
too permissive commands points the steering cannot reach (the run quoted
below), too strict refuses a route the car completes - so the two measured
poses are the cases, not a restatement of the arithmetic.
"""

import math

from ral_eval.mavros_driver import carrot_unreachable


# The car's own plant limit (config/robots/car.yaml plant.min_turn_radius_m)
# and the margin its command block configures.
CAR_MIN_RADIUS = 0.611
CAR_MARGIN = 1.15


def _candidate(distance: float, bearing_deg: float) -> tuple:
    """Place a candidate `distance` away, `bearing_deg` off a body at the origin."""
    angle = math.radians(bearing_deg)
    return (distance * math.cos(angle), distance * math.sin(angle))


def test_the_measured_failure_pose_is_refused():
    """
    Refuse the carrot the car was measured creeping at.

    MEASURED, runs/ui_car_20261026T221042Z at t = 8.4 s sim: the driver was
    commanding a setpoint 0.63 m away at 65 deg off the nose. That needs a
    0.63 / (2 sin 65) = 0.35 m turn radius against this car's 0.611 m
    minimum, so the vehicle could not turn onto it; it crept at 0.03 m/s and
    the run ended mavros:stalled_no_progress 3.04 m short.
    """
    assert carrot_unreachable((0.0, 0.0, 0.0), _candidate(0.63, 65.0),
                              CAR_MIN_RADIUS, CAR_MARGIN) is True


def test_a_carrot_far_enough_round_the_same_corner_is_commanded():
    """
    Command the carrot the same corner offers further along the path.

    The healthy half of the same run: 2.67 m away at 48 deg needs a 1.80 m
    radius, which this body drives comfortably. The guard must not refuse
    these, or the carrot never advances and the car never turns at all.
    """
    assert carrot_unreachable((0.0, 0.0, 0.0), _candidate(2.67, 48.0),
                              CAR_MIN_RADIUS, CAR_MARGIN) is False


def test_a_close_carrot_straight_ahead_is_commanded():
    """
    Command a carrot dead ahead however close it is.

    No turn is demanded, so no turning radius is required. A distance test
    alone would refuse this one - 0.30 m is well inside the 0.70 m effective
    radius - and refusing the final approach to a waypoint dead ahead would
    stop the vehicle on a straight corridor.
    """
    assert carrot_unreachable((0.0, 0.0, 0.0), _candidate(0.30, 0.0),
                              CAR_MIN_RADIUS, CAR_MARGIN) is False
