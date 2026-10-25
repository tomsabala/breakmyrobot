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
The capability gate and the parameter bounds, without a simulator.

Every /faults/* request passes `supports()` before anything is called, and
every numeric one passes `check_params()`. Both answer with the string an
operator reads: it is published verbatim as the FaultEvent `rejected` detail
and answered as the service message, so this file asserts the text as well as
the verdict.

The YAML is read by SOURCE path, as test_fault_catalogue.py does: no ament
index, so no installed share directory is needed, and no rclpy, so these run
in any build. The robot profiles live in ral_bringup, two directories up -
read directly rather than through a <test_depend>, because ral_bringup
exec_depends this package and colcon would see the reverse edge as a cycle.
test_bridge_contract.py reads this package's own source tree the same way.
"""

import math
import os

from ral_faults.catalogue import (
    check_params,
    load_catalogue,
    target_for,
)
from ral_eval.robot_profile import load_profile, profile_path, supports

import pytest

CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          'config')
BRINGUP_SHARE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    'ral_bringup')
CATALOGUE = load_catalogue(os.path.join(CONFIG_DIR, 'fault_catalogue.yaml'))
ROOMBA = load_profile(profile_path(BRINGUP_SHARE, 'roomba'))
CAR = load_profile(profile_path(BRINGUP_SHARE, 'car'))


def test_a_supported_fault_is_not_refused():
    assert supports(ROOMBA, 'kidnap') is None
    assert supports(ROOMBA, 'scan_degradation') is None
    assert supports(CAR, 'kidnap') is None


def test_an_unsupported_fault_refuses_with_the_profiles_own_reason_verbatim():
    """That string is what the FaultEvent and the service reply carry."""
    stated = CAR['unsupported']['scan_degradation']
    assert supports(CAR, 'scan_degradation') == stated
    assert 'ArduRover' in stated


def test_a_kind_absent_with_no_stated_reason_still_refuses():
    """Silence in the table is not consent."""
    reason = supports(CAR, 'mast_collapse')
    assert reason is not None
    assert 'mast_collapse is not listed under car' in reason
    assert 'car.yaml' in reason
    assert 'silence in the table is not consent' in reason


def test_a_profile_whose_robot_key_disagrees_with_its_stem_raises(tmp_path):
    """Callers look a profile up by stem; a disagreement addresses two robots."""
    source = profile_path(BRINGUP_SHARE, 'car')
    with open(source) as handle:
        text = handle.read()
    forged = tmp_path / 'roomba.yaml'
    forged.write_text(text)
    with pytest.raises(ValueError) as mismatch:
        load_profile(str(forged))
    assert "'car'" in str(mismatch.value)
    assert "'roomba'" in str(mismatch.value)


def test_a_profile_missing_a_required_key_raises_naming_it(tmp_path):
    forged = tmp_path / 'car.yaml'
    forged.write_text('robot: car\nmodel: ral_car\n')
    with pytest.raises(ValueError) as missing:
        load_profile(str(forged))
    assert 'is missing' in str(missing.value)
    assert 'estimate' in str(missing.value)


def test_an_unknown_target_rule_raises_naming_the_catalogue():
    broken = {'spawn_obstacle': dict(CATALOGUE['spawn_obstacle'],
                                     target_rule='whatever')}
    with pytest.raises(ValueError) as unknown_rule:
        target_for(broken, 'spawn_obstacle', {'name': 'blocker_1'}, 'ral_roomba')
    assert 'unknown target_rule' in str(unknown_rule.value)


def test_a_non_finite_parameter_is_refused_for_every_kind():
    """A kind with an empty param_ranges block is still checked for finiteness."""
    for kind, field in (('mobility_fault', 'force_x_n'),
                        ('kidnap', 'new_pose.position.x'),
                        ('spawn_obstacle', 'size.x')):
        refusal = check_params(CATALOGUE, kind, {field: float('nan')})
        assert refusal is not None, kind
        assert 'not a finite number' in refusal
        assert check_params(CATALOGUE, kind, {field: math.inf}) is not None, kind
        assert check_params(CATALOGUE, kind, {field: 1.0}) is None, kind


def test_a_scan_parameter_outside_the_declared_range_is_refused():
    """max_range_m 0.0 - a partially filled request - blanks every beam."""
    refusal = check_params(CATALOGUE, 'scan_degradation', {'max_range_m': 0.0})
    assert refusal is not None
    assert 'fault_catalogue.yaml declares scan_degradation.param_ranges.max_range_m' \
        in refusal
    assert check_params(CATALOGUE, 'scan_degradation', {'max_range_m': 4.0}) is None
    assert check_params(CATALOGUE, 'scan_degradation',
                        {'dropout_probability': 1.5}) is not None
    assert check_params(CATALOGUE, 'scan_degradation',
                        {'range_noise_stddev_m': -0.01}) is not None
    # Unbounded above: `[0.0, null]` means no upper bound, not "unchecked".
    assert check_params(CATALOGUE, 'scan_degradation',
                        {'range_noise_stddev_m': 50.0}) is None
