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
The scenario schema, exercised without a ROS graph or a simulator.

The rejections are the point. A scenario is the tracked input that makes a run
reproducible, and every case below is one where accepting the file would
produce a run that LOOKS fine and measures the wrong thing - a different
world's map, a fault that never fires, a defaulted parameter nobody chose.
Each assertion checks the message names the offending key, because an error
that does not say which line to fix gets worked around instead.
"""

import os

from ral_eval.frames import map_pose_to_world
from ral_eval.scenario import (
    ALLOWED_WORLDS,
    ScenarioError,
    load_scenario,
    mission_poses,
    validate_scenario,
)

import pytest

MINIMAL = {
    'experiment_id': 'baseline_roomba',
    'robot': 'roomba',
    'world': 'ral_apartment',
    'seed': 1,
    'initial_pose': {'x': 0.5, 'y': 0.5, 'yaw': 0.0},
    'goal': {'x': 10.5, 'y': 6.5, 'yaw': 1.57},
    'timeout_sec': 180,
}


def _with(**overrides) -> dict:
    doc = dict(MINIMAL)
    doc.update(overrides)
    return doc


def _rejects(doc, needle: str) -> str:
    with pytest.raises(ScenarioError) as excinfo:
        validate_scenario(doc, 'scenario.yaml')
    message = str(excinfo.value)
    assert 'scenario.yaml' in message, f'the message does not name the file: {message}'
    assert needle in message, f'the message does not name {needle!r}: {message}'
    return message


def test_a_minimal_scenario_loads_and_coerces_numbers():
    scenario = validate_scenario(_with(timeout_sec=180, seed=7), 'scenario.yaml')
    assert scenario['timeout_sec'] == 180.0
    assert isinstance(scenario['timeout_sec'], float)
    assert isinstance(scenario['goal']['x'], float)
    assert scenario['interventions'] == []


def test_interventions_are_sorted_by_time_and_given_stable_ids():
    """The id joins a scenario entry to the FaultEvents the manager publishes."""
    doc = _with(interventions=[
        {'at_sec': 40.0, 'type': 'kidnap', 'params': {'pose': {'x': 1.0, 'y': 2.0,
                                                               'yaw': 0.0}}},
        {'at_sec': 20.0, 'type': 'spawn_obstacle',
         'params': {'name': 'blocker_1', 'pose': {'x': 6.0, 'y': 3.0, 'yaw': 0.0},
                    'size': {'x': 1.2, 'y': 0.2, 'z': 0.6}}},
    ])
    scenario = validate_scenario(doc, 'scenario.yaml')
    assert [item['id'] for item in scenario['interventions']] == \
        ['spawn_obstacle_1', 'kidnap_2']
    assert [item['at_sec'] for item in scenario['interventions']] == [20.0, 40.0]


def test_an_unknown_world_is_refused_with_what_a_new_world_needs():
    message = _rejects(_with(world='ral_kitchen'), 'world')
    assert 'bridge_roomba_' in message
    assert 'frames.yaml' in message


def test_the_car_is_refused_until_phase_5():
    _rejects(_with(robot='car'), 'robot')


def test_a_missing_goal_is_refused():
    doc = _with()
    del doc['goal']
    _rejects(doc, 'goal')


def test_a_boolean_yaw_is_not_an_angle():
    """A YAML `yaw: true` is not an angle, though bool is an int subclass."""
    _rejects(_with(goal={'x': 1.0, 'y': 1.0, 'yaw': True}), 'goal.yaw')


def test_a_negative_timeout_is_refused():
    _rejects(_with(timeout_sec=-5), 'timeout_sec')


def test_an_intervention_after_the_timeout_is_refused():
    doc = _with(timeout_sec=180, interventions=[
        {'at_sec': 200.0, 'type': 'clear_obstacle', 'params': {'name': 'x'}}])
    message = _rejects(doc, 'at_sec')
    assert 'timeout_sec' in message


def test_a_negative_intervention_time_is_refused():
    doc = _with(interventions=[
        {'at_sec': -1.0, 'type': 'clear_obstacle', 'params': {'name': 'x'}}])
    _rejects(doc, 'at_sec')


def test_an_unknown_intervention_type_is_refused():
    doc = _with(interventions=[{'at_sec': 1.0, 'type': 'explode', 'params': {}}])
    _rejects(doc, 'type')


def test_a_spawn_without_a_size_is_refused():
    doc = _with(interventions=[
        {'at_sec': 1.0, 'type': 'spawn_obstacle',
         'params': {'name': 'b', 'pose': {'x': 1.0, 'y': 1.0, 'yaw': 0.0}}}])
    _rejects(doc, 'size')


def test_two_obstacles_may_not_share_a_name():
    spawn = {'at_sec': 1.0, 'type': 'spawn_obstacle',
             'params': {'name': 'blocker_1', 'pose': {'x': 1.0, 'y': 1.0, 'yaw': 0.0},
                        'size': {'x': 1.0, 'y': 1.0, 'z': 1.0}}}
    doc = _with(interventions=[spawn, dict(spawn, at_sec=2.0)])
    _rejects(doc, 'blocker_1')


def test_clearing_an_obstacle_that_was_never_spawned_is_refused():
    doc = _with(interventions=[
        {'at_sec': 5.0, 'type': 'clear_obstacle', 'params': {'name': 'ghost'}}])
    _rejects(doc, 'ghost')


def test_an_experiment_id_that_is_not_a_directory_name_is_refused():
    _rejects(_with(experiment_id='Baseline Roomba'), 'experiment_id')


def test_an_unexpected_top_level_key_is_refused():
    """A key the runner does not read is a setting the author believes is applied."""
    _rejects(_with(use_sim_time=False), 'use_sim_time')


def test_mission_poses_converts_both_poses_and_round_trips():
    wtm = (0.5, 0.5, 0.3)
    scenario = validate_scenario(_with(), 'scenario.yaml')
    poses = mission_poses(scenario, wtm)
    for key in ('initial', 'goal'):
        back = map_pose_to_world(*poses[key]['map'], wtm)
        for got, want in zip(back, poses[key]['world']):
            assert abs(got - want) < 1e-9, f'{key}: {back} != {poses[key]["world"]}'
    # A non-zero alignment yaw must actually rotate; equality would mean the
    # conversion was skipped.
    assert poses['goal']['map'] != poses['goal']['world']


_SCENARIO_DIR = os.path.join(os.path.dirname(__file__), '..', 'scenarios')


@pytest.mark.parametrize('name', sorted(os.listdir(_SCENARIO_DIR)))
def test_every_shipped_scenario_passes_its_own_schema(name):
    """A scenario the harness installs must load; a broken one is a broken run."""
    scenario = load_scenario(os.path.join(_SCENARIO_DIR, name))
    assert scenario['experiment_id']
    assert scenario['world'] in ALLOWED_WORLDS
    assert scenario['source_path'].endswith(name)


# -- values that are ACCEPTED today and break a downstream consumer -----------
# Each of these was executed against the pre-fix schema and loaded cleanly.

def test_a_nan_timeout_cannot_end_a_mission_loop():
    """`elapsed >= nan` is permanently false; the mission would never time out."""
    _rejects(_with(timeout_sec=float('nan')), 'timeout_sec')


def test_an_infinite_timeout_is_refused():
    _rejects(_with(timeout_sec=float('inf')), 'timeout_sec')


def test_a_nan_goal_coordinate_is_refused_before_a_run_directory_exists():
    """int(math.floor(nan)) raises inside preflight_goal, after the bag started."""
    _rejects(_with(goal={'x': float('nan'), 'y': 1.0, 'yaw': 0.0}), 'goal.x')


def test_a_nan_intervention_time_is_refused():
    doc = _with(interventions=[
        {'at_sec': float('nan'), 'type': 'clear_obstacle', 'params': {'name': 'x'}}])
    _rejects(doc, 'at_sec')


def test_a_nan_box_extent_is_refused():
    doc = _with(interventions=[
        {'at_sec': 1.0, 'type': 'spawn_obstacle',
         'params': {'name': 'b', 'pose': {'x': 1.0, 'y': 1.0, 'yaw': 0.0},
                    'size': {'x': float('nan'), 'y': 1.0, 'z': 1.0}}}])
    _rejects(doc, 'size.x')


def test_an_unexpected_size_key_is_refused_like_an_unexpected_pose_key():
    """`w: 9.0` was silently dropped; an ignored key reads as applied."""
    doc = _with(interventions=[
        {'at_sec': 1.0, 'type': 'spawn_obstacle',
         'params': {'name': 'b', 'pose': {'x': 1.0, 'y': 1.0, 'yaw': 0.0},
                    'size': {'x': 1.0, 'y': 1.0, 'z': 1.0, 'w': 9.0}}}])
    _rejects(doc, "['w']")


def _scan(**overrides) -> dict:
    params = {'enabled': True, 'range_noise_stddev_m': 0.02, 'max_range_m': 8.0,
              'blind_sector_start_deg': -30.0, 'blind_sector_end_deg': 30.0,
              'dropout_probability': 0.1}
    params.update(overrides)
    return _with(interventions=[
        {'at_sec': 10.0, 'type': 'scan_degradation', 'params': params}])


def test_a_dropout_probability_above_one_is_not_a_probability():
    _rejects(_scan(dropout_probability=5.0), 'dropout_probability')


def test_a_negative_noise_stddev_is_refused():
    _rejects(_scan(range_noise_stddev_m=-3.0), 'range_noise_stddev_m')


def test_a_max_range_beyond_the_modelled_lidar_is_refused():
    """model.sdf gives the RPLIDAR A1 0.15-12.0 m; 20 m is not degradation."""
    _rejects(_scan(max_range_m=20.0), 'max_range_m')


def test_clearing_an_obstacle_at_its_own_spawn_time_is_refused():
    """Both fire on consecutive loop iterations; the box exists for ~0 sim s."""
    spawn = {'at_sec': 20.0, 'type': 'spawn_obstacle',
             'params': {'name': 'blocker_1', 'pose': {'x': 1.0, 'y': 1.0, 'yaw': 0.0},
                        'size': {'x': 1.0, 'y': 1.0, 'z': 1.0}}}
    clear = {'at_sec': 20.0, 'type': 'clear_obstacle', 'params': {'name': 'blocker_1'}}
    _rejects(_with(interventions=[spawn, clear]), 'at_sec')


def test_an_experiment_id_that_cannot_lead_a_directory_name_is_refused():
    _rejects(_with(experiment_id='_leading'), 'experiment_id')


def test_an_explicit_null_interventions_key_is_not_an_empty_list():
    """`interventions:` with no value is a list the author meant to fill in."""
    _rejects(_with(interventions=None), 'interventions')


def test_a_falsy_params_value_reaches_its_own_error():
    """`params: 0` used to be reported as missing params, not as a non-mapping."""
    doc = _with(interventions=[{'at_sec': 1.0, 'type': 'clear_obstacle', 'params': 0}])
    message = _rejects(doc, 'params')
    assert 'not a mapping' in message, message
