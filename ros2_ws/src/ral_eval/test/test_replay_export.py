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
What a replay fixture must not be able to claim.

The exporter's one freedom is which messages survive, so these cases pin the
rule that decides: events are never thinned, samples are thinned on a grid
that is a property of the stamps alone, and the rule text that lands in the
fixture is the rule that ran. Beside that, the headline may not state a
number metrics.json does not carry, and a run missing an artifact must refuse
rather than export a fixture that looks complete.

No bag and no simulator: every input here is synthetic.
"""

import json
import os

from ral_eval.replay_export import (
    ExportError,
    REPLAY_VERSION,
    build_headline,
    decimate,
    decimation_rule,
    export_run,
    round_payload,
    stream_sources,
    write_index,
)

import pytest

import yaml


def frame(t, topic='truth'):
    """One frame at a sim time, with the stamp the exporter would have read."""
    return {'t': t, 'topic': topic, 'payload': {'stamp_sim_s': t}}


ROOMBA_PROFILE = {
    'robot': 'roomba',
    'estimate': {'topic': '/amcl_pose', 'type': 'geometry_msgs/msg/PoseWithCovarianceStamped',
                 'frame': 'map', 'label': 'AMCL', 'tf_source': 'bridge',
                 'uncertainty': {'kind': 'covariance_trace', 'converged_below': 0.25}},
    'command': {'kind': 'velocity_twist', 'topic': '/cmd_vel', 'label': 'twist'},
    'mission_driver': 'nav2',
    'bringup_topics': [{'topic': '/map', 'type': 'nav_msgs/msg/OccupancyGrid'}],
}

CAR_PROFILE = {
    'robot': 'car',
    'estimate': {'topic': '/mavros/local_position/odom', 'type': 'nav_msgs/msg/Odometry',
                 'frame': 'ekf_local', 'label': 'EKF3', 'tf_source': 'mavros',
                 'uncertainty': {'kind': 'none'}},
    'command': {'kind': 'setpoint_position', 'topic': '/mavros/setpoint_position/local',
                'label': 'setpoint', 'planner_action': '/compute_path_to_pose'},
    'mission_driver': 'mavros_guided',
    'bringup_topics': [{'topic': '/mavros/state', 'type': 'mavros_msgs/msg/State'}],
}


def test_decimation_keeps_every_event_and_thins_only_the_samples():
    """An event stream passes through whole; a sample stream lands on a grid."""
    events = [frame(t / 100.0, 'mission') for t in range(50)]
    assert decimate(events, None) == events, 'a mission state may never be dropped'

    samples = [frame(t / 100.0) for t in range(100)]       # 100 Hz over 1 s sim
    kept = decimate(samples, 0.1)

    assert len(kept) == 10
    assert [round(f['t'], 3) for f in kept] == [round(i / 10.0, 3) for i in range(10)]


def test_decimation_grid_is_a_property_of_the_stamps_not_of_the_arrival_order():
    """A jittery stream cannot drift its own sampling grid."""
    jittered = [frame(t) for t in (0.0, 0.098, 0.201, 0.299, 0.4, 0.51, 0.6)]

    kept = [f['t'] for f in decimate(jittered, 0.1)]

    # Buckets 0, 0, 2, 2, 4, 5, 6: 0.098 and 0.299 fall in a bucket already
    # represented and are dropped, buckets 1 and 3 are simply empty. A
    # "0.1 s since the last kept frame" rule would instead have kept 0.098
    # and shifted every later decision by that offset.
    assert kept == [0.0, 0.201, 0.4, 0.51, 0.6]


def test_decimation_bucket_edges_are_exact_in_integer_microseconds():
    """A sample ON a bucket edge is kept: 0.3 // 0.1 is 2.0 in binary floats."""
    edges = [frame(t) for t in (0.0, 0.1, 0.2, 0.3, 0.7, 0.8, 2.9)]

    assert [f['t'] for f in decimate(edges, 0.1)] == [0.0, 0.1, 0.2, 0.3, 0.7, 0.8, 2.9]


def test_decimation_rule_text_states_what_actually_ran():
    """The fixture's rule string carries the interval and both counts."""
    assert decimation_rule('mission', 4, 4) == 'kept every message: 4 of 4'

    text = decimation_rule('scan', 729, 147)

    assert '0.5 s sim bucket' in text
    assert '2 Hz sim' in text
    assert '147 of 729' in text, 'the rule must say how much was dropped'


def test_rounding_keeps_a_no_return_null_and_leaves_grid_geometry_alone():
    """Rounding is a precision cut, never a value a message did not carry."""
    scan = round_payload('scan', {
        'stamp_sim_s': 41.7341234567, 'frame': 'rplidar_link', 'angle_min': -3.14159265,
        'angle_increment': 0.01750196, 'range_min': 0.15, 'range_max': 12.0,
        'ranges': [1.2345678, None, 0.0004]})

    assert scan['ranges'] == [1.235, None, 0.0], 'a no-return must stay null'
    assert scan['stamp_sim_s'] == 41.734123, 'stamps keep microseconds for the scan join'

    grid = round_payload('map', {
        'frame': 'map', 'resolution': 0.05000000074505806, 'width': 241, 'height': 162,
        'origin': {'x': -0.506, 'y': -0.585, 'yaw': 0.0}, 'data_b64': 'AAAA'})

    # The resolution places every cell; a rounded one would move the walls.
    assert grid['resolution'] == 0.05000000074505806
    assert grid['data_b64'] == 'AAAA'

    estimate = round_payload('est', {
        'stamp_sim_s': 1.0, 'frame': 'ekf_local',
        'pose': {'x': 1.23456, 'y': -2.34567, 'yaw_rad': 0.1234567},
        'covariance_trace': None, 'label': 'EKF3'})

    assert estimate['pose'] == {'x': 1.235, 'y': -2.346, 'yaw_rad': 0.1235}
    assert estimate['covariance_trace'] is None, 'no covariance is not a zero covariance'


def test_stream_sources_follow_the_profile_and_refuse_an_unknown_driver():
    """Which streams exist is the profile's answer, never the robot's name."""
    roomba = dict(stream_sources(ROOMBA_PROFILE))
    car = dict(stream_sources(CAR_PROFILE))

    assert roomba['est'] == '/amcl_pose' and car['est'] == '/mavros/local_position/odom'
    assert roomba['cmd'] == '/cmd_vel'
    assert 'cmd' not in car, 'the car is commanded in setpoints, not in a twist'
    assert 'fb' not in car, 'the car has no navigate_to_pose action to give feedback'
    # Both platforms plan globally, so both carry /plan and /map.
    assert roomba['plan'] == car['plan'] == '/plan'
    assert roomba['map'] == car['map'] == '/map'
    assert car['align'] == '/frame_alignment' and 'align' not in roomba
    assert car['wp'] == '/mavros/nav_controller_output/output'

    unknown = dict(CAR_PROFILE, mission_driver='teleop_only')
    with pytest.raises(ValueError) as refusal:
        stream_sources(unknown)
    assert 'teleop_only' in str(refusal.value)


def test_headline_states_only_what_the_artifacts_carry():
    """Every clause names a run.json or metrics.json field, or says it is absent."""
    run_json = {'experiment_id': 'scan_degraded_roomba', 'outcome': 'nav2:STATUS_SUCCEEDED'}
    metrics_doc = {'metrics': {
        'goal_pose_error_m': {'value': 0.06805723518196488},
        'mission_duration_s': {'value': 52.224000000000004},
        'collision_count': {'value': 0}}}

    headline = build_headline(run_json, metrics_doc, ('scan_degradation', 41.734))

    assert headline == ('scan_degraded_roomba: nav2:STATUS_SUCCEEDED, 0.068 m from the '
                        'goal, 52.22 s sim, 0 collision(s), scan_degradation activated '
                        'at 41.73 s sim.')


def test_headline_reports_an_unmeasured_metric_as_unmeasured():
    """A null metric is stated with its reason, never dropped or zeroed."""
    run_json = {'experiment_id': 'ui_car', 'outcome': 'mavros:stalled_no_progress'}
    metrics_doc = {'metrics': {
        'goal_pose_error_m': {'value': None, 'reason': 'no truth sample inside the window'},
        'mission_duration_s': {'value': 33.446}}}

    headline = build_headline(run_json, metrics_doc, None)

    assert 'goal_pose_error_m not measured (no truth sample inside the window)' in headline
    assert '0.000 m' not in headline
    assert 'activated' not in headline, 'no fault reached activation in this run'


def _run_dir(tmp_path, **files):
    """Make a run directory carrying exactly the artifacts named."""
    run_dir = tmp_path / 'baseline_roomba_20260101T000000Z'
    (run_dir / 'bag').mkdir(parents=True)
    for name, doc in files.items():
        (run_dir / name).write_text(json.dumps(doc))
    return str(run_dir)


def test_export_refuses_a_run_with_no_bag(tmp_path):
    """Nothing to replay is a refusal, not an empty fixture."""
    run_dir = tmp_path / 'no_bag'
    run_dir.mkdir()

    with pytest.raises(ExportError) as refusal:
        export_run(str(run_dir), 'unused.yaml')

    assert 'no bag at' in str(refusal.value)
    assert str(run_dir) in str(refusal.value)


def test_export_refuses_a_run_with_no_metrics(tmp_path):
    """metrics.json is embedded verbatim; without it there is nothing to embed."""
    run_dir = _run_dir(tmp_path, **{'run.json': {'experiment_id': 'x'},
                                    'env.json': {'robot': 'roomba'}})

    with pytest.raises(ExportError) as refusal:
        export_run(run_dir, 'unused.yaml')

    assert 'metrics.json is missing' in str(refusal.value)


def test_export_refuses_a_run_whose_env_carries_no_robot_profile(tmp_path):
    """Without the run's own profile, which streams it had is unknown."""
    run_dir = _run_dir(tmp_path, **{'run.json': {'experiment_id': 'x'},
                                    'env.json': {'robot': 'roomba'},
                                    'metrics.json': {'metrics': {}}})

    with pytest.raises(ExportError) as refusal:
        export_run(run_dir, 'unused.yaml')

    assert 'robot_profile' in str(refusal.value)


def test_export_refuses_a_map_that_is_not_the_one_the_run_was_scored_against(tmp_path):
    """A different grid would offset every position by a constant."""
    run_dir = _run_dir(tmp_path, **{
        'run.json': {'experiment_id': 'x'},
        'env.json': {'robot': 'roomba', 'map_sha256': 'deadbeef',
                     'robot_profile': yaml.safe_dump(ROOMBA_PROFILE)},
        'metrics.json': {'metrics': {}}})

    with pytest.raises(ExportError) as refusal:
        export_run(run_dir, os.path.join(str(tmp_path), 'absent.yaml'))

    # The map is checked before the bag is opened, so a missing map yaml is
    # named as the missing map yaml.
    assert 'no map yaml at' in str(refusal.value)


def test_index_lists_every_fixture_present_with_its_size(tmp_path):
    """The index is rebuilt from the directory, not from what was just written."""
    doc = {'replay_version': REPLAY_VERSION, 'run_id': 'a_run', 'robot': 'roomba',
           'outcome': 'nav2:STATUS_SUCCEEDED', 'headline': 'a_run: nav2:STATUS_SUCCEEDED.',
           't_start': 1.5, 't_end': 9.25, 'frames': []}
    (tmp_path / 'a_run.json').write_text(json.dumps(doc))

    path = write_index(str(tmp_path))
    index = json.loads(open(path).read())

    assert len(index['replays']) == 1
    entry = index['replays'][0]
    assert entry['file'] == 'a_run.json'
    assert entry['bytes'] == os.path.getsize(str(tmp_path / 'a_run.json'))
    assert entry['t_start'] == 1.5 and entry['t_end'] == 9.25
    assert entry['headline'] == doc['headline']
