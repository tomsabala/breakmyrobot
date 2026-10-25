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
Mechanical enforcement of the bridge's localization-honesty rule.

The rule - the robot's own estimate reaches /tf, simulator truth never does -
used to be enforced by a comment in the YAML plus one runtime check that had
to be run by hand. These tests need no simulator and no ROS graph: they read
every config/bridge_<robot>_<world>.yaml in this package, pair each with its
robot profile in config/robots/, and fail the build if a row would put
simulator truth into the TF tree.

Why this matters more than it looks: gz publishes truth as gz.msgs.Pose_V, the
same type DiffDrive uses for /tf, so a single wrong `ros_topic_name` is all it
takes. AMCL, the costmaps and the controller would then silently consume a
perfect pose, and every Phase 3 localization metric would be meaningless while
every topic still looked healthy.

WHAT /tf CARRIES IS PER ROBOT, and the profile says which. A DiffDrive robot
bridges odom->base_link from gz; the car has no DiffDrive at all, so that
transform does not exist for it and its estimate chain is published by
ral_eval's ekf_local_tf from /mavros/local_position/odom. A bridge config for
a robot with no profile fails loudly rather than being skipped.

This file also pins two cross-file facts that have no other home: the
footprint radius in a profile against the costmap it was taken from, and
every scenario's interventions against the faults its robot's profile
declares.
"""

from pathlib import Path

import pytest
import yaml

SRC_DIR = Path(__file__).resolve().parents[2]
CONFIG_DIR = Path(__file__).resolve().parents[1] / 'config'
PROFILE_DIR = CONFIG_DIR / 'robots'
SCENARIO_DIR = SRC_DIR / 'ral_eval' / 'scenarios'

# gz topics that carry simulator truth. PosePublisher's model pose plus the two
# world-level pose feeds that expose every model's true pose.
TRUTH_GZ_SUFFIXES = ('/pose', '/pose/info', '/dynamic_pose/info')

# Where truth is allowed to land in ROS. Anything under this prefix is an
# evaluation topic that a consumer has to subscribe to by name.
TRUTH_ROS_PREFIX = '/ground_truth/'

# What the EVIDENCE pipeline needs out of the simulator, for every robot.
# Not every bridged topic - these three are the ones whose absence is silent:
#   /clock      sim time; without it every windowed metric is null
#   /scan_raw   the relay's input, so /scan has nothing to pass through
#   /bumper     the contact path collision_count is counted from. A contact
#               sensor publishes only on contact, so a missing row reads as a
#               clean run rather than as a missing bridge - which is exactly
#               how /bumper went ungated by both nav harnesses.
SIM_EVIDENCE_TOPICS = ('/clock', '/scan_raw', '/bumper')


def _configs():
    configs = sorted(CONFIG_DIR.glob('bridge_*_*.yaml'))
    assert configs, f'no bridge config found in {CONFIG_DIR}'
    return configs


def _robot_of(path) -> str:
    """
    bridge_<robot>_<world>.yaml -> <robot>, by matching a profile that exists.

    NOT `stem.split('_')[1]`: the world name itself contains underscores
    (`ral_apartment`), so a positional split only happens to work while every
    robot name is a single token. The rule here is: strip the `bridge_`
    prefix, then take the LONGEST profile stem in config/robots that the
    remainder starts with, followed by `_`. A robot named `big_car` resolves
    to `big_car` rather than to `big`.
    """
    rest = path.stem[len('bridge_'):]
    candidates = sorted((profile.stem for profile in PROFILE_DIR.glob('*.yaml')
                         if rest.startswith(f'{profile.stem}_')), key=len)
    assert candidates, (
        f'{path.name}: no robot profile in {PROFILE_DIR} matches it. The name is '
        'parsed as bridge_<robot>_<world>.yaml by taking the longest profile stem '
        f'the remainder {rest!r} starts with, so either the profile is missing or '
        'the config is named for a robot nothing can launch, gate faults for or '
        'score a run against')
    return candidates[-1]


def _profile(robot: str) -> dict:
    path = PROFILE_DIR / f'{robot}.yaml'
    assert path.is_file(), (
        f'{path} does not exist, so the bridge config for {robot!r} describes a '
        'robot nothing can launch, gate faults for or score a run against')
    return yaml.safe_load(path.read_text())


def _rows(path):
    rows = yaml.safe_load(path.read_text())
    assert isinstance(rows, list), f'{path.name}: expected a list of bridge rows'
    return rows


@pytest.mark.parametrize('config', _configs(), ids=lambda p: p.name)
def test_every_row_is_complete(config):
    for row in _rows(config):
        missing = {'ros_topic_name', 'gz_topic_name', 'ros_type_name',
                   'gz_type_name', 'direction'} - set(row)
        assert not missing, f'{config.name}: row {row} is missing {sorted(missing)}'


@pytest.mark.parametrize('config', _configs(), ids=lambda p: p.name)
@pytest.mark.parametrize('topic', SIM_EVIDENCE_TOPICS)
def test_the_evidence_pipeline_gets_what_it_needs_from_the_simulator(config, topic):
    """
    Every bridge config carries the topics a run's evidence is computed from.

    Each of these fails SILENTLY when deleted: a missing /clock leaves every
    windowed metric null, a missing /scan_raw leaves the relay with nothing to
    pass through, and a missing /bumper makes a dead contact path look like a
    collision-free run. The harnesses now prove /bumper's liveness at runtime;
    this is the static half, so a deleted row fails here rather than in a
    document that reports zero collisions.
    """
    bridged = {row['ros_topic_name'] for row in _rows(config)}
    assert topic in bridged, (
        f'{config.name} does not bridge {topic}; the evidence pipeline reads it for '
        f'every robot. Bridged: {sorted(bridged)}')


@pytest.mark.parametrize('config', _configs(), ids=lambda p: p.name)
def test_tf_is_published_only_by_the_robots_own_estimate(config):
    """
    What may claim /tf is the robot's own dead reckoning, or nothing at all.

    `estimate.tf_source` in the profile decides which: `diff_drive_bridge`
    means gz publishes odom->base_link and exactly one row may carry it;
    `ekf_local_tf` means this platform has no such transform and NO row may.
    """
    profile = _profile(_robot_of(config))
    tf_source = profile['estimate']['tf_source']
    tf_rows = [row for row in _rows(config) if row['ros_topic_name'] == '/tf']

    if tf_source == 'ekf_local_tf':
        assert not tf_rows, (
            f"{config.name}: {len(tf_rows)} /tf row(s) "
            f'{[row["gz_topic_name"] for row in tf_rows]}, but this robot has no '
            'DiffDrive: its `odom`->`base_link` does not exist, and its estimate '
            'chain is published by ral_eval\'s ekf_local_tf from '
            '/mavros/local_position/odom. A bridged /tf here would give base_link '
            'two parents and the winner would depend on timing')
        return

    assert tf_source == 'diff_drive_bridge', (
        f'{config.name}: profile tf_source is {tf_source!r}, which this test does '
        'not know how to check; add the rule rather than letting it pass')
    assert len(tf_rows) == 1, (
        f'{config.name}: expected exactly one /tf row, found '
        f'{[row["gz_topic_name"] for row in tf_rows]}')
    assert tf_rows[0]['gz_topic_name'].endswith('/tf'), (
        f'{config.name}: /tf is fed from {tf_rows[0]["gz_topic_name"]}, which is not '
        'a DiffDrive tf topic')
    assert tf_rows[0]['gz_topic_name'].startswith('/model/'), (
        f'{config.name}: /tf must come from a model tf topic, got '
        f'{tf_rows[0]["gz_topic_name"]}')


@pytest.mark.parametrize('config', _configs(), ids=lambda p: p.name)
def test_truth_topics_never_reach_tf(config):
    for row in _rows(config):
        if not row['gz_topic_name'].endswith(TRUTH_GZ_SUFFIXES):
            continue
        assert row['ros_topic_name'].startswith(TRUTH_ROS_PREFIX), (
            f'{config.name}: simulator truth {row["gz_topic_name"]} is bridged to '
            f'{row["ros_topic_name"]}; truth may only reach ROS under '
            f'{TRUTH_ROS_PREFIX}* (ENGINEERING.md section 4)')


@pytest.mark.parametrize('config', _configs(), ids=lambda p: p.name)
def test_ground_truth_topics_carry_only_truth(config):
    """The converse: nothing but a truth pose feed may claim the truth namespace."""
    for row in _rows(config):
        if not row['ros_topic_name'].startswith(TRUTH_ROS_PREFIX):
            continue
        assert row['gz_topic_name'].endswith(TRUTH_GZ_SUFFIXES), (
            f'{config.name}: {row["ros_topic_name"]} is fed from '
            f'{row["gz_topic_name"]}, which is not a simulator truth pose topic')


@pytest.mark.parametrize('config', _configs(), ids=lambda p: p.name)
def test_commands_flow_to_the_simulator_and_sensors_flow_back(config):
    """
    /cmd_vel is the only row that may flow into the simulator.

    A config with NO /cmd_vel row is legal: ArduRover is commanded over
    MAVLink, not by a gz Twist, so the car has no command row at all.
    """
    for row in _rows(config):
        if row['ros_topic_name'] == '/cmd_vel':
            assert row['direction'] == 'ROS_TO_GZ', (
                f'{config.name}: /cmd_vel must be ROS_TO_GZ')
        else:
            assert row['direction'] == 'GZ_TO_ROS', (
                f'{config.name}: {row["ros_topic_name"]} must be GZ_TO_ROS; a ROS-side '
                'publisher on a sensor or state topic would let a non-simulator source '
                'inject it')


@pytest.mark.parametrize('config', _configs(), ids=lambda p: p.name)
def test_message_types_use_the_gz_prefix(config):
    """ignition.msgs.* silently matches nothing on Harmonic (Phase 0, docs/gate.md)."""
    for row in _rows(config):
        assert row['gz_type_name'].startswith('gz.msgs.'), (
            f'{config.name}: {row["gz_topic_name"]} uses {row["gz_type_name"]}')


def test_the_roomba_profile_footprint_matches_the_costmap_it_came_from():
    """
    One radius, two readers: the pre-flight's and the costmaps'.

    The runner pre-flights a goal against the profile's footprint radius while
    Nav2 plans against the costmap's. If they drift, a goal the planner would
    refuse is sent, or one it would accept is refused before it is tried - and
    either way the refusal reason names a footprint the stack did not use.
    """
    profile = yaml.safe_load((PROFILE_DIR / 'roomba.yaml').read_text())
    nav2 = yaml.safe_load((CONFIG_DIR / 'nav2_roomba.yaml').read_text())
    costmap = nav2['global_costmap']['global_costmap']['ros__parameters']
    assert float(profile['footprint']['radius_m']) == float(costmap['robot_radius']), (
        'roomba.yaml footprint.radius_m and nav2_roomba.yaml '
        'global_costmap.robot_radius disagree; the pre-flight and the planner '
        'would be checking different bodies')


@pytest.mark.parametrize(
    'scenario', sorted(SCENARIO_DIR.glob('*.yaml')), ids=lambda p: p.name)
def test_every_scenario_fault_is_one_its_robot_supports(scenario):
    """
    A scenario cannot be authored whose faults are refused at run time.

    The capability gate would reject them honestly, but the run would be a
    baseline wearing a fault scenario's name - and nothing would have tested
    what the file says it tests.
    """
    doc = yaml.safe_load(scenario.read_text()) or {}
    profile = _profile(doc['robot'])
    supported = set(profile['faults'])
    for intervention in doc.get('interventions') or []:
        kind = intervention['type']
        assert kind in supported, (
            f'{scenario.name} schedules {kind}, which {doc["robot"]}\'s profile does '
            f'not list (it supports {sorted(supported)}); the manager would reject it '
            f'with: {(profile.get("unsupported") or {}).get(kind, "no stated reason")}')
