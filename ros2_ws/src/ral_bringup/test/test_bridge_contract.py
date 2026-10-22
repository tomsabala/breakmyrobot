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

The rule - wheel odometry reaches /tf, simulator truth never does - used to be
enforced by a comment in the YAML plus one runtime check that had to be run by
hand. These tests need no simulator and no ROS graph: they read every
config/bridge_roomba_*.yaml in this package and fail the build if a row would
put simulator truth into the TF tree.

Why this matters more than it looks: gz publishes truth as gz.msgs.Pose_V, the
same type DiffDrive uses for /tf, so a single wrong `ros_topic_name` is all it
takes. AMCL, the costmaps and the controller would then silently consume a
perfect pose, and every Phase 3 localization metric would be meaningless while
every topic still looked healthy.
"""

from pathlib import Path

import pytest
import yaml

CONFIG_DIR = Path(__file__).resolve().parents[1] / 'config'

# gz topics that carry simulator truth. PosePublisher's model pose plus the two
# world-level pose feeds that expose every model's true pose.
TRUTH_GZ_SUFFIXES = ('/pose', '/pose/info', '/dynamic_pose/info')

# Where truth is allowed to land in ROS. Anything under this prefix is an
# evaluation topic that a consumer has to subscribe to by name.
TRUTH_ROS_PREFIX = '/ground_truth/'


def _configs():
    configs = sorted(CONFIG_DIR.glob('bridge_roomba_*.yaml'))
    assert configs, f'no bridge config found in {CONFIG_DIR}'
    return configs


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
def test_tf_is_published_only_by_the_robots_own_odometry(config):
    """/tf may carry exactly one thing: DiffDrive's dead-reckoned odom->base_link."""
    tf_rows = [row for row in _rows(config) if row['ros_topic_name'] == '/tf']

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
