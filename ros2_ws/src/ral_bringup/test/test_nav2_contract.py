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
The Nav2/SLAM configuration contract, checked against the plant, no simulator.

Nav2's parameters are copied from a TurtleBot3 reference. Three classes of
mistake survive that copy silently, and each one is checked here:

  * commanding a plant that cannot execute the command - every velocity and
    acceleration limit Nav2 may emit must sit inside the DiffDrive envelope
    in models/ral_roomba/model.sdf, or the plant saturates and the
    saturation is read as a controller fault;
  * naming frames, topics and ranges that do not exist on this robot -
    `base_footprint`, `laser_max_range: 100.0`, a relative `scan` topic;
  * routing around the fault boundary - anything reading /scan_raw instead
    of /scan bypasses the node Phase 4.2 injects sensor faults into, so the
    fault would have no effect on the subsystem it is supposed to break.

Everything here is a file parse. It runs in every `colcon test`, with no gz,
no ROS graph and no map.
"""

from functools import lru_cache
from pathlib import Path
from xml.etree import ElementTree

import yaml

CONFIG_DIR = Path(__file__).resolve().parents[1] / 'config'
MODEL_SDF = (Path(__file__).resolve().parents[2]
             / 'ral_description' / 'models' / 'ral_roomba' / 'model.sdf')

SCAN_TOPIC = '/scan'


# Each of the three inputs is parsed once; every helper below reads from the
# same tree, so a test cannot be reading a different file than its neighbour.
@lru_cache(maxsize=None)
def _nav2():
    return yaml.safe_load((CONFIG_DIR / 'nav2_roomba.yaml').read_text())


@lru_cache(maxsize=None)
def _slam():
    return yaml.safe_load((CONFIG_DIR / 'slam_roomba.yaml').read_text())


@lru_cache(maxsize=None)
def _model_root():
    assert MODEL_SDF.is_file(), f'robot description not found at {MODEL_SDF}'
    return ElementTree.parse(MODEL_SDF).getroot().find('model')


def _diff_drive():
    """Return the DiffDrive plugin's limits: the actual plant envelope."""
    for plugin in _model_root().findall('plugin'):
        if plugin.get('name') == 'gz::sim::systems::DiffDrive':
            return {child.tag: child.text.strip() for child in plugin}
    raise AssertionError(f'no DiffDrive plugin in {MODEL_SDF}')


def _lidar_range():
    for link in _model_root().findall('link'):
        for sensor in link.findall('sensor'):
            if sensor.get('type') != 'gpu_lidar':
                continue
            rng = sensor.find('lidar/range')
            return float(rng.find('min').text), float(rng.find('max').text)
    raise AssertionError(f'no gpu_lidar sensor in {MODEL_SDF}')


def _base_link_radius():
    for link in _model_root().findall('link'):
        if link.get('name') != 'base_link':
            continue
        return float(link.find('collision/geometry/cylinder/radius').text)
    raise AssertionError(f'no base_link collision cylinder in {MODEL_SDF}')


def _follow_path():
    return _nav2()['controller_server']['ros__parameters']['FollowPath']


def _costmaps():
    nav2 = _nav2()
    return {
        'local_costmap': nav2['local_costmap']['local_costmap']['ros__parameters'],
        'global_costmap': nav2['global_costmap']['global_costmap']['ros__parameters'],
    }


def test_dwb_limits_stay_inside_the_diffdrive_envelope():
    plant = _diff_drive()
    dwb = _follow_path()
    checks = [
        ('max_vel_x', dwb['max_vel_x'], float(plant['max_linear_velocity']), 'le'),
        ('max_speed_xy', dwb['max_speed_xy'], float(plant['max_linear_velocity']), 'le'),
        ('max_vel_theta', dwb['max_vel_theta'], float(plant['max_angular_velocity']), 'le'),
        ('acc_lim_x', dwb['acc_lim_x'], float(plant['max_linear_acceleration']), 'le'),
        ('acc_lim_theta', dwb['acc_lim_theta'], float(plant['max_angular_acceleration']), 'le'),
        ('decel_lim_x', dwb['decel_lim_x'], float(plant['min_linear_acceleration']), 'ge'),
        ('decel_lim_theta', dwb['decel_lim_theta'],
         float(plant['min_angular_acceleration']), 'ge'),
    ]
    for name, configured, limit, direction in checks:
        if direction == 'le':
            assert configured <= limit, (
                f'FollowPath.{name} = {configured} exceeds the plant limit {limit} from '
                f'{MODEL_SDF.name}: the simulated drive would saturate and the clamp would '
                'look like a controller that cannot follow its own plan')
        else:
            assert configured >= limit, (
                f'FollowPath.{name} = {configured} is below the plant limit {limit} from '
                f'{MODEL_SDF.name}')
    assert dwb['min_vel_x'] >= -float(plant['max_linear_velocity'])


def test_velocity_smoother_matches_the_controller_envelope():
    """The smoother publishes /cmd_vel; it must shape, never reshape, DWB's output."""
    dwb = _follow_path()
    smoother = _nav2()['velocity_smoother']['ros__parameters']
    shipped = [0.26, 0.0, 1.0]
    reason = (
        'velocity_smoother is the node that actually publishes /cmd_vel '
        '(nav2_bringup/navigation_launch.py remaps cmd_vel_smoothed -> cmd_vel), so its '
        f'envelope must equal DWB\'s. Left at the shipped TurtleBot3 default {shipped} it '
        'silently truncates every command above 0.26 m/s, and the truncation is '
        'indistinguishable from a controller bug. This test exists to catch exactly that.')
    assert smoother['max_velocity'][0] == dwb['max_vel_x'], reason
    assert smoother['max_velocity'][2] == dwb['max_vel_theta'], reason
    assert smoother['min_velocity'][0] == dwb['min_vel_x'], reason
    assert smoother['max_accel'][0] == dwb['acc_lim_x'], reason
    assert smoother['max_accel'][2] == dwb['acc_lim_theta'], reason
    assert smoother['max_decel'][0] == dwb['decel_lim_x'], reason
    assert smoother['max_decel'][2] == dwb['decel_lim_theta'], reason


def test_velocity_limit_arrays_are_floats():
    """
    Integer entries in a double_array parameter are rejected at node start.

    Measured in ral:dev - a node declaring a double_array default and given
    `[1, 0, 2]` from a params file raises
    `InvalidParameterTypeException: ... of type 'INTEGER_ARRAY', expecting
    type 'DOUBLE_ARRAY'`. Under use_composition that kills the whole Nav2
    container, far from the YAML line that caused it. The equality checks
    above cannot see it: in Python `1 == 1.0`.
    """
    smoother = _nav2()['velocity_smoother']['ros__parameters']
    for name in ('max_velocity', 'min_velocity', 'max_accel', 'max_decel',
                 'deadband_velocity'):
        for index, value in enumerate(smoother[name]):
            assert isinstance(value, float), (
                f'velocity_smoother.{name}[{index}] is the integer {value!r}; write it '
                'as a float or the parameter is rejected and Nav2 never starts')


def test_both_costmaps_inflate_identically():
    """One number is reported as "the inflation radius in force"; make it one number."""
    local, global_ = (_costmaps()[name]['inflation_layer'] for name in
                      ('local_costmap', 'global_costmap'))
    assert local['inflation_radius'] == global_['inflation_radius'], (
        'the costmaps disagree about inflation: the planner would route through a gap '
        'the controller treats as lethal, or the reverse')
    assert local['cost_scaling_factor'] == global_['cost_scaling_factor']


def test_recovery_behaviours_stay_inside_the_plant_envelope():
    """
    Check behavior_server, which publishes /cmd_vel bypassing the smoother.

    Verified in nav2_bringup/navigation_launch.py: controller_server and
    velocity_smoother remap cmd_vel, behavior_server does not. Its rotation
    limits are therefore commands straight into the plant.
    """
    plant = _diff_drive()
    behaviors = _nav2()['behavior_server']['ros__parameters']
    assert behaviors['max_rotational_vel'] <= float(plant['max_angular_velocity'])
    assert behaviors['rotational_acc_lim'] <= float(plant['max_angular_acceleration'])


def test_amcl_uses_a_frame_this_robot_actually_has():
    amcl = _nav2()['amcl']['ros__parameters']
    links = {link.get('name') for link in _model_root().findall('link')}
    assert amcl['base_frame_id'] == 'base_link', (
        f"amcl.base_frame_id = {amcl['base_frame_id']!r}; the shipped default "
        "'base_footprint' does not exist on this robot and AMCL never resolves TF")
    assert 'base_link' in links
    assert 'base_footprint' not in links, (
        'a base_footprint link has appeared in the model; decide which frame AMCL and '
        'slam_toolbox use and update both configs together')


def test_every_lidar_consumer_reads_the_relay_output():
    amcl = _nav2()['amcl']['ros__parameters']
    assert amcl['scan_topic'] == SCAN_TOPIC, (
        f"amcl.scan_topic = {amcl['scan_topic']!r}; it must be {SCAN_TOPIC!r}, the output "
        'of ral_faults/scan_fault_relay')
    for name, costmap in _costmaps().items():
        topic = costmap['obstacle_layer']['scan']['topic']
        assert topic == SCAN_TOPIC, (
            f'{name}.obstacle_layer.scan.topic = {topic!r}; reading /scan_raw would bypass '
            'the relay that Phase 4.2 injects sensor degradation into, so a lidar fault '
            'would leave obstacle avoidance untouched')
    assert _slam()['slam_toolbox']['ros__parameters']['scan_topic'] == SCAN_TOPIC


def test_no_nav2_component_is_pointed_at_simulator_truth():
    text = (CONFIG_DIR / 'nav2_roomba.yaml').read_text()
    for forbidden in ('ground_truth', 'base_link_truth'):
        assert forbidden not in text, (
            f'{forbidden!r} appears in nav2_roomba.yaml. Simulator truth must never reach '
            'localization, planning or control (ENGINEERING.md section 4)')


def test_amcl_laser_range_matches_the_modelled_sensor():
    amcl = _nav2()['amcl']['ros__parameters']
    min_range, max_range = _lidar_range()
    assert amcl['laser_min_range'] >= min_range, (
        f"amcl.laser_min_range = {amcl['laser_min_range']} is below the sensor's "
        f'{min_range} m minimum')
    assert amcl['laser_max_range'] <= max_range, (
        f"amcl.laser_max_range = {amcl['laser_max_range']} exceeds the sensor's "
        f'{max_range} m maximum; the shipped 100.0 asks AMCL to weight beams the lidar '
        'cannot produce')


def test_costmap_footprints_enclose_the_robot():
    radius = _base_link_radius()
    for name, costmap in _costmaps().items():
        assert costmap['robot_radius'] >= radius, (
            f"{name}.robot_radius = {costmap['robot_radius']} is smaller than the "
            f'{radius} m collision cylinder in {MODEL_SDF.name}: the planner would route '
            'the body through obstacles it does not know it occupies')


def test_amcl_does_not_hardcode_an_initial_pose():
    amcl = _nav2()['amcl']['ros__parameters']
    assert amcl['set_initial_pose'] is False, (
        'set_initial_pose true makes AMCL ignore the /initialpose a scenario publishes, so '
        'every run would start from the same hardcoded belief regardless of what was asked')


def test_slam_config_agrees_with_the_robot_and_the_costmaps():
    slam = _slam()['slam_toolbox']['ros__parameters']
    _, max_range = _lidar_range()
    assert slam['base_frame'] == 'base_link'
    assert slam['max_laser_range'] <= max_range, (
        f"slam_toolbox.max_laser_range = {slam['max_laser_range']} exceeds the sensor's "
        f'{max_range} m; the map would be rastered from beams that cannot exist')
    local = _costmaps()['local_costmap']
    assert slam['resolution'] == local['resolution'], (
        f"map resolution {slam['resolution']} != costmap resolution "
        f"{local['resolution']}; the static layer would be resampled")
