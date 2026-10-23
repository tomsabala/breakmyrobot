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
Bring up the full navigation stack: sim, scan relay, AMCL and Nav2.

What runs
---------
  * roomba_sim.launch.py - gz, the ROS/gz bridge, robot_state_publisher and
    ral_eval's ground_truth_node (evaluation only, no TF).
  * ral_faults/scan_fault_relay - the sole publisher of /scan. AMCL and both
    costmaps read it; Phase 4.2 degrades it at this boundary.
  * nav2_bringup/bringup_launch.py with ral_bringup/config/nav2_roomba.yaml -
    map_server, AMCL, the planner, DWB, the behaviours, the BT navigator and
    the velocity smoother that actually publishes /cmd_vel.
  * a static, DISPLAY-ONLY map -> world transform derived from the measured
    alignment (see below).

The three map artifacts are a MATCHED SET: maps/apartment.pgm,
maps/apartment.yaml and maps/apartment_frames.yaml. The alignment belongs to
that specific grid, and this launch file refuses to start if it does not:
the frames file records the .pgm's sha256 and ral_eval.frames checks it.
ral_bringup/maps/README.md has the procedure.

No RViz is started here. The container image is ros:humble-ros-base-jammy and
has no rviz2; RViz runs on the HOST:

    source /opt/ros/humble/setup.bash && rviz2 -d ros2_ws/src/ral_bringup/rviz/ral.rviz

Nothing in this stack consumes simulator truth. /ground_truth/odom has no
subscriber among the Nav2 nodes, and the static transform below carries a
fixed frame alignment, never a robot pose. scripts/checks/roomba_nav_check.py
asserts both at runtime (checks 4 and 5).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ral_eval.frames import load_alignment_for_map, map_to_world_tf


def _map_to_world_static_tf(context, *args, **kwargs):
    """
    Build the display-only map -> world transform from the measured alignment.

    An OpaqueFunction because the `map` argument has to be resolved to a real
    string before the alignment file beside it can be read. It is added
    OUTSIDE the TimerAction that delays Nav2, so a bad alignment stops the
    launch before gz is spawned rather than ten seconds into a run.

    This transform is DISPLAY ONLY. It carries a fixed, measured frame
    alignment and no robot pose, so it does not leak simulator truth into the
    navigation path (ENGINEERING.md section 4): nothing downstream can derive
    where the robot is from it. It exists so RViz, whose fixed frame is `map`,
    can draw /ground_truth/odom - which is stamped in the gz `world` frame -
    on top of the map.

    Phase 3's metrics must read maps/apartment_frames.yaml directly and apply
    ral_eval.frames, never listen to this TF: a metric that depends on a
    transform published by the stack it is measuring stops being independent
    of it.

    load_alignment_for_map raises if the file is missing, malformed, unsealed
    or measured against a different grid, naming the file and the command
    that fixes it. That is deliberate - an identity fallback, or an alignment
    belonging to some other .pgm, publishes a frame offset that every later
    number reports as localization error.
    """
    map_path = LaunchConfiguration('map').perform(context)
    x, y, yaw = map_to_world_tf(load_alignment_for_map(map_path))
    return [Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_world_static_tf',
        arguments=['--x', f'{x:.6f}', '--y', f'{y:.6f}', '--z', '0',
                   '--yaw', f'{yaw:.6f}', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'map', '--child-frame-id', 'world'],
        parameters=[{'use_sim_time': True}],
        output='screen')]


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('ral_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')

    world = LaunchConfiguration('world')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='ral_apartment',
            description='World name: selects the SDF, the bridge config and the '
                        'truth frame name together.'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(bringup_share, 'maps', 'apartment.yaml'),
            description='Static map. Its <map>_frames.yaml alignment must have been '
                        'measured for this exact grid.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_share, 'config', 'nav2_roomba.yaml'),
            description='Nav2 parameters.'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Let the lifecycle manager activate the Nav2 nodes.'),
        DeclareLaunchArgument(
            'use_composition', default_value='True',
            description='Load the Nav2 nodes into one component container.'),
        DeclareLaunchArgument(
            'nav2_start_delay', default_value='10.0',
            description='WALL seconds to wait for gz before starting Nav2.'),

        # First, so that an alignment that is missing, unsealed or measured
        # against a different grid aborts the launch before anything is
        # spawned, rather than after gz and the whole Nav2 stack are up.
        OpaqueFunction(function=_map_to_world_static_tf),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'roomba_sim.launch.py')),
            launch_arguments={'world': world}.items()),

        Node(
            package='ral_faults',
            executable='scan_fault_relay',
            name='scan_fault_relay',
            parameters=[{'use_sim_time': True}],
            output='screen'),

        # Nav2 activates its costmaps as soon as it starts. If gz has not yet
        # published /clock and the bridge has not yet published
        # odom -> base_link, costmap activation aborts with "Timed out waiting
        # for transform" and the lifecycle manager leaves the stack down.
        #
        # The delay is measured in WALL seconds on purpose: at t=0 sim time
        # does not exist yet, so a sim-time timer would never fire. If
        # activation still fails on a slower machine, raise this - do NOT
        # raise transform_tolerance, which would mask genuinely stale TF
        # during Phase 4's fault runs.
        TimerAction(
            period=LaunchConfiguration('nav2_start_delay'),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
                    launch_arguments={
                        'map': LaunchConfiguration('map'),
                        'params_file': LaunchConfiguration('params_file'),
                        'use_sim_time': 'true',
                        'autostart': LaunchConfiguration('autostart'),
                        'use_composition': LaunchConfiguration('use_composition'),
                        'slam': 'False',
                    }.items()),
            ]),
    ])
