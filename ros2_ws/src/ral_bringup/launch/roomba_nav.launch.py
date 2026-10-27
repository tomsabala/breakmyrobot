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
Bring up the full navigation stack: sim, fault injection, AMCL and Nav2.

What runs
---------
  * robot_sim.launch.py - gz, the ROS/gz bridge, robot_state_publisher, the
    runtime spawn of the body itself and ral_eval's ground_truth_node
    (evaluation only, no TF).
  * faults.launch.py - ral_faults/scan_fault_relay (the sole publisher of
    /scan, and where lidar degradation is injected), ral_faults/fault_manager
    (every /faults/* service, the FaultEvent lifecycle and /faults/active) and
    a second parameter_bridge, ros_gz_fault_bridge, carrying the gz create,
    remove, set_pose services and the two wrench topics.
  * nav2_bringup/bringup_launch.py with ral_bringup/config/nav2_roomba.yaml -
    map_server, AMCL, the planner, DWB, the behaviours, the BT navigator and
    the velocity smoother that actually publishes /cmd_vel.
  * a static, DISPLAY-ONLY map -> world transform derived from the measured
    alignment (see below).

`robot` and `seed` exist for the fault layer: `robot` selects the capability
row and the gz model a fault addresses, `seed` is the scenario's and governs
the scan relay's RNG. The scenario runner passes both from the scenario file.

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
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# The launch system executes launch files by PATH, not as a package, so a
# sibling module is importable only once its directory is on sys.path. The
# map/alignment refusal lives in common.py because car_nav.launch.py needs
# exactly the same guarantee, and two copies would eventually disagree about
# which map is acceptable.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import map_to_world_static_tf          # noqa: E402


def _nav2_after_gate(event, context):
    """
    Start Nav2 only if the gate PASSED; otherwise shut the launch down.

    Nav2 must NEVER start behind a failed gate. MEASURED,
    logs/stack_roomba_20261026T205614Z.log: started into an incomplete TF
    tree, the costmap looped on "two or more unconnected trees", the
    component container ignored SIGTERM for 10 s and was SIGKILLed (exit -9),
    and the gateway failed the stack 90 s later with "no messages on /map".
    A named shutdown here is the same failure, 90 s earlier, with the missing
    precondition on the gate's own stderr.
    """
    if event.returncode == 0:
        nav2_share = get_package_share_directory('nav2_bringup')
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': 'true',
                'autostart': LaunchConfiguration('autostart'),
                'use_composition': LaunchConfiguration('use_composition'),
                'slam': 'False',
            }.items())]
    return [
        LogInfo(msg='nav2_gate: odom -> base_link and an advancing /clock were '
                    'not both observed (exit '
                    f'{event.returncode}); Nav2 is NOT started, because a '
                    'costmap activated against an incomplete TF tree wedges '
                    'the component container instead of failing. See the '
                    "gate's stderr above for which precondition is missing."),
        EmitEvent(event=Shutdown(
            reason='nav2_gate failed: odom -> base_link did not resolve, or '
                   '/clock was not advancing')),
    ]


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('ral_bringup')

    world = LaunchConfiguration('world')
    robot = LaunchConfiguration('robot')
    seed = LaunchConfiguration('seed')

    # The gate is held in a local because the event handler below must target
    # this exact action, not a second copy of it.
    nav2_gate = ExecuteProcess(
        cmd=['ros2', 'run', 'ral_eval', 'wait_for_tf',
             '--parent', 'odom', '--child', 'base_link', '--timeout', '120'],
        name='nav2_gate', output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='ral_apartment',
            description='World name: selects the SDF, the bridge config and the '
                        'truth frame name together.'),
        DeclareLaunchArgument(
            'robot', default_value='roomba',
            description='Robot key: selects ral_bringup/config/robots/<robot>.yaml, '
                        'and with it the gz model spawned, which faults the manager '
                        'will apply and which gz model they address.'),
        DeclareLaunchArgument(
            'seed', default_value='1',
            description="The scenario's seed, forwarded to scan_fault_relay's RNG. "
                        'It governs the lidar noise and dropout only.'),
        DeclareLaunchArgument(
            'spawn_x', default_value='',
            description="World-frame spawn x, forwarded to robot_sim.launch.py. "
                        "Empty means the profile's spawn.x. The scenario runner "
                        'passes its declared initial pose, so the declared and the '
                        'actual pose cannot disagree.'),
        DeclareLaunchArgument(
            'spawn_y', default_value='',
            description="World-frame spawn y. Empty means the profile's spawn.y."),
        DeclareLaunchArgument(
            'spawn_yaw', default_value='',
            description="World-frame spawn yaw in rad. Empty means the profile's "
                        'spawn.yaw.'),
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

        # First, so that an alignment that is missing, unsealed or measured
        # against a different grid aborts the launch before anything is
        # spawned, rather than after gz and the whole Nav2 stack are up.
        OpaqueFunction(function=map_to_world_static_tf),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'robot_sim.launch.py')),
            launch_arguments={
                'world': world,
                'robot': robot,
                'spawn_x': LaunchConfiguration('spawn_x'),
                'spawn_y': LaunchConfiguration('spawn_y'),
                'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            }.items()),

        # Fault injection: the scan relay (sole publisher of /scan), the fault
        # manager and the gz interfaces the faults are applied through. The
        # relay used to be declared here; it moved so that everything taking
        # the scenario's seed and robot lives in one file.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'faults.launch.py')),
            # `map` goes through so the fault manager pre-flights a reposition
            # against the same grid this stack is scored in.
            launch_arguments={'world': world, 'robot': robot, 'seed': seed,
                              'map': LaunchConfiguration('map')}.items()),

        # Nav2 activates its costmaps as soon as it starts, and a costmap asks
        # for odom -> base_link at the LATEST available time. Starting it
        # before the bridge publishes that transform, or before gz publishes
        # an advancing /clock, is a measured stack failure (see
        # _nav2_after_gate). This gate waits for exactly those two conditions
        # and nothing more, so the wait is as short as the machine allows.
        #
        # It replaced a fixed 11.5 s wall timer that was BOTH wasteful and
        # insufficient: logs/stack_roomba_20261026T204117Z.log spent 11.5 s
        # idle in a ~30 s bring-up, and logs/stack_roomba_20261026T205614Z.log
        # fired into an incomplete TF tree anyway.
        #
        # Do NOT raise transform_tolerance instead: that would mask genuinely
        # stale TF during Phase 4's fault runs.
        nav2_gate,
        RegisterEventHandler(OnProcessExit(target_action=nav2_gate,
                                           on_exit=_nav2_after_gate)),
    ])
