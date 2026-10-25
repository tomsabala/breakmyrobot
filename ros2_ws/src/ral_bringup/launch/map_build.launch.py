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

r"""
Build the static map: sim + scan relay + slam_toolbox + alignment capture.

This is an OPERATOR session, not an automated step. It produces three
committed artifacts that every later phase depends on - maps/apartment.pgm,
maps/apartment.yaml and maps/apartment_frames.yaml - and they are a matched
set. See ral_bringup/maps/README.md for why the map must not be regenerated
from world geometry.

Procedure
---------
    scripts/build.sh

    # shell 1 - this launch file
    scripts/dev.sh bash -lc 'source /ws/ros2_ws/install/setup.bash && \
      ros2 launch ral_bringup map_build.launch.py'
    # WAIT for:
    #   [capture_world_to_map] world_to_map captured: x=... y=... yaw=... -> ...

    # shell 2 (HOST; the container image is ros-base and has no rviz2)
    source /opt/ros/humble/setup.bash && rviz2 -d ros2_ws/src/ral_bringup/rviz/ral.rviz

    # shell 3 - drive
    scripts/dev.sh bash -lc 'source /ws/ros2_ws/install/setup.bash && \
      ros2 run teleop_twist_keyboard teleop_twist_keyboard'

    # shell 4 - save, with the stack still running
    scripts/dev.sh bash -lc 'source /ws/ros2_ws/install/setup.bash && \
      ros2 run nav2_map_server map_saver_cli -t /map \
        -f /ws/ros2_ws/src/ral_bringup/maps/apartment \
        --ros-args -p use_sim_time:=true'

    # shell 5 - check and SEAL the pair, then install all three artifacts.
    # map_sanity.py writes the accepted grid's sha256 into
    # apartment_frames.yaml; until it has, roomba_nav.launch.py refuses to
    # start on an alignment that cannot be shown to belong to this map.
    python3 scripts/checks/map_sanity.py
    scripts/build.sh --packages-select ral_bringup

**Do not drive until capture_world_to_map has logged the capture.** It samples
truth at slam_toolbox's first /map to record where the `map` frame was placed;
if the robot has already moved, that pose is unobservable and the node refuses
to write (exit 2) rather than recording a wrong alignment. It also refuses if
/odom is not at its origin (exit 4): slam_toolbox puts `map` on the odom
origin, so a stationary robot parked elsewhere is not evidence of anything.

Drive requirements and the acceptance criteria for the resulting map are in
ral_bringup/maps/README.md.

The static map -> world transform that roomba_nav.launch.py publishes is
deliberately NOT started here: it is computed from apartment_frames.yaml,
which this session is what creates.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The source tree, not the share directory. colcon --symlink-install cannot
# symlink a file that does not exist at build time, so on a fresh checkout
# share/ral_bringup/maps/apartment_frames.yaml is not a link to anything:
# writing there would produce a file that vanishes on the next build. Writing
# into src/ and rebuilding afterwards is what makes the artifact committed.
DEFAULT_FRAMES_OUTPUT = '/ws/ros2_ws/src/ral_bringup/maps/apartment_frames.yaml'


def generate_launch_description() -> LaunchDescription:
    world = LaunchConfiguration('world')
    map_name = LaunchConfiguration('map_name')
    frames_output = LaunchConfiguration('frames_output')

    bringup_share = get_package_share_directory('ral_bringup')
    slam_params = os.path.join(bringup_share, 'config', 'slam_roomba.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='ral_apartment',
            description='World name: selects the SDF, the bridge config and the '
                        'truth frame name together.'),
        DeclareLaunchArgument(
            'map_name', default_value='apartment',
            description='Name recorded in the alignment file; must match the '
                        'name map_saver_cli is given.'),
        DeclareLaunchArgument(
            'frames_output', default_value=DEFAULT_FRAMES_OUTPUT,
            description='Where to write the measured world->map alignment. '
                        'Defaults to the source tree; rebuild afterwards.'),

        # gz, the bridge, robot_state_publisher, the spawn and
        # ground_truth_node. `robot: roomba` is explicit and not a default:
        # the committed map is the grid the ROOMBA's lidar saw from the
        # Roomba's sensor height, and rebuilding it from another body would
        # silently change what every committed alignment belongs to.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'robot_sim.launch.py')),
            launch_arguments={'world': world, 'robot': 'roomba'}.items()),

        # The sole publisher of /scan. slam_toolbox maps from the same stream
        # the robot's own stack consumes, so the map is built from what the
        # robot can actually see.
        Node(
            package='ral_faults',
            executable='scan_fault_relay',
            name='scan_fault_relay',
            parameters=[{'use_sim_time': True}],
            output='screen'),

        # Started directly rather than through slam_toolbox's
        # online_async_launch.py: that launch file defaults to the shipped
        # params, whose base_frame (`base_footprint`) does not exist on this
        # robot and whose max_laser_range (20 m) exceeds the modelled sensor.
        # Both failures are quiet - no map, or a map rastered from beams that
        # cannot exist.
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params, {'use_sim_time': True}],
            output='screen'),

        # Evaluation-side. Samples /ground_truth/odom once, at slam_toolbox's
        # first /map, to record where `map` was placed. Nothing in the
        # navigation path consumes its output.
        Node(
            package='ral_eval',
            executable='capture_world_to_map',
            name='capture_world_to_map',
            parameters=[{
                'use_sim_time': True,
                'output_path': frames_output,
                'map_name': map_name,
                'world_name': world,
            }],
            output='screen'),
    ])
