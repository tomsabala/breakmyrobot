"""Bring up ral_roomba in a simulated world: gz, bridge, TF, truth republisher.

One launch argument, `world`, selects three things that must stay consistent or
sensor streams silently vanish:

  * the world SDF                    share/ral_description/worlds/<world>.sdf
  * the bridge config                share/ral_bringup/config/bridge_roomba_<world>.yaml
    (three of its gz topic paths embed the world name)
  * ground_truth_node's world_frame  (gz PosePublisher stamps the world name)

Nothing else in Phase 1 varies. There is no map, no AMCL, no Nav2 and no fault
injection here - those are Phases 2 and 4.

robot_state_publisher prints three kinds of warning on startup, all benign:
  * `XML Element[gz_frame_id], child of element[sensor], not defined in SDF` -
    sdformat_urdf does not know the gz-only frame override; gz does, and the
    element is what makes the scan and IMU frames resolvable at all. gz logs
    the same warning and honours the element regardless.
  * `SDFormat link [...] has a <sensor>, but URDF does not support this` -
    URDF has no sensor concept; the sensors run in gz, and rsp only needs the
    kinematic chain.
  * a KDL warning that the root link `base_link` has an inertia that KDL drops -
    again, only the chain is needed here; the inertia is what gz integrates.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# The robot does not depend on the `world` argument, so its SDF is read here
# with plain Python. sdformat_urdf parses this SDF in place: there is no URDF or
# xacro copy of the robot anywhere in this project, so there is nothing to drift
# out of sync with what the simulator actually spawns.
_DESCRIPTION_SHARE = get_package_share_directory('ral_description')
_MODEL_SDF = os.path.join(_DESCRIPTION_SHARE, 'models', 'ral_roomba', 'model.sdf')

with open(_MODEL_SDF, 'r') as handle:
    ROBOT_DESCRIPTION = handle.read()


def generate_launch_description() -> LaunchDescription:
    world = LaunchConfiguration('world')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='ral_apartment',
            description='World name: selects the SDF, the matching bridge config '
                        'and ground_truth_node\'s world_frame together.',
        ),

        # `model://ral_roomba` in the world resolves only if this points at the
        # directory *containing* the model directory. Prepended, so a caller's
        # own resource path still works.
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(_DESCRIPTION_SHARE, 'models')
            + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ),

        # -s: server only (no GUI). --headless-rendering: EGL offscreen context
        # for the gpu_lidar, verified working on this machine in Phase 0.
        ExecuteProcess(
            cmd=[
                'gz', 'sim', '-s', '-r', '--headless-rendering', '-v2',
                PathJoinSubstitution([
                    FindPackageShare('ral_description'), 'worlds', [world, '.sdf'],
                ]),
            ],
            output='screen',
        ),

        # parameter_bridge takes the config path as a parameter, so a launch
        # Substitution resolves normally here.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            parameters=[{
                'config_file': PathJoinSubstitution([
                    FindPackageShare('ral_bringup'), 'config',
                    ['bridge_roomba_', world, '.yaml'],
                ]),
                'use_sim_time': True,
            }],
            output='screen',
        ),

        # Publishes base_link -> rplidar_link / wheels / casters. The moving part
        # of the chain, odom -> base_link, comes from DiffDrive over the bridge.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': ROBOT_DESCRIPTION,
                'use_sim_time': True,
            }],
            output='screen',
        ),

        # Evaluation only. Publishes /ground_truth/odom and no TF.
        Node(
            package='ral_eval',
            executable='ground_truth_node',
            name='ground_truth_node',
            parameters=[{
                'use_sim_time': True,
                'world_frame': world,
                'model_frame': 'ral_roomba',
            }],
            output='screen',
        ),
    ])
