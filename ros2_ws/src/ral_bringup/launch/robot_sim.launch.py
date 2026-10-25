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
Bring one robot up in a simulated world: gz, bridge, TF, truth republisher.

Two launch arguments select everything that must stay consistent or sensor
streams silently vanish.

`world` selects:
  * the world SDF                    share/ral_description/worlds/<world>.sdf
  * ground_truth_node's world_frame  (gz PosePublisher stamps the world name)

`robot` selects, through ral_bringup/config/robots/<robot>.yaml:
  * the gz model spawned             models/<description_model>/model.sdf
  * the bridge config                share/ral_bringup/config/<bridge_config>
    (three of its gz topic paths embed the world name)
  * ground_truth_node's model_frame  (the gz entity name)

The body is SPAWNED here, through /world/<world>/create, rather than included
in the world SDF: a static include would put every robot in every run of that
world, and the car standing in the Roomba's lidar would invalidate the
committed map and every baseline measured against it.

The spawn POSE comes from the scenario, through `spawn_x`/`spawn_y`/
`spawn_yaw`, so the initial pose a scenario declares and the pose the body is
actually standing at cannot disagree. Empty values mean "use the profile's
`spawn` block"; z is always the profile's, because it is a property of the
body and not of the experiment.

There is no map, no AMCL, no Nav2, no autopilot and no fault injection here -
those belong to <robot>_nav.launch.py and faults.launch.py.

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
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ral_eval.robot_profile import load_profile, profile_path

# The launch system executes launch files by PATH, not as a package, so a
# sibling module needs its directory on sys.path. spawn_flag_path is shared
# with car_nav.launch.py, which waits on the file this launch touches.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import spawn_flag_path                                   # noqa: E402

# The ArduPilotPlugin's listening port, as authored in ral_car/model.sdf. It is
# rewritten per SITL instance below so two runs in two ROS domains cannot drive
# each other's vehicle through the same host-global UDP port.
DEFAULT_FDM_PORT = 9002


def _robot_description(description_model: str) -> str:
    """
    Read the model SDF as text for robot_state_publisher.

    sdformat_urdf parses this SDF in place: there is no URDF or xacro copy of
    any robot in this project, so there is nothing to drift out of sync with
    what the simulator actually spawns.

    Read inside the OpaqueFunction, not at import: at import, a missing or
    uninstalled ral_description fails while the launch file is being *parsed*,
    which surfaces as a bare FileNotFoundError with no indication of what the
    launch system was doing.
    """
    share = get_package_share_directory('ral_description')
    model_sdf = os.path.join(share, 'models', description_model, 'model.sdf')
    try:
        with open(model_sdf, 'r') as handle:
            return handle.read()
    except OSError as exc:
        raise RuntimeError(
            f'cannot read the robot description at {model_sdf}: {exc}. '
            'Build and source the workspace (scripts/build.sh) so ral_description '
            'is installed.') from exc


def _spawn_pose(profile: dict, context) -> 'tuple[float, float, float, float]':
    """(x, y, z, yaw): the scenario's where it gave one, the profile's elsewhere."""
    spawn = profile['spawn']

    def _or_profile(name: str, key: str) -> float:
        given = LaunchConfiguration(name).perform(context).strip()
        return float(given) if given else float(spawn[key])

    return (_or_profile('spawn_x', 'x'), _or_profile('spawn_y', 'y'),
            float(spawn['z']), _or_profile('spawn_yaw', 'yaw'))


def _robot_actions(context, *args, **kwargs):
    """
    Everything that depends on WHICH robot is being launched.

    An OpaqueFunction because the profile has to be read at launch time: the
    bridge config name, the gz entity name, the description directory and the
    spawn pose all come out of it, and a LaunchConfiguration cannot be used as
    a dictionary key before it is resolved.
    """
    bringup_share = get_package_share_directory('ral_bringup')
    world = LaunchConfiguration('world').perform(context)
    robot = LaunchConfiguration('robot').perform(context)
    instance_given = LaunchConfiguration('sitl_instance').perform(context).strip()

    profile = load_profile(profile_path(bringup_share, robot))
    model = profile['model']
    x, y, z, yaw = _spawn_pose(profile, context)

    description = _robot_description(profile['description_model'])

    # The SDF the spawn request carries. Written to /tmp rather than passed
    # inline because `ros2 run ros_gz_sim create` takes a file, and because a
    # per-robot copy on disk is what makes a failed spawn inspectable.
    #
    # The FDM port is substituted here, not authored per instance, so the
    # autopilot's own --instance arithmetic and the plugin's listening port are
    # derived from ONE number. A mismatch is silent: SITL waits for a frame
    # that is being sent to another port, and the vehicle never arms.
    instance = int(instance_given) if instance_given else \
        int(os.environ.get('ROS_DOMAIN_ID', '0')) % 10
    spawn_sdf = description.replace(
        f'<fdm_port_in>{DEFAULT_FDM_PORT}</fdm_port_in>',
        f'<fdm_port_in>{DEFAULT_FDM_PORT + 10 * instance}</fdm_port_in>')
    spawn_path = f'/tmp/ral_spawn_{robot}.sdf'
    with open(spawn_path, 'w') as handle:
        handle.write(spawn_sdf)
    # The sentinel a dependent process waits on. It is a FILE rather than a gz
    # query because a gz query is the one thing that does not work here: a
    # model carrying a lock-stepped ArduPilotPlugin stops the server stepping
    # the moment it is spawned, so its sensor topics never appear and
    # `gz topic -l` never lists them until the autopilot connects. Waiting on
    # gz for permission to start the autopilot is a deadlock (MEASURED: the
    # waiter polled for 240 s while gz waited for the FDM peer). Removed here,
    # before the spawn, so a previous run's sentinel cannot satisfy this one.
    spawned_flag = spawn_flag_path(robot, instance)
    if os.path.exists(spawned_flag):
        os.remove(spawned_flag)

    return [
        # parameter_bridge takes the config path as a parameter.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            parameters=[{
                'config_file': os.path.join(
                    bringup_share, 'config',
                    profile['bridge_config'].format(world=world)),
                'use_sim_time': True,
            }],
            output='screen',
        ),

        # Publishes the fixed joints on /tf_static and the revolute ones on
        # /tf from the bridged /joint_states - so a broken joint_state bridge
        # row removes those frames while the static ones stay. The moving part
        # of the chain is the robot's own estimate and comes from elsewhere:
        # odom -> base_link over the bridge for a DiffDrive robot,
        # ekf_local -> base_link from ral_eval's ekf_local_tf for the car.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': description,
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
                'model_frame': model,
            }],
            output='screen',
        ),

        # A bounded retry, because `ros2 run ros_gz_sim create` fails outright
        # if the gz server is not yet serving /world/<world>/create - and gz
        # start-up time is a property of the host, not of this launch file. A
        # fixed TimerAction long enough for this machine would be a hidden
        # assumption about another; raise the retry count instead.
        ExecuteProcess(
            cmd=['bash', '-lc',
                 'for attempt in $(seq 1 30); do '
                 f'ros2 run ros_gz_sim create -world {world} -file {spawn_path} '
                 f'-name {model} -x {x} -y {y} -z {z} -Y {yaw} && '
                 f'touch {spawned_flag} && exit 0; '
                 'sleep 1; done; '
                 f'echo "spawn failed: /world/{world}/create never accepted '
                 f'{model}" >&2; exit 1'],
            output='screen',
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    world = LaunchConfiguration('world')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='ral_apartment',
            description='World name: selects the SDF and ground_truth_node\'s '
                        'world_frame together.',
        ),
        DeclareLaunchArgument(
            'robot',
            default_value='roomba',
            description='Robot key: selects ral_bringup/config/robots/<robot>.yaml, '
                        'and with it the gz model spawned, the bridge config and the '
                        'truth model frame.',
        ),
        DeclareLaunchArgument(
            'spawn_x', default_value='',
            description='World-frame spawn x. Empty means the profile\'s spawn.x; '
                        'the scenario runner passes its declared initial pose, so '
                        'the declared and the actual pose cannot disagree.',
        ),
        DeclareLaunchArgument(
            'spawn_y', default_value='',
            description='World-frame spawn y. Empty means the profile\'s spawn.y.',
        ),
        DeclareLaunchArgument(
            'spawn_yaw', default_value='',
            description='World-frame spawn yaw in rad. Empty means the profile\'s '
                        'spawn.yaw.',
        ),
        DeclareLaunchArgument(
            'sitl_instance', default_value='',
            description='ArduPilot SITL instance number; empty derives it from '
                        'ROS_DOMAIN_ID. Only the FDM port in the spawned SDF depends '
                        'on it, and only for a robot carrying ArduPilotPlugin.',
        ),

        # `model://...` inside the world resolves only if this points at the
        # directory *containing* the model directories, and `create` resolves
        # its own `model://` references the same way. Prepended, so a caller's
        # own resource path still works.
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(get_package_share_directory('ral_description'), 'models')
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

        OpaqueFunction(function=_robot_actions),
    ])
