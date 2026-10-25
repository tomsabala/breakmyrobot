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
Bring up the car: gz, fault injection, ArduRover SITL, MAVROS and the EKF frame.

What runs
---------
  * robot_sim.launch.py - gz, the ROS/gz bridge, robot_state_publisher, the
    runtime spawn of ral_car and ral_eval's ground_truth_node (evaluation
    only, no TF).
  * faults.launch.py - ral_faults/scan_fault_relay (the sole publisher of
    /scan), ral_faults/fault_manager (every /faults/* service, gated on
    config/robots/car.yaml, which refuses scan_degradation for this platform)
    and the gz create/remove/set_pose/wrench bridge.
  * `ardurover`, the ArduPilot SITL firmware binary, with
    config/rover_ackermann.parm. It is the autopilot: it plans, it controls
    the steering and the throttle, and it owns the EKF3 state estimate.
  * mavros_node, the only link between that autopilot and ROS.
  * ral_eval/ekf_local_tf, which measures the EKF origin against truth once at
    the spawn pose, publishes it on /frame_alignment, and broadcasts
    map -> ekf_local -> base_link afterwards from the robot's own estimate.
  * the display-only map -> world transform, as for the Roomba.

There is NO Nav2 here, no map_server, no AMCL and no costmap: this platform's
planner, controller and state estimator all run inside the autopilot. The
`map` argument exists only because the EKF origin has to be expressed in the
same `map` frame every metric is scored in, and because the display transform
needs the same measured alignment.

Ports
-----
SITL's FDM port and its MAVLink port are HOST-GLOBAL under `--network host`,
so two runs in two ROS domains would otherwise drive each other's vehicle.
Both are derived from one instance number - `sitl_instance`, defaulting to
ROS_DOMAIN_ID % 10 - and the same number is forwarded to robot_sim.launch.py,
which rewrites the ArduPilotPlugin's <fdm_port_in> in the spawned SDF. A
mismatch between the two sides is silent: SITL waits for a frame being sent to
another port and the vehicle never arms.

`sim_vehicle.py` is deliberately not used. It spawns a MAVProxy tree whose
orphans hold those host-global UDP ports after a run, and every argument it
would choose is pinned here instead.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ral_eval.robot_profile import load_profile

# See roomba_nav.launch.py: launch files are executed by path, so a sibling
# module needs its directory on sys.path. The map/alignment refusal is shared
# with that file and has exactly one implementation.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import (                                                 # noqa: E402
    alignment_for_context,
    map_to_world_static_tf,
    spawn_flag_path,
)

# Base ports, offset by 10 per SITL instance. 9002 is ArduPilotPlugin's
# default FDM port and the literal authored in ral_car/model.sdf; 14550 is
# ArduPilot's default GCS port, which MAVROS listens on here.
FDM_PORT_BASE = 9002
MAVLINK_PORT_BASE = 14550


def _instance(context) -> int:
    given = LaunchConfiguration('sitl_instance').perform(context).strip()
    if given:
        return int(given)
    return int(os.environ.get('ROS_DOMAIN_ID', '0')) % 10


def _sitl(context, *args, **kwargs):
    """
    Start the autopilot process itself.

    An OpaqueFunction because the ports and the working directory are derived
    from the instance number, which is a LaunchConfiguration.

    --wipe makes every run start from rover_ackermann.parm alone, with nothing
    carried over from the previous run's EEPROM; without it a parameter changed
    by one run silently governs the next. cwd is a per-instance directory so
    eeprom.bin and the dataflash logs land somewhere bounded instead of in the
    repository root.

    --home is Canberra, ArduPilot's own default SITL home. Its absolute value
    is irrelevant here: EKF3's origin is set at the first fix wherever that is,
    and ekf_local_tf MEASURES the offset between that origin and the gz world
    rather than assuming any particular home.

    SITL WAITS FOR THE MODEL. The body is spawned at runtime, and ArduPilot's
    JSON backend handshakes with ArduPilotPlugin inside that model. MEASURED
    failure when SITL started first: the plugin was not there to answer, SITL
    kept resending servo packets with the same frame counter, and when the
    model did appear the plugin logged "Duplicate input frame" while SITL
    logged "No JSON sensor message received, resending servos" - a handshake
    that never recovered. Because <lock_step> is on, gz then stopped stepping
    entirely: /clock went silent and every bring-up topic timed out. It is a
    RACE, not a certainty - the same launch sometimes recovers - which is
    exactly why it is closed with a readiness condition rather than left to
    chance. The condition is the SENTINEL FILE robot_sim.launch.py touches
    once gz has accepted the model - not a gz query, because a model carrying
    a lock-stepped plugin stops the server stepping the moment it is spawned,
    so its topics never appear until the autopilot connects and waiting on gz
    for permission to start the autopilot is a deadlock (MEASURED: a topic
    waiter polled for 240 s while gz waited for its FDM peer).
    """
    bringup_share = get_package_share_directory('ral_bringup')
    instance = _instance(context)
    workdir = f'/tmp/ral_sitl_{instance}'
    os.makedirs(workdir, exist_ok=True)
    mavlink_port = MAVLINK_PORT_BASE + 10 * instance
    robot = LaunchConfiguration('robot').perform(context)
    profile = load_profile(os.path.join(bringup_share, 'config', 'robots',
                                        f'{robot}.yaml'))
    # The profile names the parameter file, so the autopilot configuration a
    # run flew and the one its env.json hashes are the same file by
    # construction rather than by two places agreeing on a filename.
    parm = os.path.join(bringup_share, 'config', profile['autopilot_parm'])
    flag = spawn_flag_path(robot, instance)
    sitl = ' '.join(['ardurover', '--model', 'JSON', '--speedup', '1',
                     '--defaults', parm, '--wipe', '--instance', str(instance),
                     '--home', '-35.363262,149.165237,584,0',
                     f'--serial0=udpclient:127.0.0.1:{mavlink_port}'])
    return [ExecuteProcess(
        cmd=['bash', '-lc',
             f'for attempt in $(seq 1 120); do '
             f'[ -f {flag} ] && break; sleep 1; done; '
             f'[ -f {flag} ] || '
             f'echo "WARNING: {profile["model"]} was never spawned ({flag} absent); '
             'starting SITL anyway, and the FDM handshake will not complete" >&2; '
             f'exec {sitl}'],
        cwd=workdir,
        output='screen')]


def _mavros(context, *args, **kwargs):
    """MAVROS, attached to this instance's MAVLink port."""
    bringup_share = get_package_share_directory('ral_bringup')
    mavlink_port = MAVLINK_PORT_BASE + 10 * _instance(context)
    return [Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        parameters=[
            {
                'use_sim_time': True,
                # udp://:PORT@ binds locally and learns the peer from the
                # first packet, which is what SITL's `udpclient` side needs.
                'fcu_url': f'udp://:{mavlink_port}@',
                'tgt_system': 1,
                'tgt_component': 1,
            },
            os.path.join(bringup_share, 'config', 'mavros_car.yaml'),
        ],
        output='screen')]


# MAVLink message id -> Hz, requested from the autopilot once MAVROS is up.
# MEASURED need: with rover_ackermann.parm's SR0_* stream rates alone,
# /mavros/local_position/odom reached the bag at 1 Hz - fourteen messages
# across a fourteen-second mission - and /mavros/estimator_status carried no
# messages at all. At 1 Hz, an estimate is paired with a truth sample up to
# 0.8 m of travel away at the 1.0 m/s cruise speed, so est_error_* reports
# sampling interval rather than localization error. A MAVLink2
# SET_MESSAGE_INTERVAL request does take effect: the same topic measured
# 9.9 Hz immediately after one.
#
#   32  LOCAL_POSITION_NED      the estimate every truth comparison uses
#   62  NAV_CONTROLLER_OUTPUT   wp_dist, the FCU's own arrival evidence
#  230  ESTIMATOR_STATUS        the EKF health flags metrics would report -
#                               REQUESTED AND REFUSED: this ArduRover answers
#                               MessageInterval_Response(success=False) for
#                               it, and /mavros/estimator_status is in every
#                               bag with zero messages. The request stays so
#                               the refusal is visible in launch.log rather
#                               than inferred from an empty topic, and
#                               metrics.py reports the two uncertainty
#                               metrics as null naming it.
STREAM_INTERVALS = ((32, 10.0), (62, 5.0), (230, 2.0))


def _stream_rates(context, *args, **kwargs):
    """
    Ask the autopilot for the telemetry rates the evidence pipeline needs.

    A one-shot process rather than a node: it is three service calls, and it
    must happen after MAVROS has both started and connected, which `ros2
    service call` blocking on the service name expresses directly. Each
    failure is printed rather than swallowed - a run whose estimate arrived at
    1 Hz should say so in launch.log, because every truth-vs-estimate number
    in it is then a sampling artefact.
    """
    calls = '; '.join(
        f'ros2 service call /mavros/set_message_interval '
        f'mavros_msgs/srv/MessageInterval '
        f'"{{message_id: {message_id}, message_rate: {rate}}}" '
        f'|| echo "WARNING: could not set the rate of MAVLink message '
        f'{message_id}" >&2'
        for message_id, rate in STREAM_INTERVALS)
    return [ExecuteProcess(
        cmd=['bash', '-lc',
             'for attempt in $(seq 1 120); do '
             'ros2 service list 2>/dev/null '
             '| grep -qx /mavros/set_message_interval && break; '
             'sleep 1; done; '
             + calls],
        output='screen')]


def _ekf_local_tf(context, *args, **kwargs):
    """
    Start the node that measures the EKF origin and owns the estimate's TF chain.

    The committed world -> map alignment is resolved HERE and passed as three
    doubles, so the refusal on a stale or unsealed map/alignment pair happens
    once, in one place, before anything is spawned.

    The spawn pose is forwarded for the same reason the sim launch takes it:
    the node predicts the EKF origin AT THE POSE THE BODY WAS SPAWNED AT, and
    predicting the profile default while the scenario put the body elsewhere
    refuses a healthy run.
    """
    bringup_share = get_package_share_directory('ral_bringup')
    robot = LaunchConfiguration('robot').perform(context)
    x, y, yaw = alignment_for_context(context)
    return [Node(
        package='ral_eval',
        executable='ekf_local_tf',
        name='ekf_local_tf',
        parameters=[{
            'use_sim_time': True,
            'robot': robot,
            'profile_path': os.path.join(bringup_share, 'config', 'robots',
                                         f'{robot}.yaml'),
            'world_to_map_x': x,
            'world_to_map_y': y,
            'world_to_map_yaw': yaw,
            # ParameterValue(..., str): a bare LaunchConfiguration is typed by
            # its CONTENT, so '1.0' would arrive as a double and the node -
            # which declares these as strings, because '' is how the launch
            # layer says "not given" - would refuse to start.
            'spawn_x': ParameterValue(LaunchConfiguration('spawn_x'),
                                      value_type=str),
            'spawn_y': ParameterValue(LaunchConfiguration('spawn_y'),
                                      value_type=str),
            'spawn_yaw': ParameterValue(LaunchConfiguration('spawn_yaw'),
                                        value_type=str),
        }],
        output='screen')]


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('ral_bringup')

    world = LaunchConfiguration('world')
    robot = LaunchConfiguration('robot')
    seed = LaunchConfiguration('seed')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='ral_apartment',
            description='World name: selects the SDF, the bridge config and the '
                        'truth frame name together.'),
        DeclareLaunchArgument(
            'robot', default_value='car',
            description='Robot key: selects ral_bringup/config/robots/<robot>.yaml. '
                        'Only a profile whose mission_driver is mavros_guided makes '
                        'sense here; roomba_nav.launch.py is the Nav2 stack.'),
        DeclareLaunchArgument(
            'seed', default_value='1',
            description="The scenario's seed, forwarded to scan_fault_relay's RNG. "
                        'Nothing on this platform consumes the lidar, so it governs '
                        'a stream that is recorded as evidence and read by nothing.'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(bringup_share, 'maps', 'apartment.yaml'),
            description='Static map. No node on this platform localizes against it: '
                        'it supplies the measured world->map alignment that the EKF '
                        'origin and the display transform are expressed in, and its '
                        '<map>_frames.yaml must have been measured for this grid.'),
        DeclareLaunchArgument(
            'spawn_x', default_value='',
            description="World-frame spawn x. Empty means the profile's spawn.x."),
        DeclareLaunchArgument(
            'spawn_y', default_value='',
            description="World-frame spawn y. Empty means the profile's spawn.y."),
        DeclareLaunchArgument(
            'spawn_yaw', default_value='',
            description="World-frame spawn yaw in rad. Empty means the profile's "
                        'spawn.yaw.'),
        DeclareLaunchArgument(
            'sitl_instance', default_value='',
            description='ArduPilot SITL instance; empty derives it from '
                        'ROS_DOMAIN_ID. Decides the FDM and MAVLink ports, which are '
                        'host-global under --network host.'),

        # First, so that an alignment that is missing, unsealed or measured
        # against a different grid aborts the launch before anything is
        # spawned, rather than after gz and SITL are up.
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
                'sitl_instance': LaunchConfiguration('sitl_instance'),
            }.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'faults.launch.py')),
            launch_arguments={'world': world, 'robot': robot, 'seed': seed}.items()),

        # SITL is started immediately and not behind a timer: ArduPilotPlugin
        # and the autopilot find each other through a UDP handshake that either
        # side may start, and lock_step means neither runs ahead of the other.
        OpaqueFunction(function=_sitl),
        OpaqueFunction(function=_mavros),
        OpaqueFunction(function=_stream_rates),
        OpaqueFunction(function=_ekf_local_tf),
    ])
