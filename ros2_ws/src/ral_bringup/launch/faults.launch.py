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
Everything fault injection needs, and nothing else.

What runs
---------
  * ral_faults/scan_fault_relay - the sole publisher of /scan, and the lidar
    injection point. It lives here rather than in roomba_nav.launch.py because
    its `seed` comes from the scenario, and the scenario's fault parameters
    belong together in one file.
  * ral_faults/fault_manager - serves every /faults/* service, owns the
    FaultEvent lifecycle and the latched /faults/active snapshot.
  * a SECOND ros_gz_bridge/parameter_bridge, named ros_gz_fault_bridge, for
    the five gz interfaces the faults are injected through.

Why a second bridge node, and why no YAML
-----------------------------------------
The bridge's YAML schema carries TOPICS only (verified; it is why
config/bridge_roomba_<world>.yaml says the Phase 4 services must be CLI
arguments). Services therefore have to be `<name>@<ros_srv_type>` CLI
arguments, and a CLI argument cannot RENAME: the ROS name of a bridged service
or ROS->gz topic equals its gz name. That is why the manager publishes on
/world/<world>/wrench/persistent rather than a tidier ROS name.

Keeping all five on ONE extra node is a deliberate trade: the alternative is a
second per-world YAML for the two wrench topics plus a CLI node for the three
services, i.e. another file that must be kept in step with
bridge_roomba_<world>.yaml every time a world is added.

The node NAME must differ from roomba_sim.launch.py's `ros_gz_bridge`. Two
nodes with the same name in one domain is a name collision: ROS 2 does not
refuse it, it just makes parameters, logs and `ros2 node info` ambiguous.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    world = LaunchConfiguration('world')
    robot = LaunchConfiguration('robot')
    seed = LaunchConfiguration('seed')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='ral_apartment',
            description='World name: the gz service and wrench topic paths embed it.'),
        DeclareLaunchArgument(
            'robot', default_value='roomba',
            description='Robot key in ral_faults/config/capabilities.yaml: selects '
                        'the gz model the wrench and the teleport address, and which '
                        'faults are supported at all.'),
        DeclareLaunchArgument(
            'seed', default_value='1',
            description="The scenario's seed. Governs scan_fault_relay's noise and "
                        'dropout only; nav2_amcl exposes no seed.'),

        Node(
            package='ral_faults',
            executable='scan_fault_relay',
            name='scan_fault_relay',
            parameters=[{
                'use_sim_time': True,
                # A bare LaunchConfiguration is a STRING, and the node declares
                # `seed` as an integer; without this the node dies at start-up
                # with a parameter type error.
                'seed': ParameterValue(seed, value_type=int),
            }],
            output='screen'),

        Node(
            package='ral_faults',
            executable='fault_manager',
            name='fault_manager',
            parameters=[{
                'use_sim_time': True,
                'world': world,
                'robot': robot,
            }],
            output='screen'),

        # Each argument is a substitution LIST, so `world` is resolved at
        # launch time inside the gz path. The gz request/reply types are
        # omitted: parameter_bridge falls through to add_service_bridge with
        # empty gz type names and the ros_gz_interfaces factory treats empty
        # as "match". If a launch-time error ever says otherwise, the explicit
        # forms are @gz.msgs.EntityFactory@gz.msgs.Boolean (create),
        # @gz.msgs.Entity@gz.msgs.Boolean (remove) and
        # @gz.msgs.Pose@gz.msgs.Boolean (set_pose).
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_fault_bridge',
            parameters=[{'use_sim_time': True}],
            arguments=[
                ['/world/', world, '/create@ros_gz_interfaces/srv/SpawnEntity'],
                ['/world/', world, '/remove@ros_gz_interfaces/srv/DeleteEntity'],
                ['/world/', world, '/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
                ['/world/', world,
                 '/wrench/persistent@ros_gz_interfaces/msg/EntityWrench]'
                 'gz.msgs.EntityWrench'],
                ['/world/', world,
                 '/wrench/clear@ros_gz_interfaces/msg/Entity]gz.msgs.Entity'],
            ],
            output='screen'),
    ])
