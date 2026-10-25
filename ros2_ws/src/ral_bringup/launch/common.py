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
Launch-time helpers shared by every <robot>_nav.launch.py in this package.

Not a launch file: it declares nothing and is never passed to `ros2 launch`.
It exists so the map/alignment refusal has exactly ONE implementation. Both
navigation stacks need the same guarantee - that the alignment they are about
to score a run against belongs to the grid in use - and two copies of that
check would eventually disagree about which map is acceptable.

A launch file imports it by adding its own directory to sys.path first; the
launch system executes launch files by path, not as a package, so a sibling
module is not importable any other way.
"""

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ral_eval.frames import load_alignment_for_map, map_to_world_tf


def alignment_for_context(context) -> 'tuple[float, float, float]':
    """
    Return the measured world -> map alignment for the `map` argument, or refuse.

    load_alignment_for_map raises if the file is missing, malformed, unsealed
    or measured against a different grid, naming the file and the command that
    fixes it. That is deliberate - an identity fallback, or an alignment
    belonging to some other .pgm, publishes a frame offset that every later
    number reports as localization error.
    """
    return load_alignment_for_map(LaunchConfiguration('map').perform(context))


def map_to_world_static_tf(context, *args, **kwargs):
    """
    Build the display-only map -> world transform from the measured alignment.

    An OpaqueFunction because the `map` argument has to be resolved to a real
    string before the alignment file beside it can be read. Callers add it
    FIRST, outside any TimerAction, so a bad alignment stops the launch before
    gz is spawned rather than ten seconds into a run.

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
    """
    x, y, yaw = map_to_world_tf(alignment_for_context(context))
    return [Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_world_static_tf',
        arguments=['--x', f'{x:.6f}', '--y', f'{y:.6f}', '--z', '0',
                   '--yaw', f'{yaw:.6f}', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'map', '--child-frame-id', 'world'],
        parameters=[{'use_sim_time': True}],
        output='screen')]


def spawn_flag_path(robot: str, instance: int) -> str:
    """
    Return the file robot_sim.launch.py touches once the body is in the world.

    car_nav.launch.py waits for it before starting the autopilot: ArduPilot's
    FDM peer is a plugin INSIDE the spawned model, and a SITL that starts
    first never completes the handshake.

    A FILE, not a gz query: a model carrying a lock-stepped plugin stops the
    server stepping the moment it is spawned, so its topics never appear until
    the autopilot connects, and waiting on gz for permission to start the
    autopilot is a deadlock. Keyed by instance as well as robot, so two runs
    in two ROS domains cannot read each other's.
    """
    return f'/tmp/ral_spawned_{robot}_{instance}.flag'
