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
The one implementation of the gz `world` <-> SLAM `map` conversion.

`world_to_map` is the pose of the **`map` frame origin expressed in the gz
`world` frame**. slam_toolbox registers its first scan at that scan's
ODOMETRIC pose, so `map` -> `odom` starts as identity and `map` lands on the
odom frame origin - which, for a robot that has not moved since it spawned,
is also its true pose. capture_world_to_map measures it there and checks
both halves of that coincidence; see its module docstring.

Three consumers import this module: ral_bringup's roomba_nav.launch.py,
which inverts the alignment to publish the display-only map -> world
transform, map_build's capture node, which writes it, and Phase 3's metrics,
which map truth into `map` to compare it against the estimate. One
implementation, because they must agree exactly - a sign error that affects
only one of them shows up as a localization error that is really a frame
error, and is attributed to AMCL.

Consumers that have a map path should call load_alignment_for_map, which
additionally refuses an alignment measured against a different grid.

Note the proof harness in scripts/checks/ deliberately does NOT import this
module; it re-derives the same four lines inline, because a verifier that
imports the code under test shares its bugs.
"""

import hashlib
import math
import os

import yaml


def wrap_to_pi(angle: float) -> float:
    """Fold an angle into [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_of_yaw(yaw: float):
    """Return the unit quaternion (x, y, z, w) for a rotation of `yaw` about +Z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def yaw_of_quaternion(q) -> float:
    """Return the yaw about +Z of a geometry_msgs/Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def load_world_to_map(path: str) -> 'tuple[float, float, float]':
    """
    Read a measured (x, y, yaw) alignment from a <map>_frames.yaml file.

    Raises RuntimeError naming the path and the regeneration command if the
    file is missing, lacks the `world_to_map` mapping, or carries a
    non-numeric value. There is deliberately NO identity default: publishing
    an assumed identity alignment would turn a constant frame offset into
    what every downstream metric reports as localization error.
    """
    regenerate = 'regenerate it with: ros2 launch ral_bringup map_build.launch.py'
    try:
        with open(path, 'r') as handle:
            data = yaml.safe_load(handle)
    except OSError as exc:
        raise RuntimeError(
            f'cannot read the world->map alignment at {path}: {exc}. '
            f'It belongs to the map in the same directory; {regenerate}') from exc

    if not isinstance(data, dict) or not isinstance(data.get('world_to_map'), dict):
        raise RuntimeError(
            f'{path} has no `world_to_map` mapping. The alignment must be measured, '
            f'never assumed; {regenerate}')

    wtm = data['world_to_map']
    values = []
    for key in ('x', 'y', 'yaw'):
        value = wtm.get(key)
        # bool is an int subclass; a YAML `yaw: true` is not an angle.
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise RuntimeError(
                f'{path}: world_to_map.{key} is {value!r}, which is not a number. '
                f'{regenerate}')
        values.append(float(value))
    return (values[0], values[1], values[2])


def frames_path_for_map(map_yaml_path: str) -> str:
    """Alignment file belonging to a map yaml: <map>.yaml -> <map>_frames.yaml."""
    root, _ = os.path.splitext(map_yaml_path)
    return root + '_frames.yaml'


def grid_sha256(map_yaml_path: str) -> str:
    """SHA-256 of the occupancy grid image a map yaml points at."""
    with open(map_yaml_path, 'r') as handle:
        meta = yaml.safe_load(handle)
    image = os.path.join(os.path.dirname(map_yaml_path), meta['image'])
    with open(image, 'rb') as handle:
        return hashlib.sha256(handle.read()).hexdigest()


def load_alignment_for_map(map_yaml_path: str) -> 'tuple[float, float, float]':
    """
    Load the alignment measured for THIS grid, refusing a stale pair.

    `map_sha256` in <map>_frames.yaml is the hash of the .pgm the alignment
    was measured against, sealed by scripts/checks/map_sanity.py when it
    accepts a map. A new grid beside an old alignment is the one failure mode
    the alignment cannot survive and nothing else can detect: it offsets
    truth from the estimate by a constant that reads as localization error,
    in a stack that is otherwise working. So it is checked here, on the path
    every consumer already takes, rather than in prose.

    The field is absent only between capture_world_to_map writing the file
    and map_sanity.py accepting the saved map; that window is a mapping
    session, and the error says so.
    """
    frames_path = frames_path_for_map(map_yaml_path)
    wtm = load_world_to_map(frames_path)
    with open(frames_path, 'r') as handle:
        recorded = (yaml.safe_load(handle) or {}).get('map_sha256')
    if recorded is None:
        raise RuntimeError(
            f'{frames_path} has no `map_sha256`, so it cannot be shown to belong to '
            f'{map_yaml_path}. Seal the pair with: python3 scripts/checks/map_sanity.py')
    # str(): an all-digit hash is a YAML integer, and a hand-edited one can be
    # anything at all. Both must read as a mismatch, not as a TypeError from
    # the message that is trying to explain the mismatch.
    recorded = str(recorded)
    actual = grid_sha256(map_yaml_path)
    if recorded != actual:
        raise RuntimeError(
            f'{frames_path} was measured against a different grid: it records '
            f'map_sha256 {recorded[:16]}... and {map_yaml_path} hashes to '
            f'{actual[:16]}.... A stale pair biases every estimate-error metric by a '
            'constant offset. Rebuild both together: ros2 launch ral_bringup '
            'map_build.launch.py, then python3 scripts/checks/map_sanity.py')
    return wtm


def world_pose_to_map(x: float, y: float, yaw: float,
                      wtm: 'tuple[float, float, float]') -> 'tuple[float, float, float]':
    """Express a pose given in the gz `world` frame in the `map` frame."""
    wx, wy, wyaw = wtm
    dx = x - wx
    dy = y - wy
    cos_w = math.cos(wyaw)
    sin_w = math.sin(wyaw)
    return (dx * cos_w + dy * sin_w,
            -dx * sin_w + dy * cos_w,
            wrap_to_pi(yaw - wyaw))


def map_pose_to_world(x: float, y: float, yaw: float,
                      wtm: 'tuple[float, float, float]') -> 'tuple[float, float, float]':
    """Express a pose given in the `map` frame in the gz `world` frame."""
    wx, wy, wyaw = wtm
    cos_w = math.cos(wyaw)
    sin_w = math.sin(wyaw)
    return (wx + x * cos_w - y * sin_w,
            wy + x * sin_w + y * cos_w,
            wrap_to_pi(yaw + wyaw))


def map_to_world_tf(wtm: 'tuple[float, float, float]') -> 'tuple[float, float, float]':
    """
    Return the pose of the `world` origin in `map`, for a map->world TF.

    A static_transform_publisher with `--frame-id map --child-frame-id world`
    must carry the child's pose in the parent, i.e. the INVERSE of
    world_to_map - not world_to_map itself. The two are numerically identical
    whenever yaw is 0, which is exactly the case the production measurement
    produces (the robot spawns axis-aligned), so a sign error here is
    invisible in the real data. test_frames.py therefore exercises this with
    a non-zero yaw.
    """
    return world_pose_to_map(0.0, 0.0, 0.0, wtm)
