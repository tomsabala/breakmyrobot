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
Unit tests for the world <-> map conversion. No ROS graph, no simulator.

Every rotation case here uses a NON-ZERO yaw on purpose. The alignment this
project actually measures has yaw ~ 0 (the robot spawns axis-aligned), and at
yaw = 0 a transposed rotation, a sign error and a missing inverse are all
numerically identical to the correct answer. A test built from the production
numbers would pass on every one of those bugs, and the bug would surface
later as "AMCL is badly localized" in a phase that cannot see frames.py.
"""

import hashlib
import math

from ral_eval.frames import (
    frames_path_for_map,
    load_alignment_for_map,
    load_world_to_map,
    map_pose_to_world,
    map_to_world_tf,
    world_pose_to_map,
    wrap_to_pi,
)

import pytest

TOL = 1e-9


def _close(actual, expected, tol=TOL):
    assert len(actual) == len(expected)
    for index, (got, want) in enumerate(zip(actual, expected)):
        assert abs(got - want) <= tol, f'component {index}: {got} != {want}'


def test_pure_translation():
    """The production case: map origin offset from world, no rotation."""
    _close(world_pose_to_map(10.5, 6.5, 1.57, (0.5, 0.5, 0.0)), (10.0, 6.0, 1.57))


def test_rotation_is_applied_in_the_right_direction():
    """A quarter turn: transposing the rotation would give (2.0, 0.0, ...)."""
    _close(world_pose_to_map(3.0, 2.0, 0.0, (1.0, 2.0, math.pi / 2)),
           (0.0, -2.0, -math.pi / 2))


def test_map_to_world_tf_is_the_inverse_not_the_alignment():
    """
    The TF carries the world origin's pose in map, i.e. the inverse.

    Publishing world_to_map itself here would give (1.0, 2.0, +pi/2) - which
    is what a careless implementation returns, and what yaw = 0 hides.
    """
    _close(map_to_world_tf((1.0, 2.0, math.pi / 2)), (-2.0, 1.0, -math.pi / 2))


def test_round_trip():
    wtm = (1.0, 2.0, 0.7)
    pose = (4.2, -1.3, 2.9)
    _close(map_pose_to_world(*world_pose_to_map(*pose, wtm), wtm), pose)


def test_wrap_to_pi_folds_a_step_across_the_branch_cut():
    """+3.10 -> -3.10 is a +0.0832 rad step, not a -6.2 rad one."""
    assert abs(wrap_to_pi(-3.10 - 3.10) - 0.0831853071795865) <= 1e-9


def test_load_rejects_a_missing_file(tmp_path):
    missing = tmp_path / 'nope_frames.yaml'
    with pytest.raises(RuntimeError) as excinfo:
        load_world_to_map(str(missing))
    assert str(missing) in str(excinfo.value)


def test_load_rejects_a_file_without_the_key(tmp_path):
    path = tmp_path / 'frames.yaml'
    path.write_text('map: apartment\n')
    with pytest.raises(RuntimeError) as excinfo:
        load_world_to_map(str(path))
    assert str(path) in str(excinfo.value)


def test_load_rejects_a_non_numeric_yaw(tmp_path):
    path = tmp_path / 'frames.yaml'
    path.write_text('world_to_map: {x: 0.5, y: 0.5, yaw: unknown}\n')
    with pytest.raises(RuntimeError) as excinfo:
        load_world_to_map(str(path))
    assert str(path) in str(excinfo.value)


def test_load_accepts_a_measured_file(tmp_path):
    path = tmp_path / 'frames.yaml'
    path.write_text('world_to_map: {x: 0.5003, y: 0.4998, yaw: 0.0001}\n')
    _close(load_world_to_map(str(path)), (0.5003, 0.4998, 0.0001))


def _map_pair(tmp_path, grid: bytes, sealed_to: 'str | None'):
    """Write a minimal map + alignment pair, optionally sealed to a hash."""
    (tmp_path / 'apartment.pgm').write_bytes(grid)
    map_yaml = tmp_path / 'apartment.yaml'
    map_yaml.write_text('image: apartment.pgm\nresolution: 0.05\n')
    frames = 'world_to_map: {x: 0.5, y: 0.5, yaw: 0.0}\n'
    if sealed_to is not None:
        frames += f'map_sha256: {sealed_to}\n'
    (tmp_path / 'apartment_frames.yaml').write_text(frames)
    return str(map_yaml)


def test_frames_path_sits_beside_the_map_it_belongs_to():
    assert frames_path_for_map('/maps/apartment.yaml') == '/maps/apartment_frames.yaml'


def test_alignment_loads_when_it_was_measured_against_this_grid(tmp_path):
    grid = b'P5 1 1 255 \x00'
    digest = hashlib.sha256(grid).hexdigest()
    _close(load_alignment_for_map(_map_pair(tmp_path, grid, digest)), (0.5, 0.5, 0.0))


def test_alignment_measured_against_another_grid_is_refused(tmp_path):
    """The stale pair: a new .pgm beside the alignment of the old one."""
    stale = hashlib.sha256(b'the map this alignment was measured against').hexdigest()
    with pytest.raises(RuntimeError) as excinfo:
        load_alignment_for_map(_map_pair(tmp_path, b'P5 1 1 255 \xff', stale))
    assert 'measured against a different grid' in str(excinfo.value)


def test_unsealed_alignment_is_refused(tmp_path):
    """Between capture and map_sanity there is no evidence of which grid it is."""
    with pytest.raises(RuntimeError) as excinfo:
        load_alignment_for_map(_map_pair(tmp_path, b'P5 1 1 255 \x00', None))
    assert 'map_sanity' in str(excinfo.value)
