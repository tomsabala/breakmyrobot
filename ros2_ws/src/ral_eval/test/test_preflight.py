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
The goal pre-flight, on synthetic grids, without a simulator.

`preflight_goal` is the ONLY path to MissionState `unreachable`, and until now
only its occupied branch had ever run. The rotated-origin branch in particular
is correct but was never exercised: nav_msgs/OccupancyGrid permits a rotated
origin, apartment.yaml does not use one, and an indexing error there would
refuse or accept the wrong cell in silence.
"""

import math

from nav_msgs.msg import OccupancyGrid
from ral_eval.run_scenario import preflight_goal

RADIUS_M = 0.18
OCCUPIED = 65
RES = 0.05


def _grid(width: int, height: int, origin=(0.0, 0.0), yaw: float = 0.0,
          fill: int = 0) -> OccupancyGrid:
    grid = OccupancyGrid()
    grid.info.resolution = RES
    grid.info.width = width
    grid.info.height = height
    grid.info.origin.position.x = float(origin[0])
    grid.info.origin.position.y = float(origin[1])
    grid.info.origin.orientation.z = math.sin(yaw / 2.0)
    grid.info.origin.orientation.w = math.cos(yaw / 2.0)
    grid.data = [fill] * (width * height)
    return grid


def _set(grid: OccupancyGrid, col: int, row: int, value: int) -> None:
    data = list(grid.data)
    data[row * grid.info.width + col] = value
    grid.data = data


def _map_of_cell(grid: OccupancyGrid, col: int, row: int) -> 'tuple[float, float]':
    """Independently computed map coordinate of a cell centre, including yaw."""
    gx = (col + 0.5) * RES
    gy = (row + 0.5) * RES
    yaw = 2.0 * math.atan2(grid.info.origin.orientation.z,
                           grid.info.origin.orientation.w)
    return (grid.info.origin.position.x + gx * math.cos(yaw) - gy * math.sin(yaw),
            grid.info.origin.position.y + gx * math.sin(yaw) + gy * math.cos(yaw))


def test_an_occupied_cell_in_the_footprint_refuses_and_names_the_cell():
    grid = _grid(40, 40)
    _set(grid, 20, 20, 100)
    goal = _map_of_cell(grid, 20, 20)
    verdict = preflight_goal(grid, (goal[0], goal[1], 0.0), RADIUS_M, OCCUPIED)
    assert verdict['ok'] is False
    assert verdict['outcome'] == 'not_sent:goal_cell_occupied'
    assert '(20,20)=100' in verdict['detail'], verdict['detail']


def test_unknown_cells_are_allowed_and_counted():
    """planner_server.GridBased.allow_unknown is true; refusing would disagree."""
    grid = _grid(40, 40, fill=-1)
    goal = _map_of_cell(grid, 20, 20)
    verdict = preflight_goal(grid, (goal[0], goal[1], 0.0), RADIUS_M, OCCUPIED)
    assert verdict['ok'] is True, verdict
    assert verdict['outcome'] == ''
    assert 'unknown (allowed' in verdict['detail'], verdict['detail']
    assert '0 occupied' in verdict['detail'], verdict['detail']


def test_a_footprint_crossing_the_border_is_named_as_the_footprint():
    """
    The centre cell is well inside; only the footprint leaves the grid.

    The old name, not_sent:goal_outside_map, said the goal was outside the map
    when the goal's own cell was inside it.
    """
    grid = _grid(40, 40)
    goal = _map_of_cell(grid, 1, 20)                    # 0.075 m from the border
    verdict = preflight_goal(grid, (goal[0], goal[1], 0.0), RADIUS_M, OCCUPIED)
    assert verdict['ok'] is False
    assert verdict['outcome'] == 'not_sent:footprint_outside_map'
    assert 'the goal centre cell is inside the grid' in verdict['detail'], verdict['detail']


def test_a_rotated_origin_indexes_the_cell_the_map_coordinate_names():
    """
    Correct today, and this keeps it that way.

    An OccupancyGrid with origin.yaw = pi/2 is legal. If the inverse rotation
    in preflight_goal were dropped or signed the wrong way, the occupied cell
    below would be missed and a goal inside a wall would be sent.
    """
    grid = _grid(40, 40, origin=(-1.0, -1.0), yaw=math.pi / 2.0)
    _set(grid, 30, 5, 100)
    occupied_goal = _map_of_cell(grid, 30, 5)
    verdict = preflight_goal(grid, (occupied_goal[0], occupied_goal[1], 0.0),
                             RADIUS_M, OCCUPIED)
    assert verdict['ok'] is False, verdict
    assert verdict['outcome'] == 'not_sent:goal_cell_occupied'
    assert '(30, 5)' in verdict['detail'], verdict['detail']
    assert '(30,5)=100' in verdict['detail'], verdict['detail']

    free_goal = _map_of_cell(grid, 10, 20)
    assert preflight_goal(grid, (free_goal[0], free_goal[1], 0.0),
                          RADIUS_M, OCCUPIED)['ok'] is True
