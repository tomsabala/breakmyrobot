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
The one reader of the committed occupancy grid.

Two consumers, and that is why it lives here rather than in either of them:

  plot_run.py      DRAWS the grid behind panel A when the bag recorded no
                   /map - which is every run of a platform whose stack has no
                   map_server.
  run_scenario.py  PRE-FLIGHTS the goal against it on exactly those
                   platforms: with no /map on the wire there is no grid from
                   the stack to refuse an occupied goal cell with, and the
                   committed PGM is the same artifact the alignment was
                   sealed against.

One reader is what keeps them from disagreeing. A second copy would be a
second set of assumptions about negate, about the origin rotation and about
the free/occupied thresholds, and the figure would then be able to show a
goal in free space that the pre-flight refused as occupied - or the reverse,
which is worse.
"""

import os

import numpy as np

import yaml


def grid_from_pgm(map_yaml_path: str):
    """Read the committed grid, for a bag that recorded no /map: (array, info)."""
    with open(map_yaml_path) as handle:
        meta = yaml.safe_load(handle)
    # This reader inverts the pixel ratio for negate: 0 and places the grid
    # with an axis-aligned extent. Both assumptions hold for apartment.yaml;
    # neither is checked anywhere else, so a map that breaks one is refused
    # rather than drawn wrong.
    if int(meta.get('negate', 0)) != 0:
        raise ValueError(f'{map_yaml_path} sets negate: 1; this reader inverts the '
                         'ratio for negate: 0 only and would draw free space as wall')
    if len(meta['origin']) > 2 and abs(float(meta['origin'][2])) > 1e-9:
        raise ValueError(f'{map_yaml_path} has a rotated origin '
                         f'({meta["origin"][2]} rad); _draw_grid places the grid with '
                         'an axis-aligned extent and would put every cell elsewhere')
    image_path = os.path.join(os.path.dirname(map_yaml_path), meta['image'])
    with open(image_path, 'rb') as handle:
        data = handle.read()
    # Minimal binary PGM (P5) reader: nav2's map_saver writes exactly this.
    fields = []
    index = 0
    while len(fields) < 4:
        while index < len(data) and data[index:index + 1].isspace():
            index += 1
        if data[index:index + 1] == b'#':
            while index < len(data) and data[index:index + 1] != b'\n':
                index += 1
            continue
        start = index
        while index < len(data) and not data[index:index + 1].isspace():
            index += 1
        fields.append(data[start:index])
    index += 1
    magic, width, height, maxval = fields[0], int(fields[1]), int(fields[2]), int(fields[3])
    if magic != b'P5':
        raise ValueError(f'{image_path} is {magic!r}, not a binary P5 pgm')
    pixels = np.frombuffer(data[index:index + width * height], dtype=np.uint8)
    pixels = pixels.reshape(height, width)[::-1]        # pgm is top-down, the grid is not
    occupancy = np.full(pixels.shape, -1, dtype=np.int16)
    free = meta.get('free_thresh', 0.25)
    occupied = meta.get('occupied_thresh', 0.65)
    ratio = (maxval - pixels.astype(np.float32)) / maxval
    occupancy[ratio >= occupied] = 100
    occupancy[ratio <= free] = 0
    info = {'resolution': meta['resolution'], 'origin': meta['origin'],
            'width': width, 'height': height}
    return occupancy, info
