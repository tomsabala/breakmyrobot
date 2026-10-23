#!/usr/bin/env python3
"""
Acceptance check for a freshly built map, against the world it was built in.

Run after a mapping session (ral_bringup/maps/README.md) and before trusting
anything downstream:

    python3 scripts/checks/map_sanity.py

Pure Python: no ROS, no simulator, no container. It parses the saved
`apartment.pgm`/`.yaml`, the measured `apartment_frames.yaml` and the world
SDF, and asserts that the map the robot built matches the world it drove in.

Why this exists
---------------
A SLAM map can be badly wrong in ways that look fine in a thumbnail. This was
not hypothetical: the first mapping run of this project produced a map
containing a GHOST copy of room 1 offset by -7.95 m in x, with wall A and the
east wall missing entirely - the apartment is three near-identical 4 m rooms
in a row, so a scan registered in one room matches its neighbours almost as
well as its own, and loop closure merged the wrong ones. Both doorways were
still visible as gaps, so a "do the doorways look open?" eyeball test passed
on a map that was structurally destroyed.

Every check below is therefore about global consistency with known geometry,
not local appearance:

  1. the four outer walls appear at the x/y the world SDF puts them, and the
     grid does not extend far beyond them (a ghost copy shows up here);
  2. both interior walls appear at their SDF x, with a gap at the right y and
     roughly the right width;
  3. the robot's start pose is in free space.

On acceptance it SEALS the pair: it writes the grid's sha256 into
apartment_frames.yaml as `map_sha256`, and on later runs it verifies that
field instead. That is what stops a new .pgm from being used with an old
alignment - the one failure mode that leaves a working stack reporting a
constant frame offset as localization error. ral_eval.frames refuses to load
an unsealed or mismatched pair, so roomba_nav.launch.py will not start on
one.

Exits 0 if the map is usable, 1 otherwise, printing a measured value per
check. It does NOT judge map QUALITY: the map is expected to carry real SLAM
error, and thickened or slightly skewed walls are accepted by design.
"""

import hashlib
import math
import os
import sys
from xml.etree import ElementTree

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MAPS_DIR = os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'ral_bringup', 'maps')
WORLD_SDF = os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'ral_description',
                         'worlds', 'ral_apartment.sdf')

OCCUPIED_MAX = 100       # pgm value at or below which a cell is an obstacle
FREE_MIN = 201           # pgm value at or above which a cell is free space

WALL_TOLERANCE = 0.30    # m, how far a mapped wall may sit from its SDF face
OUTSIDE_MARGIN = 1.50    # m, occupancy allowed beyond the outer walls
DOORWAY_WIDTH_SLACK = 0.45   # m, how much wider a mapped gap may read


def read_pgm(path):
    """Return (width, height, bytes) from a binary P5 PGM."""
    data = open(path, 'rb').read()

    def token(index):
        while data[index:index + 1].isspace():
            index += 1
        if data[index:index + 1] == b'#':
            while data[index:index + 1] not in (b'\n', b''):
                index += 1
            return token(index)
        end = index
        while not data[end:end + 1].isspace():
            end += 1
        return data[index:end], end

    magic, index = token(0)
    if magic != b'P5':
        raise SystemExit(f'{path}: not a binary PGM (magic {magic!r})')
    width, index = token(index)
    height, index = token(index)
    _maxval, index = token(index)
    return int(width), int(height), data[index + 1:]


class Grid:
    """A saved occupancy grid, addressable in gz WORLD coordinates."""

    def __init__(self, maps_dir):
        meta = yaml.safe_load(open(os.path.join(maps_dir, 'apartment.yaml')))
        self.resolution = float(meta['resolution'])
        self.origin = (float(meta['origin'][0]), float(meta['origin'][1]))
        self.image = meta['image']
        image_path = os.path.join(maps_dir, self.image)
        self.sha256 = hashlib.sha256(open(image_path, 'rb').read()).hexdigest()
        self.width, self.height, self.cells = read_pgm(image_path)
        self.frames_path = os.path.join(maps_dir, 'apartment_frames.yaml')
        frames = yaml.safe_load(open(self.frames_path))
        wtm = frames['world_to_map']
        self.world_to_map = (float(wtm['x']), float(wtm['y']), float(wtm['yaw']))
        self.frames = frames

    def value(self, world_x, world_y):
        """pgm value at a world point, or None if outside the grid."""
        wx, wy, wyaw = self.world_to_map
        dx, dy = world_x - wx, world_y - wy
        map_x = dx * math.cos(wyaw) + dy * math.sin(wyaw)
        map_y = -dx * math.sin(wyaw) + dy * math.cos(wyaw)
        col = int((map_x - self.origin[0]) / self.resolution)
        row_from_bottom = int((map_y - self.origin[1]) / self.resolution)
        row = self.height - 1 - row_from_bottom
        if not (0 <= col < self.width and 0 <= row < self.height):
            return None
        return self.cells[row * self.width + col]

    def occupied(self, world_x, world_y):
        value = self.value(world_x, world_y)
        return value is not None and value <= OCCUPIED_MAX

    def free(self, world_x, world_y):
        value = self.value(world_x, world_y)
        return value is not None and value >= FREE_MIN

    def occupied_near(self, world_x, world_y, radius):
        """True if any cell within `radius` of the point is an obstacle."""
        steps = int(radius / self.resolution)
        for i in range(-steps, steps + 1):
            for j in range(-steps, steps + 1):
                if self.occupied(world_x + i * self.resolution,
                                 world_y + j * self.resolution):
                    return True
        return False

    def world_extent(self):
        """The grid's own bounds, expressed in world coordinates."""
        corners = []
        wx, wy, wyaw = self.world_to_map
        for map_x in (self.origin[0], self.origin[0] + self.width * self.resolution):
            for map_y in (self.origin[1], self.origin[1] + self.height * self.resolution):
                corners.append((wx + map_x * math.cos(wyaw) - map_y * math.sin(wyaw),
                                wy + map_x * math.sin(wyaw) + map_y * math.cos(wyaw)))
        return (min(c[0] for c in corners), max(c[0] for c in corners),
                min(c[1] for c in corners), max(c[1] for c in corners))


def world_boxes():
    """Every model's (centre_x, centre_y, size_x, size_y) from the world SDF."""
    tree = ElementTree.parse(WORLD_SDF)
    boxes = {}
    for model in tree.getroot().find('world').findall('model'):
        pose = model.find('pose')
        size = model.find('link/collision/geometry/box/size')
        if pose is None or size is None:
            continue
        px, py = (float(v) for v in pose.text.split()[:2])
        sx, sy = (float(v) for v in size.text.split()[:2])
        boxes[model.get('name')] = (px, py, sx, sy)
    return boxes


def seal(grid):
    """Record the accepted grid's hash in the alignment file beside it."""
    kept = [line for line in open(grid.frames_path).read().splitlines()
            if not line.startswith('map_sha256:')]
    temporary = grid.frames_path + '.partial'
    with open(temporary, 'w') as handle:
        # Quoted: a sha256 that happens to be all digits is a YAML integer,
        # and an integer loses the leading zeros the hash needs.
        handle.write('\n'.join(kept) + f"\nmap_sha256: '{grid.sha256}'\n")
    os.replace(temporary, grid.frames_path)
    print(f'--- sealed {os.path.basename(grid.frames_path)} to '
          f'sha256({grid.image}) = {grid.sha256}', flush=True)


def main() -> int:
    grid = Grid(MAPS_DIR)
    boxes = world_boxes()
    results = []

    def record(name, passed, detail):
        results.append((name, passed, detail))
        print(f'{"PASS" if passed else "FAIL"}  {name}: {detail}', flush=True)

    # -- resolution and provenance -------------------------------------------
    record('resolution and image reference',
           abs(grid.resolution - 0.05) < 1e-9 and grid.image == 'apartment.pgm',
           f'resolution {grid.resolution} m/cell, image {grid.image!r}, '
           f'{grid.width} x {grid.height} cells')

    # -- 1. outer shell: present where the SDF says, and not far beyond ------
    # Sampled along each wall rather than at one point, because a wall can be
    # mapped for part of its length and missing for the rest.
    shell = {}
    for name, (axis, fixed, lo, hi) in {
        'wall_south': ('y', 0.0, 0.6, 11.4),
        'wall_north': ('y', 8.0, 0.6, 11.4),
        'wall_west': ('x', 0.0, 0.6, 7.4),
        'wall_east': ('x', 12.0, 0.6, 7.4),
    }.items():
        samples = [lo + (hi - lo) * i / 20.0 for i in range(21)]
        if axis == 'y':
            hits = sum(grid.occupied_near(s, fixed, WALL_TOLERANCE) for s in samples)
        else:
            hits = sum(grid.occupied_near(fixed, s, WALL_TOLERANCE) for s in samples)
        shell[name] = hits / len(samples)
    worst = min(shell.values())
    record('outer walls mapped',
           worst >= 0.9,
           ', '.join(f'{name} {fraction * 100:.0f}%' for name, fraction in shell.items())
           + f' of sample points have an obstacle within {WALL_TOLERANCE} m '
             '(>= 90% required for every wall)')

    # A ghost copy of the apartment lands outside the real walls, which is
    # what this bound catches; the eyeball test does not.
    min_x, max_x, min_y, max_y = grid.world_extent()
    outside = []
    step = 0.25
    x = min_x
    while x <= max_x:
        y = min_y
        while y <= max_y:
            if (x < -OUTSIDE_MARGIN or x > 12.0 + OUTSIDE_MARGIN
                    or y < -OUTSIDE_MARGIN or y > 8.0 + OUTSIDE_MARGIN):
                if grid.occupied(x, y):
                    outside.append((round(x, 2), round(y, 2)))
            y += step
        x += step
    record('no occupancy outside the apartment',
           len(outside) <= 5,
           f'{len(outside)} sampled obstacle cell(s) more than {OUTSIDE_MARGIN} m '
           f'outside the 12 x 8 m shell (<= 5 allowed for noise)'
           + (f'; first few: {outside[:5]}' if outside else ''))

    # -- 2. interior walls and their doorways ---------------------------------
    for lower, upper, label in (('wall_a_lower', 'wall_a_upper', 'wall A'),
                                ('wall_b_lower', 'wall_b_upper', 'wall B')):
        cx, lower_cy, _, lower_sy = boxes[lower]
        _, upper_cy, _, upper_sy = boxes[upper]
        gap_lo = lower_cy + lower_sy / 2.0
        gap_hi = upper_cy - upper_sy / 2.0
        gap_width = gap_hi - gap_lo
        gap_mid = (gap_lo + gap_hi) / 2.0

        # The wall must be there on both sides of the gap ...
        solid_samples = ([lower_cy - lower_sy / 2.0 + lower_sy * i / 10.0
                          for i in range(1, 10)]
                         + [upper_cy - upper_sy / 2.0 + upper_sy * i / 10.0
                            for i in range(1, 10)])
        solid_hits = sum(grid.occupied_near(cx, y, WALL_TOLERANCE)
                         for y in solid_samples)
        solid_fraction = solid_hits / len(solid_samples)

        # ... and the gap must be open. Measured by growing outward from the
        # gap centre until an obstacle is met on each side.
        def open_extent(direction):
            distance = 0.0
            while distance < 3.0:
                if grid.occupied(cx, gap_mid + direction * distance):
                    return distance
                distance += grid.resolution
            return distance

        measured = open_extent(+1.0) + open_extent(-1.0)
        centre_free = grid.free(cx, gap_mid)
        passed = (solid_fraction >= 0.9
                  and gap_width - 0.1 <= measured <= gap_width + DOORWAY_WIDTH_SLACK
                  and centre_free)
        record(f'{label} with its doorway', passed,
               f'wall at x = {cx}: {solid_fraction * 100:.0f}% of the solid span has an '
               f'obstacle within {WALL_TOLERANCE} m (>= 90% required); gap at y = '
               f'{gap_mid:.2f} measures {measured:.2f} m open (SDF {gap_width:.2f} m, '
               f'accepted {gap_width - 0.1:.2f}-{gap_width + DOORWAY_WIDTH_SLACK:.2f}); '
               f'gap centre is free = {centre_free}')

    # -- 3. the start pose is in free space ------------------------------------
    start_x, start_y = grid.world_to_map[0], grid.world_to_map[1]
    record('start pose in free space', grid.free(start_x, start_y),
           f'world ({start_x}, {start_y}) -> pgm value {grid.value(start_x, start_y)} '
           f'(free requires >= {FREE_MIN})')

    # -- 4. the alignment belongs to THIS grid ---------------------------------
    # str(): a hand-edited or all-digit value must read as a mismatch rather
    # than crash the check that exists to report it.
    recorded = grid.frames.get('map_sha256')
    recorded = None if recorded is None else str(recorded)
    if recorded is None:
        record('alignment belongs to this grid', True,
               f'apartment_frames.yaml is unsealed; it will be sealed to '
               f'{grid.sha256[:16]}... if this map is accepted')
    else:
        record('alignment belongs to this grid', recorded == grid.sha256,
               f'apartment_frames.yaml records map_sha256 {recorded[:16]}..., '
               f'apartment.pgm hashes to {grid.sha256[:16]}...')

    passed = all(result[1] for result in results)
    print(f'\n{"MAP ACCEPTED" if passed else "MAP REJECTED"} '
          f'({sum(1 for r in results if r[1])}/{len(results)} checks passed)', flush=True)
    if passed and recorded != grid.sha256:
        seal(grid)
    if not passed:
        print('Redo the mapping drive (ral_bringup/maps/README.md). Do not patch the '
              'map by hand and do not rasterise it from the world SDF.', flush=True)
        if recorded is not None and recorded != grid.sha256:
            print('The alignment was measured against a different grid: capture it in '
                  'the SAME session that builds the map. Nothing was sealed.',
                  flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
