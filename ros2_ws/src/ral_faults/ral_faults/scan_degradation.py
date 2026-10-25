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
What a degraded lidar reports, as a pure function of the healthy scan.

No ROS, no node, no message: a list of ranges in, a list of ranges out, with
the RNG passed in so a caller owns the seed. test_scan_degradation.py tests
this module directly; scan_fault_relay only wires a LaserScan into it.

HONESTY NOTE, repeated verbatim from fault_catalogue.yaml: this corrupts the
LaserScan stream between the sensor bridge and its consumers. The simulated
lidar itself stays healthy - Gazebo Harmonic cannot change SDF sensor noise at
runtime.

`+inf` is the ONLY marker for a beam that returned nothing. A real driver
reports an out-of-range or absent return as a value outside [range_min,
range_max] and REP 117 names +inf for "no return within range"; 0.0 would be
read by the costmaps as an obstacle on the sensor itself, and NaN is the
marker for an erroneous reading rather than an absent one. Writing either
would be inventing a measurement the modelled device does not make.
"""

import math
import random
from typing import NamedTuple

INVALID = float('inf')


class Degradation(NamedTuple):
    """The five parameters of SetScanDegradation, after validation."""

    range_noise_stddev_m: float
    max_range_m: float
    blind_sector_start_deg: float
    blind_sector_end_deg: float
    dropout_probability: float


def wrap_deg(angle_deg: float) -> float:
    """Wrap an angle to (-180, 180], the range the blind sector is given in."""
    wrapped = math.fmod(angle_deg, 360.0)
    if wrapped <= -180.0:
        wrapped += 360.0
    elif wrapped > 180.0:
        wrapped -= 360.0
    return wrapped


def in_blind_sector(angle_deg: float, start_deg: float, end_deg: float) -> bool:
    """
    Report whether `angle_deg` lies in the sector running CCW from start to end.

    The direction is what makes this unambiguous: a sector is not "between two
    numbers", it is an arc with a start and an end. When start > end the arc
    WRAPS through +-180 deg, so (150, -150) is the 60 deg behind the robot and
    NOT the 300 deg in front of it. Reading it as min/max would blank five
    sixths of the scan and the run would be reported as a 60 deg fault.

    start == end is a ZERO-WIDTH arc and blanks nothing. The service has no
    other way to say "no blind sector", and a request that leaves both fields
    at 0.0 means exactly that; reading it as the full circle would silently
    blank every beam of a run whose author only wanted noise. Which single
    beam a zero-width sector would otherwise catch depends on floating-point
    accumulation of angle_min + i * angle_increment, so the alternative is
    not even stable.
    """
    start = wrap_deg(start_deg)
    end = wrap_deg(end_deg)
    if start == end:
        return False
    angle = wrap_deg(angle_deg)
    if start < end:
        return start <= angle <= end
    return angle >= start or angle <= end


def degrade_ranges(ranges, angle_min: float, angle_increment: float,
                   range_min: float, cfg: Degradation, rng: random.Random) -> list:
    """
    Return a new range list with `cfg` applied, in a fixed order.

    The order matters and is part of the contract, because each step feeds the
    next:

      1. additive zero-mean Gaussian noise, per beam that currently carries a
         finite range (a beam that already reported nothing cannot be made
         noisy);
      2. a noisy range below `range_min` becomes +inf - the noise can push a
         close return below what the device can range, and that is exactly
         what a driver reports for it;
      3. truncation: a range above `max_range_m` becomes +inf. Applied to the
         NOISY value, because a real lidar's reach limit acts on the reading
         it actually produced;
      4. the blind sector: every beam whose angle lies inside it becomes +inf,
         noisy or not - an occluded or dead sector returns nothing at all;
      5. per-beam dropout at `dropout_probability`, independently, over the
         beams that survived the previous steps.

    Determinism: each ENABLED random step draws exactly once per beam, in beam
    order, whether or not that beam can use the draw. A dead beam therefore
    does not shift every later beam's noise, so the corruption is a function
    of (seed, scans since activation, beam index) and nothing else. Drawing
    only for usable beams would make the stream depend on the sensor's own
    no-return pattern, and two runs of the same scenario would diverge the
    first time the simulated lidar missed a return.

    `rng` is a random.Random owned by the caller. A module-level RNG would
    make two relays in one process share a stream, and the whole point of the
    seed is that a rerun with the same scenario produces the same corruption.
    """
    out = []
    for index, value in enumerate(ranges):
        reading = float(value)
        noise = rng.gauss(0.0, cfg.range_noise_stddev_m) \
            if cfg.range_noise_stddev_m > 0.0 else 0.0
        dropout_draw = rng.random() if cfg.dropout_probability > 0.0 else 1.0

        if math.isfinite(reading):
            reading += noise
            if reading < range_min:
                reading = INVALID
            elif reading > cfg.max_range_m:
                reading = INVALID
        else:
            # NaN or -inf arrived from the sensor. Both already mean "no usable
            # reading"; normalising them to +inf keeps one invalid marker in
            # the degraded stream rather than passing two through.
            reading = INVALID

        angle_deg = math.degrees(angle_min + index * angle_increment)
        if in_blind_sector(angle_deg, cfg.blind_sector_start_deg,
                           cfg.blind_sector_end_deg):
            reading = INVALID

        if math.isfinite(reading) and dropout_draw < cfg.dropout_probability:
            reading = INVALID

        out.append(reading)
    return out
