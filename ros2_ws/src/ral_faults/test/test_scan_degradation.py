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
The lidar degradation maths, on synthetic scans, without ROS.

Every case is chosen so a plausible WRONG implementation gives a different
answer: a blind sector read as min/max rather than as a directed arc, a
truncation applied to the clean range instead of the noisy one, a dropout
that reuses one draw for every beam, a noise term that leaves a physically
impossible sub-minimum reading in the stream.
"""

import math
import random

from ral_faults.scan_degradation import (
    Degradation,
    degrade_ranges,
    in_blind_sector,
)

INF = float('inf')

# A 36-beam scan over the full circle: beam i is at -180 + 10*i degrees.
BEAMS = 36
ANGLE_MIN = -math.pi
ANGLE_INCREMENT = 2.0 * math.pi / BEAMS
RANGE_MIN = 0.15


def cfg(noise=0.0, max_range=12.0, start=0.0, end=0.0, dropout=0.0) -> Degradation:
    """Build a Degradation with everything off except what a case turns on."""
    return Degradation(range_noise_stddev_m=noise, max_range_m=max_range,
                       blind_sector_start_deg=start, blind_sector_end_deg=end,
                       dropout_probability=dropout)


def run(ranges, config, seed=1):
    return degrade_ranges(ranges, ANGLE_MIN, ANGLE_INCREMENT, RANGE_MIN,
                          config, random.Random(seed))


def angle_deg(index: int) -> float:
    return math.degrees(ANGLE_MIN + index * ANGLE_INCREMENT)


def test_everything_off_returns_the_same_numbers():
    """The maths itself must not distort a scan it was told not to degrade."""
    ranges = [1.0 + 0.1 * index for index in range(BEAMS)]
    assert run(ranges, cfg()) == ranges


def test_noise_is_deterministic_for_a_seed_and_differs_for_another():
    ranges = [2.0] * BEAMS
    first = run(ranges, cfg(noise=0.05), seed=7)
    assert first == run(ranges, cfg(noise=0.05), seed=7)
    assert first != run(ranges, cfg(noise=0.05), seed=8)
    # Noise, not a rewrite: every beam moved, and none moved absurdly far.
    assert all(value != 2.0 for value in first)
    assert all(abs(value - 2.0) < 0.5 for value in first)


def test_truncation_blanks_only_the_beams_beyond_the_limit():
    ranges = [1.0, 2.49, 2.51, 9.0] + [1.0] * (BEAMS - 4)
    out = run(ranges, cfg(max_range=2.5))
    assert out[0] == 1.0
    assert out[1] == 2.49
    assert out[2] == INF
    assert out[3] == INF


def test_truncation_sees_the_NOISY_value_not_the_clean_one():
    """
    Order matters: noise feeds truncation.

    Beam 0 starts 0.02 m under the limit. With this seed its noise is
    +0.0644 m, which carries it over - an implementation that truncated
    before adding noise would leave it finite (and 0.02 m short of a limit it
    actually exceeded).
    """
    clean = 2.48
    noisy_only = run([clean] + [0.5] * (BEAMS - 1), cfg(noise=0.05), seed=1)[0]
    assert abs(noisy_only - 2.544409) < 1e-6, noisy_only
    out = run([clean] + [0.5] * (BEAMS - 1), cfg(noise=0.05, max_range=2.5), seed=1)
    assert out[0] == INF


def test_noise_below_range_min_is_reported_as_no_return():
    """A reading the device cannot range is +inf, never a sub-minimum number."""
    out = run([0.16] * BEAMS, cfg(noise=0.5), seed=5)
    blanked = [value for value in out if value == INF]
    finite = [value for value in out if value != INF]
    assert blanked, out
    assert all(value >= RANGE_MIN for value in finite), finite


def test_a_non_wrapping_sector_blanks_exactly_the_beams_inside_it():
    """
    Beams 27..33 are at 90..150 degrees; nothing adjacent may be touched.

    The sector bounds sit BETWEEN beams (85 and 155) on purpose: a bound on a
    beam angle would make the case turn on whether
    degrees(angle_min + i * increment) lands a float epsilon above or below
    it, which is a property of binary floating point rather than of the
    degradation.
    """
    ranges = [3.0] * BEAMS
    out = run(ranges, cfg(start=85.0, end=155.0))
    blanked = {index for index, value in enumerate(out) if value == INF}
    assert blanked == {27, 28, 29, 30, 31, 32, 33}, \
        sorted((index, angle_deg(index)) for index in blanked)
    assert out[26] == 3.0 and out[34] == 3.0


def test_a_wrapping_sector_blanks_the_rear_and_not_the_front():
    """145 -> -145 is the 60-odd degrees behind the robot, not the 300 in front."""
    ranges = [3.0] * BEAMS
    out = run(ranges, cfg(start=145.0, end=-145.0))
    blanked = {index for index, value in enumerate(out) if value == INF}
    # Beams 33, 34, 35 at 150..170 and beams 0..3 at -180..-150, i.e. the arc
    # through +-180. A min/max reading of the two bounds would have blanked
    # beams 4..32 instead - the entire front of the robot.
    assert blanked == {33, 34, 35, 0, 1, 2, 3}, \
        sorted((index, angle_deg(index)) for index in blanked)
    assert all(out[index] == 3.0 for index in range(4, 33))


def test_a_zero_width_sector_blanks_nothing():
    """`start == end` is how a request says 'no blind sector'."""
    ranges = [3.0] * BEAMS
    assert run(ranges, cfg(start=0.0, end=0.0)) == ranges
    assert run(ranges, cfg(start=45.0, end=45.0)) == ranges


def test_dropout_is_all_or_nothing_at_the_probability_extremes():
    ranges = [3.0] * BEAMS
    assert run(ranges, cfg(dropout=1.0)) == [INF] * BEAMS
    assert run(ranges, cfg(dropout=0.0)) == ranges


def test_dropout_draws_once_per_beam_rather_than_once_per_scan():
    """A single draw reused for the whole sweep gives 0 or 36 blanks, never a mix."""
    ranges = [3.0] * BEAMS
    out = run(ranges, cfg(dropout=0.5), seed=11)
    blanked = out.count(INF)
    assert 0 < blanked < BEAMS, out
    assert run(ranges, cfg(dropout=0.5), seed=11) == out


def test_a_dead_beam_does_not_shift_the_noise_of_every_later_beam():
    """
    +inf in, +inf out - and the RNG stream is unaffected by it.

    The RNG advances once per beam whether or not the beam can use the draw,
    so beams 1..35 get exactly the noise they would have got had beam 0 been
    alive. Drawing only for usable beams would make two runs with the same
    seed diverge the first time the simulated lidar missed one return.
    """
    out = run([INF] + [2.0] * (BEAMS - 1), cfg(noise=0.05), seed=2)
    assert out[0] == INF
    reference = run([2.0] * BEAMS, cfg(noise=0.05), seed=2)
    assert out[1:] == reference[1:]


def test_nan_is_normalised_to_the_single_invalid_marker():
    out = run([float('nan')] + [2.0] * (BEAMS - 1), cfg())
    assert out[0] == INF


def test_in_blind_sector_wraps_the_angle_it_is_given():
    assert in_blind_sector(190.0, -180.0, -170.0) is True    # 190 == -170
    assert in_blind_sector(-170.0, 150.0, -150.0) is True
    assert in_blind_sector(0.0, 150.0, -150.0) is False
