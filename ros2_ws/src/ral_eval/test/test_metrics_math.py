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
The metric computations, on synthetic data, without a bag.

Every case here has a number chosen so that a plausible WRONG implementation
gives a different answer: a path length that counts a teleport, a stopped-time
integral that forgets the zero-order hold, a pairing that matches the nearest
sample regardless of how far away it is, a plan comparison that reads a
shortened plan as a reroute, an angular difference that does not wrap.
"""

import math

from ral_eval.metrics import (
    RESPONSE_BASELINE_WINDOW_S,
    RESPONSE_HORIZON_S,
    central_difference,
    debounce_events,
    first_response,
    metric,
    min_clearance,
    moving_average,
    pair_nearest,
    path_length,
    plan_divergence,
    point_to_box,
    resample_linear,
    resample_zoh,
    scan_divergence,
    stopped_time,
    unwrap_angles,
)

import pytest


def test_path_length_of_a_straight_metre_at_50_hz():
    track = [(index * 0.02, index * 0.02, 0.0) for index in range(51)]
    walked = path_length(track)
    assert abs(walked['length'] - 1.0) < 1e-9, walked
    assert walked['excluded'] == 0.0


def test_a_teleport_is_excluded_and_reported_not_counted_as_travel():
    """4 m in 0.02 s is a kidnap; the robot's own metre must survive it."""
    track = [(index * 0.02, index * 0.02, 0.0) for index in range(26)]   # 0.50 m
    jump_t = track[-1][0] + 0.02
    track.append((jump_t, track[-1][1] + 4.0, 0.0))                      # the teleport
    track += [(jump_t + 0.02 * k, track[-1][1] + 0.02 * k, 0.0) for k in range(1, 26)]
    walked = path_length(track)
    assert abs(walked['length'] - 1.0) < 1e-9, walked
    assert abs(walked['excluded'] - 4.0) < 1e-9, walked
    assert walked['excluded_steps'] == 1


def test_stopped_time_holds_each_command_until_the_next():
    cmds = [(0.0, 0.2, 0.0), (2.0, 0.0, 0.0), (5.0, 0.3, 0.0)]
    assert abs(stopped_time(cmds, 0.0, 6.0) - 3.0) < 1e-9


def test_stopped_time_carries_the_command_in_force_at_the_window_start():
    """The robot does not forget its last command because a window began."""
    cmds = [(-3.0, 0.0, 0.0), (4.0, 0.25, 0.0)]
    assert abs(stopped_time(cmds, 0.0, 10.0) - 4.0) < 1e-9


def test_pair_nearest_respects_the_window_and_prefers_the_closer_sample():
    primary = [(1.0, 'a')]
    assert pair_nearest(primary, [(1.04, 'near')], window=0.05)[0][2] == 'near'
    assert pair_nearest(primary, [(1.06, 'far')], window=0.05) == []
    pairs = pair_nearest(primary, [(0.97, 'behind'), (1.01, 'ahead')], window=0.05)
    assert pairs[0][2] == 'ahead', pairs


def test_debounce_collapses_a_sustained_contact_but_keeps_a_second_one():
    events = [(0.0, 'a/b'), (0.1, 'a/b'), (0.2, 'a/b'), (1.1, 'a/b')]
    assert debounce_events(events, window=0.5) == 2


def test_debounce_counts_distinct_pairs_separately():
    events = [(0.0, 'a/b'), (0.05, 'a/c')]
    assert debounce_events(events, window=0.5) == 2


def test_plan_divergence_is_zero_for_the_same_path():
    path = [(float(index), 0.0) for index in range(11)]
    assert plan_divergence(path, path) < 1e-9


def test_plan_divergence_sees_a_displaced_path():
    path = [(float(index), 0.0) for index in range(11)]
    moved = [(x, y + 0.4) for x, y in path]
    assert abs(plan_divergence(moved, path) - 0.4) < 1e-6


def test_a_plan_that_merely_got_shorter_is_not_a_divergence():
    """1 Hz replanning shortens the path every tick; that is not a reaction."""
    path = [(float(index), 0.0) for index in range(11)]
    assert plan_divergence(path[5:], path) < 1e-9


def test_plan_divergence_handles_paths_of_different_lengths():
    coarse = [(0.0, 0.0), (10.0, 0.0)]
    fine = [(index * 0.25, 0.4) for index in range(41)]
    assert abs(plan_divergence(fine, coarse) - 0.4) < 1e-6


def test_a_null_metric_must_carry_a_reason():
    """The rule every "null + reason, never a default" claim in metrics.py rests on."""
    with pytest.raises(AssertionError):
        metric(None, 'm', 'src', 'defn')
    recorded = metric(None, 'm', 'src', 'defn', reason='x')
    assert recorded['value'] is None and recorded['reason'] == 'x'


def test_first_response_picks_the_command_when_it_moves_before_the_next_plan():
    plans = [(0.0, [(0.0, 0.0), (10.0, 0.0)]), (2.0, [(0.0, 0.0), (10.0, 0.0)])]
    # Two commands before the activation, so the cmd trigger has a baseline to
    # calibrate against; both are 0.40, so the excursion is 0 and the floor rules.
    cmds = [(-1.0, 0.40, 0.0), (0.0, 0.40, 0.0), (0.5, 0.10, 0.0)]
    found = first_response(0.2, plans, cmds, [])
    assert found['trigger'] == 'cmd_changed', found
    assert abs(found['response_sim_s'] - 0.5) < 1e-9
    assert abs(found['delta_s'] - 0.3) < 1e-9
    assert found['cmd_baseline_samples'] == 2


def test_first_response_picks_the_plan_when_only_the_plan_moves():
    plans = [(0.0, [(0.0, 0.0), (10.0, 0.0)]), (1.0, [(0.0, 0.9), (10.0, 0.9)])]
    cmds = [(0.0, 0.40, 0.0), (0.5, 0.40, 0.0)]
    found = first_response(0.2, plans, cmds, [])
    assert found['trigger'] == 'plan_diverged', found
    assert abs(found['response_sim_s'] - 1.0) < 1e-9


def test_first_response_sees_a_recovery_increment():
    found = first_response(1.0, [], [], [(0.5, 0), (1.5, 1)])
    assert found['trigger'] == 'recovery_started', found


def test_first_response_reports_the_search_when_nothing_reacted():
    """Never None: the calibration must be visible even when nothing fired."""
    plans = [(0.0, [(0.0, 0.0), (10.0, 0.0)]), (3.0, [(0.0, 0.0), (10.0, 0.0)])]
    cmds = [(0.0, 0.40, 0.0), (2.0, 0.42, 0.0)]
    found = first_response(1.0, plans, cmds, [(0.0, 2), (2.0, 2)])
    assert found['trigger'] is None, found
    assert found['reason'], found
    assert found['horizon_s'] == RESPONSE_HORIZON_S


def test_a_command_change_after_the_horizon_is_not_a_response():
    """A change five seconds later is not attributable to the fault."""
    base = [(-1.0, 0.40, 0.0), (0.0, 0.40, 0.0)]
    late = first_response(0.0, [], base + [(5.0, 0.0, 0.0)], [], horizon=3.0)
    assert late['trigger'] is None, late
    early = first_response(0.0, [], base + [(1.0, 0.0, 0.0)], [], horizon=3.0)
    assert early['trigger'] == 'cmd_changed', early
    assert abs(early['delta_s'] - 1.0) < 1e-9


def test_first_response_skips_the_command_trigger_without_a_baseline():
    """No command before the activation means no null hypothesis to test."""
    found = first_response(0.0, [], [(1.0, 0.4, 0.0)], [])
    assert found['trigger'] is None, found
    assert found['cmd_baseline_samples'] == 0
    assert found['cmd_threshold_m_s'] is None
    assert 'skipped' in found['reason'], found['reason']


def test_the_command_threshold_scales_with_this_run_s_own_variation():
    """A 0.2 m/s step is a response after a quiet pre-window and not after a noisy one."""
    quiet = [(-5.0, 0.40, 0.0), (-2.0, 0.40, 0.0), (1.0, 0.60, 0.0)]
    assert first_response(0.0, [], quiet, [])['trigger'] == 'cmd_changed'
    noisy = [(-5.0, 0.40, 0.0), (-4.0, 0.10, 0.0), (-2.0, 0.40, 0.0), (1.0, 0.60, 0.0)]
    found = first_response(0.0, [], noisy, [])
    assert found['trigger'] is None, found
    assert abs(found['cmd_baseline_excursion_m_s'] - 0.30) < 1e-9, found


def test_the_baseline_window_does_not_reach_back_for_ever():
    """A command older than the window cannot calibrate the threshold."""
    old = -(RESPONSE_BASELINE_WINDOW_S + 5.0)
    found = first_response(0.0, [], [(old, 0.40, 0.0), (-1.0, 0.40, 0.0)], [])
    assert found['cmd_baseline_samples'] == 1, found
    assert found['cmd_threshold_m_s'] is None, found


def test_stopped_time_is_unknown_when_no_command_precedes_the_window():
    """0.0 would assert the robot never stopped in a window nothing was known about."""
    assert stopped_time([(11.0, 0.0, 0.0)], 0.0, 10.0) is None


def test_path_length_refuses_a_track_whose_stamps_decrease():
    """A negative dt collapses the limit and sheds real travel into teleports."""
    with pytest.raises(ValueError):
        path_length([(0.0, 0.0, 0.0), (1.0, 0.1, 0.0), (0.5, 0.2, 0.0)])


def test_central_difference_and_smoothing_recover_a_constant_speed():
    step = 0.1
    xs = [0.4 * index * step for index in range(20)]
    speeds = moving_average(central_difference(xs, step), 3)
    for value in speeds:
        assert abs(value - 0.4) < 1e-3, speeds


def test_the_angular_difference_wraps_across_the_branch_cut():
    """+3.10 -> -3.10 over 0.1 s is +0.832 rad/s, not -62 rad/s."""
    rates = central_difference([3.10, -3.10], 0.1, angular=True)
    assert abs(rates[0] - 0.831853071795865) < 1e-9, rates


def test_unwrap_makes_yaw_safe_to_interpolate():
    unwrapped = unwrap_angles([3.10, -3.10])
    assert abs(unwrapped[1] - (3.10 + 0.0831853071795865)) < 1e-9, unwrapped


def test_zero_order_hold_and_linear_resampling_differ_where_it_matters():
    samples = [(0.0, 0.0), (1.0, 1.0)]
    grid = [0.0, 0.5, 1.0]
    assert resample_zoh(samples, grid) == [0.0, 0.0, 1.0]
    linear = resample_linear(samples, grid)
    assert abs(linear[1] - 0.5) < 1e-12, linear


def test_resampling_outside_the_sampled_interval_is_none_not_an_extrapolation():
    samples = [(1.0, 5.0), (2.0, 6.0)]
    assert resample_linear(samples, [0.5, 3.0]) == [None, None]
    assert resample_zoh(samples, [0.5])[0] is None


def test_path_length_of_an_empty_track_is_zero_with_the_sample_count_reported():
    walked = path_length([])
    assert walked['length'] == 0.0 and walked['samples'] == 0


def test_moving_average_ignores_missing_samples():
    assert moving_average([1.0, None, 3.0], 3) == [1.0, 2.0, 3.0]


def test_wrapping_is_consistent_with_math_for_a_full_turn():
    rates = central_difference([0.0, 2.0 * math.pi], 1.0, angular=True)
    assert abs(rates[0]) < 1e-9, rates


def _box(name='blocker', cx=2.0, cy=0.0, yaw=0.0, sx=1.0, sy=0.2,
         t_start=0.0, t_end=100.0) -> dict:
    return {'name': name, 'cx': cx, 'cy': cy, 'yaw': yaw, 'sx': sx, 'sy': sy,
            't_start': t_start, 't_end': t_end}


def test_point_to_box_measures_to_the_surface_of_a_ROTATED_rectangle():
    """
    A 1.0 x 0.2 m box turned 90 degrees presents its SHORT side.

    A centre-to-centre distance gives 2.0 and an axis-aligned reading gives
    1.5; only rotating the point into the box frame gives 1.9.
    """
    assert abs(point_to_box(2.0, 0.0, 0.0, 0.0, math.pi / 2.0, 1.0, 0.2) - 1.9) < 1e-12
    assert abs(point_to_box(2.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.2) - 1.5) < 1e-12


def test_point_to_box_is_negative_inside_and_measures_the_corner_outside():
    assert point_to_box(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.2) == -0.1
    # Diagonally off a corner: 3-4-5 triangle from the corner at (0.5, 0.1).
    assert abs(point_to_box(3.5, 4.1, 0.0, 0.0, 0.0, 1.0, 0.2) - 5.0) < 1e-12


def test_min_clearance_subtracts_the_footprint_and_reports_penetration():
    track = [(1.0, 0.0, 0.0, 0.0), (2.0, 1.3, 0.0, 0.0)]
    # At t=2 the robot centre is 0.2 m from the box face; a 0.18 m footprint
    # leaves 0.02 m.
    result = min_clearance(track, [_box()], radius=0.18)
    assert abs(result['value'] - 0.02) < 1e-12, result
    assert result['samples'] == 2
    # A wider footprint on the same track is a penetration, not a clamp to 0.
    overlapping = min_clearance(track, [_box()], radius=0.5)
    assert abs(overlapping['value'] + 0.3) < 1e-12, overlapping


def test_min_clearance_ignores_samples_outside_an_obstacle_s_lifetime():
    """A box that was not there yet cannot be approached."""
    track = [(1.0, 1.45, 0.0, 0.0), (5.0, 0.0, 0.0, 0.0)]
    result = min_clearance(track, [_box(t_start=4.0, t_end=9.0)], radius=0.18)
    assert result['samples'] == 1
    assert abs(result['value'] - (1.5 - 0.18)) < 1e-12, result


def test_min_clearance_reports_an_obstacle_with_no_samples_as_unmeasured():
    result = min_clearance([(1.0, 0.0, 0.0, 0.0)],
                           [_box(name='late', t_start=50.0, t_end=60.0)], radius=0.18)
    assert result['per_obstacle'] == {'late': None}
    assert result['value'] is None


def test_scan_divergence_joins_on_the_EXACT_stamp_and_drops_the_unpaired():
    scans = [((10, 0), [1.0, 2.0]), ((10, 100000000), [1.0, 9.0])]
    raw = [((10, 0), [1.0, 2.0])]
    result = scan_divergence(scans, raw, activation_t=10.05)
    # The second scan has no raw partner at that exact stamp: a nearest-stamp
    # join would have paired it with the first and called it divergent.
    assert result['pairs_compared'] == 1
    # ...and the sweep that did not join is COUNTED, not silently dropped:
    # 1 of 2 compared reads very differently from 1 of 1.
    assert result['pairs_unmatched'] == 1
    assert result['pairs_length_mismatch'] == 0
    assert result['pairs_divergent_before_activation'] == 0
    assert result['first_divergent_sim_s'] is None


def test_scan_divergence_counts_a_partner_of_a_different_length_as_uncompared():
    """A relay that changed the beam count compares nothing, and must say so."""
    result = scan_divergence([((7, 0), [1.0, 2.0])], [((7, 0), [1.0, 2.0, 3.0])],
                             activation_t=0.0)
    assert result['pairs_compared'] == 0
    assert result['pairs_length_mismatch'] == 1
    assert result['pairs_unmatched'] == 0
    assert result['mean_divergent_beam_fraction_after'] is None


def test_scan_divergence_counts_pre_activation_difference_rather_than_hiding_it():
    scans = [((5, 0), [1.0, 2.0]), ((20, 0), [1.0, 9.0])]
    raw = [((5, 0), [1.0, 2.5]), ((20, 0), [1.0, 2.0])]
    result = scan_divergence(scans, raw, activation_t=10.0)
    assert result['pairs_compared'] == 2
    assert result['pairs_divergent_before_activation'] == 1
    assert result['first_divergent_sim_s'] == 5.0
    assert abs(result['mean_divergent_beam_fraction_after'] - 0.5) < 1e-12


def test_scan_divergence_treats_two_no_returns_as_equal_and_inf_against_a_range_as_not():
    inf = float('inf')
    identical = scan_divergence([((1, 0), [inf, 1.0])], [((1, 0), [inf, 1.0])], 0.0)
    assert identical['mean_divergent_beam_fraction_after'] == 0.0
    blanked = scan_divergence([((1, 0), [inf, 1.0])], [((1, 0), [3.0, 1.0])], 0.0)
    assert abs(blanked['mean_divergent_beam_fraction_after'] - 0.5) < 1e-12
