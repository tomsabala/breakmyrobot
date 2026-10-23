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
The bag of one run in, metrics.json out.

Every metric is an OBJECT, never a bare number:

    {"value": ..., "unit": ..., "source": ..., "definition": ..., "reason": ...}

so a reader never has to guess where a number came from or what it counted.
`value` is null with a `reason` whenever an input is missing. There is no
default and no zero-for-no-data: "0 collisions" and "collisions could not be
counted because /bumper is not in the bag" are different claims, and only one
of them is a measurement (ENGINEERING.md section 8).

The BAG is the source of record. run.json is consulted only for the goal's
UUID, for the action status when the hidden action topic did not record, for
the mission window when /mission_state did not, and for the bring-up liveness
proof that makes `collision_count: 0` a measurement; meta says which source
each of those came from.

Truth arrives in the gz `world` frame and AMCL reports in `map`. Every
comparison between them applies world_to_map from the alignment sealed to the
map in use, through ral_eval.frames - never an implicit identity, which would
publish a constant frame offset as localization error.

The computations are pure functions over lists of (stamp, value) tuples so
they can be tested without a bag or a simulator; write_metrics only wires bag
topics into them.
"""

import argparse
import json
import math
import os
import sys

from action_msgs.msg import GoalStatus
from ral_eval.bagio import ral_stamp, read_bag, stamp_of, truth_track
from ral_eval.frames import wrap_to_pi
from ral_interfaces.msg import FaultEvent, MissionState

import yaml

# 2: est_error_floor_m was renamed to est_error_at_start_pose_m, and
# fault_to_response_s' null case became a null instead of an empty list. A v1
# metrics.json cannot be read by the v2 report script.
METRICS_VERSION = 2

STAMP_MATCH_S = 0.05        # truth<->estimate pairing window, as in the Phase 2 harness
CMD_ZERO_EPS = 0.01         # m/s and rad/s below which a command is "stopped"
COLLISION_DEBOUNCE_S = 0.5  # sim seconds, per colliding pair
PLANT_MAX_LINEAR = 0.46     # model.sdf DiffDrive max_linear_velocity
TELEPORT_FACTOR = 3.0       # a step longer than 3x the plant maximum is not travel
TELEPORT_SLACK_M = 0.05     # absolute slack, so stamp jitter alone cannot trip it
PLAN_RESAMPLE_M = 0.1       # plan comparison spacing
RESAMPLE_HZ = 10.0          # grid for commanded-vs-achieved comparison
SMOOTH_SAMPLES = 3          # moving-average width on the differentiated truth
COVARIANCE_LIMIT = 0.25     # trace below which AMCL is converged; == the runner's

# Fault-response detection. A fixed threshold with an unbounded forward search
# measures the controller, not the fault: sweeping synthetic activation times
# across runs where NO fault was ever applied, the old fixed 0.05 m/s
# cmd_changed trigger fired on 31 of 31 instants in a baseline run and 52 of 63
# in the obstacle run. Both material-change triggers are now calibrated against
# the run's own pre-activation window AND bounded to a horizon.
#
# RESPONSE_HORIZON_S is how long after an activation a change can still be
# attributed to the fault: 3.0 s sim is three replan opportunities at the BT's
# 1 Hz RateController. RESPONSE_BASELINE_WINDOW_S is the pre-activation window
# the threshold is calibrated on, and RESPONSE_BASELINE_MARGIN the multiple of
# the largest excursion found there that a post-activation change must exceed.
RESPONSE_HORIZON_S = 3.0
RESPONSE_BASELINE_WINDOW_S = 10.0
RESPONSE_BASELINE_MARGIN = 1.5
CMD_CHANGE_FLOOR = 0.05      # m/s, the floor the derived threshold cannot go below
PLAN_DIVERGENCE_FLOOR = 0.25  # m, likewise

# est_error_at_start_pose_m is measured while the body has not translated. An
# in-place warm-up rotation keeps xy fixed; this bound is what makes that an
# assertion rather than an assumption.
FLOOR_MAX_DISPLACEMENT_M = 0.05

# The BT wraps ComputePathToPose in <RateController hz="1.0">, so /plan
# republishes at 1 Hz with or without a fault. No plan-based detection can
# resolve a response faster than that, and the figure is reported beside the
# metric rather than left for a reader to assume.
PLAN_DETECTION_RESOLUTION_S = 1.0

GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'STATUS_UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'STATUS_ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'STATUS_EXECUTING',
    GoalStatus.STATUS_CANCELING: 'STATUS_CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'STATUS_SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'STATUS_CANCELED',
    GoalStatus.STATUS_ABORTED: 'STATUS_ABORTED',
}

TERMINAL_MISSION_STATES = (
    MissionState.STATE_SUCCEEDED,
    MissionState.STATE_ABORTED,
    MissionState.STATE_CANCELED,
    MissionState.STATE_UNREACHABLE,
    MissionState.STATE_REJECTED,
    MissionState.STATE_UNKNOWN,
)

TOPIC_TRUTH = '/ground_truth/odom'
TOPIC_AMCL = '/amcl_pose'
TOPIC_CMD = '/cmd_vel'
TOPIC_PLAN = '/plan'
TOPIC_BUMPER = '/bumper'
TOPIC_MISSION = '/mission_state'
TOPIC_FAULTS = '/fault_events'
TOPIC_INITIALPOSE = '/initialpose'
TOPIC_MAP = '/map'
TOPIC_ACTION_STATUS = '/navigate_to_pose/_action/status'
TOPIC_ACTION_FEEDBACK = '/navigate_to_pose/_action/feedback'

READ_TOPICS = [TOPIC_TRUTH, TOPIC_AMCL, TOPIC_CMD, TOPIC_PLAN, TOPIC_BUMPER,
               TOPIC_MISSION, TOPIC_FAULTS, TOPIC_INITIALPOSE, TOPIC_MAP,
               TOPIC_ACTION_STATUS, TOPIC_ACTION_FEEDBACK]


def metric(value, unit: str, source: str, definition: str, reason: str = None) -> dict:
    """One metric with its provenance. `value` None REQUIRES a reason."""
    if value is None and not reason:
        raise AssertionError('a null metric must carry a reason')
    return {'value': value, 'unit': unit, 'source': source,
            'definition': definition, 'reason': reason}


def _topic_reason(topic: str, status: dict, samples: list) -> 'str | None':
    """Why this topic yields no measurement, or None when it has samples."""
    state = status.get(topic, 'absent')
    if state == 'absent':
        return f'{topic} is not in the bag'
    if state.startswith('unresolved_type:'):
        return (f'{topic} is in the bag but its message type could not be loaded '
                f'({state.split(":", 1)[1].strip()}); its messages were NOT read')
    if not samples:
        return f'{topic} is in the bag and carries no messages'
    return None


# -- pure functions -----------------------------------------------------------
# Everything below is a function of plain tuples. No ROS, no bag, no file.

def pair_nearest(primary, secondary, window: float = STAMP_MATCH_S) -> list:
    """
    Pair each `primary` (t, value) with the nearest `secondary` within `window`.

    Returns [(t_primary, value_primary, value_secondary), ...]. A primary
    sample with no secondary inside the window is DROPPED, not paired with the
    closest one regardless of distance: a 2 s old truth pose compared against
    an estimate measures how fast the robot drove, not how well it localized.

    Not a matching: MANY primaries may consume the same secondary, which is
    the intended behaviour when the two streams run at different rates (AMCL
    publishes on filter update, truth at 50 Hz). An exact tie - a secondary
    equidistant on both sides - resolves to the LATER sample, because the
    candidate loop accepts `delta <= best_delta`.
    """
    if not primary or not secondary:
        return []
    ordered = sorted(secondary, key=lambda item: item[0])
    times = [item[0] for item in ordered]
    out = []
    for stamp, value in primary:
        index = _bisect(times, stamp)
        best = None
        best_delta = window
        for candidate in (index - 1, index):
            if 0 <= candidate < len(ordered):
                delta = abs(times[candidate] - stamp)
                if delta <= best_delta:
                    best = ordered[candidate]
                    best_delta = delta
        if best is not None:
            out.append((stamp, value, best[1]))
    return out


def _bisect(values: list, target: float) -> int:
    low, high = 0, len(values)
    while low < high:
        middle = (low + high) // 2
        if values[middle] < target:
            low = middle + 1
        else:
            high = middle
    return low


def path_length(track: list) -> dict:
    """
    Distance actually travelled, with teleports excluded and reported.

    `track` is [(t, x, y), ...] and MUST be time-ordered; bagio.truth_track
    sorts it. A decreasing stamp raises ValueError rather than being clamped:
    a negative dt collapses the step limit and would silently shed real travel
    into `teleport_excluded_m`, which is a wrong number reported confidently.

    A step longer than TELEPORT_FACTOR times what the plant could cover in its
    own dt (plus TELEPORT_SLACK_M for stamp jitter) is not travel: it is a
    kidnap, a respawn or a dropped interval. Including it would credit the
    robot with distance it never drove. Note that two samples sharing a stamp
    collapse the limit to TELEPORT_SLACK_M alone, so a genuine step between
    them would be excluded; that is reported in `excluded_steps`.
    """
    length = 0.0
    excluded = 0.0
    excluded_steps = 0
    for before, after in zip(track, track[1:]):
        dt = after[0] - before[0]
        if dt < 0.0:
            raise ValueError(
                f'path_length received a track whose stamps decrease: {before[0]} '
                f'followed by {after[0]}. The caller must sort before measuring; a '
                'negative dt would shed real travel into teleport_excluded_m.')
        step = math.hypot(after[1] - before[1], after[2] - before[2])
        limit = TELEPORT_FACTOR * PLANT_MAX_LINEAR * dt + TELEPORT_SLACK_M
        if step > limit:
            excluded += step
            excluded_steps += 1
        else:
            length += step
    return {'length': length, 'excluded': excluded, 'excluded_steps': excluded_steps,
            'samples': len(track)}


def stopped_time(cmds: list, t_start: float, t_end: float,
                 eps: float = CMD_ZERO_EPS) -> 'float | None':
    """
    Sim seconds inside [t_start, t_end] with the command below `eps`.

    `cmds` is [(t, linear, angular), ...]. Zero-order hold: a command is in
    force from its stamp until the next one, and the last one is held to
    t_end - which is what the plant does with it. Commands before t_start
    contribute the state at t_start, because the robot does not forget the
    last thing it was told at the window boundary.

    Returns None when NO command is at or before t_start: the state in force
    at the window start is then unknown, and reporting 0.0 would assert the
    robot never stopped during a window in which nothing was known.
    """
    if t_end <= t_start:
        return 0.0
    ordered = sorted(cmds, key=lambda item: item[0])
    total = 0.0
    current = None
    previous_t = t_start
    for stamp, linear, angular in ordered:
        if stamp <= t_start:
            current = (abs(linear) < eps and abs(angular) < eps)
            continue
        if stamp >= t_end:
            break
        if current is None:
            return None
        if current:
            total += stamp - previous_t
        previous_t = stamp
        current = (abs(linear) < eps and abs(angular) < eps)
    if current is None:
        return None
    if current:
        total += t_end - previous_t
    return total


def debounce_events(events: list, window: float = COLLISION_DEBOUNCE_S) -> int:
    """
    Count events, collapsing repeats of the SAME key within `window`.

    `events` is [(t, key), ...]. gz reports a contact every physics step while
    two bodies touch, so the raw message count measures how long a collision
    lasted, not how many there were. Debouncing is per key (the colliding
    pair), so two simultaneous contacts are two events.
    """
    last = {}
    count = 0
    for stamp, key in sorted(events, key=lambda item: item[0]):
        if key not in last or stamp - last[key] > window:
            count += 1
        last[key] = stamp
    return count


def resample_zoh(samples: list, grid: list):
    """Zero-order hold onto `grid`; None before the first sample."""
    ordered = sorted(samples, key=lambda item: item[0])
    out = []
    index = 0
    current = None
    for point in grid:
        while index < len(ordered) and ordered[index][0] <= point:
            current = ordered[index][1]
            index += 1
        out.append(current)
    return out


def resample_linear(samples: list, grid: list):
    """Linear interpolation onto `grid`; None outside the sampled interval."""
    ordered = sorted(samples, key=lambda item: item[0])
    if not ordered:
        return [None] * len(grid)
    times = [item[0] for item in ordered]
    out = []
    for point in grid:
        if point < times[0] or point > times[-1]:
            out.append(None)
            continue
        index = _bisect(times, point)
        if index == 0:
            out.append(ordered[0][1])
            continue
        t0, v0 = ordered[index - 1]
        t1, v1 = ordered[index]
        if t1 == t0:
            out.append(v1)
        else:
            out.append(v0 + (v1 - v0) * (point - t0) / (t1 - t0))
    return out


def unwrap_angles(values: list) -> list:
    """Continuous angle series, so interpolation never crosses the branch cut."""
    out = []
    offset = 0.0
    previous = None
    for value in values:
        if value is None:
            out.append(None)
            continue
        if previous is not None:
            offset += wrap_to_pi(value - previous) - (value - previous)
        out.append(value + offset)
        previous = value
    return out


def central_difference(values: list, dt: float, angular: bool = False) -> list:
    """
    d/dt of an evenly spaced series; one-sided at the ends, None where absent.

    `angular` folds each difference with wrap_to_pi, so a +3.10 -> -3.10 step
    is +0.0832 rad, not -6.2.
    """
    def delta(a, b):
        return wrap_to_pi(b - a) if angular else (b - a)

    out = []
    for index in range(len(values)):
        before = values[index - 1] if index > 0 else None
        after = values[index + 1] if index + 1 < len(values) else None
        here = values[index]
        if before is not None and after is not None:
            out.append(delta(before, after) / (2.0 * dt))
        elif here is not None and after is not None:
            out.append(delta(here, after) / dt)
        elif before is not None and here is not None:
            out.append(delta(before, here) / dt)
        else:
            out.append(None)
    return out


def moving_average(values: list, width: int = SMOOTH_SAMPLES) -> list:
    """Centred moving average over `width` samples, skipping Nones."""
    half = width // 2
    out = []
    for index in range(len(values)):
        window = [value for value in values[max(0, index - half):index + half + 1]
                  if value is not None]
        out.append(sum(window) / len(window) if window else None)
    return out


def _point_to_segment(px, py, ax, ay, bx, by) -> float:
    dx, dy = bx - ax, by - ay
    if dx == 0.0 and dy == 0.0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def resample_polyline(points: list, spacing: float = PLAN_RESAMPLE_M) -> list:
    """Points every `spacing` metres along a polyline, ends included."""
    if len(points) < 2:
        return list(points)
    lengths = [0.0]
    for before, after in zip(points, points[1:]):
        lengths.append(lengths[-1] + math.hypot(after[0] - before[0], after[1] - before[1]))
    total = lengths[-1]
    if total == 0.0:
        return [points[0]]
    count = max(2, int(math.ceil(total / spacing)) + 1)
    out = []
    for index in range(count):
        target = total * index / (count - 1)
        segment = _bisect(lengths, target)
        segment = max(1, min(segment, len(points) - 1))
        span = lengths[segment] - lengths[segment - 1]
        ratio = 0.0 if span == 0.0 else (target - lengths[segment - 1]) / span
        ax, ay = points[segment - 1]
        bx, by = points[segment]
        out.append((ax + (bx - ax) * ratio, ay + (by - ay) * ratio))
    return out


def plan_divergence(new_plan: list, reference: list,
                    spacing: float = PLAN_RESAMPLE_M) -> float:
    """
    How far the new plan leaves the reference plan, in metres.

    One-sided: the maximum over points of the NEW plan of the distance to the
    nearest point ON the reference polyline. Deliberately not a pointwise
    comparison by index or by arclength fraction - a plan that merely got
    shorter as the robot drove along it still lies on the old one and must
    score ~0, or every 1 Hz replan would read as a reaction to a fault.
    """
    if len(new_plan) < 1 or len(reference) < 2:
        return 0.0
    worst = 0.0
    for px, py in resample_polyline(new_plan, spacing):
        best = min(_point_to_segment(px, py, *reference[index], *reference[index + 1])
                   for index in range(len(reference) - 1))
        worst = max(worst, best)
    return worst


def first_response(activated_t: float, plans: list, cmds: list, recoveries: list,
                   horizon: float = RESPONSE_HORIZON_S) -> dict:
    """
    Return the first MATERIAL change after a fault activated, and what it was.

    `plans` is [(t, [(x, y), ...]), ...], `cmds` is [(t, linear, angular), ...],
    `recoveries` is [(t, count), ...] from Nav2's own feedback.

    Triggers, earliest wins, all bounded to (activated_t, activated_t + horizon]:
      plan_diverged    a plan leaving the last pre-activation plan by more than
                       max(PLAN_DIVERGENCE_FLOOR, RESPONSE_BASELINE_MARGIN x the
                       largest plan-to-plan divergence in the pre-activation
                       window)
      cmd_changed      |linear.x| moving from the command in force at activation
                       by more than max(CMD_CHANGE_FLOOR, RESPONSE_BASELINE_MARGIN
                       x the largest pre-activation excursion). SKIPPED, not
                       floored, when the pre-activation window holds fewer than
                       two commands: there is no null hypothesis to test against.
      recovery_started Nav2's own number_of_recoveries incrementing. An EVENT,
                       not an inference, so it keeps the floor-free comparison.

    # No angular sign-flip trigger. DWB reverses yaw rate constantly while
    # tracking a path: measured 18/31 and 44/63 firings at synthetic activation
    # times in runs where no fault was ever applied. A trigger that fires on
    # more than half of all instants measures the controller, not the fault.

    ALWAYS returns a dict, so the calibration is visible even when nothing
    fired. `trigger` is None in that case and `reason` says what was searched.
    """
    baseline_start = activated_t - RESPONSE_BASELINE_WINDOW_S
    limit = activated_t + horizon
    candidates = []

    # -- plan_diverged --------------------------------------------------------
    ordered_plans = sorted(plans, key=lambda item: item[0])
    reference = None
    for stamp, points in ordered_plans:
        if stamp <= activated_t:
            reference = points
    baseline_plans = [(stamp, points) for stamp, points in ordered_plans
                      if baseline_start <= stamp <= activated_t]
    plan_noise = 0.0
    for (_, before), (_, after) in zip(baseline_plans, baseline_plans[1:]):
        plan_noise = max(plan_noise, plan_divergence(after, before))
    plan_threshold = PLAN_DIVERGENCE_FLOOR
    if len(baseline_plans) >= 2:
        plan_threshold = max(PLAN_DIVERGENCE_FLOOR,
                             RESPONSE_BASELINE_MARGIN * plan_noise)
    if reference is not None and len(reference) >= 2:
        for stamp, points in ordered_plans:
            if not (activated_t < stamp <= limit):
                continue
            if plan_divergence(points, reference) > plan_threshold:
                candidates.append(('plan_diverged', stamp))
                break

    # -- cmd_changed ----------------------------------------------------------
    ordered_cmds = sorted(cmds, key=lambda item: item[0])
    base = None
    for stamp, linear, angular in ordered_cmds:
        if stamp <= activated_t:
            base = (linear, angular)
    baseline_cmds = [(stamp, linear, angular) for stamp, linear, angular in ordered_cmds
                     if baseline_start <= stamp <= activated_t]
    cmd_threshold = None
    cmd_excursion = None
    if base is not None and len(baseline_cmds) >= 2:
        cmd_excursion = max(abs(abs(linear) - abs(base[0]))
                            for _, linear, _ in baseline_cmds)
        cmd_threshold = max(CMD_CHANGE_FLOOR, RESPONSE_BASELINE_MARGIN * cmd_excursion)
        for stamp, linear, angular in ordered_cmds:
            if not (activated_t < stamp <= limit):
                continue
            if abs(abs(linear) - abs(base[0])) > cmd_threshold:
                candidates.append(('cmd_changed', stamp))
                break

    # -- recovery_started -----------------------------------------------------
    ordered_recoveries = sorted(recoveries, key=lambda item: item[0])
    base_recoveries = None
    for stamp, count in ordered_recoveries:
        if stamp <= activated_t:
            base_recoveries = count
    if base_recoveries is not None:
        for stamp, count in ordered_recoveries:
            if activated_t < stamp <= limit and count > base_recoveries:
                candidates.append(('recovery_started', stamp))
                break

    out = {
        'trigger': None, 'response_sim_s': None, 'delta_s': None,
        'horizon_s': horizon,
        'cmd_threshold_m_s': cmd_threshold,
        'cmd_baseline_excursion_m_s': cmd_excursion,
        'cmd_baseline_samples': len(baseline_cmds),
        'plan_threshold_m': plan_threshold,
        'plan_baseline_divergence_m': plan_noise,
        'plan_baseline_samples': len(baseline_plans),
        'reason': None,
    }
    if not candidates:
        searched = (f'no plan divergence above {plan_threshold:.3f} m, '
                    + ('command change above '
                       f'{cmd_threshold:.3f} m/s' if cmd_threshold is not None else
                       'command change (the cmd_changed trigger was skipped: '
                       f'{len(baseline_cmds)} command(s) in the '
                       f'{RESPONSE_BASELINE_WINDOW_S:.0f} s sim pre-activation window, '
                       'fewer than the 2 needed to calibrate a threshold)')
                    + f' or recovery within {horizon:.1f} s sim of activation')
        out['reason'] = searched
        return out
    trigger, stamp = min(candidates, key=lambda item: item[1])
    out['trigger'] = trigger
    out['response_sim_s'] = stamp
    out['delta_s'] = stamp - activated_t
    return out


# -- bag wiring ---------------------------------------------------------------

def _window(track, t0, t1):
    return [sample for sample in track if t0 <= sample[0] <= t1]


def _mission_window(mission_msgs, run_json):
    """[t_accept, t_terminal] from /mission_state, else run.json, else none."""
    accept = None
    terminal = None
    state = None
    for _, msg in mission_msgs:
        if msg.state == MissionState.STATE_GOAL_ACCEPTED and accept is None:
            accept = ral_stamp(msg)
        if msg.state in TERMINAL_MISSION_STATES and terminal is None:
            terminal = ral_stamp(msg)
            state = msg.state
    refused = None
    if accept is not None and terminal is not None:
        if terminal >= accept:
            return {'t_accept_sim': accept, 't_terminal_sim': terminal,
                    'source': TOPIC_MISSION, 'final_state': state}
        # A window that runs backwards would make mission_duration_s negative.
        refused = (f'{TOPIC_MISSION} rejected: terminal stamp precedes goal_accepted '
                   f'({terminal:.3f} < {accept:.3f} s sim)')
    if run_json.get('t_accept_sim') is not None \
            and run_json.get('t_terminal_sim') is not None:
        return {'t_accept_sim': run_json['t_accept_sim'],
                't_terminal_sim': run_json['t_terminal_sim'],
                'source': refused or 'run.json (no usable /mission_state pair in the bag)',
                'final_state': run_json.get('mission_state')}
    return {'t_accept_sim': None, 't_terminal_sim': None,
            'source': refused or (TOPIC_MISSION if mission_msgs else 'none'),
            'final_state': run_json.get('mission_state')}


def write_metrics(run_dir: str) -> dict:
    """Compute every metric for one run directory and write metrics.json."""
    bag_dir = os.path.join(run_dir, 'bag')
    with open(os.path.join(run_dir, 'run.json')) as handle:
        run_json = json.load(handle)
    with open(os.path.join(run_dir, 'env.json')) as handle:
        env = json.load(handle)
    with open(os.path.join(run_dir, 'scenario.yaml')) as handle:
        scenario = yaml.safe_load(handle)

    # The alignment is read from the run's OWN env.json copy, not from the
    # repository: a run carries the alignment it was scored with, so a later
    # remap cannot retroactively change an old run's numbers.
    alignment_doc = yaml.safe_load(env['alignment'])
    wtm = (float(alignment_doc['world_to_map']['x']),
           float(alignment_doc['world_to_map']['y']),
           float(alignment_doc['world_to_map']['yaw']))
    # The seal, verified from the artifact alone: the alignment copied into
    # env.json must have been measured against the grid env.json names. A
    # mismatch offsets every truth-vs-estimate number by a constant that reads
    # as localization error, which is the one failure the alignment cannot
    # survive and nothing downstream can detect.
    recorded_grid = str(alignment_doc.get('map_sha256'))
    if recorded_grid != str(env.get('map_sha256')):
        raise RuntimeError(
            f'{run_dir}: env.json records map_sha256 {env.get("map_sha256")} but the '
            f'alignment copied into it was measured against {recorded_grid}. Every '
            'truth-vs-estimate number would carry a constant frame offset.')
    alignment_source = (f'{os.path.basename(env["alignment_path"])} copied verbatim into '
                        'env.json')

    data, status = read_bag(bag_dir, READ_TOPICS)
    missing_topics = [name for name, state in status.items() if state == 'absent']

    truth_all = truth_track(data[TOPIC_TRUTH], wtm)
    amcl_all = sorted(
        ((stamp_of(msg, log_time), msg) for log_time, msg in data[TOPIC_AMCL]),
        key=lambda item: item[0])
    # geometry_msgs/Twist has no header: /cmd_vel can only be stamped by the
    # bag log time, which the recorder takes in sim time (--use-sim-time).
    cmd_all = [(log_time, msg.linear.x, msg.angular.z) for log_time, msg in data[TOPIC_CMD]]
    plan_all = [(stamp_of(msg, log_time),
                 [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses])
                for log_time, msg in data[TOPIC_PLAN]]
    mission_msgs = data[TOPIC_MISSION]
    fault_msgs = data[TOPIC_FAULTS]

    window = _mission_window(mission_msgs, run_json)
    t0 = window['t_accept_sim']
    t1 = window['t_terminal_sim']
    have_window = t0 is not None and t1 is not None
    no_window = (f'no mission window: final state '
                 f'{window["final_state"]!r}, outcome {run_json.get("outcome")!r}')

    metrics = {}

    def windowed(name, compute, unit, source, definition, missing=None):
        """Record a metric that needs the mission window and possibly a topic."""
        if missing:
            metrics[name] = metric(None, unit, source, definition, reason=missing)
            return
        if not have_window:
            metrics[name] = metric(None, unit, source, definition, reason=no_window)
            return
        metrics[name] = compute()

    # -- mission_duration_s ---------------------------------------------------
    windowed(
        'mission_duration_s',
        lambda: metric(t1 - t0, 's (sim)', window['source'],
                       'terminal MissionState stamp minus the goal_accepted stamp'),
        's (sim)', window['source'],
        'terminal MissionState stamp minus the goal_accepted stamp')

    # -- path_length_m / teleport_excluded_m ----------------------------------
    truth_window = _window(truth_all, t0, t1) if have_window else []
    truth_missing = _topic_reason(TOPIC_TRUTH, status, data[TOPIC_TRUTH])
    path_source = (f'{TOPIC_TRUTH} (frame world) mapped to map by {alignment_source}')
    path_definition = (
        'sum of |delta xy| between consecutive truth samples inside the mission window, '
        f'excluding any step longer than {TELEPORT_FACTOR}x the plant maximum '
        f'({PLANT_MAX_LINEAR} m/s) for its own dt plus {TELEPORT_SLACK_M} m')

    if truth_missing or not have_window:
        reason = truth_missing or no_window
        metrics['path_length_m'] = metric(None, 'm', path_source, path_definition, reason)
        metrics['teleport_excluded_m'] = metric(
            None, 'm', path_source, 'summed length of the steps path_length_m excluded',
            reason)
    else:
        walked = path_length([(s[0], s[1], s[2]) for s in truth_window])
        metrics['path_length_m'] = metric(
            walked['length'], 'm', path_source, path_definition)
        metrics['teleport_excluded_m'] = metric(
            walked['excluded'], 'm', path_source,
            f'summed length of the {walked["excluded_steps"]} step(s) path_length_m '
            'excluded as non-travel (kidnap, respawn or a dropped interval); a measured '
            'zero means no such step occurred')

    # -- goal_pose_error_m ----------------------------------------------------
    goal_map = run_json.get('goal_map')
    goal_source = (f'{TOPIC_TRUTH} mapped to map by {alignment_source}, against the '
                   'scenario goal converted the same way')
    goal_definition = ('|last truth sample at or before the terminal stamp, in map, '
                       'minus the goal in map|')
    if truth_missing or not have_window or not goal_map:
        metrics['goal_pose_error_m'] = metric(
            None, 'm', goal_source, goal_definition,
            truth_missing or (no_window if not have_window else 'run.json has no goal_map'))
    elif not truth_window:
        metrics['goal_pose_error_m'] = metric(
            None, 'm', goal_source, goal_definition,
            'no truth sample inside the mission window')
    else:
        final = truth_window[-1]
        metrics['goal_pose_error_m'] = metric(
            math.hypot(final[1] - goal_map[0], final[2] - goal_map[1]),
            'm', goal_source, goal_definition)

    # -- est_error_mean_m / est_error_max_m -----------------------------------
    est_source = (f'{TOPIC_AMCL} paired with {TOPIC_TRUTH} by header stamp within '
                  f'{STAMP_MATCH_S * 1000:.0f} ms, truth mapped to map by {alignment_source}')
    est_definition = ('mean and max |AMCL - truth| over the mission window; each AMCL '
                      'sample is paired with the nearest truth sample by HEADER stamp, '
                      'and unpaired samples are dropped rather than matched further out')
    amcl_missing = _topic_reason(TOPIC_AMCL, status, data[TOPIC_AMCL])
    amcl_window = [(stamp, msg) for stamp, msg in amcl_all if have_window and t0 <= stamp <= t1]
    pairs = pair_nearest(
        [(stamp, (msg.pose.pose.position.x, msg.pose.pose.position.y))
         for stamp, msg in amcl_window],
        [(sample[0], (sample[1], sample[2])) for sample in truth_window])
    pair_counts = {'est_error': len(pairs)}
    if not have_window:
        reason = no_window
    elif amcl_missing:
        reason = amcl_missing
    elif truth_missing:
        reason = truth_missing
    elif not pairs:
        reason = (f'no AMCL/truth pair within {STAMP_MATCH_S * 1000:.0f} ms inside the '
                  'mission window')
    else:
        reason = None
    if reason:
        metrics['est_error_mean_m'] = metric(None, 'm', est_source, est_definition, reason)
        metrics['est_error_max_m'] = metric(None, 'm', est_source, est_definition, reason)
    else:
        errors = [math.hypot(est[0] - truth[0], est[1] - truth[1])
                  for _, est, truth in pairs]
        metrics['est_error_mean_m'] = metric(
            sum(errors) / len(errors), 'm', est_source,
            f'{est_definition}; mean over {len(errors)} pairs')
        metrics['est_error_max_m'] = metric(
            max(errors), 'm', est_source, f'{est_definition}; max over {len(errors)} pairs')

    # -- est_error_at_start_pose_m --------------------------------------------
    # The scale every estimate-error number should be read against: while the
    # robot is still AT ITS START POSE, what is left is map error plus
    # world->map alignment error, not localization error. Phase 2 measured
    # 0.1128 m for this map. It is NOT a floor - the name used to say so and
    # the committed evidence already contradicted it.
    #
    # The window runs from convergence to goal acceptance, not from the last
    # non-zero command: AMCL publishes on filter UPDATE, and its filter updates
    # on motion, so a robot standing still publishes nothing at all. Measured:
    # zero /amcl_pose messages in the 2 s settle after the warm-up rotation
    # stopped. The warm-up rotation is in the window instead, which is sound
    # because an in-place rotation leaves xy unchanged - and that is asserted
    # here rather than assumed, by requiring every paired truth sample to lie
    # within FLOOR_MAX_DISPLACEMENT_M of the truth position at acceptance.
    start_pose_source = est_source
    start_pose_definition = (
        'mean |AMCL - truth| while the robot is at its start pose and rotating in place: '
        f'from the first {TOPIC_AMCL} whose covariance trace is below {COVARIANCE_LIMIT} '
        'to goal acceptance, excluding any pair whose truth position has already moved '
        f'more than {FLOOR_MAX_DISPLACEMENT_M} m from the truth position at acceptance. '
        'The robot is NOT stationary in this window - AMCL publishes on filter update and '
        'its filter updates on motion, so a stopped robot publishes nothing at all; the '
        'in-place warm-up rotation leaves xy unchanged, which is what the '
        f'{FLOOR_MAX_DISPLACEMENT_M} m guard asserts. The guard is applied to the TRUTH '
        'half of each pair; within the '
        f'{STAMP_MATCH_S * 1000:.0f} ms pairing window the body can travel at most '
        f'{PLANT_MAX_LINEAR * STAMP_MATCH_S:.3f} m, so the bound actually enforced on the '
        f'AMCL half is {FLOOR_MAX_DISPLACEMENT_M} + {PLANT_MAX_LINEAR * STAMP_MATCH_S:.3f} '
        'm. It is map error plus world->map alignment error AT THAT ONE POSE, measured, '
        'and it is the scale against which in-mission estimate error should be read. It '
        'is NOT a lower bound: both terms vary across the map, so in-mission error below '
        'this value is normal.')
    if not have_window:
        metrics['est_error_at_start_pose_m'] = metric(
            None, 'm', start_pose_source, start_pose_definition, no_window)
    else:
        converged_before = [stamp for stamp, msg in amcl_all
                            if stamp <= t0
                            and (msg.pose.covariance[0] + msg.pose.covariance[7]
                                 + msg.pose.covariance[35]) < COVARIANCE_LIMIT]
        truth_at_accept = [sample for sample in truth_all if sample[0] <= t0]
        if not converged_before:
            metrics['est_error_at_start_pose_m'] = metric(
                None, 'm', start_pose_source, start_pose_definition,
                f'no {TOPIC_AMCL} reached a covariance trace below {COVARIANCE_LIMIT} '
                'before goal acceptance, so there is no converged start-pose window')
        elif not truth_at_accept:
            metrics['est_error_at_start_pose_m'] = metric(
                None, 'm', start_pose_source, start_pose_definition,
                f'no {TOPIC_TRUTH} sample at or before goal acceptance')
        else:
            start_pose_begin = min(converged_before)
            anchor = truth_at_accept[-1]
            settle_pairs = pair_nearest(
                [(stamp, (msg.pose.pose.position.x, msg.pose.pose.position.y))
                 for stamp, msg in amcl_all if start_pose_begin <= stamp <= t0],
                [(sample[0], (sample[1], sample[2]))
                 for sample in _window(truth_all, start_pose_begin, t0)
                 if math.hypot(sample[1] - anchor[1],
                               sample[2] - anchor[2]) <= FLOOR_MAX_DISPLACEMENT_M])
            pair_counts['est_error_at_start_pose'] = len(settle_pairs)
            if not settle_pairs:
                metrics['est_error_at_start_pose_m'] = metric(
                    None, 'm', start_pose_source, start_pose_definition,
                    f'no AMCL/truth pair in the start-pose window '
                    f'[{start_pose_begin:.2f}, {t0:.2f}] s sim within '
                    f'{FLOOR_MAX_DISPLACEMENT_M} m of the start pose')
            else:
                values = [math.hypot(est[0] - truth[0], est[1] - truth[1])
                          for _, est, truth in settle_pairs]
                metrics['est_error_at_start_pose_m'] = metric(
                    sum(values) / len(values), 'm', start_pose_source,
                    f'{start_pose_definition}; mean over {len(values)} pairs in '
                    f'[{start_pose_begin:.2f}, {t0:.2f}] s sim')

    # -- est_cov_trace_max ----------------------------------------------------
    cov_definition = 'max of covariance[0] + covariance[7] + covariance[35] in the window'
    if not have_window:
        metrics['est_cov_trace_max'] = metric(None, '', TOPIC_AMCL, cov_definition, no_window)
    elif not amcl_window:
        metrics['est_cov_trace_max'] = metric(
            None, '', TOPIC_AMCL, cov_definition,
            amcl_missing or f'no {TOPIC_AMCL} message inside the mission window')
    else:
        metrics['est_cov_trace_max'] = metric(
            max(msg.pose.covariance[0] + msg.pose.covariance[7] + msg.pose.covariance[35]
                for _, msg in amcl_window),
            '', TOPIC_AMCL, cov_definition)

    # -- localization_converged_sim_s -----------------------------------------
    converge_source = f'{TOPIC_INITIALPOSE}, {TOPIC_AMCL}'
    converge_definition = (
        f'first {TOPIC_AMCL} whose covariance trace is below {COVARIANCE_LIMIT}, minus '
        f'the first {TOPIC_INITIALPOSE} stamp. AMCL updates its filter on MOTION, so the '
        'runner rotates the robot in place while waiting; this measures that wait')
    initialpose_missing = _topic_reason(TOPIC_INITIALPOSE, status, data[TOPIC_INITIALPOSE])
    if initialpose_missing:
        metrics['localization_converged_sim_s'] = metric(
            None, 's (sim)', converge_source, converge_definition, initialpose_missing)
    else:
        start = min(stamp_of(msg, log_time)
                    for log_time, msg in data[TOPIC_INITIALPOSE])
        converged = [stamp for stamp, msg in amcl_all
                     if stamp >= start
                     and (msg.pose.covariance[0] + msg.pose.covariance[7]
                          + msg.pose.covariance[35]) < COVARIANCE_LIMIT]
        if not converged:
            metrics['localization_converged_sim_s'] = metric(
                None, 's (sim)', converge_source, converge_definition,
                amcl_missing or (
                    f'no {TOPIC_AMCL} reached a covariance trace below '
                    f'{COVARIANCE_LIMIT} after the initial pose; the filter never '
                    'converged and the run says so'))
        else:
            metrics['localization_converged_sim_s'] = metric(
                min(converged) - start, 's (sim)', converge_source, converge_definition)

    # -- collision_count ------------------------------------------------------
    # /bumper is a gz contact sensor bridged GZ_TO_ROS, and a contact sensor
    # publishes ONLY on contact. `0 messages` is therefore indistinguishable
    # from a dead bridge unless the run itself proved the path was alive, which
    # is what run.json's bumper_liveness records at bring-up.
    liveness = run_json.get('bumper_liveness') or {}
    collision_definition = (
        f'{TOPIC_BUMPER} messages with a non-empty contacts list inside the window, '
        f'debounced at {COLLISION_DEBOUNCE_S} s sim per colliding pair (gz reports a '
        'contact every physics step while two bodies touch)')
    bumper_total = len(data[TOPIC_BUMPER])
    bumper_reason = None
    if status.get(TOPIC_BUMPER) != 'read':
        bumper_reason = _topic_reason(TOPIC_BUMPER, status, data[TOPIC_BUMPER])
    elif not have_window:
        bumper_reason = no_window
    elif bumper_total == 0 and not liveness.get('alive'):
        bumper_reason = (
            f'{TOPIC_BUMPER} recorded 0 messages and the contact path was not shown '
            f'to be alive in this run (ROS publishers: '
            f'{liveness.get("ros_publishers", "not checked")}; gz topic '
            f'{liveness.get("gz_topic", "?")}: {liveness.get("gz_detail", "not checked")}). '
            'A gz contact sensor publishes only on contact, so no-contact and '
            'no-sensor cannot be told apart here.')
    if bumper_reason:
        metrics['collision_count'] = metric(
            None, 'count', TOPIC_BUMPER, collision_definition, bumper_reason)
    else:
        raw = 0
        events = []
        for log_time, msg in data[TOPIC_BUMPER]:
            stamp = stamp_of(msg, log_time)
            if not (t0 <= stamp <= t1) or not msg.contacts:
                continue
            raw += 1
            for contact in msg.contacts:
                events.append((stamp, f'{contact.collision1.name}/{contact.collision2.name}'))
        if bumper_total:
            proof = (f'; {bumper_total} message(s) recorded and {raw} of them carried '
                     'contacts inside the window, so the contact path is live by its own '
                     'traffic')
        else:
            proof = (f'; 0 message(s) recorded. A gz contact sensor publishes only on '
                     'contact, so this zero is a measurement only because the path was '
                     f'shown to be alive in this run: {liveness.get("ros_publishers")} '
                     f'publishing {TOPIC_BUMPER}, and gz topic '
                     f'{liveness.get("gz_topic")} {liveness.get("gz_detail")}')
        metrics['collision_count'] = metric(
            debounce_events(events), 'count', TOPIC_BUMPER,
            f'{collision_definition}{proof}')

    # -- plan_publications ----------------------------------------------------
    plan_definition = (
        f'count of {TOPIC_PLAN} messages inside the window. This is how often the '
        'behaviour tree\'s RateController (hz=1.0) published a path, NOT a count of '
        'replans: the BT republishes at 1 Hz with or without a fault')
    plan_missing = _topic_reason(TOPIC_PLAN, status, data[TOPIC_PLAN])
    if status.get(TOPIC_PLAN, 'absent') != 'read':
        metrics['plan_publications'] = metric(
            None, 'count', TOPIC_PLAN, plan_definition, plan_missing)
    elif not have_window:
        metrics['plan_publications'] = metric(
            None, 'count', TOPIC_PLAN, plan_definition, no_window)
    else:
        metrics['plan_publications'] = metric(
            len([stamp for stamp, _ in plan_all if t0 <= stamp <= t1]),
            'count', TOPIC_PLAN, plan_definition)

    # -- stopped_time_s -------------------------------------------------------
    stopped_definition = (
        f'sim seconds inside the window with |linear.x| < {CMD_ZERO_EPS} and '
        f'|angular.z| < {CMD_ZERO_EPS}, zero-order hold on the bag log time (sim), the '
        'last command held to the terminal stamp')
    cmd_missing = _topic_reason(TOPIC_CMD, status, data[TOPIC_CMD])
    if cmd_missing:
        metrics['stopped_time_s'] = metric(
            None, 's (sim)', TOPIC_CMD, stopped_definition, cmd_missing)
    elif not have_window:
        metrics['stopped_time_s'] = metric(
            None, 's (sim)', TOPIC_CMD, stopped_definition, no_window)
    else:
        stopped = stopped_time(cmd_all, t0, t1)
        if stopped is None:
            metrics['stopped_time_s'] = metric(
                None, 's (sim)', TOPIC_CMD, stopped_definition,
                f'no {TOPIC_CMD} message at or before the window start, so the command '
                'in force at t_accept is unknown')
        else:
            metrics['stopped_time_s'] = metric(
                stopped, 's (sim)', TOPIC_CMD, stopped_definition)

    # -- nav2_result ----------------------------------------------------------
    result_definition = (
        'the action status of THIS goal, by UUID, as the server reported it. '
        'NavigateToPose\'s result payload is std_msgs/Empty, so the GoalStatus code is '
        'the only verbatim outcome the action offers')
    goal_uuid = run_json.get('goal_uuid')
    status_source = TOPIC_ACTION_STATUS
    # Explicitly the LATEST by bag log time, not "whatever the reader yielded
    # last": a future reader that returns stamp order could otherwise pick a
    # mid-mission STATUS_EXECUTING and report it as the outcome.
    status_entries = []
    for log_time, msg in data[TOPIC_ACTION_STATUS]:
        for entry in msg.status_list:
            if goal_uuid and bytes(entry.goal_info.goal_id.uuid).hex() == goal_uuid:
                status_entries.append((log_time, int(entry.status)))
    status_value = max(status_entries)[1] if status_entries else None
    if status_value is None:
        status_source = 'run.json'
        status_value = run_json.get('nav2_status_code')
    if status_value is None:
        metrics['nav2_result'] = metric(
            None, '', status_source, result_definition,
            f'no status for goal {goal_uuid} on {TOPIC_ACTION_STATUS} and none recorded '
            'in run.json')
    else:
        metrics['nav2_result'] = metric(
            {'code': status_value,
             'name': GOAL_STATUS_NAMES.get(status_value, f'unknown:status_{status_value}')},
            '', status_source, result_definition)

    # -- recoveries -----------------------------------------------------------
    recovery_definition = (
        'max number_of_recoveries across NavigateToPose feedback inside the window. '
        'Nav2\'s own count of behaviour-tree recovery executions, never inferred from '
        '/plan or /cmd_vel')
    feedback_samples = [(stamp_of(msg.feedback, log_time), msg.feedback)
                        for log_time, msg in data[TOPIC_ACTION_FEEDBACK]]
    feedback_missing = _topic_reason(
        TOPIC_ACTION_FEEDBACK, status, data[TOPIC_ACTION_FEEDBACK])
    if status.get(TOPIC_ACTION_FEEDBACK, 'absent') == 'absent':
        metrics['recoveries'] = metric(
            None, 'count', TOPIC_ACTION_FEEDBACK, recovery_definition,
            f'{TOPIC_ACTION_FEEDBACK} is not in the bag (the hidden action topic did not '
            'record); a recovery count is not synthesised from other streams')
    elif feedback_missing:
        metrics['recoveries'] = metric(
            None, 'count', TOPIC_ACTION_FEEDBACK, recovery_definition, feedback_missing)
    elif not have_window:
        metrics['recoveries'] = metric(
            None, 'count', TOPIC_ACTION_FEEDBACK, recovery_definition, no_window)
    else:
        counts = [int(feedback.number_of_recoveries)
                  for stamp, feedback in feedback_samples if t0 <= stamp <= t1]
        if not counts:
            metrics['recoveries'] = metric(
                None, 'count', TOPIC_ACTION_FEEDBACK, recovery_definition,
                'no action feedback inside the mission window')
        else:
            metrics['recoveries'] = metric(
                max(counts), 'count', TOPIC_ACTION_FEEDBACK, recovery_definition)

    # -- cmd_vs_achieved ------------------------------------------------------
    achieved_source = f'{TOPIC_CMD} (zero-order hold) and {TOPIC_TRUTH} (differentiated)'
    achieved_definition = (
        f'commanded and achieved velocity resampled onto a {RESAMPLE_HZ} Hz grid over the '
        'window and compared sample by sample. Achieved velocity is a central difference '
        'of truth POSE, smoothed over '
        f'{SMOOTH_SAMPLES} samples: forward speed is the body-frame projection of the xy '
        'difference on the heading, angular rate is the wrapped yaw difference. The truth '
        'message does carry a twist, but it is a raw 50 Hz finite difference stamped on a '
        'different grid and is marked unknown (twist.covariance[0] = -1) whenever two '
        'poses share a stamp, so it is not used here. The command series is stamped by '
        'the recorder and the truth series by the publisher; the measured offset between '
        'the two bases in this stack is under 35 ms, against a '
        f'{1000.0 / RESAMPLE_HZ:.0f} ms grid step')
    if not have_window:
        metrics['cmd_vs_achieved'] = metric(
            None, 'm/s and rad/s', achieved_source, achieved_definition, no_window)
    elif not cmd_all or len(truth_window) < 3:
        metrics['cmd_vs_achieved'] = metric(
            None, 'm/s and rad/s', achieved_source, achieved_definition,
            cmd_missing or truth_missing or (
                f'{TOPIC_CMD} has {len(cmd_all)} message(s) and truth has '
                f'{len(truth_window)} sample(s) in the window; at least 3 truth samples '
                'are needed for a central difference'))
    else:
        step = 1.0 / RESAMPLE_HZ
        count = max(3, int((t1 - t0) / step) + 1)
        grid = [t0 + index * step for index in range(count)]
        cmd_linear = resample_zoh([(s[0], s[1]) for s in cmd_all], grid)
        cmd_angular = resample_zoh([(s[0], s[2]) for s in cmd_all], grid)
        truth_x = resample_linear([(s[0], s[1]) for s in truth_window], grid)
        truth_y = resample_linear([(s[0], s[2]) for s in truth_window], grid)
        truth_yaw = resample_linear(
            list(zip([s[0] for s in truth_window],
                     unwrap_angles([s[3] for s in truth_window]))), grid)
        dx = moving_average(central_difference(truth_x, step))
        dy = moving_average(central_difference(truth_y, step))
        achieved_angular = moving_average(central_difference(truth_yaw, step))
        achieved_linear = []
        for index in range(count):
            if dx[index] is None or dy[index] is None or truth_yaw[index] is None:
                achieved_linear.append(None)
            else:
                achieved_linear.append(dx[index] * math.cos(truth_yaw[index])
                                       + dy[index] * math.sin(truth_yaw[index]))
        linear_errors = [abs(cmd_linear[index] - achieved_linear[index])
                         for index in range(count)
                         if cmd_linear[index] is not None
                         and achieved_linear[index] is not None]
        angular_errors = [abs(cmd_angular[index] - achieved_angular[index])
                          for index in range(count)
                          if cmd_angular[index] is not None
                          and achieved_angular[index] is not None]
        pair_counts['cmd_vs_achieved'] = len(linear_errors)
        if not linear_errors or not angular_errors:
            metrics['cmd_vs_achieved'] = metric(
                None, 'm/s and rad/s', achieved_source, achieved_definition,
                'no grid sample had both a command in force and a differentiable truth '
                'pose')
        else:
            metrics['cmd_vs_achieved'] = metric(
                {'linear': {'mean_abs_error': sum(linear_errors) / len(linear_errors),
                            'max_abs_error': max(linear_errors)},
                 'angular': {'mean_abs_error': sum(angular_errors) / len(angular_errors),
                             'max_abs_error': max(angular_errors)},
                 'samples': len(linear_errors)},
                'm/s and rad/s', achieved_source, achieved_definition)

    # -- fault_to_response_s --------------------------------------------------
    response_source = f'{TOPIC_FAULTS}, {TOPIC_PLAN}, {TOPIC_CMD}, {TOPIC_ACTION_FEEDBACK}'
    response_definition = (
        'per FaultEvent that reached state=activated: the first MATERIAL change after it. '
        f'plan_diverged = a {TOPIC_PLAN} leaving the last pre-activation plan by more than '
        f'max({PLAN_DIVERGENCE_FLOOR} m, {RESPONSE_BASELINE_MARGIN}x the largest '
        'plan-to-plan divergence in this run\'s pre-activation window); cmd_changed = '
        f'|linear.x| moving by more than max({CMD_CHANGE_FLOOR} m/s, '
        f'{RESPONSE_BASELINE_MARGIN}x the largest pre-activation excursion), skipped '
        'entirely when fewer than two commands precede the activation; recovery_started = '
        'Nav2\'s number_of_recoveries incrementing. Earliest wins, and the trigger is '
        'recorded. Both material-change triggers are calibrated against this run\'s own '
        f'pre-activation window ({RESPONSE_BASELINE_WINDOW_S:.0f} s sim) and bounded to '
        f'{RESPONSE_HORIZON_S:.0f} s sim after activation. At these settings, synthetic '
        'activation times swept across fault-free runs still fired cmd_changed on 2 of 31 '
        'and 5 of 63 instants and plan_diverged on 0 of 31 and 3 of 63: a single reported '
        'response is weak evidence on its own and must be read beside a control run')
    activations = [(ral_stamp(msg), msg) for _, msg in fault_msgs
                   if msg.state == FaultEvent.STATE_ACTIVATED]
    if not activations:
        states = sorted({msg.state for _, msg in fault_msgs})
        metrics['fault_to_response_s'] = metric(
            None, 'list of s (sim)', response_source, response_definition,
            reason=f'no FaultEvent reached state=activated; states seen: '
                   f'{states or "none"}')
    else:
        recovery_series = [(stamp, int(feedback.number_of_recoveries))
                           for stamp, feedback in feedback_samples]
        entries = []
        for stamp, msg in activations:
            found = first_response(stamp, plan_all, cmd_all, recovery_series)
            entries.append({
                'id': msg.id, 'kind': msg.kind, 'activated_sim_s': stamp,
                'response_sim_s': found['response_sim_s'],
                'delta_s': found['delta_s'],
                'trigger': found['trigger'],
                'horizon_s': found['horizon_s'],
                'cmd_threshold_m_s': found['cmd_threshold_m_s'],
                'cmd_baseline_excursion_m_s': found['cmd_baseline_excursion_m_s'],
                'cmd_baseline_samples': found['cmd_baseline_samples'],
                'plan_threshold_m': found['plan_threshold_m'],
                'plan_baseline_divergence_m': found['plan_baseline_divergence_m'],
                'plan_baseline_samples': found['plan_baseline_samples'],
                'reason': found['reason'],
            })
        metrics['fault_to_response_s'] = metric(
            entries, 'list of s (sim)', response_source, response_definition)

    payload = {
        'meta': {
            'experiment_id': run_json.get('experiment_id'),
            'run_dir': run_dir,
            'bag': bag_dir,
            'scenario': {'world': scenario.get('world'), 'robot': scenario.get('robot'),
                         'seed': scenario.get('seed')},
            'mission_window': window,
            'world_to_map': {'x': wtm[0], 'y': wtm[1], 'yaw': wtm[2]},
            'world_to_map_source': env['alignment_path'],
            'map_sha256': env.get('map_sha256'),
            'nav2_result_source': status_source,
            'pair_counts': pair_counts,
            'missing_topics': missing_topics,
            'topic_status': status,
            'plan_detection_resolution_s': PLAN_DETECTION_RESOLUTION_S,
            'plan_detection_resolution_source':
                'nav2_bt_navigator navigate_to_pose_w_replanning_and_recovery.xml: '
                'ComputePathToPose is wrapped in <RateController hz="1.0">',
            'metrics_version': METRICS_VERSION,
        },
        'metrics': metrics,
    }
    path = os.path.join(run_dir, 'metrics.json')
    temporary = path + '.partial'
    with open(temporary, 'w') as handle:
        json.dump(payload, handle, indent=2)
        handle.write('\n')
    os.replace(temporary, path)
    print(f'--- wrote {path}', flush=True)
    return payload


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description='Compute metrics.json for one run.')
    parser.add_argument('--run-dir', required=True)
    parsed = parser.parse_args(args)
    write_metrics(parsed.run_dir)
    sys.exit(0)


if __name__ == '__main__':
    main()
