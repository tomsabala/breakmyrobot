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
the mission window when /mission_state did not, for the measured
world->ekf_local alignment of an estimator with its own origin, and for the
bring-up liveness proof that makes `collision_count: 0` a measurement; meta
says which source each of those came from.

Nothing here knows a robot by name. WHAT the estimate is, which topic carries
it, whether the platform is commanded in velocity at all, how fast its plant
can go and how wide its footprint is are read from the ROBOT PROFILE the run
copied verbatim into env.json - the same discipline the alignment follows, so
a later profile edit cannot change an old run's numbers. A metric a platform
cannot support is a null with a reason naming why, never an absent key: an
absent key and an unmeasurable quantity read identically.

Truth arrives in the gz `world` frame; an estimate arrives in whatever frame
its profile declares. Every comparison between them applies world_to_map from
the alignment sealed to the map in use, through ral_eval.frames - and, for an
estimator whose origin is its own (ArduPilot's EKF3), the per-run MEASURED
world->ekf_local first. Never an implicit identity, which would publish a
constant frame offset as localization error.

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
from ral_eval.frames import map_pose_to_world, world_pose_to_map, wrap_to_pi
from ral_interfaces.msg import FaultEvent, MissionState

import yaml

# 2: est_error_floor_m was renamed to est_error_at_start_pose_m, and
# fault_to_response_s' null case became a null instead of an empty list. A v1
# metrics.json cannot be read by the v2 report script.
# 3: obstacle_min_clearance_m and scan_degradation_divergence were added, and
# /scan and /scan_raw are read from the bag to compute the second. A v2
# reader sees two metric keys it does not know; a v3 reader asked for a v2
# file finds them absent.
# 4: scan_degradation_divergence gained pairs_unmatched and
# pairs_length_mismatch - the sweeps that were NOT compared, which a v3 file
# does not carry at all and whose absence cannot be told from zero.
# scripts/checks/repeatability_report.py asserts this number against its own
# METRICS_VERSION_SUPPORTED and refuses an older run, so the bump is
# load-bearing rather than documentation.
# 5: the estimate source, the plant maximum and the footprint radius are read
# from the run's own robot profile instead of module constants; `nav2_result`
# became `mission_result`; `est_pos_horiz_accuracy_max_m`, `est_status_flags`
# and `speed_vs_target` were added for platforms with no pose covariance and
# no velocity command. A v4 reader sees a renamed key.
METRICS_VERSION = 5

STAMP_MATCH_S = 0.05        # truth<->estimate pairing window, as in the Phase 2 harness
CMD_ZERO_EPS = 0.01         # m/s and rad/s below which a command is "stopped"
COLLISION_DEBOUNCE_S = 0.5  # sim seconds, per colliding pair
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
# attributed to the fault: 2.0 s sim is two replan opportunities at the BT's
# 1 Hz RateController. It was 3.0 s until a sweep over the regenerated
# fault-free bags fired cmd_changed on 6 of 38 instants in one baseline, above
# the 4-in-31 rate the setting was chosen against. The horizon is the term
# that dominates, so it is the one that was lowered: widening
# RESPONSE_BASELINE_MARGIN instead would hide a genuine slow-down fault.
# RESPONSE_BASELINE_WINDOW_S is the pre-activation window the threshold is
# calibrated on, and RESPONSE_BASELINE_MARGIN the multiple of the largest
# excursion found there that a post-activation change must exceed.
RESPONSE_HORIZON_S = 2.0
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

# The mission state each terminal GoalStatus corresponds to, the same mapping
# the runner publishes with. Any other code is `unknown`: a non-terminal
# status arriving as the last one means the mission's end was never reported,
# and naming it `aborted` would claim a status the server never sent.
NAV2_STATUS_STATES = {
    GoalStatus.STATUS_SUCCEEDED: MissionState.STATE_SUCCEEDED,
    GoalStatus.STATUS_ABORTED: MissionState.STATE_ABORTED,
    GoalStatus.STATUS_CANCELED: MissionState.STATE_CANCELED,
}

TERMINAL_MISSION_STATES = (
    MissionState.STATE_SUCCEEDED,
    MissionState.STATE_ABORTED,
    MissionState.STATE_CANCELED,
    MissionState.STATE_UNREACHABLE,
    MissionState.STATE_REJECTED,
    MissionState.STATE_UNKNOWN,
)

# The topics whose NAMES are a property of a stack rather than of a robot:
# every platform's truth, mission and fault streams, and the fixed topics the
# Nav2 and MAVROS stacks each publish under. The estimate topic, the
# uncertainty topic and the command topic are NOT here: they are per robot,
# they come out of the run's own profile, and a constant for them is exactly
# the hardcoding this module stopped doing in v5.
TOPIC_TRUTH = '/ground_truth/odom'
TOPIC_PLAN = '/plan'
TOPIC_BUMPER = '/bumper'
TOPIC_MISSION = '/mission_state'
TOPIC_FAULTS = '/fault_events'
TOPIC_INITIALPOSE = '/initialpose'
TOPIC_MAP = '/map'
TOPIC_ACTION_STATUS = '/navigate_to_pose/_action/status'
TOPIC_ACTION_FEEDBACK = '/navigate_to_pose/_action/feedback'
TOPIC_MAVROS_STATE = '/mavros/state'
TOPIC_MAVROS_NAV_OUTPUT = '/mavros/nav_controller_output/output'

# Both halves of the lidar path: scan_degradation_divergence is the proof
# that the degraded stream and the raw one differ only after activation.
TOPIC_SCAN = '/scan'
TOPIC_SCAN_RAW = '/scan_raw'

# Read by every platform: truth, the mission vocabulary, the fault stream, the
# two lidar halves and the contact sensor exist on any robot this project
# integrates, because they are what an experiment is scored from.
READ_TOPICS_COMMON = (TOPIC_TRUTH, TOPIC_MISSION, TOPIC_FAULTS,
                      TOPIC_SCAN, TOPIC_SCAN_RAW, TOPIC_BUMPER)


def read_topics(profile: dict) -> list:
    """
    Every topic this run's metrics are computed from, for ONE robot profile.

    Driven entirely by the profile, so adding a platform does not edit this
    module: the estimate and its uncertainty come from `estimate`, the
    commanded velocity from `command` only when the platform IS commanded in
    velocity, and the per-stack topics from `mission_driver`. A topic asked
    for and never recorded is reported by read_bag as 'absent' and becomes a
    null metric with a reason; a topic never asked for would instead vanish
    silently, which is why this list is derived rather than guessed.

    test_mission_vocabulary asserts, for every committed profile, that this
    list is covered by what the runner actually records.
    """
    wanted = list(READ_TOPICS_COMMON)
    if TOPIC_MAP in [entry['topic'] for entry in profile['bringup_topics']]:
        wanted.append(TOPIC_MAP)
    estimate = profile['estimate']
    wanted.append(estimate['topic'])
    uncertainty = estimate.get('uncertainty') or {}
    if uncertainty.get('topic'):
        wanted.append(uncertainty['topic'])
    command = profile['command']
    if command['kind'] == 'velocity_twist':
        wanted.append(command['topic'])
    driver = profile['mission_driver']
    if driver == 'nav2':
        # /initialpose is the start of localization_converged_sim_s, and the
        # two action topics are the only verbatim outcome NavigateToPose has.
        wanted += [TOPIC_INITIALPOSE, TOPIC_PLAN,
                   TOPIC_ACTION_STATUS, TOPIC_ACTION_FEEDBACK]
    elif driver == 'mavros_guided':
        # mode/armed decide every terminal outcome, and wp_dist is the FCU's
        # own distance to go.
        wanted += [TOPIC_MAVROS_STATE, TOPIC_MAVROS_NAV_OUTPUT]
    else:
        raise ValueError(
            f'robot profile {profile.get("robot")!r} declares mission_driver '
            f'{driver!r}, which metrics.py has no topic set for. Add it here beside '
            'the driver in ral_eval/mission_driver.py rather than letting a run be '
            'scored from the common topics alone.')
    # Deduplicated, order preserved: a profile may name a topic the common set
    # already carries, and read_bag keys its status dict by name.
    return list(dict.fromkeys(wanted))


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


def estimate_track(samples, estimate_frame, wtm, wtl) -> list:
    """
    Return the estimate as [(stamp, x_map, y_map)], converted once.

    `wtl` is the measured world->ekf_local alignment, or None for a frame that
    is already `map`. An ekf_local pose is composed into `world` first, then
    into `map` by the same world_to_map every truth sample uses.

    `samples` is [(stamp, msg)]. The message SHAPE comes from the profile's
    estimate descriptor, not from inspection: PoseWithCovarianceStamped and
    Odometry both carry the pose at msg.pose.pose, and any other type raises
    naming the profile key rather than being guessed at - a guessed accessor
    would publish some other message's numbers as this robot's estimate.

    map_pose_to_world is the right call for the first composition even though
    neither frame is `map`: it composes a pose expressed in a CHILD frame
    into the parent, given the child origin's pose in the parent, which is
    exactly what `wtl` is (world_to_ekf_local = the ekf_local origin measured
    in `world`). Writing a second SE(2) composition here would be a second
    place for a sign error to live.
    """
    out = []
    for stamp, msg in samples:
        type_name = str(getattr(msg, '_type', '')) or type(msg).__name__
        if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
        else:
            raise ValueError(
                f'the robot profile declares estimate.type for a message shaped like '
                f'{type_name}, which carries no pose.pose. metrics.py reads '
                'geometry_msgs/msg/PoseWithCovarianceStamped and nav_msgs/msg/Odometry; '
                'add the accessor beside this check rather than guessing one.')
        if estimate_frame == 'map':
            out.append((stamp, x, y))
            continue
        if wtl is None:
            raise ValueError(
                f'the robot profile declares estimate.frame {estimate_frame!r}, which is '
                'not `map`, and run.json carries no world_to_ekf_local measurement to '
                'compose it with. Scoring the estimate as though it were already in '
                '`map` would report the unmeasured frame offset as localization error.')
        wx, wy, _ = map_pose_to_world(x, y, 0.0, wtl)
        mx, my, _ = world_pose_to_map(wx, wy, 0.0, wtm)
        out.append((stamp, mx, my))
    return out


def path_length(track: list, max_linear: float) -> dict:
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

    `max_linear` is the plant maximum, in m/s, and has NO default: it is a
    property of the robot, read from the run's own profile. A guessed value
    silently re-labels real travel as a teleport on a faster platform, and
    real teleports as travel on a slower one.
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
        limit = TELEPORT_FACTOR * max_linear * dt + TELEPORT_SLACK_M
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


def achieved_velocity(track: list, t_start: float, t_end: float):
    """
    (grid, forward m/s, yaw rate rad/s) differentiated from truth POSE.

    `track` is [(t, x, y, yaw), ...]. One implementation, three readers:
    cmd_vs_achieved compares it against the velocity command, speed_vs_target
    compares it against a cruise speed on a platform that has no velocity
    command, and plot_run draws it. A second copy would let the figure and
    metrics.json disagree about phase or sample count.

    Forward speed is the body-frame projection of the xy difference on the
    heading, so a reversing vehicle reads negative rather than fast.
    """
    step = 1.0 / RESAMPLE_HZ
    count = max(3, int((t_end - t_start) / step) + 1)
    grid = [t_start + index * step for index in range(count)]
    truth_x = resample_linear([(s[0], s[1]) for s in track], grid)
    truth_y = resample_linear([(s[0], s[2]) for s in track], grid)
    truth_yaw = resample_linear(
        list(zip([s[0] for s in track], unwrap_angles([s[3] for s in track]))), grid)
    dx = moving_average(central_difference(truth_x, step), SMOOTH_SAMPLES)
    dy = moving_average(central_difference(truth_y, step), SMOOTH_SAMPLES)
    angular = moving_average(central_difference(truth_yaw, step), SMOOTH_SAMPLES)
    linear = []
    for index in range(count):
        if dx[index] is None or dy[index] is None or truth_yaw[index] is None:
            linear.append(None)
        else:
            linear.append(dx[index] * math.cos(truth_yaw[index])
                          + dy[index] * math.sin(truth_yaw[index]))
    return grid, linear, angular


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


def first_response_mavros(activated_t: float, modes: list, speeds: list,
                          horizon: float = RESPONSE_HORIZON_S) -> dict:
    """
    Measure the same response on a platform with no plan and no velocity command.

    `modes` is [(t, mode_string), ...] from /mavros/state and `speeds` is
    [(t, achieved_m_s), ...] - achieved forward speed differentiated from
    SIMULATOR TRUTH. The truth speed is an EVALUATION signal, not something
    the robot knew: ArduRover publishes no velocity command to compare
    against, so the only thing left that a fault can be seen to change is the
    motion itself, and the honest label for that is "what the simulator says
    happened".

    Triggers, earliest wins, all bounded to (activated_t, activated_t + horizon]:
      mode_changed    the autopilot's own mode string differing from the one
                      in force at activation. An EVENT, not an inference, so
                      it keeps the floor-free comparison exactly as
                      recovery_started does in first_response.
      speed_changed   achieved speed moving from the speed at activation by
                      more than max(CMD_CHANGE_FLOOR, RESPONSE_BASELINE_MARGIN
                      x the largest pre-activation excursion). SKIPPED, not
                      floored, when the pre-activation window holds fewer than
                      two samples: there is no null hypothesis to test against.

    ALWAYS returns a dict, so the calibration is visible even when nothing
    fired. `trigger` is None in that case and `reason` says what was searched.
    """
    baseline_start = activated_t - RESPONSE_BASELINE_WINDOW_S
    limit = activated_t + horizon
    candidates = []

    # -- mode_changed ---------------------------------------------------------
    ordered_modes = sorted(modes, key=lambda item: item[0])
    base_mode = None
    for stamp, mode in ordered_modes:
        if stamp <= activated_t:
            base_mode = mode
    if base_mode is not None:
        for stamp, mode in ordered_modes:
            if activated_t < stamp <= limit and mode != base_mode:
                candidates.append(('mode_changed', stamp))
                break

    # -- speed_changed --------------------------------------------------------
    ordered_speeds = sorted(speeds, key=lambda item: item[0])
    base_speed = None
    for stamp, speed in ordered_speeds:
        if stamp <= activated_t:
            base_speed = speed
    baseline_speeds = [(stamp, speed) for stamp, speed in ordered_speeds
                       if baseline_start <= stamp <= activated_t]
    speed_threshold = None
    speed_excursion = None
    if base_speed is not None and len(baseline_speeds) >= 2:
        speed_excursion = max(abs(abs(speed) - abs(base_speed))
                              for _, speed in baseline_speeds)
        speed_threshold = max(CMD_CHANGE_FLOOR,
                              RESPONSE_BASELINE_MARGIN * speed_excursion)
        for stamp, speed in ordered_speeds:
            if not (activated_t < stamp <= limit):
                continue
            if abs(abs(speed) - abs(base_speed)) > speed_threshold:
                candidates.append(('speed_changed', stamp))
                break

    out = {
        'trigger': None, 'response_sim_s': None, 'delta_s': None,
        'horizon_s': horizon,
        'speed_threshold_m_s': speed_threshold,
        'speed_baseline_excursion_m_s': speed_excursion,
        'speed_baseline_samples': len(baseline_speeds),
        'mode_at_activation': base_mode,
        'reason': None,
    }
    if not candidates:
        out['reason'] = (
            'no autopilot mode change, '
            + ('no achieved-speed change above '
               f'{speed_threshold:.3f} m/s' if speed_threshold is not None else
               'no achieved-speed change (the speed_changed trigger was skipped: '
               f'{len(baseline_speeds)} truth speed sample(s) in the '
               f'{RESPONSE_BASELINE_WINDOW_S:.0f} s sim pre-activation window, '
               'fewer than the 2 needed to calibrate a threshold)')
            + f', within {horizon:.1f} s sim of activation')
        return out
    trigger, stamp = min(candidates, key=lambda item: item[1])
    out['trigger'] = trigger
    out['response_sim_s'] = stamp
    out['delta_s'] = stamp - activated_t
    return out


def point_to_box(px: float, py: float, cx: float, cy: float, yaw: float,
                 sx: float, sy: float) -> float:
    """
    Signed distance from a point to an ORIENTED rectangle's surface, in map.

    Negative inside. The box is rotated, so the point is rotated into the
    box's frame first: a centre-to-centre distance, or an axis-aligned
    comparison in `map`, gives a different number for every obstacle whose
    yaw is not zero - and `map` is rotated relative to `world` by the measured
    alignment, so EVERY obstacle is rotated here.
    """
    dx_world = px - cx
    dy_world = py - cy
    local_x = dx_world * math.cos(-yaw) - dy_world * math.sin(-yaw)
    local_y = dx_world * math.sin(-yaw) + dy_world * math.cos(-yaw)
    dx = abs(local_x) - sx / 2.0
    dy = abs(local_y) - sy / 2.0
    if dx > 0.0 or dy > 0.0:
        return math.hypot(max(dx, 0.0), max(dy, 0.0))
    return max(dx, dy)


def min_clearance(track: list, boxes: list, radius: float) -> dict:
    """
    Smallest (distance from truth centre to box surface - radius) over time.

    `track`  [(stamp, x_map, y_map, yaw_map)]
    `boxes`  [{'name', 'cx', 'cy', 'yaw', 'sx', 'sy', 't_start', 't_end'}], in
             map, each measured only while it EXISTED: a box is not an
             obstacle before it was spawned or after it was removed, and
             counting those samples would report a clearance against thin air.
    Returns {'value', 'per_obstacle': {name: value}, 'samples'}. A NEGATIVE
    value is penetration: the footprint circle overlapped the box.

    The footprint is a circle of `radius`, which is what the costmaps were
    configured with (nav2_roomba.yaml robot_radius) - not the true square
    hull. For a differential-drive disc that is the same shape; the number is
    labelled with the radius it used.
    """
    per_obstacle = {}
    samples = 0
    for box in boxes:
        best = None
        for stamp, x, y, _yaw in track:
            if stamp < box['t_start'] or stamp > box['t_end']:
                continue
            clearance = point_to_box(x, y, box['cx'], box['cy'], box['yaw'],
                                     box['sx'], box['sy']) - radius
            samples += 1
            if best is None or clearance < best:
                best = clearance
        per_obstacle[box['name']] = best
    measured = [value for value in per_obstacle.values() if value is not None]
    return {'value': min(measured) if measured else None,
            'per_obstacle': per_obstacle, 'samples': samples}


def scan_divergence(scan_samples: list, raw_samples: list,
                    activation_t: float) -> dict:
    """
    Compare /scan against /scan_raw on IDENTICAL header stamps.

    `scan_samples` and `raw_samples` are [((sec, nanosec), ranges)]. The join
    is EXACT on the stamp key, not pair_nearest: the relay republishes the
    sensor's own header, so the same sweep carries the same stamp on both
    topics, and a near-miss pairing would compare two different sweeps and
    report the robot's motion as degradation.

    Two ranges are equal when both are +inf or when they are bit-for-bit
    equal. A tolerance would hide exactly the small additive noise the fault
    injects.

    Returns pairs_compared, pairs_divergent_before_activation (which must be 0
    for "the streams differ only after activation" to hold - a non-zero value
    is the measurement, not an error), first_divergent_sim_s,
    mean_divergent_beam_fraction_after, and the two counts of sweeps that were
    NOT compared: pairs_unmatched (no /scan_raw entry carries that stamp) and
    pairs_length_mismatch (a partner was found with a different beam count).
    Without those, pairs_compared = 40 reads identically whether 40 of 40 or
    40 of 400 sweeps joined; a non-zero count means the relay or the recorder
    dropped sweeps, and the comparison covers less than it appears to.
    """
    raw_by_stamp = dict(raw_samples)
    compared = 0
    unmatched = 0
    length_mismatch = 0
    divergent_before = 0
    first_divergent = None
    fractions_after = []
    for key, ranges in sorted(scan_samples, key=lambda item: item[0]):
        raw = raw_by_stamp.get(key)
        if raw is None:
            unmatched += 1
            continue
        if len(raw) != len(ranges):
            length_mismatch += 1
            continue
        compared += 1
        differing = sum(1 for a, b in zip(ranges, raw) if not _same_range(a, b))
        stamp = key[0] + key[1] * 1e-9
        if differing:
            if first_divergent is None or stamp < first_divergent:
                first_divergent = stamp
            if stamp < activation_t:
                divergent_before += 1
        if stamp >= activation_t:
            fractions_after.append(differing / len(ranges) if ranges else 0.0)
    return {
        'pairs_compared': compared,
        'pairs_unmatched': unmatched,
        'pairs_length_mismatch': length_mismatch,
        'pairs_divergent_before_activation': divergent_before,
        'first_divergent_sim_s': first_divergent,
        'mean_divergent_beam_fraction_after':
            (sum(fractions_after) / len(fractions_after)) if fractions_after else None,
        'activated_sim_s': activation_t,
    }


def _same_range(a: float, b: float) -> bool:
    """Equal when both are the no-return marker, or exactly equal."""
    a_inf = math.isinf(a)
    b_inf = math.isinf(b)
    if a_inf or b_inf:
        return a_inf and b_inf
    return a == b


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

    # The robot profile, read from the run's OWN env.json copy for the same
    # reason the alignment is: what the estimate IS, how fast the plant can
    # go and how wide the footprint is are properties of the robot that ran,
    # and a later profile edit must not change an old run's numbers.
    profile = yaml.safe_load(env['robot_profile'])
    estimate_descriptor = profile['estimate']
    command_descriptor = profile['command']
    driver = profile['mission_driver']
    is_nav2 = driver == 'nav2'
    estimate_topic = estimate_descriptor['topic']
    estimate_label = estimate_descriptor['label']
    uncertainty = estimate_descriptor.get('uncertainty') or {}
    uncertainty_kind = uncertainty.get('kind')
    uncertainty_topic = uncertainty.get('topic')
    command_topic = command_descriptor['topic']
    is_velocity_command = command_descriptor['kind'] == 'velocity_twist'
    plant_max_linear = float(profile['plant']['max_linear_m_s'])

    # world -> ekf_local: the MEASURED pose of the estimator's own origin in
    # the gz world frame, made per run by ekf_local_tf and recorded in
    # run.json. None for a platform whose estimate is already in `map`;
    # estimate_track refuses rather than assuming an identity.
    wtl_doc = run_json.get('world_to_ekf_local')
    wtl = None
    if wtl_doc:
        wtl = (float(wtl_doc['x']), float(wtl_doc['y']), float(wtl_doc['yaw']))

    data, status = read_bag(bag_dir, read_topics(profile))
    missing_topics = [name for name, state in status.items() if state == 'absent']

    def samples(topic):
        """Bag samples for a topic THIS profile may not have asked for."""
        return data.get(topic, [])

    truth_all = truth_track(data[TOPIC_TRUTH], wtm)
    estimate_msgs = sorted(
        ((stamp_of(msg, log_time), msg) for log_time, msg in samples(estimate_topic)),
        key=lambda item: item[0])
    # [(stamp, x_map, y_map)]: converted ONCE, here, so every estimate number
    # below is in the same frame as truth.
    estimate_all = estimate_track(estimate_msgs, estimate_descriptor['frame'], wtm, wtl)
    # geometry_msgs/Twist has no header: /cmd_vel can only be stamped by the
    # bag log time, which the recorder takes in sim time (--use-sim-time).
    cmd_all = [(log_time, msg.linear.x, msg.angular.z)
               for log_time, msg in samples(command_topic)] if is_velocity_command else []
    plan_all = [(stamp_of(msg, log_time),
                 [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses])
                for log_time, msg in samples(TOPIC_PLAN)]
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
        f'excluding any step longer than {TELEPORT_FACTOR}x this plant maximum '
        f'({plant_max_linear} m/s, from the run\'s robot profile) for its own dt plus '
        f'{TELEPORT_SLACK_M} m')

    if truth_missing or not have_window:
        reason = truth_missing or no_window
        metrics['path_length_m'] = metric(None, 'm', path_source, path_definition, reason)
        metrics['teleport_excluded_m'] = metric(
            None, 'm', path_source, 'summed length of the steps path_length_m excluded',
            reason)
    else:
        walked = path_length([(s[0], s[1], s[2]) for s in truth_window], plant_max_linear)
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
    est_source = (f'{estimate_topic} ({estimate_label}) paired with {TOPIC_TRUTH} by '
                  f'header stamp within {STAMP_MATCH_S * 1000:.0f} ms, truth mapped to '
                  f'map by {alignment_source}'
                  + ('' if estimate_descriptor['frame'] == 'map' else
                     f', and the estimate composed from {estimate_descriptor["frame"]} '
                     'into map through the measured world_to_ekf_local in run.json'))
    est_definition = (f'mean and max |estimate - truth| over the mission window, the '
                      f'estimate being {estimate_label}; each estimate sample is paired '
                      'with the nearest truth sample by HEADER stamp, and unpaired '
                      'samples are dropped rather than matched further out')
    estimate_missing = _topic_reason(estimate_topic, status, samples(estimate_topic))
    estimate_window = [sample for sample in estimate_all
                       if have_window and t0 <= sample[0] <= t1]
    # The covariance half needs the messages, not the converted track.
    estimate_msg_window = [(stamp, msg) for stamp, msg in estimate_msgs
                           if have_window and t0 <= stamp <= t1]
    pairs = pair_nearest(
        [(sample[0], (sample[1], sample[2])) for sample in estimate_window],
        [(sample[0], (sample[1], sample[2])) for sample in truth_window])
    pair_counts = {'est_error': len(pairs)}
    if not have_window:
        reason = no_window
    elif estimate_missing:
        reason = estimate_missing
    elif truth_missing:
        reason = truth_missing
    elif not pairs:
        reason = (f'no estimate/truth pair within {STAMP_MATCH_S * 1000:.0f} ms inside '
                  'the mission window')
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
    #
    # The whole measurement exists only for an estimator that reports a
    # covariance AND is initialized by an operator: it is the covariance
    # crossing that opens the window. A platform with neither reports null
    # with that as the reason, rather than a number taken from some other
    # instant.
    no_initialization = (
        f'{estimate_label} has no operator-triggered initialization to measure from: '
        'the estimator sets its own origin at the autopilot\'s first GPS fix, nothing '
        f'publishes {TOPIC_INITIALPOSE} on this platform, and its uncertainty kind is '
        f'{uncertainty_kind!r} rather than a pose covariance whose crossing could open '
        'a start-pose window')
    start_pose_source = est_source
    start_pose_definition = (
        'mean |estimate - truth| while the robot is at its start pose and rotating in '
        f'place: from the first {estimate_topic} whose covariance trace is below '
        f'{COVARIANCE_LIMIT} to goal acceptance, excluding any pair whose truth position '
        f'has already moved more than {FLOOR_MAX_DISPLACEMENT_M} m from the truth '
        'position at acceptance. '
        'The robot is NOT stationary in this window - AMCL publishes on filter update and '
        'its filter updates on motion, so a stopped robot publishes nothing at all; the '
        'in-place warm-up rotation leaves xy unchanged, which is what the '
        f'{FLOOR_MAX_DISPLACEMENT_M} m guard asserts. The guard is applied to the TRUTH '
        'half of each pair; within the '
        f'{STAMP_MATCH_S * 1000:.0f} ms pairing window the body can travel at most '
        f'{plant_max_linear * STAMP_MATCH_S:.3f} m, so the bound actually enforced on the '
        f'estimate half is {FLOOR_MAX_DISPLACEMENT_M} + '
        f'{plant_max_linear * STAMP_MATCH_S:.3f} '
        'm. It is map error plus world->map alignment error AT THAT ONE POSE, measured, '
        'and it is the scale against which in-mission estimate error should be read. It '
        'is NOT a lower bound: both terms vary across the map, so in-mission error below '
        'this value is normal.')
    if uncertainty_kind != 'covariance_trace':
        metrics['est_error_at_start_pose_m'] = metric(
            None, 'm', start_pose_source, start_pose_definition, no_initialization)
    elif not have_window:
        metrics['est_error_at_start_pose_m'] = metric(
            None, 'm', start_pose_source, start_pose_definition, no_window)
    else:
        converged_before = [stamp for stamp, msg in estimate_msgs
                            if stamp <= t0
                            and (msg.pose.covariance[0] + msg.pose.covariance[7]
                                 + msg.pose.covariance[35]) < COVARIANCE_LIMIT]
        truth_at_accept = [sample for sample in truth_all if sample[0] <= t0]
        if not converged_before:
            metrics['est_error_at_start_pose_m'] = metric(
                None, 'm', start_pose_source, start_pose_definition,
                f'no {estimate_topic} reached a covariance trace below {COVARIANCE_LIMIT} '
                'before goal acceptance, so there is no converged start-pose window')
        elif not truth_at_accept:
            metrics['est_error_at_start_pose_m'] = metric(
                None, 'm', start_pose_source, start_pose_definition,
                f'no {TOPIC_TRUTH} sample at or before goal acceptance')
        else:
            start_pose_begin = min(converged_before)
            anchor = truth_at_accept[-1]
            settle_pairs = pair_nearest(
                [(sample[0], (sample[1], sample[2])) for sample in estimate_all
                 if start_pose_begin <= sample[0] <= t0],
                [(sample[0], (sample[1], sample[2]))
                 for sample in _window(truth_all, start_pose_begin, t0)
                 if math.hypot(sample[1] - anchor[1],
                               sample[2] - anchor[2]) <= FLOOR_MAX_DISPLACEMENT_M])
            pair_counts['est_error_at_start_pose'] = len(settle_pairs)
            if not settle_pairs:
                metrics['est_error_at_start_pose_m'] = metric(
                    None, 'm', start_pose_source, start_pose_definition,
                    f'no estimate/truth pair in the start-pose window '
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
    if uncertainty_kind != 'covariance_trace':
        metrics['est_cov_trace_max'] = metric(
            None, '', estimate_topic, cov_definition,
            'ArduPilot does not send LOCAL_POSITION_NED_COV, so '
            '/mavros/local_position/* carries no covariance. What this platform\'s '
            'estimator CAN report about itself is its ESTIMATOR_STATUS flags, in '
            'est_status_flags - whether they actually arrived on this run is stated '
            'in that metric\'s own reason')
    elif not have_window:
        metrics['est_cov_trace_max'] = metric(
            None, '', estimate_topic, cov_definition, no_window)
    elif not estimate_msg_window:
        metrics['est_cov_trace_max'] = metric(
            None, '', estimate_topic, cov_definition,
            estimate_missing or f'no {estimate_topic} message inside the mission window')
    else:
        metrics['est_cov_trace_max'] = metric(
            max(msg.pose.covariance[0] + msg.pose.covariance[7] + msg.pose.covariance[35]
                for _, msg in estimate_msg_window),
            '', estimate_topic, cov_definition)

    # -- est_pos_horiz_accuracy_max_m -----------------------------------------
    # The counterpart for an estimator that reports a status rather than a
    # covariance. VERIFIED, not assumed: `ros2 interface show
    # mavros_msgs/msg/EstimatorStatus` in this image lists a header and twelve
    # booleans - the ESTIMATOR_STATUS_FLAGS bitmask, republished flag by flag.
    # The MAVLink ESTIMATOR_STATUS message's pos_horiz_accuracy and
    # pos_horiz_ratio fields are NOT carried across by this version of mavros,
    # so there is no accuracy number in the bag to take a maximum of. The
    # metric stays, null with that as its reason, because the alternative is
    # an absent key - and a reader cannot tell an absent key from an
    # unmeasured one. The accessor is probed rather than hardcoded, so a mavros
    # that does carry the fields fills this in without an edit here.
    accuracy_source = uncertainty_topic or 'none (the profile names no uncertainty topic)'
    accuracy_definition = (
        f'max pos_horiz_accuracy and max pos_horiz_ratio over {accuracy_source} inside '
        'the mission window: the estimator\'s OWN assessment of its horizontal position '
        'accuracy, which is what a platform with no pose covariance can honestly report')
    # The uncertainty window, computed ONCE for both metrics below. It used to
    # be bound inside the accuracy metric's `else:` branch while the flags
    # metric read the same names - so on a covariance platform the flags
    # metric referred to a name that branch had never bound.
    uncertainty_status_msgs = []
    uncertainty_in_window = []
    uncertainty_carried = []
    if uncertainty_kind == 'estimator_status':
        uncertainty_status_msgs = [(stamp_of(msg, log_time), msg)
                                   for log_time, msg in samples(uncertainty_topic)]
        uncertainty_in_window = [msg for stamp, msg in uncertainty_status_msgs
                                 if have_window and t0 <= stamp <= t1]
        uncertainty_carried = [msg for msg in uncertainty_in_window
                               if hasattr(msg, 'pos_horiz_accuracy')
                               and hasattr(msg, 'pos_horiz_ratio')]

    if uncertainty_kind != 'estimator_status':
        metrics['est_pos_horiz_accuracy_max_m'] = metric(
            None, 'm', accuracy_source, accuracy_definition,
            'nav2_amcl publishes no ESTIMATOR_STATUS; this platform reports '
            'est_cov_trace_max instead')
    elif not have_window:
        metrics['est_pos_horiz_accuracy_max_m'] = metric(
            None, 'm', accuracy_source, accuracy_definition, no_window)
    elif not uncertainty_in_window:
        metrics['est_pos_horiz_accuracy_max_m'] = metric(
            None, 'm', accuracy_source, accuracy_definition,
            _topic_reason(uncertainty_topic, status, samples(uncertainty_topic))
            or f'no {uncertainty_topic} message inside the mission window')
    elif not uncertainty_carried:
        metrics['est_pos_horiz_accuracy_max_m'] = metric(
            None, 'm', accuracy_source, accuracy_definition,
            f'{uncertainty_topic} carries '
            f'{type(uncertainty_in_window[0]).__module__.split(".")[0]}'
            f'/{type(uncertainty_in_window[0]).__name__}, which has no '
            'pos_horiz_accuracy and no pos_horiz_ratio field: mavros republishes '
            'ESTIMATOR_STATUS as its flag bits only (verified with `ros2 interface '
            'show mavros_msgs/msg/EstimatorStatus`), so the accuracy numbers MAVLink '
            f'defines never reach the bag. {len(uncertainty_in_window)} status '
            'message(s) were recorded inside the window and their flags are in the '
            'bag; this platform therefore publishes NO numeric pose uncertainty at all')
    else:
        metrics['est_pos_horiz_accuracy_max_m'] = metric(
            {'pos_horiz_accuracy_m': max(float(msg.pos_horiz_accuracy)
                                         for msg in uncertainty_carried),
             'pos_horiz_ratio_max': max(float(msg.pos_horiz_ratio)
                                        for msg in uncertainty_carried),
             'samples': len(uncertainty_carried)},
            'm', accuracy_source, accuracy_definition)

    # -- est_status_flags -----------------------------------------------------
    # The EKF's own health flags, summarised as the fraction of in-window
    # messages in which each was true: the estimator's assessment of itself,
    # published by the robot and never simulator truth. pos_horiz_abs and
    # gps_glitch are the two a kidnap run is read through.
    #
    # MEASURED on this build: it does not arrive. The launch file's MAVLink2
    # SET_MESSAGE_INTERVAL request for ESTIMATOR_STATUS is answered
    # success=False by this ArduRover (visible in every car launch.log), so
    # /mavros/estimator_status is recorded and EMPTY and all three uncertainty
    # metrics - est_cov_trace_max, est_pos_horiz_accuracy_max_m and this one -
    # are null. That is an absence of EVIDENCE, not an absence of a number,
    # and each reason says so rather than pointing at a sibling metric that is
    # also null. The truth-vs-estimate error is unaffected: it is measured
    # from the estimate topic, which does stream.
    flags_definition = (
        f'fraction of {accuracy_source} messages inside the mission window in which '
        'each ESTIMATOR_STATUS flag was true, plus the message count. The estimator\'s '
        'OWN assessment of which of its states it considers healthy')
    if uncertainty_kind != 'estimator_status':
        metrics['est_status_flags'] = metric(
            None, 'fraction', accuracy_source, flags_definition,
            'nav2_amcl publishes no ESTIMATOR_STATUS; this platform reports its '
            'filter health as est_cov_trace_max instead')
    elif not have_window:
        metrics['est_status_flags'] = metric(
            None, 'fraction', accuracy_source, flags_definition, no_window)
    elif not uncertainty_in_window:
        metrics['est_status_flags'] = metric(
            None, 'fraction', accuracy_source, flags_definition,
            _topic_reason(uncertainty_topic, status, samples(uncertainty_topic))
            or f'no {uncertainty_topic} message inside the mission window')
    else:
        names = sorted(
            field for field in type(uncertainty_in_window[0]).get_fields_and_field_types()
            if field.endswith('_status_flag'))
        metrics['est_status_flags'] = metric(
            dict({name: sum(bool(getattr(msg, name)) for msg in uncertainty_in_window)
                  / len(uncertainty_in_window) for name in names},
                 samples=len(uncertainty_in_window)),
            'fraction', accuracy_source, flags_definition)

    # -- localization_converged_sim_s -----------------------------------------
    converge_source = f'{TOPIC_INITIALPOSE}, {estimate_topic}'
    converge_definition = (
        f'first {estimate_topic} whose covariance trace is below {COVARIANCE_LIMIT}, '
        f'minus the first {TOPIC_INITIALPOSE} stamp. AMCL updates its filter on MOTION, '
        'so the runner rotates the robot in place while waiting; this measures that wait')
    if uncertainty_kind != 'covariance_trace':
        metrics['localization_converged_sim_s'] = metric(
            None, 's (sim)', converge_source, converge_definition, no_initialization)
    else:
        initialpose_missing = _topic_reason(
            TOPIC_INITIALPOSE, status, samples(TOPIC_INITIALPOSE))
        if initialpose_missing:
            metrics['localization_converged_sim_s'] = metric(
                None, 's (sim)', converge_source, converge_definition, initialpose_missing)
        else:
            start = min(stamp_of(msg, log_time)
                        for log_time, msg in samples(TOPIC_INITIALPOSE))
            converged = [stamp for stamp, msg in estimate_msgs
                         if stamp >= start
                         and (msg.pose.covariance[0] + msg.pose.covariance[7]
                              + msg.pose.covariance[35]) < COVARIANCE_LIMIT]
            if not converged:
                metrics['localization_converged_sim_s'] = metric(
                    None, 's (sim)', converge_source, converge_definition,
                    estimate_missing or (
                        f'no {estimate_topic} reached a covariance trace below '
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
    not_nav2_plan = (f'{TOPIC_PLAN} exists only in the Nav2 stack; this platform plans '
                     'inside the autopilot and publishes no path')
    if not is_nav2:
        metrics['plan_publications'] = metric(
            None, 'count', TOPIC_PLAN, plan_definition, not_nav2_plan)
    elif status.get(TOPIC_PLAN, 'absent') != 'read':
        metrics['plan_publications'] = metric(
            None, 'count', TOPIC_PLAN, plan_definition,
            _topic_reason(TOPIC_PLAN, status, samples(TOPIC_PLAN)))
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
    not_velocity_commanded = (
        'ArduRover is commanded by position setpoint, not velocity; its throttle output '
        'is PWM and not a velocity command')
    cmd_missing = (_topic_reason(command_topic, status, samples(command_topic))
                   if is_velocity_command else None)
    if not is_velocity_command:
        metrics['stopped_time_s'] = metric(
            None, 's (sim)', command_topic, stopped_definition, not_velocity_commanded)
    elif cmd_missing:
        metrics['stopped_time_s'] = metric(
            None, 's (sim)', command_topic, stopped_definition, cmd_missing)
    elif not have_window:
        metrics['stopped_time_s'] = metric(
            None, 's (sim)', command_topic, stopped_definition, no_window)
    else:
        stopped = stopped_time(cmd_all, t0, t1)
        if stopped is None:
            metrics['stopped_time_s'] = metric(
                None, 's (sim)', command_topic, stopped_definition,
                f'no {command_topic} message at or before the window start, so the '
                'command in force at t_accept is unknown')
        else:
            metrics['stopped_time_s'] = metric(
                stopped, 's (sim)', command_topic, stopped_definition)

    # -- mission_result -------------------------------------------------------
    # One key for both stacks, and the value says WHICH stack spoke. For Nav2
    # that is the action status of this goal by UUID; for any other driver it
    # is the terminal /mission_state the runner published, whose `outcome` the
    # driver itself filled in from the autopilot's own telemetry. The outcome
    # string carries its source as a prefix in either case, exactly as
    # MissionState.msg defines the closed set.
    nav2_result_definition = (
        'the action status of THIS goal, by UUID, as the server reported it. '
        'NavigateToPose\'s result payload is std_msgs/Empty, so the GoalStatus code is '
        'the only verbatim outcome the action offers')
    mission_result_definition = (
        'the terminal MissionState the runner published: its state, its source-prefixed '
        'outcome and the topic it was read from. The driver fills the outcome in from '
        'the autopilot\'s own telemetry - mode, armed and wp_dist - never from truth '
        'and never from an inference about why the mission ended')
    goal_uuid = run_json.get('goal_uuid')
    if is_nav2:
        result_definition = nav2_result_definition
        status_source = TOPIC_ACTION_STATUS
        # Explicitly the LATEST by bag log time, not "whatever the reader yielded
        # last": a future reader that returns stamp order could otherwise pick a
        # mid-mission STATUS_EXECUTING and report it as the outcome.
        status_entries = []
        for log_time, msg in samples(TOPIC_ACTION_STATUS):
            for entry in msg.status_list:
                if goal_uuid and bytes(entry.goal_info.goal_id.uuid).hex() == goal_uuid:
                    status_entries.append((log_time, int(entry.status)))
        status_value = max(status_entries)[1] if status_entries else None
        if status_value is None:
            status_source = 'run.json'
            status_value = run_json.get('nav2_status_code')
        if status_value is None:
            metrics['mission_result'] = metric(
                None, '', status_source, result_definition,
                f'no status for goal {goal_uuid} on {TOPIC_ACTION_STATUS} and none '
                'recorded in run.json')
        else:
            name = GOAL_STATUS_NAMES.get(status_value, f'unknown:status_{status_value}')
            metrics['mission_result'] = metric(
                {'state': NAV2_STATUS_STATES.get(status_value,
                                                 MissionState.STATE_UNKNOWN),
                 'outcome': f'nav2:{name}',
                 'source': status_source},
                '', status_source, result_definition)
    else:
        result_definition = mission_result_definition
        status_source = TOPIC_MISSION
        terminal_msg = next((msg for _, msg in mission_msgs
                             if msg.state in TERMINAL_MISSION_STATES), None)
        if terminal_msg is None:
            metrics['mission_result'] = metric(
                None, '', status_source, result_definition,
                f'no terminal state on {TOPIC_MISSION} in the bag; run.json records '
                f'state {run_json.get("mission_state")!r} and outcome '
                f'{run_json.get("outcome")!r}, which is the runner\'s own copy rather '
                'than a recorded message')
        else:
            metrics['mission_result'] = metric(
                {'state': terminal_msg.state,
                 'outcome': terminal_msg.outcome,
                 'source': TOPIC_MISSION},
                '', status_source, result_definition)

    # -- recoveries -----------------------------------------------------------
    recovery_definition = (
        'max number_of_recoveries across NavigateToPose feedback inside the window. '
        'Nav2\'s own count of behaviour-tree recovery executions, never inferred from '
        '/plan or /cmd_vel')
    feedback_samples = [(stamp_of(msg.feedback, log_time), msg.feedback)
                        for log_time, msg in samples(TOPIC_ACTION_FEEDBACK)]
    feedback_missing = _topic_reason(
        TOPIC_ACTION_FEEDBACK, status, samples(TOPIC_ACTION_FEEDBACK))
    if not is_nav2:
        metrics['recoveries'] = metric(
            None, 'count', TOPIC_ACTION_FEEDBACK, recovery_definition,
            f'{TOPIC_ACTION_FEEDBACK} exists only in the Nav2 stack; this platform plans '
            'inside the autopilot and publishes no path')
    elif status.get(TOPIC_ACTION_FEEDBACK, 'absent') == 'absent':
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
    achieved_source = (f'{command_topic} (zero-order hold) and {TOPIC_TRUTH} '
                       '(differentiated)')
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
    if not is_velocity_command:
        metrics['cmd_vs_achieved'] = metric(
            None, 'm/s and rad/s', achieved_source, achieved_definition,
            not_velocity_commanded)
    elif not have_window:
        metrics['cmd_vs_achieved'] = metric(
            None, 'm/s and rad/s', achieved_source, achieved_definition, no_window)
    elif not cmd_all or len(truth_window) < 3:
        metrics['cmd_vs_achieved'] = metric(
            None, 'm/s and rad/s', achieved_source, achieved_definition,
            cmd_missing or truth_missing or (
                f'{command_topic} has {len(cmd_all)} message(s) and truth has '
                f'{len(truth_window)} sample(s) in the window; at least 3 truth samples '
                'are needed for a central difference'))
    else:
        grid, achieved_linear, achieved_angular = achieved_velocity(truth_window, t0, t1)
        count = len(grid)
        cmd_linear = resample_zoh([(s[0], s[1]) for s in cmd_all], grid)
        cmd_angular = resample_zoh([(s[0], s[2]) for s in cmd_all], grid)
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

    # -- speed_vs_target ------------------------------------------------------
    # The mobility-fault evidence for a platform that publishes no velocity
    # command: there is nothing to compare the motion against except the speed
    # the autopilot was CONFIGURED to cruise at, and the motion itself. The
    # achieved half is simulator TRUTH, differentiated - an EVALUATION signal,
    # not something the robot knew, and that is said in the definition because
    # a reader must not take `deficit_fraction` for the vehicle's own estimate
    # of how slow it is.
    target_source = (f'{TOPIC_TRUTH} (differentiated) against the robot profile\'s '
                     'plant.max_linear_m_s')
    target_definition = (
        f'achieved forward speed on the same {RESAMPLE_HZ} Hz grid cmd_vs_achieved uses, '
        f'against {plant_max_linear} m/s - the cruise speed this platform is configured '
        'with (CRUISE_SPEED/WP_SPEED, recorded in the profile as plant.max_linear_m_s). '
        'deficit_fraction is 1 - mean_achieved / target, clamped at 0, so a vehicle at '
        'its configured speed reads 0 and a halted one reads 1. after_activation repeats '
        'the measurement from the first mobility_fault that reached state=activated to '
        'the end of the window, which is the number a mobility fault is read from. '
        'ACHIEVED SPEED IS SIMULATOR TRUTH, used here as an evaluation signal: the robot '
        'neither measured nor reported it, and no claim is made that it knew it was slow')
    if is_velocity_command:
        metrics['speed_vs_target'] = metric(
            None, 'm/s and fraction', target_source, target_definition,
            f'this platform IS commanded in velocity on {command_topic}; its '
            'speed is scored against that command by cmd_vs_achieved rather than '
            'against a configured cruise speed')
    elif not have_window:
        metrics['speed_vs_target'] = metric(
            None, 'm/s and fraction', target_source, target_definition, no_window)
    elif len(truth_window) < 3:
        metrics['speed_vs_target'] = metric(
            None, 'm/s and fraction', target_source, target_definition,
            truth_missing or (f'truth has {len(truth_window)} sample(s) in the window; '
                              'at least 3 are needed for a central difference'))
    else:
        grid, speeds, _ = achieved_velocity(truth_window, t0, t1)
        speed_track = [(stamp, value) for stamp, value in zip(grid, speeds)
                       if value is not None]
        mobility_activation = next(
            (ral_stamp(msg) for _, msg in fault_msgs
             if msg.kind == FaultEvent.KIND_MOBILITY_FAULT
             and msg.state == FaultEvent.STATE_ACTIVATED), None)

        def _summary(series):
            mean = sum(value for _, value in series) / len(series)
            deficit = max(0.0, 1.0 - mean / plant_max_linear) if plant_max_linear else None
            return mean, deficit, len(series)

        if not speed_track:
            metrics['speed_vs_target'] = metric(
                None, 'm/s and fraction', target_source, target_definition,
                'no grid sample had a differentiable truth pose')
        else:
            mean_all, deficit_all, samples_all = _summary(speed_track)
            value = {'target_m_s': plant_max_linear,
                     'mean_achieved_m_s': mean_all,
                     'deficit_fraction': deficit_all,
                     'samples': samples_all}
            if mobility_activation is None:
                value['after_activation'] = None
            else:
                after = [(stamp, speed) for stamp, speed in speed_track
                         if stamp >= mobility_activation]
                if not after:
                    value['after_activation'] = None
                else:
                    mean_after, deficit_after, samples_after = _summary(after)
                    value['after_activation'] = {
                        'activated_sim_s': mobility_activation,
                        'mean_achieved_m_s': mean_after,
                        'deficit_fraction': deficit_after,
                        'samples': samples_after}
            metrics['speed_vs_target'] = metric(
                value, 'm/s and fraction', target_source, target_definition)

    # -- fault_to_response_s --------------------------------------------------
    # The triggers are chosen by the DRIVER, because what counts as a material
    # change is a property of the stack: Nav2 publishes a plan, a velocity
    # command and its own recovery count, and ArduRover publishes none of the
    # three. The discipline is the same on both - every threshold calibrated
    # against this run's own pre-activation window, every search bounded to a
    # horizon, every event trigger left floor-free.
    nav2_response_source = (f'{TOPIC_FAULTS}, {TOPIC_PLAN}, {command_topic}, '
                            f'{TOPIC_ACTION_FEEDBACK}')
    nav2_response_definition = (
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
        'activation times swept at 1 s intervals across four fault-free missions still '
        'fired cmd_changed on 3 of 38, 5 of 38, 3 of 39 and 3 of 38 instants, and '
        'plan_diverged on none of them: a single reported response is weak evidence on '
        'its own and must be read beside a control run')
    mavros_response_source = (f'{TOPIC_FAULTS}, {TOPIC_MAVROS_STATE}, {TOPIC_TRUTH} '
                              '(differentiated)')
    mavros_response_definition = (
        'per FaultEvent that reached state=activated: the first MATERIAL change after it, '
        'on a platform with no plan topic, no velocity command and no recovery count. '
        f'mode_changed = the autopilot\'s own {TOPIC_MAVROS_STATE}.mode differing from '
        'the mode in force at activation - an EVENT, so it keeps the floor-free '
        'comparison; speed_changed = achieved forward speed moving by more than '
        f'max({CMD_CHANGE_FLOOR} m/s, {RESPONSE_BASELINE_MARGIN}x the largest '
        'pre-activation excursion), skipped entirely when fewer than two speed samples '
        'precede the activation. Earliest wins, and the trigger is recorded. The search '
        f'is bounded to {RESPONSE_HORIZON_S:.0f} s sim after activation and the '
        f'threshold calibrated on the {RESPONSE_BASELINE_WINDOW_S:.0f} s sim window '
        'before it. ACHIEVED SPEED IS SIMULATOR TRUTH, used as an evaluation signal: the '
        'autopilot published no velocity of its own to compare against, and no claim is '
        'made that the vehicle detected the change')
    response_source = nav2_response_source if is_nav2 else mavros_response_source
    response_definition = (nav2_response_definition if is_nav2
                           else mavros_response_definition)
    activations = [(ral_stamp(msg), msg) for _, msg in fault_msgs
                   if msg.state == FaultEvent.STATE_ACTIVATED]
    if not activations:
        states = sorted({msg.state for _, msg in fault_msgs})
        metrics['fault_to_response_s'] = metric(
            None, 'list of s (sim)', response_source, response_definition,
            reason=f'no FaultEvent reached state=activated; states seen: '
                   f'{states or "none"}')
    elif is_nav2:
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
    else:
        # The mode series is the autopilot's own, and the speed series is the
        # same differentiated truth speed_vs_target reports - over the WHOLE
        # truth track, not the mission window, because an activation may
        # precede the window's end by less than the baseline window.
        modes = sorted((stamp_of(msg, log_time), str(msg.mode))
                       for log_time, msg in samples(TOPIC_MAVROS_STATE))
        if len(truth_all) >= 3:
            speed_grid, speed_values, _ = achieved_velocity(
                truth_all, truth_all[0][0], truth_all[-1][0])
            speed_series = [(stamp, value)
                            for stamp, value in zip(speed_grid, speed_values)
                            if value is not None]
        else:
            speed_series = []
        entries = []
        for stamp, msg in activations:
            found = first_response_mavros(stamp, modes, speed_series)
            entries.append({
                'id': msg.id, 'kind': msg.kind, 'activated_sim_s': stamp,
                'response_sim_s': found['response_sim_s'],
                'delta_s': found['delta_s'],
                'trigger': found['trigger'],
                'horizon_s': found['horizon_s'],
                'speed_threshold_m_s': found['speed_threshold_m_s'],
                'speed_baseline_excursion_m_s': found['speed_baseline_excursion_m_s'],
                'speed_baseline_samples': found['speed_baseline_samples'],
                'mode_at_activation': found['mode_at_activation'],
                'reason': found['reason'],
            })
        metrics['fault_to_response_s'] = metric(
            entries, 'list of s (sim)', response_source, response_definition)

    # -- obstacle_min_clearance_m ---------------------------------------------
    clearance_source = (f'{TOPIC_TRUTH} mapped to map by {alignment_source}, against '
                        'the spawn_obstacle poses run.json records as applied, '
                        'converted the same way')
    clearance_definition = (
        'smallest (distance from the truth position to the surface of an applied '
        'spawned obstacle, minus the footprint radius this robot is scored with) over '
        'the interval each obstacle EXISTED - from its activated FaultEvent to its '
        'cleared one, or to the end of the mission window. The obstacle is treated '
        'as the oriented rectangle it was spawned as; the footprint as a circle of '
        'the radius run.json\'s pre-flight recorded, falling back to the run\'s robot '
        'profile footprint.radius_m. A negative value is penetration')
    radius_source = 'run.json preflight.robot_radius_m'
    radius = (run_json.get('preflight') or {}).get('robot_radius_m')
    if radius is None:
        # The robot's own profile, copied into env.json by the run: a radius
        # guessed here would turn "zero penetration" into an unverifiable
        # claim, and the profile is the same number the pre-flight used.
        radius = (profile.get('footprint') or {}).get('radius_m')
        radius_source = 'the run\'s robot profile footprint.radius_m'
    radius = None if radius is None else float(radius)
    applied_boxes = []
    for entry in run_json.get('interventions', []):
        if entry.get('type') != 'spawn_obstacle' or entry.get('result') != 'applied':
            continue
        pose = entry['params']['pose']
        size = entry['params']['size']
        # The obstacle pose is WORLD frame (run.json records pose_frame);
        # truth is already in map here, so the box is converted the same way.
        mx, my, myaw = world_pose_to_map(pose['x'], pose['y'], pose['yaw'], wtm)
        activated = next((ral_stamp(msg) for _, msg in fault_msgs
                          if msg.id == entry['id']
                          and msg.state == FaultEvent.STATE_ACTIVATED), None)
        cleared = next((ral_stamp(msg) for _, msg in fault_msgs
                        if msg.id == entry['id']
                        and msg.state == FaultEvent.STATE_CLEARED), None)
        applied_boxes.append({
            'name': entry['params']['name'], 'cx': mx, 'cy': my, 'yaw': myaw,
            'sx': size['x'], 'sy': size['y'],
            't_start': activated, 't_end': cleared,
        })
    unanchored = [box['name'] for box in applied_boxes if box['t_start'] is None]
    if truth_missing or not have_window:
        metrics['obstacle_min_clearance_m'] = metric(
            None, 'm', clearance_source, clearance_definition,
            truth_missing or no_window)
    elif not applied_boxes:
        metrics['obstacle_min_clearance_m'] = metric(
            None, 'm', clearance_source, clearance_definition,
            'no spawn_obstacle intervention was applied in this run')
    elif unanchored:
        metrics['obstacle_min_clearance_m'] = metric(
            None, 'm', clearance_source, clearance_definition,
            f'no activated FaultEvent in the bag for {unanchored}; without the '
            'activation stamp there is no interval in which the obstacle existed')
    elif radius is None:
        metrics['obstacle_min_clearance_m'] = metric(
            None, 'm', clearance_source, clearance_definition,
            'no robot_radius available: run.json has no preflight.robot_radius_m and '
            'the robot profile copied into env.json carries no footprint.radius_m')
    else:
        for box in applied_boxes:
            if box['t_end'] is None:
                box['t_end'] = t1
        clearance = min_clearance(truth_all, applied_boxes, radius)
        if clearance['value'] is None:
            metrics['obstacle_min_clearance_m'] = metric(
                None, 'm', clearance_source, clearance_definition,
                'no truth sample falls inside any obstacle\'s existence interval')
        else:
            metrics['obstacle_min_clearance_m'] = metric(
                {'value': clearance['value'], 'per_obstacle': clearance['per_obstacle'],
                 'samples': clearance['samples'], 'robot_radius_m': radius,
                 'robot_radius_source': radius_source},
                'm', clearance_source, clearance_definition)

    # -- scan_degradation_divergence ------------------------------------------
    divergence_source = f'{TOPIC_SCAN} and {TOPIC_SCAN_RAW}, joined on header stamp'
    divergence_definition = (
        f'{TOPIC_SCAN} compared against {TOPIC_SCAN_RAW} beam by beam on IDENTICAL '
        'header stamps - the relay republishes the sensor\'s own header, so an exact '
        'stamp key is the only join that compares the same sweep. Two ranges are '
        'equal when both are +inf or exactly equal; no tolerance, which would hide '
        'the injected noise. pairs_divergent_before_activation must be 0 for "the '
        'streams differ only after the fault" to hold, and a non-zero value is the '
        'measurement rather than an error. pairs_unmatched and '
        'pairs_length_mismatch count the sweeps that were NOT compared - no raw '
        'entry at that stamp, or a partner with a different beam count - so the '
        'denominator behind pairs_compared is reported rather than hidden; a '
        'non-zero count means the relay or the recorder dropped sweeps')
    scan_missing = _topic_reason(TOPIC_SCAN, status, data[TOPIC_SCAN])
    raw_missing = _topic_reason(TOPIC_SCAN_RAW, status, data[TOPIC_SCAN_RAW])
    scan_activation = next(
        (ral_stamp(msg) for _, msg in fault_msgs
         if msg.kind == FaultEvent.KIND_SCAN_DEGRADATION
         and msg.state == FaultEvent.STATE_ACTIVATED), None)
    if scan_missing or raw_missing:
        metrics['scan_degradation_divergence'] = metric(
            None, 'counts and fraction', divergence_source, divergence_definition,
            scan_missing or raw_missing)
    elif scan_activation is None:
        metrics['scan_degradation_divergence'] = metric(
            None, 'counts and fraction', divergence_source, divergence_definition,
            'no scan_degradation FaultEvent reached state=activated in this run')
    else:
        metrics['scan_degradation_divergence'] = metric(
            scan_divergence(
                [((msg.header.stamp.sec, msg.header.stamp.nanosec), list(msg.ranges))
                 for _, msg in data[TOPIC_SCAN]],
                [((msg.header.stamp.sec, msg.header.stamp.nanosec), list(msg.ranges))
                 for _, msg in data[TOPIC_SCAN_RAW]],
                scan_activation),
            'counts and fraction', divergence_source, divergence_definition)

    payload = {
        'meta': {
            'experiment_id': run_json.get('experiment_id'),
            'run_dir': run_dir,
            'bag': bag_dir,
            'scenario': {'world': scenario.get('world'), 'robot': scenario.get('robot'),
                         'seed': scenario.get('seed')},
            'mission_window': window,
            'robot': profile['robot'],
            'robot_profile_path': env.get('robot_profile_path'),
            # The whole descriptor, not a summary: a reader of metrics.json
            # must be able to see WHAT the estimate was without the profile
            # file beside them.
            'estimate_source': estimate_descriptor,
            'mission_driver': driver,
            'world_to_map': {'x': wtm[0], 'y': wtm[1], 'yaw': wtm[2]},
            'world_to_map_source': env['alignment_path'],
            # The MEASURED pose of the estimator's own origin in `world`, as
            # ekf_local_tf published it; None for a platform that estimates
            # in `map` directly.
            'world_to_ekf_local': wtl_doc,
            'map_sha256': env.get('map_sha256'),
            'mission_result_source': status_source,
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
