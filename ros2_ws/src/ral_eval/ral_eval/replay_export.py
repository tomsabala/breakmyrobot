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
Turn ONE completed run directory into the console's replay fixture.

The public site has no gateway: no ROS, no simulator, no process to subscribe
to. A replay fixture is therefore the only thing the console can be driven by
there, and it must be the SAME console - so a frame in this file has to be
indistinguishable from a frame the live gateway would have sent.

That is why nothing here encodes anything. Every payload comes out of
`dashboard_api`'s own `encode_*`, the functions the live `/api/stream` calls,
and every message comes out of `ral_eval.bagio`, the one bag reader. A second
encoder would let the replayed console and the live console disagree about
what a message meant, and a second deserialisation path would let them
disagree about when it happened.

What this tool DOES decide is which messages survive, because the fixtures are
committed to git and a 27 MB bag is not. Three rules, and every one of them is
written verbatim into `provenance.decimation` of the file it produced:

  events        kept in full - mission states, fault events, active-fault
                snapshots, plans, action feedback, waypoint distances, the
                autopilot's state, the frame alignment, contacts and the map.
                A dropped event is a hole in the record the operator reads,
                and the live gateway never drops one either (NEVER_DROP).
  samples       truth, estimate and command to ~10 Hz sim, both lidar halves
                to ~2 Hz sim, /clock to ~1 Hz. These are streams that were
                sampled to begin with; the console interpolates nothing and
                draws what it is given.
  precision     positions to the millimetre, ranges to the millimetre, angles
                to 0.1 mrad, stamps to the microsecond. A 360-beam scan at
                full float repr is most of the file.

Decimation is a REDUCTION, never a rewrite: a kept frame is bit-identical in
shape to the live one and its numbers are the message's own, rounded to a
precision this file states. Nothing is resampled, smoothed or interpolated.

The window is the whole bag, not the mission window: bring-up, the warm-up
rotation and the post-terminal tail are part of what happened.

Refusals (exit 2, naming the missing thing) rather than a fixture that looks
complete: no bag, no run.json/env.json/metrics.json, a robot profile this
build has no topic set for, a grid that can be read from neither the bag nor
the committed PGM, a map whose hash is not the one the run was scored against,
and a file over the size budget.
"""

import argparse
import json
import os
import sys
from datetime import datetime, timezone

from ament_index_python.packages import get_package_share_directory

from ral_eval import frames
from ral_eval.bagio import read_bag, stamp_of
from ral_eval.dashboard_api import (
    WORLD,
    _stamp_s,
    build_reference,
    encode_active,
    encode_alignment,
    encode_bumper,
    encode_cmd,
    encode_estimate,
    encode_fault_event,
    encode_feedback,
    encode_grid,
    encode_mavros_state,
    encode_mission,
    encode_plan,
    encode_scan,
    encode_truth,
    encode_wp,
)
from ral_eval.metrics import (
    TOPIC_ACTION_FEEDBACK,
    TOPIC_BUMPER,
    TOPIC_FAULTS,
    TOPIC_MAP,
    TOPIC_MAVROS_NAV_OUTPUT,
    TOPIC_MAVROS_STATE,
    TOPIC_MISSION,
    TOPIC_PLAN,
    TOPIC_SCAN,
    TOPIC_SCAN_RAW,
    TOPIC_TRUTH,
    read_topics,
)
from ral_eval.run_scenario import occupancy_grid_from_pgm

import yaml

# Bumped when the shape of the file changes, so a console reading an old
# fixture refuses it instead of drawing half of it.
REPLAY_VERSION = 1

# Topics the gateway subscribes to but metrics.py does not score from, so
# their names live here rather than in metrics.TOPIC_*: the clock the console
# shows, the manager's latched snapshot, the car's measured frame offset and
# the costmap. /global_costmap/costmap is listed for completeness - no
# committed recorder profile records it, and an absent topic is reported as
# absent rather than quietly omitted.
TOPIC_CLOCK = '/clock'
TOPIC_ACTIVE = '/faults/active'
TOPIC_ALIGNMENT = '/frame_alignment'
TOPIC_COSTMAP = '/global_costmap/costmap'

# Minimum sim interval between kept samples, per SSE topic. A topic absent
# from BOTH this table and the special cases below is an EVENT and is kept in
# full; see the module docstring.
MIN_INTERVAL_S = {
    'truth': 0.1,
    'est': 0.1,
    'cmd': 0.1,
    'scan': 0.5,
    'scan_raw': 0.5,
    'clock': 1.0,
}

# Two streams are neither samples nor events, and a fixed bucket would either
# bloat the file or change a number the console displays. Each gets a rule
# derived from what the CONSOLE does with it.
#
# /bumper: a gz contact sensor publishes every physics step while two bodies
# touch - 14,170 messages for the two contacts of ui_car_20261026T224207Z,
# 2.64 MB of a 3.35 MB fixture. The console does not show that stream; it
# derives a collision count by debouncing at 0.5 s sim per colliding pair
# (store.ts `ingestBumper`: a hit is recorded when the gap to the previous
# KEPT hit of that pair exceeds the window). Replaying only the messages that
# WOULD become hits reproduces that list exactly - every kept frame is more
# than a window after the previous one by construction - so the displayed
# count is unchanged and the rest is 2.6 MB of bytes nobody reads. The last
# message is kept too, because the console also shows the latest /bumper.
BUMPER_DEBOUNCE_S = 0.5

# /navigate_to_pose feedback: 5,244 messages, 0.62 MB of a 2.05 MB fixture.
# The console keeps only the LATEST (store.ts `case 'fb'`), reading
# `distance_remaining` and `number_of_recoveries` off it, so a 5 Hz grid
# shows the same numbers moving at the same speed. Every message at which
# `number_of_recoveries` CHANGES is kept regardless of the grid: a recovery
# is an event in Nav2's own count, and dropping the instant it incremented
# would move it in the timeline.
FB_INTERVAL_S = 0.2

# Rounding, in decimal places. Positions and ranges to the millimetre: the
# map is 0.05 m per cell and the lidar resolves nothing finer than its own
# noise. Angles to 0.1 mrad, which is 0.02 deg. Stamps to the microsecond,
# because /scan and /scan_raw are joined on an EXACT stamp equality in the
# console and a coarser stamp could merge two sweeps into one.
POS_DP = 3
ANG_DP = 4
RANGE_DP = 3
RATE_DP = 4
TRACE_DP = 6
STAMP_DP = 6

# Default size budget for ONE fixture, before gzip. These files are committed.
DEFAULT_MAX_BYTES = 4 * 1024 * 1024


class ExportError(Exception):
    """A refusal that names the missing or inconsistent thing."""


def _read_json(path: str, what: str):
    """Load one required run artifact, or refuse naming it."""
    if not os.path.isfile(path):
        raise ExportError(f'{what} is missing: no file at {path}')
    try:
        with open(path) as handle:
            return json.load(handle)
    except (OSError, ValueError) as exc:
        raise ExportError(f'{what} at {path} is unreadable: {exc}') from exc


def _profile_of(env: dict, run_dir: str) -> dict:
    """
    Load the robot profile the RUN carried, from its own env.json copy.

    metrics.py and plot_run.py both read the profile from env.json rather than
    from the repository, so an old run is replayed with the robot it ran as.
    """
    doc = env.get('robot_profile')
    if not doc:
        raise ExportError(
            f'{run_dir}/env.json carries no `robot_profile`, so which estimate, '
            'command and stack topics this run had is unknown')
    try:
        profile = yaml.safe_load(doc)
    except yaml.YAMLError as exc:
        raise ExportError(f'{run_dir}/env.json `robot_profile` is not YAML: {exc}') from exc
    if not isinstance(profile, dict) or 'estimate' not in profile:
        raise ExportError(
            f'{run_dir}/env.json `robot_profile` is not a robot profile '
            f'(type {type(profile).__name__}, no `estimate` block)')
    return profile


def stream_sources(profile: dict) -> list:
    """
    [(sse topic, bag topic)] for one profile: what TelemetryNode would publish.

    Mirrors dashboard_api.TelemetryNode's static subscriptions and `_attach`,
    gated on the same profile keys and never on the robot's name - /plan and
    /map belong to whatever plans, and the car plans too. metrics.read_topics
    is called beside this for its own refusal: a profile naming a
    mission_driver this build has no topic set for must stop the export rather
    than produce a fixture missing that stack's streams.
    """
    read_topics(profile)
    estimate = profile['estimate']
    command = profile['command']
    sources = [
        ('clock', TOPIC_CLOCK),
        ('truth', TOPIC_TRUTH),
        ('scan', TOPIC_SCAN),
        ('scan_raw', TOPIC_SCAN_RAW),
        ('mission', TOPIC_MISSION),
        ('fault_event', TOPIC_FAULTS),
        ('active', TOPIC_ACTIVE),
        ('bumper', TOPIC_BUMPER),
        ('est', estimate['topic']),
    ]
    if command['kind'] == 'velocity_twist':
        sources += [('cmd', command['topic']),
                    ('fb', TOPIC_ACTION_FEEDBACK),
                    ('costmap', TOPIC_COSTMAP)]
    if command['kind'] == 'velocity_twist' or 'planner_action' in command:
        sources += [('plan', TOPIC_PLAN), ('map', TOPIC_MAP)]
    if profile['mission_driver'] == 'mavros_guided':
        sources += [('mavros_state', TOPIC_MAVROS_STATE),
                    ('wp', TOPIC_MAVROS_NAV_OUTPUT),
                    ('align', TOPIC_ALIGNMENT)]
    return sources


def encoders_for(profile: dict) -> dict:
    """
    {sse topic: f(msg, log_time_s) -> payload}, every one of them the gateway's.

    `log_time_s` is the bag's own log time, which the recorder takes in sim
    time (--use-sim-time). It is passed to exactly the four encoders the live
    gateway passes its sim_now() to - Twist, NavControllerOutput, State and
    Contacts carry no stamp of their own - so the replayed frame keeps the
    same `stamp_source: 'received'` label and means the same thing: when the
    sample was RECEIVED, by the recorder here and by the gateway there.
    """
    estimate = profile['estimate']
    return {
        # The gateway inlines this one in TelemetryNode._on_clock; it is a
        # transcription of a single field, not an encoder.
        'clock': lambda msg, t: {'sim_s': _stamp_s(msg.clock)},
        'truth': lambda msg, t: encode_truth(msg),
        'est': lambda msg, t: encode_estimate(msg, estimate['label'], estimate['frame']),
        'scan': lambda msg, t: encode_scan(msg),
        'scan_raw': lambda msg, t: encode_scan(msg),
        'plan': lambda msg, t: encode_plan(msg),
        'cmd': lambda msg, t: encode_cmd(msg, t),
        'fb': lambda msg, t: encode_feedback(msg),
        'wp': lambda msg, t: encode_wp(msg, t),
        'mavros_state': lambda msg, t: encode_mavros_state(msg, t),
        'align': lambda msg, t: encode_alignment(msg),
        'mission': lambda msg, t: encode_mission(msg),
        'fault_event': lambda msg, t: encode_fault_event(msg),
        'active': lambda msg, t: encode_active(msg),
        'bumper': lambda msg, t: encode_bumper(msg, t),
        'map': lambda msg, t: encode_grid(msg),
        'costmap': lambda msg, t: encode_grid(msg),
    }


def _r(value, digits: int):
    """Round, preserving a null: a missing number must not become a zero."""
    if value is None:
        return None
    return round(float(value), digits)


def _round_pose(pose: dict) -> dict:
    for key in ('x', 'y'):
        if key in pose:
            pose[key] = _r(pose[key], POS_DP)
    for key in ('yaw', 'yaw_rad'):
        if key in pose:
            pose[key] = _r(pose[key], ANG_DP)
    return pose


def round_payload(topic: str, payload: dict) -> dict:
    """
    Round one payload in place, by topic, to the precision this file states.

    Every rounded field is named here rather than rounded by key name across
    the whole document: a grid's `resolution` and `origin` place every cell
    and must survive untouched, and so must the base64 cells themselves.
    """
    if 'stamp_sim_s' in payload:
        payload['stamp_sim_s'] = _r(payload['stamp_sim_s'], STAMP_DP)
    if topic == 'clock':
        payload['sim_s'] = _r(payload['sim_s'], STAMP_DP)
    elif topic == 'truth':
        _round_pose(payload['pose'])
        twist = payload['twist']
        twist['linear_x'] = _r(twist['linear_x'], RATE_DP)
        twist['angular_z'] = _r(twist['angular_z'], RATE_DP)
    elif topic == 'est':
        _round_pose(payload['pose'])
        payload['covariance_trace'] = _r(payload['covariance_trace'], TRACE_DP)
    elif topic in ('scan', 'scan_raw'):
        payload['ranges'] = [_r(value, RANGE_DP) for value in payload['ranges']]
        for key in ('angle_min', 'angle_increment', 'range_min', 'range_max'):
            payload[key] = _r(payload[key], TRACE_DP)
    elif topic == 'plan':
        payload['poses'] = [_round_pose(pose) for pose in payload['poses']]
    elif topic == 'cmd':
        payload['linear_x'] = _r(payload['linear_x'], RATE_DP)
        payload['angular_z'] = _r(payload['angular_z'], RATE_DP)
    elif topic == 'fb':
        payload['distance_remaining'] = _r(payload['distance_remaining'], POS_DP)
    elif topic == 'align':
        payload['x'] = _r(payload['x'], POS_DP)
        payload['y'] = _r(payload['y'], POS_DP)
        payload['yaw'] = _r(payload['yaw'], ANG_DP)
        payload['residual_m'] = _r(payload['residual_m'], POS_DP)
        payload['residual_yaw_rad'] = _r(payload['residual_yaw_rad'], TRACE_DP)
    elif topic == 'mission':
        _round_pose(payload['goal'])
    elif topic == 'active':
        for event in payload['active']:
            event['stamp_sim_s'] = _r(event['stamp_sim_s'], STAMP_DP)
    return payload


def decimate(frames_in: list, interval_s) -> list:
    """
    Keep the FIRST frame of each `interval_s` sim bucket, or all of them.

    Bucketing on floor(t / interval) rather than on "interval since the last
    kept frame" makes the result a property of the stamps alone: a jittery
    stream cannot drift its own sampling grid, and re-running the export on
    the same bag selects the same frames.

    The arithmetic is in integer microseconds, the precision the stamps were
    rounded to, because the float form is not exact at the boundary: in
    binary, 0.3 // 0.1 is 2.0, so a sample landing exactly on a bucket edge
    was dropped as a duplicate of the previous bucket.
    """
    if interval_s is None:
        return list(frames_in)
    width = int(round(interval_s * 1e6))
    kept = []
    last_bucket = None
    for frame in frames_in:
        bucket = int(round(frame['t'] * 1e6)) // width
        if bucket != last_bucket:
            kept.append(frame)
            last_bucket = bucket
    return kept


def thin_bumper(frames_in: list) -> list:
    """
    Keep only the contacts the console would COUNT, plus the last message.

    This is `store.ts ingestBumper` run at export time: per colliding pair, a
    frame survives when the gap to the previous SURVIVING frame of that pair
    exceeds BUMPER_DEBOUNCE_S. Because the console applies the same rule to
    what it receives, and every frame kept here is already more than a window
    after its predecessor, replaying the thinned stream produces the identical
    hit list - the collision count a reader sees is the same number.

    The final message is kept whichever side of the window it falls, because
    the console also displays the latest /bumper as a live field.
    """
    kept = []
    last_hit = {}
    for frame in frames_in:
        pairs = frame['payload'].get('pairs') or []
        stamp = frame['t']
        if any(pair not in last_hit or stamp - last_hit[pair] > BUMPER_DEBOUNCE_S
               for pair in pairs):
            for pair in pairs:
                if pair not in last_hit or stamp - last_hit[pair] > BUMPER_DEBOUNCE_S:
                    last_hit[pair] = stamp
            kept.append(frame)
    if frames_in and kept and kept[-1] is not frames_in[-1]:
        kept.append(frames_in[-1])
    return kept


def thin_feedback(frames_in: list) -> list:
    """
    Keep a 5 Hz grid, plus every message at which the recovery count changed.

    The console keeps the latest feedback only, so the grid is about how
    smoothly `distance_remaining` moves. `number_of_recoveries` is Nav2's own
    count of behaviour-tree recoveries and is the one field whose INSTANT
    matters, so a message that changes it is never dropped.
    """
    grid = {f['t']: f for f in decimate(frames_in, FB_INTERVAL_S)}
    previous = None
    for frame in frames_in:
        recoveries = frame['payload'].get('number_of_recoveries')
        if recoveries != previous:
            grid[frame['t']] = frame
            previous = recoveries
    return [grid[t] for t in sorted(grid)]


def decimation_rule(topic: str, read_count: int, kept_count: int) -> str:
    """Describe the rule actually applied to one topic, as the fixture records it."""
    if topic == 'bumper':
        return (f'kept only the contacts the console debounce would count, at '
                f'{BUMPER_DEBOUNCE_S:g} s sim per colliding pair, plus the last '
                f'message: {kept_count} of {read_count}')
    if topic == 'fb':
        return (f'kept a {FB_INTERVAL_S:g} s sim grid plus every change of '
                f'number_of_recoveries: {kept_count} of {read_count}')
    interval = MIN_INTERVAL_S.get(topic)
    if interval is None:
        return f'kept every message: {kept_count} of {read_count}'
    return (f'kept the first message of each {interval:g} s sim bucket '
            f'(about {1.0 / interval:g} Hz sim): {kept_count} of {read_count}')


def _metric(metrics_doc: dict, name: str):
    """(value, reason) for one metric, or (None, reason) when it has none."""
    entry = (metrics_doc.get('metrics') or {}).get(name)
    if not isinstance(entry, dict):
        return None, f'metrics.json has no `{name}`'
    return entry.get('value'), entry.get('reason')


def build_headline(run_json: dict, metrics_doc: dict, activation) -> str:
    """
    One sentence, every clause naming a field of run.json or metrics.json.

    A metric with no value says so instead of being left out, because an
    absent clause and an unmeasurable quantity read identically. `activation`
    is (kind, stamp) from the first /fault_events message that reached
    state=activated in this bag, or None.
    """
    experiment = run_json.get('experiment_id') or 'unnamed experiment'
    outcome = run_json.get('outcome') or 'no outcome recorded'
    parts = [f'{experiment}: {outcome}']

    error, error_reason = _metric(metrics_doc, 'goal_pose_error_m')
    if isinstance(error, (int, float)):
        parts.append(f'{float(error):.3f} m from the goal')
    else:
        parts.append(f'goal_pose_error_m not measured ({error_reason or "no reason given"})')

    duration, duration_reason = _metric(metrics_doc, 'mission_duration_s')
    if isinstance(duration, (int, float)):
        parts.append(f'{float(duration):.2f} s sim')
    else:
        parts.append(
            f'mission_duration_s not measured ({duration_reason or "no reason given"})')

    collisions, _ = _metric(metrics_doc, 'collision_count')
    if isinstance(collisions, (int, float)):
        parts.append(f'{int(collisions)} collision(s)')

    if activation is not None:
        kind, stamp = activation
        parts.append(f'{kind} activated at {stamp:.2f} s sim')
    return ', '.join(parts) + '.'


def _grid_frame(map_yaml: str, t: float) -> dict:
    """
    Build a `map` frame from the committed grid, for a bag that recorded none.

    The car's stack runs no map_server, so its bag has no /map at all - and
    without a grid the console draws trajectories over nothing. The committed
    PGM is the same artifact the run's alignment was sealed against, read
    through the one reader (ral_eval.grid, via occupancy_grid_from_pgm), and
    `provenance.grid_source` says which of the two it was.
    """
    try:
        grid = occupancy_grid_from_pgm(map_yaml)
    except (OSError, ValueError, KeyError) as exc:
        raise ExportError(
            f'the bag recorded no {TOPIC_MAP} and the committed grid at {map_yaml} '
            f'cannot be read either ({type(exc).__name__}: {exc})') from exc
    grid.header.frame_id = 'map'
    return {'t': round(t, STAMP_DP), 'topic': 'map', 'payload': encode_grid(grid)}


def export_run(run_dir: str, map_yaml: str) -> dict:
    """Build the whole fixture document for one run directory."""
    run_dir = os.path.abspath(run_dir)
    if not os.path.isdir(run_dir):
        raise ExportError(f'no run directory at {run_dir}')
    bag_dir = os.path.join(run_dir, 'bag')
    if not os.path.isdir(bag_dir):
        raise ExportError(f'no bag at {bag_dir}: this run recorded nothing to replay')

    run_json = _read_json(os.path.join(run_dir, 'run.json'), 'run.json')
    env = _read_json(os.path.join(run_dir, 'env.json'), 'env.json')
    metrics_doc = _read_json(os.path.join(run_dir, 'metrics.json'), 'metrics.json')
    profile = _profile_of(env, run_dir)

    # The map the console will draw must be the map the run was scored
    # against: world_to_map comes with the grid, and replaying a run over a
    # different grid would offset every truth and estimate position by a
    # constant the viewer cannot see.
    if not os.path.isfile(map_yaml):
        raise ExportError(f'no map yaml at {map_yaml}')
    try:
        reference_sha = frames.grid_sha256(map_yaml)
    except (OSError, ValueError) as exc:
        raise ExportError(f'the grid behind {map_yaml} cannot be hashed: {exc}') from exc
    run_sha = str(env.get('map_sha256'))
    if run_sha != reference_sha:
        raise ExportError(
            f'{run_dir} was scored against map_sha256 {run_sha} but {map_yaml} hashes to '
            f'{reference_sha}; every position in the replay would carry a constant frame '
            'offset. Export this run against the map it ran on (--map).')

    sources = stream_sources(profile)
    bag_topics = list(dict.fromkeys(topic for _, topic in sources))
    data, status = read_bag(bag_dir, bag_topics)
    encoders = encoders_for(profile)

    by_topic = {}
    read_counts = {}
    for sse_topic, bag_topic in sources:
        encode = encoders[sse_topic]
        out = []
        for log_time, msg in data.get(bag_topic, []):
            payload = round_payload(sse_topic, encode(msg, log_time))
            # WHERE the frame sits in the replay is bagio's decision, not the
            # payload's: `stamp_of` prefers the publisher's stamp and falls
            # back to the log time for a message that carries none - or whose
            # stamp is all-zero. That last case is real here. A latched
            # ActiveFaults published before the simulator's clock started
            # carries stamp 0, and using it would place an empty fault
            # snapshot 42 s before the recording began. The PAYLOAD keeps the
            # stamp the gateway would have shown, 0 included; only the
            # replay position comes from the bag.
            if sse_topic == 'clock':
                # rosgraph_msgs/Clock has no header and no `stamp`: the sim
                # time IS its payload.
                stamp = payload['sim_s']
            else:
                stamp = round(stamp_of(msg, log_time), STAMP_DP)
            out.append({'t': stamp, 'topic': sse_topic, 'payload': payload})
        out.sort(key=lambda frame: frame['t'])
        read_counts[sse_topic] = len(out)
        if sse_topic == 'bumper':
            by_topic[sse_topic] = thin_bumper(out)
        elif sse_topic == 'fb':
            by_topic[sse_topic] = thin_feedback(out)
        else:
            by_topic[sse_topic] = decimate(out, MIN_INTERVAL_S.get(sse_topic))

    all_frames = [frame for topic_frames in by_topic.values() for frame in topic_frames]
    if not all_frames:
        raise ExportError(
            f'{bag_dir} yielded no frame on any topic the console draws '
            f'({", ".join(bag_topics)}); per-topic bag status: {status}')
    all_frames.sort(key=lambda frame: frame['t'])
    t_start = all_frames[0]['t']
    t_end = all_frames[-1]['t']

    grid_source = f'{TOPIC_MAP} in the bag'
    grid_rule = decimation_rule('map', read_counts.get('map', 0),
                                len(by_topic.get('map', [])))
    if not by_topic.get('map'):
        # Placed at the start of the record: it is the static map, latched on
        # the wire, and the console needs it before the first trajectory frame.
        frame = _grid_frame(map_yaml, t_start)
        by_topic['map'] = [frame]
        all_frames.insert(0, frame)
        grid_source = (f'{map_yaml} (the bag recorded no {TOPIC_MAP}; this stack runs no '
                       'map_server), read through ral_eval.grid')
        grid_rule = (f'the bag recorded no {TOPIC_MAP}: ONE frame read from the committed '
                     f'grid at {map_yaml} through ral_eval.grid, placed at t_start '
                     f'({t_start:g} s sim). Nothing from this run was decimated')

    activation = None
    for frame in by_topic.get('fault_event', []):
        if frame['payload']['state'] == 'activated':
            activation = (frame['payload']['kind'], frame['t'])
            break

    decimation = {topic: decimation_rule(topic, read_counts[topic], len(by_topic[topic]))
                  for topic in sorted(read_counts)}
    decimation['map'] = grid_rule

    return {
        'replay_version': REPLAY_VERSION,
        'run_id': os.path.basename(run_dir),
        'robot': profile['robot'],
        'world': env.get('world') or WORLD,
        'outcome': run_json.get('outcome') or '',
        'headline': build_headline(run_json, metrics_doc, activation),
        'provenance': {
            'git_sha': env.get('git_sha'),
            'git_dirty': env.get('git_dirty'),
            'image_ref': env.get('image_ref'),
            'map_sha256': env.get('map_sha256'),
            'exported_utc': datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
            'bag': f'{os.path.basename(run_dir)}/bag',
            'grid_source': grid_source,
            # Per-topic read status from the one bag reader: 'absent' is a
            # fact about the recording, and it must not read as a stream that
            # happened to be silent.
            'topic_status': {sse: status.get(bag_topic, 'absent')
                             for sse, bag_topic in sources},
            'decimation': decimation,
        },
        't_start': t_start,
        't_end': t_end,
        'reference': build_reference(map_yaml),
        'metrics': metrics_doc,
        'frames': all_frames,
    }


def write_index(out_dir: str) -> str:
    """Rebuild index.json from the fixtures actually present in out_dir."""
    replays = []
    for name in sorted(os.listdir(out_dir)):
        if not name.endswith('.json') or name == 'index.json':
            continue
        path = os.path.join(out_dir, name)
        with open(path) as handle:
            doc = json.load(handle)
        replays.append({
            'run_id': doc['run_id'],
            'robot': doc['robot'],
            'outcome': doc['outcome'],
            'headline': doc['headline'],
            'file': name,
            'bytes': os.path.getsize(path),
            't_start': doc['t_start'],
            't_end': doc['t_end'],
        })
    path = os.path.join(out_dir, 'index.json')
    with open(path, 'w') as handle:
        json.dump({'replays': replays}, handle, separators=(',', ':'),
                  allow_nan=False)
        handle.write('\n')
    return path


def write_replay(run_dir: str, out_dir: str, map_yaml: str,
                 max_bytes: int = DEFAULT_MAX_BYTES) -> dict:
    """
    Export one run and rebuild the index. Returns a summary for the caller.

    The document is serialised BEFORE anything is written: a fixture over the
    budget is refused rather than committed, because the budget exists for the
    repository and not for the console.
    """
    document = export_run(run_dir, map_yaml)
    # Compact separators: these files are read by a browser, not by a human
    # with a diff, and the whitespace of a 360-beam scan is not free.
    body = json.dumps(document, separators=(',', ':'), allow_nan=False)
    size = len(body.encode('utf-8'))
    if max_bytes and size > max_bytes:
        raise ExportError(
            f'{document["run_id"]} serialises to {size} bytes, over the {max_bytes} byte '
            'budget. Nothing was written. Raise --max-bytes, or tighten MIN_INTERVAL_S '
            'in replay_export.py and re-export - and say which rule changed.')

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f'{document["run_id"]}.json')
    with open(path, 'w') as handle:
        handle.write(body)
        handle.write('\n')

    counts = {}
    for frame in document['frames']:
        counts[frame['topic']] = counts.get(frame['topic'], 0) + 1
    return {'path': path, 'bytes': os.path.getsize(path), 'frames': len(document['frames']),
            'frames_by_topic': counts, 't_start': document['t_start'],
            't_end': document['t_end'], 'index': write_index(out_dir),
            'headline': document['headline']}


def main(args=None) -> None:
    """CLI: export one run directory into the console's replay directory."""
    parser = argparse.ArgumentParser(
        description='Export one completed run directory as a console replay fixture.')
    parser.add_argument('--run-dir', required=True, help='runs/<id>, with its bag')
    parser.add_argument('--out', required=True,
                        help='output directory, normally dashboard/public/replay')
    parser.add_argument('--map', default=None,
                        help='map yaml; defaults to ral_bringup share maps/apartment.yaml')
    parser.add_argument('--max-bytes', type=int, default=DEFAULT_MAX_BYTES,
                        help=f'size budget for one fixture (default {DEFAULT_MAX_BYTES})')
    parsed = parser.parse_args(args)

    map_yaml = parsed.map or os.path.join(
        get_package_share_directory('ral_bringup'), 'maps', 'apartment.yaml')
    try:
        summary = write_replay(parsed.run_dir, parsed.out, map_yaml, parsed.max_bytes)
    except ExportError as exc:
        print(f'--- replay_export refused: {exc}', file=sys.stderr, flush=True)
        sys.exit(2)

    print(f'--- wrote {summary["path"]} ({summary["bytes"]} bytes, '
          f'{summary["frames"]} frames, t {summary["t_start"]:.3f}..'
          f'{summary["t_end"]:.3f} s sim)', flush=True)
    for topic in sorted(summary['frames_by_topic']):
        print(f'    {topic:13s} {summary["frames_by_topic"][topic]}', flush=True)
    print(f'--- {summary["headline"]}', flush=True)
    print(f'--- index {summary["index"]}', flush=True)
    sys.exit(0)


if __name__ == '__main__':
    main()
