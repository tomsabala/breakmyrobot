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
One 2x2 figure per run: what happened, where, and when.

Panels
------
  A (top left)     the map frame: occupancy grid, truth and estimate
                   trajectories, the last plan, start, goal and its tolerance
                   circle, spawned obstacles, fault activation markers.
  B (top right)    |estimate - truth| against sim time over the mission
                   window, with the measured RESIDUAL AT THE START POSE drawn
                   as a dashed line - map plus alignment error at that one
                   pose, which is a scale to read the curve against and NOT a
                   lower bound; in-mission error below it is normal.
  C (bottom left)  commanded versus achieved velocity, for a platform
                   commanded in velocity; achieved speed against the
                   configured cruise speed for one commanded by position
                   setpoint. This is the panel Phase 4.3's mobility fault is
                   read from.
  D (bottom right) the event timeline: mission states, fault events,
                   recoveries, plan publications.

B, C and D share the sim-time axis. A panel with no data is drawn EMPTY with
the reason written in it: a missing panel looks like an oversight, an empty
one labelled "no data: ..." is evidence.

Which topics are read, what the estimate IS and what the command IS come from
the ROBOT PROFILE the run copied into env.json, through metrics.read_topics -
not from a list here. A second copy of that list would let the figure read a
stream metrics.json did not score, and the two artifacts of one run would be
able to disagree.

Every series here is recomputed from the same bag over the same window as
metrics.json, through the same functions, so the figure and the JSON cannot
disagree. Colours match ral_bringup/rviz/ral.rviz so the screen and the paper
agree; truth is solid and the estimate dashed, so the two are distinguishable
without colour.
"""

import argparse
import json
import math
import os
import sys

import matplotlib
from ral_eval.bagio import ral_stamp, read_bag, stamp_of, truth_track
from ral_eval.frames import world_pose_to_map
from ral_eval.grid import grid_from_pgm
from ral_eval.metrics import (
    RESAMPLE_HZ,
    SMOOTH_SAMPLES,
    STAMP_MATCH_S,
    TOPIC_ACTION_FEEDBACK,
    TOPIC_FAULTS,
    TOPIC_MAP,
    TOPIC_MISSION,
    TOPIC_PLAN,
    TOPIC_TRUTH,
    achieved_velocity,
    estimate_track,
    pair_nearest,
    read_topics,
    resample_zoh,
)

import numpy as np

import yaml

matplotlib.use('Agg')                                   # no display in the container
import matplotlib.pyplot as plt                         # noqa: E402 - after use('Agg')

# ral.rviz: truth 255;0;0, estimate 0;170;255, plan 0;255;0, odometry 0;100;200.
COLOR_TRUTH = '#FF0000'
COLOR_ESTIMATE = '#00AAFF'
COLOR_PLAN = '#00FF00'
COLOR_COMMAND = '#FF8C00'
COLOR_FAULT = '#CC33CC'

# The Nav2 goal tolerance is read per run from env.json's nav2_params_path
# (the run records the path and its SHA), never duplicated here: a copy that
# drifts from nav2_roomba.yaml would draw a circle the controller does not
# use. A platform with no Nav2 has no such file, and its tolerance comes from
# its profile instead - which is the number ITS controller stops at.
GOAL_TOLERANCE_KEY = ('controller_server', 'ros__parameters', 'general_goal_checker',
                      'xy_goal_tolerance')

FAULT_STATE_MARKERS = {
    'requested': ('o', '#888888'),
    'activated': ('^', COLOR_FAULT),
    'rejected': ('x', '#AA0000'),
    'cleared': ('s', '#008800'),
}


def _legend(axis) -> None:
    """Draw a legend only when something is labelled, so an empty panel is quiet."""
    handles, _ = axis.get_legend_handles_labels()
    if handles:
        axis.legend(loc='upper left', fontsize=8)


def _empty(axis, reason: str) -> None:
    axis.text(0.5, 0.5, f'no data: {reason}', ha='center', va='center',
              transform=axis.transAxes, fontsize=11, color='#AA0000', wrap=True)


def _goal_tolerance(profile: dict, nav2_params_path: str) -> 'tuple':
    """(tolerance, label) from the source THIS run's controller actually used."""
    if profile['mission_driver'] != 'nav2':
        return (float(profile['goal_tolerance_m']),
                'the robot profile\'s goal_tolerance_m')
    try:
        with open(nav2_params_path) as handle:
            node = yaml.safe_load(handle) or {}
        for key in GOAL_TOLERANCE_KEY:
            node = node[key]
        return float(node), f'{os.path.basename(nav2_params_path)} xy_goal_tolerance'
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
        return None, 'nav2 params unreadable'


def _draw_grid(axis, occupancy, info) -> None:
    res = info['resolution']
    ox, oy = info['origin'][0], info['origin'][1]
    shown = np.array(occupancy, dtype=float)
    shown[shown < 0] = 50.0                             # unknown drawn grey, not free
    axis.imshow(shown, origin='lower', cmap='gray_r', vmin=0, vmax=100,
                extent=[ox, ox + info['width'] * res, oy, oy + info['height'] * res])
    axis.set_aspect('equal')


def write_plot(run_dir: str) -> str:
    """Render runs/<id>/plot.png from the bag and the computed metrics."""
    with open(os.path.join(run_dir, 'run.json')) as handle:
        run_json = json.load(handle)
    with open(os.path.join(run_dir, 'env.json')) as handle:
        env = json.load(handle)
    with open(os.path.join(run_dir, 'metrics.json')) as handle:
        computed = json.load(handle)
    metrics = computed['metrics']
    meta = computed['meta']

    # The SAME sources metrics.py scores against: the alignment and the robot
    # profile copied into the run's env.json, never run.json's convenience
    # copy and never the current repository files. A run carries what it was
    # scored with.
    alignment = yaml.safe_load(env['alignment'])['world_to_map']
    wtm = (float(alignment['x']), float(alignment['y']), float(alignment['yaw']))
    profile = yaml.safe_load(env['robot_profile'])
    estimate_descriptor = profile['estimate']
    command_descriptor = profile['command']
    estimate_label = estimate_descriptor['label']
    is_velocity_command = command_descriptor['kind'] == 'velocity_twist'
    plant_max_linear = float(profile['plant']['max_linear_m_s'])
    wtl_doc = run_json.get('world_to_ekf_local')
    wtl = None
    if wtl_doc:
        wtl = (float(wtl_doc['x']), float(wtl_doc['y']), float(wtl_doc['yaw']))
    tolerance, tolerance_source = _goal_tolerance(profile, env.get('nav2_params_path') or '')

    topics = read_topics(profile)
    data, _status = read_bag(os.path.join(run_dir, 'bag'), topics)

    def read(topic):
        """Bag samples for a topic THIS profile may not have asked for."""
        return data.get(topic, [])

    truth = truth_track(data[TOPIC_TRUTH], wtm)
    # Converted through the same composition metrics.py uses, so the panel and
    # the JSON cannot place the estimate differently.
    estimate = estimate_track(
        [(stamp_of(msg, log_time), msg)
         for log_time, msg in read(estimate_descriptor['topic'])],
        estimate_descriptor['frame'], wtm, wtl)
    have_plan_topic = TOPIC_PLAN in topics
    plans = [(stamp_of(msg, log_time),
              [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses])
             for log_time, msg in read(TOPIC_PLAN)]
    cmds = ([(log_time, msg.linear.x, msg.angular.z)
             for log_time, msg in read(command_descriptor['topic'])]
            if is_velocity_command else [])
    mission = [(ral_stamp(msg), msg) for _, msg in data[TOPIC_MISSION]]
    faults = [(ral_stamp(msg), msg) for _, msg in data[TOPIC_FAULTS]]
    feedback = [(log_time, msg.feedback) for log_time, msg in read(TOPIC_ACTION_FEEDBACK)]

    window = meta['mission_window']
    t0 = window.get('t_accept_sim')
    t1 = window.get('t_terminal_sim')
    have_window = t0 is not None and t1 is not None

    activations = [(stamp, msg) for stamp, msg in faults if msg.state == 'activated']

    figure, axes = plt.subplots(2, 2, figsize=(16, 10), dpi=100)
    outcome = run_json.get('outcome') or 'no outcome recorded'
    figure.suptitle(f'{run_json.get("experiment_id")} — {outcome} — '
                    f'{os.path.basename(os.path.normpath(run_dir))}', fontsize=15)

    # -- A: the map frame -----------------------------------------------------
    axis = axes[0][0]
    grid_source = None
    if read(TOPIC_MAP):
        grid_msg = read(TOPIC_MAP)[0][1]
        occupancy = np.array(grid_msg.data, dtype=np.int16).reshape(
            grid_msg.info.height, grid_msg.info.width)
        info = {'resolution': grid_msg.info.resolution,
                'origin': [grid_msg.info.origin.position.x, grid_msg.info.origin.position.y],
                'width': grid_msg.info.width, 'height': grid_msg.info.height}
        grid_source = 'map: from the bag'
        _draw_grid(axis, occupancy, info)
    else:
        try:
            occupancy, info = grid_from_pgm(env['map_yaml'])
            grid_source = 'map: from file (the bag recorded no /map)'
            _draw_grid(axis, occupancy, info)
        except (OSError, ValueError, KeyError) as exc:
            grid_source = f'map: unavailable ({type(exc).__name__}: {exc})'

    if truth:
        axis.plot([s[1] for s in truth], [s[2] for s in truth], color=COLOR_TRUTH,
                  linewidth=1.8, label='truth (simulator, evaluation only)')
        axis.plot(truth[0][1], truth[0][2], marker='o', color=COLOR_TRUTH, markersize=9,
                  linestyle='none', label='start (truth)')
    if estimate:
        # Dashed, so truth and estimate are distinguishable without colour.
        axis.plot([s[1] for s in estimate], [s[2] for s in estimate],
                  color=COLOR_ESTIMATE, linewidth=1.4, linestyle='--',
                  label=f'estimate ({estimate_label})')
    if not have_plan_topic:
        # An empty legend entry, because a missing plan must read as "this
        # platform has none" rather than as a plan that failed to draw.
        axis.plot([], [], color=COLOR_PLAN, linewidth=1.6, linestyle='--',
                  label='no plan: this platform plans inside the autopilot')
    last_plan = None
    for stamp, points in plans:
        if not have_window or stamp <= t1:
            last_plan = points
    if last_plan:
        # The pose count is in the label because the LAST plan of a successful
        # mission is a stub a few centimetres long, and a reader who cannot see
        # it should be told it is short rather than left to think it is missing.
        # With no mission window there IS no terminal, and the label says so.
        axis.plot([p[0] for p in last_plan], [p[1] for p in last_plan], color=COLOR_PLAN,
                  linewidth=1.6, linestyle='--', marker='.', markersize=3,
                  label=(f'last plan before terminal ({len(last_plan)} poses)'
                         if have_window else
                         f'last plan in the bag, no terminal state '
                         f'({len(last_plan)} poses)'))

    goal = run_json.get('goal_map')
    if goal:
        axis.plot(goal[0], goal[1], marker='*', color='#000000', markersize=16,
                  linestyle='none',
                  label=(f'goal (± {tolerance:.2f} m xy tolerance, from '
                         f'{tolerance_source})' if tolerance is not None
                         else f'goal (xy tolerance unavailable: {tolerance_source})'))
        if tolerance is not None:
            axis.add_patch(plt.Circle((goal[0], goal[1]), tolerance, fill=False,
                                      linestyle='--', edgecolor='#000000', linewidth=1.0))

    # Spawned obstacles, drawn DASHED when the intervention was not applied -
    # the figure must never show an obstacle that never existed - and as the
    # ORIENTED rectangle metrics.point_to_box measures. `map` is rotated
    # relative to `world` by the measured alignment, so an axis-aligned patch
    # would draw a box the clearance number was never computed against.
    for entry in run_json.get('interventions', []):
        if entry['type'] != 'spawn_obstacle':
            continue
        pose = entry['params']['pose']
        size = entry['params']['size']
        applied = entry.get('result') == 'applied'
        # The obstacle pose is WORLD frame; the panel is in `map`.
        mx, my, myaw = world_pose_to_map(pose['x'], pose['y'], pose['yaw'], wtm)
        hx, hy = size['x'] / 2.0, size['y'] / 2.0
        cos_yaw, sin_yaw = math.cos(myaw), math.sin(myaw)
        # Polygon, not Rectangle(angle=..., rotation_point='center'):
        # rotation_point needs matplotlib >= 3.6 and the container installs
        # the distro python3-matplotlib, whose version is not guaranteed.
        corners = [(mx + dx * cos_yaw - dy * sin_yaw, my + dx * sin_yaw + dy * cos_yaw)
                   for dx, dy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]
        axis.add_patch(plt.Polygon(
            corners, closed=True,
            fill=applied, facecolor=COLOR_FAULT if applied else 'none',
            edgecolor=COLOR_FAULT, linewidth=1.6,
            linestyle='-' if applied else '--', alpha=0.6,
            label=(f'obstacle {entry["params"]["name"]}'
                   + ('' if applied else ' (NOT applied)'))))

    # pair_nearest, not an unbounded min(): a marker placed against a truth
    # sample seconds away would show the robot somewhere it was not. When the
    # pairing drops, the axis lines on panels B/C/D still carry the instant.
    for stamp, msg in activations:
        nearest = pair_nearest([(stamp, msg)],
                               [(s[0], (s[1], s[2])) for s in truth],
                               window=STAMP_MATCH_S)
        if not nearest:
            continue
        px, py = nearest[0][2]
        axis.plot(px, py, marker='x', color=COLOR_FAULT, markersize=12,
                  markeredgewidth=2.5, linestyle='none')
        axis.annotate(msg.kind, (px, py), textcoords='offset points',
                      xytext=(6, 6), color=COLOR_FAULT, fontsize=9)

    axis.set_title(f'A. map frame — {grid_source}')
    axis.set_xlabel('x (m, map)')
    axis.set_ylabel('y (m, map)')
    _legend(axis)
    if not truth and not estimate:
        _empty(axis, 'no truth and no estimate in the bag')

    # -- B: estimate error ----------------------------------------------------
    # Scored over the MISSION WINDOW, exactly as est_error_max_m is: plotting
    # the whole bag would put the warm-up rotation and the post-terminal tail
    # into a curve a reader compares against a windowed number.
    axis = axes[0][1]
    estimate_b = ([s for s in estimate if t0 <= s[0] <= t1] if have_window
                  else estimate)
    truth_b = [s for s in truth if t0 <= s[0] <= t1] if have_window else truth
    pairs = pair_nearest([(s[0], (s[1], s[2])) for s in estimate_b],
                         [(s[0], (s[1], s[2])) for s in truth_b])
    if pairs:
        axis.plot([p[0] for p in pairs],
                  [math.hypot(p[1][0] - p[2][0], p[1][1] - p[2][1]) for p in pairs],
                  color=COLOR_ESTIMATE, linewidth=1.2, label='|estimate − truth|')
        entry = metrics['est_error_at_start_pose_m']
        if entry['value'] is not None:
            axis.axhline(entry['value'], linestyle='--', color='#666666',
                         label=f'residual at the start pose, in-place rotation: '
                               f'{entry["value"]:.3f} m (map + alignment error; '
                               'not a bound)')
        else:
            axis.annotate(f'start-pose residual: not measured ({entry["reason"]})',
                          (0.02, 0.92), xycoords='axes fraction', fontsize=8,
                          color='#AA0000', wrap=True)
    else:
        _empty(axis, f'no estimate/truth pair within {STAMP_MATCH_S * 1000:.0f} ms')
    axis.set_title('B. estimate error vs sim time' if have_window else
                   'B. estimate error vs sim time — NO mission window, whole bag shown')
    axis.set_xlabel('sim time (s)')
    axis.set_ylabel('m')
    _legend(axis)

    # -- C: commanded vs achieved --------------------------------------------
    # Two shapes, because the platforms are commanded differently: a velocity
    # command can be drawn beside the motion it asked for, a position
    # setpoint cannot. For the second the only honest reference line is the
    # cruise speed the autopilot is configured with, and the achieved curve is
    # labelled as differentiated SIMULATOR TRUTH in both cases.
    axis = axes[1][0]
    start = t0 if have_window else (truth[0][0] if truth else 0.0)
    end = t1 if have_window else (truth[-1][0] if truth else 0.0)
    if is_velocity_command and cmds and len(truth) >= 3:
        # The metric's own grid when there is a window, so the figure and
        # metrics.json cannot disagree about phase or sample count.
        grid, achieved_linear, achieved_angular = achieved_velocity(truth, start, end)
        cmd_linear = resample_zoh([(s[0], s[1]) for s in cmds], grid)
        cmd_angular = resample_zoh([(s[0], s[2]) for s in cmds], grid)
        axis.step(grid, [v if v is not None else np.nan for v in cmd_linear], where='post',
                  color=COLOR_COMMAND, linewidth=1.2, label='commanded linear.x (m/s)')
        axis.plot(grid, [v if v is not None else np.nan for v in achieved_linear],
                  color=COLOR_TRUTH, linewidth=1.2,
                  label=(f'achieved forward speed (m/s, '
                         f'{SMOOTH_SAMPLES / RESAMPLE_HZ:.1f} s moving average of '
                         'differentiated truth)'))
        axis.set_ylabel('m/s')
        twin = axis.twinx()
        twin.step(grid, [v if v is not None else np.nan for v in cmd_angular], where='post',
                  color=COLOR_COMMAND, linewidth=1.0, alpha=0.35, linestyle='-.',
                  label='commanded angular.z (rad/s)')
        twin.plot(grid, [v if v is not None else np.nan for v in achieved_angular],
                  color=COLOR_TRUTH, linewidth=1.0, alpha=0.35, linestyle='-.',
                  label=(f'achieved yaw rate (rad/s, '
                         f'{SMOOTH_SAMPLES / RESAMPLE_HZ:.1f} s moving average of '
                         'differentiated truth)'))
        twin.set_ylabel('rad/s')
        handles, labels = axis.get_legend_handles_labels()
        twin_handles, twin_labels = twin.get_legend_handles_labels()
        axis.legend(handles + twin_handles, labels + twin_labels, loc='upper left',
                    fontsize=8)
    elif not is_velocity_command and len(truth) >= 3:
        grid, achieved_linear, _ = achieved_velocity(truth, start, end)
        axis.plot(grid, [v if v is not None else np.nan for v in achieved_linear],
                  color=COLOR_TRUTH, linewidth=1.2,
                  label=(f'achieved forward speed (m/s, '
                         f'{SMOOTH_SAMPLES / RESAMPLE_HZ:.1f} s moving average of '
                         'differentiated truth)'))
        axis.axhline(plant_max_linear, linestyle='--', color=COLOR_COMMAND,
                     linewidth=1.2, label='commanded cruise speed (WP_SPEED)')
        axis.set_ylabel('m/s')
        _legend(axis)
    elif is_velocity_command:
        _empty(axis, f'{len(cmds)} {command_descriptor["topic"]} message(s), '
                     f'{len(truth)} truth sample(s); a central difference needs at '
                     'least 3')
    else:
        _empty(axis, f'{len(truth)} truth sample(s); a central difference needs at '
                     'least 3')
    axis.set_title('C. commanded vs achieved (achieved = differentiated truth pose)'
                   if is_velocity_command else
                   'C. achieved speed vs commanded cruise speed')
    axis.set_xlabel('sim time (s)')

    # -- D: event timeline ----------------------------------------------------
    axis = axes[1][1]
    drew = False
    if feedback:
        axis.step([s[0] for s in feedback],
                  [int(s[1].number_of_recoveries) for s in feedback], where='post',
                  color='#000000', linewidth=1.2, label='number_of_recoveries (Nav2)')
        drew = True
    if plans:
        axis.plot([s[0] for s in plans], [0.0] * len(plans), marker='|', linestyle='none',
                  color=COLOR_PLAN, markersize=10, label=f'/plan ({len(plans)})')
        drew = True
    for index, (stamp, msg) in enumerate(mission):
        axis.axvline(stamp, color='#444444', linestyle=':', linewidth=0.8)
        # Stagger the labels: goal_accepted and navigating are often within a
        # second of each other and would otherwise overprint.
        axis.annotate(msg.state, (stamp, 0), textcoords='offset points',
                      xytext=(2, 14 + 26 * (index % 2)),
                      rotation=90, fontsize=8, color='#444444')
        drew = True
    seen = set()
    for stamp, msg in faults:
        marker, color = FAULT_STATE_MARKERS.get(msg.state, ('d', COLOR_FAULT))
        label = f'fault {msg.state}' if msg.state not in seen else None
        seen.add(msg.state)
        axis.plot([stamp], [0.5], marker=marker, color=color, markersize=11,
                  linestyle='none', markeredgewidth=2.0, label=label)
        axis.annotate(f'{msg.kind}\n{msg.id}', (stamp, 0.5), textcoords='offset points',
                      xytext=(6, 4), fontsize=8, color=color)
        drew = True
    if have_window:
        for other in (axes[0][1], axes[1][0], axis):
            other.axvspan(t0, t1, color='#000000', alpha=0.05)
        axis.annotate(f'terminal: {run_json.get("outcome")}', (t1, 0),
                      textcoords='offset points', xytext=(4, -16), fontsize=9)
    for stamp, _ in activations:
        for other in (axes[0][1], axes[1][0]):
            other.axvline(stamp, color=COLOR_FAULT, linestyle=':', linewidth=1.2)
    if not drew:
        _empty(axis, 'no mission states, faults, plans or feedback in the bag')
    axis.set_title('D. event timeline')
    axis.set_xlabel('sim time (s)')
    axis.set_ylabel('recoveries (count); event markers at fixed y')
    _legend(axis)

    # B, C and D share the sim-time axis; A is a map and does not.
    spans = [s[0] for s in truth] + [s[0] for s in estimate] + [s[0] for s in cmds]
    if spans:
        for other in (axes[0][1], axes[1][0], axes[1][1]):
            other.set_xlim(min(spans), max(spans))

    plan_label = ('nav2 global planner' if have_plan_topic
                  else 'none (ArduRover plans inside the autopilot)')
    figure.text(0.5, 0.012,
                'truth = simulator ground truth (evaluation only) · estimate = '
                f'{estimate_label} · plan = {plan_label} · command = '
                f'{command_descriptor["label"]} · all times are sim time off /clock',
                ha='center', fontsize=9, color='#333333')
    figure.tight_layout(rect=[0, 0.03, 1, 0.96])
    path = os.path.join(run_dir, 'plot.png')
    figure.savefig(path)
    plt.close(figure)
    print(f'--- wrote {path}', flush=True)
    return path


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description='Render plot.png for one run.')
    parser.add_argument('--run-dir', required=True)
    parsed = parser.parse_args(args)
    write_plot(parsed.run_dir)
    sys.exit(0)


if __name__ == '__main__':
    main()
