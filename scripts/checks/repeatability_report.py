#!/usr/bin/env python3
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
Report the measured spread of N identical runs. Never assert determinism.

Reads the N newest runs/<experiment_id>_* directories, takes their
metrics.json and env.json verbatim, and writes docs/repeatability.md: a
per-run table, a spread row for the two metrics the MVP plan names, the
provenance of each run, and a section stating exactly what the numbers do and
do not show.

Refuses rather than substitutes. The N newest matching directories are taken
UNCONDITIONALLY: an incomplete one is a failed run, and silently skipping it
to reach an older complete one would report a spread over runs that were not
the last N, under a document that says they were produced in sequence. The
same applies across runs - a differing image, commit or scenario hash means
the three runs are not three runs of one thing.

Exits non-zero when fewer than N runs are found, when one is incomplete, or
when they do not share their provenance.

Run through scripts/checks/repeatability_check.sh, which produces the runs
first.
"""

import argparse
import json
import os
import re
import sys
from datetime import datetime, timezone

EXIT_OK = 0
EXIT_TOO_FEW_RUNS = 2

# The metrics reported per run. `nav2_result` is rendered from its {code, name}.
PER_RUN_METRICS = ('mission_duration_s', 'path_length_m', 'goal_pose_error_m',
                   'est_error_mean_m', 'est_error_at_start_pose_m',
                   'plan_publications', 'recoveries', 'collision_count')

# The two the MVP plan asks for a spread on.
SPREAD_METRICS = ('mission_duration_s', 'path_length_m')


def _format(value) -> str:
    if value is None:
        return '`null`'
    if isinstance(value, float):
        return f'{value:.3f}'
    if isinstance(value, dict) and 'name' in value:
        return f'`{value["name"]}`'
    return str(value)


def _load_runs(runs_dir: str, experiment_id: str, count: int) -> 'tuple[list, str]':
    """
    The `count` newest run directories of this experiment, or a refusal.

    Returns (runs, failure); exactly one of the two is non-empty.
    """
    if not os.path.isdir(runs_dir):
        return [], f'{runs_dir} is not a directory'
    # The prefix must be followed by the UTC stamp, so `baseline_roomba` never
    # absorbs a future `baseline_roomba_ackermann_*`.
    pattern = re.compile(r'^' + re.escape(experiment_id) + r'_\d{8}T\d{6}Z$')
    names = sorted(name for name in os.listdir(runs_dir) if pattern.match(name))
    chosen = list(reversed(names))[:count]               # newest first by UTC name
    if len(chosen) < count:
        return [], (f'found {len(chosen)} run directory(ies) of {experiment_id} under '
                    f'{runs_dir}, need {count}. A spread over fewer runs than it claims '
                    'is a weaker measurement presented as a stronger one.')
    runs = []
    for name in chosen:
        run_dir = os.path.join(runs_dir, name)
        loaded = {'name': name}
        for key, filename in (('metrics', 'metrics.json'), ('env', 'env.json'),
                              ('run', 'run.json')):
            path = os.path.join(run_dir, filename)
            if not os.path.isfile(path):
                return [], (f'{run_dir} has no {filename}; it is one of the {count} '
                            f'newest {experiment_id} runs, so the spread cannot be over '
                            'the runs it claims. Re-run the experiment or delete the '
                            'incomplete directory deliberately.')
            with open(path) as handle:
                loaded[key] = json.load(handle)
        runs.append(loaded)
    return list(reversed(runs)), ''                      # oldest first in the table


def _same_across_runs(runs: list, experiment_id: str) -> str:
    """'' when the runs are comparable, else what differs."""
    for run in runs:
        found = run['metrics']['meta'].get('experiment_id')
        if found != experiment_id:
            return (f'{run["name"]}: metrics.json meta.experiment_id is {found!r}, not '
                    f'{experiment_id!r}; a directory name is not a guarantee of what ran')
    for field in ('image_ref', 'git_sha', 'scenario_sha256'):
        values = {run['name']: run['env'].get(field) for run in runs}
        if len(set(values.values())) > 1:
            return (f'the runs do not share env.{field}: {values}. A spread across '
                    'different builds, commits or scenarios is not a spread')
    return ''


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--runs-dir', default='/ws/runs')
    parser.add_argument('--experiment-id', default='baseline_roomba')
    parser.add_argument('--count', type=int, default=3)
    parser.add_argument('--repo-root', default='/ws')
    args = parser.parse_args()

    if args.count < 2:
        print(f'FAIL: --count is {args.count}; a spread needs at least two runs.',
              file=sys.stderr)
        return EXIT_TOO_FEW_RUNS

    runs, failure = _load_runs(args.runs_dir, args.experiment_id, args.count)
    if failure:
        print(f'FAIL: {failure}', file=sys.stderr)
        return EXIT_TOO_FEW_RUNS
    mismatch = _same_across_runs(runs, args.experiment_id)
    if mismatch:
        print(f'FAIL: {mismatch}', file=sys.stderr)
        return EXIT_TOO_FEW_RUNS

    doc = [
        '# Repeatability — measured spread, not a determinism claim',
        '',
        f'Measured {datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")} by',
        '`scripts/checks/repeatability_check.sh`, which runs',
        f'`scripts/run_scenario.sh {args.experiment_id}` {args.count} times in sequence',
        'and reads the resulting `metrics.json` files. Every duration is sim time off',
        '`/clock`.',
        '',
        '## Per-run results',
        '',
        '| Metric | ' + ' | '.join(f'run {index + 1}' for index in range(len(runs))) + ' |',
        '| --- |' + ' --- |' * len(runs),
    ]

    # A null metric carries a reason that metric() refuses to omit; dropping it
    # here would turn "could not be measured, because X" back into a bare null.
    reasons = []

    def render(name):
        out = []
        for run in runs:
            entry = run['metrics']['metrics'].get(name)
            if entry is None:
                out.append('`absent`')
                continue
            if entry.get('value') is None:
                reason = entry.get('reason') or 'no reason recorded'
                if reason not in reasons:
                    reasons.append(reason)
                out.append(f'`null` ({reason[:60]}…)' if len(reason) > 60
                           else f'`null` ({reason})')
                continue
            out.append(_format(entry['value']))
        return out

    def values_of(name):
        out = []
        for run in runs:
            entry = run['metrics']['metrics'].get(name)
            out.append(None if entry is None else entry.get('value'))
        return out

    for name in PER_RUN_METRICS:
        doc.append(f'| `{name}` | ' + ' | '.join(render(name)) + ' |')
    doc.append('| `nav2_result` | ' + ' | '.join(render('nav2_result')) + ' |')
    doc.append('| `fault_to_response_s` | ' + ' | '.join(render('fault_to_response_s'))
               + ' |')

    if reasons:
        doc += ['', 'Why a value above is `null`:', '']
        doc += [f'- {reason}' for reason in reasons]

    doc += ['', '## Spread', '',
            '| Metric | min | max | max − min | max − min as % of mean |',
            '| --- | --- | --- | --- | --- |']
    for name in SPREAD_METRICS:
        values = [value for value in values_of(name) if isinstance(value, (int, float))]
        if len(values) < len(runs):
            doc.append(f'| `{name}` | — | — | — | not computed: '
                       f'{len(runs) - len(values)} run(s) reported `null` |')
            continue
        low, high = min(values), max(values)
        mean = sum(values) / len(values)
        percent = (high - low) / mean * 100.0 if mean else float('nan')
        doc.append(f'| `{name}` | {low:.3f} | {high:.3f} | {high - low:.3f} | '
                   f'{percent:.1f}% |')

    doc += ['', '## Run provenance', '',
            '| Run | Image | `git_sha` | dirty | `map_sha256` | `world_to_map` | RTF |',
            '| --- | --- | --- | --- | --- | --- | --- |']
    for run in runs:
        env = run['env']
        wtm = run['metrics']['meta']['world_to_map']
        doc.append(
            f'| `{run["name"]}` | `{str(env.get("image_ref", "unknown"))[:24]}` | '
            f'`{str(env.get("git_sha", "unknown"))[:12]}` | {env.get("git_dirty")} | '
            f'`{str(env.get("map_sha256"))[:16]}` | '
            f'x = {wtm["x"]}, y = {wtm["y"]}, yaw = {wtm["yaw"]} | '
            f'{run["run"].get("rtf_samples", "not measured")} |')

    doc += [
        '',
        '## What this does and does not show',
        '',
        '- **It is a measured spread over '
        f'{len(runs)} runs, not a reproducibility guarantee.** The '
        f'{len(runs)} runs use the',
        '  same scenario file, the same image and the same map; what differs is',
        '  everything the project does not control.',
        '- **Provenance.** Every run above records `git_dirty` in its own `env.json`; a',
        '  `True` there means the recorded `git_sha` is the last commit, not the code that',
        '  ran, and `git_diff_sha256` identifies the uncommitted delta. The report refuses',
        '  to mix runs whose `image_ref`, `git_sha` or `scenario_sha256` differ.',
        '- **`seed:` seeds only `ral_faults`\' RNG** — Phase 4.2\'s scan noise and',
        '  dropout. It does not seed the simulator, the planner or the localizer, so a',
        '  baseline run with no fault is not affected by it at all.',
        '- **`nav2_amcl` exposes no seed.** Its particle filter draws from `drand48`,',
        '  seeded outside our control, so the estimate differs run to run even with',
        '  identical sensor input. Every `est_error_*` number inherits that.',
        '- **ROS callback scheduling and the 20 Hz DWB loop are not deterministic.**',
        '  Control decisions land on slightly different sim instants each run, which',
        '  shows up directly in `mission_duration_s` and `path_length_m`.',
        '- **The start HEADING is aligned before the goal is sent, and it dominates**',
        '  **everything else.** The localization warm-up is an in-place rotation, so it',
        '  ends wherever convergence happened to land. With that arbitrary heading the',
        '  measured spread of `mission_duration_s` over three runs was **12.5%** (71.0 to',
        '  80.5 s): facing away from the goal, DWB reversed at 0.1 m/s for 40 s instead',
        '  of turning. The runner now turns back to the scenario\'s declared initial yaw',
        '  first, using `/amcl_pose` — the robot\'s own estimate, never ground truth —',
        '  and `run.json` records `initial_yaw_aligned` and the residual AMCL believed',
        '  it had. The reversing run was real Nav2 behaviour, but it answered a heading',
        '  no scenario asked for.',
        '- **The real-time factor varies with recorder I/O and host load.** It changes',
        '  how long a run takes in wall time, not what it measures: every duration',
        '  above is sim time.',
        '- **A wider spread is a result, not a failure.** If a later change widens it,',
        '  that is evidence about the change; it is not a reason to re-run until the',
        '  numbers agree.',
        '',
    ]

    path = os.path.join(args.repo_root, 'docs', 'repeatability.md')
    with open(path, 'w') as handle:
        handle.write('\n'.join(doc))
    print(f'--- wrote {path} from {len(runs)} run(s)', flush=True)
    return EXIT_OK


if __name__ == '__main__':
    sys.exit(main())
