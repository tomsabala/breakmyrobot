# Repeatability — measured spread, not a determinism claim

Measured 2026-10-23 13:07 UTC by
`scripts/checks/repeatability_check.sh`, which runs
`scripts/run_scenario.sh baseline_roomba` 3 times in sequence
and reads the resulting `metrics.json` files. Every duration is sim time off
`/clock`.

## Per-run results

| Metric | run 1 | run 2 | run 3 |
| --- | --- | --- | --- |
| `mission_duration_s` | 37.752 | 37.976 | 37.674 |
| `path_length_m` | 13.504 | 13.558 | 13.503 |
| `goal_pose_error_m` | 0.048 | 0.024 | 0.047 |
| `est_error_mean_m` | 0.059 | 0.067 | 0.066 |
| `est_error_floor_m` | 0.064 | 0.074 | 0.076 |
| `plan_publications` | 37 | 38 | 37 |
| `recoveries` | 0 | 0 | 0 |
| `collision_count` | 0 | 0 | 0 |
| `nav2_result` | `STATUS_SUCCEEDED` | `STATUS_SUCCEEDED` | `STATUS_SUCCEEDED` |
| `fault_to_response_s` | `[]` | `[]` | `[]` |

## Spread

| Metric | min | max | max − min | max − min as % of mean |
| --- | --- | --- | --- | --- |
| `mission_duration_s` | 37.674 | 37.976 | 0.302 | 0.8% |
| `path_length_m` | 13.503 | 13.558 | 0.055 | 0.4% |

## Run provenance

| Run | Image | `git_sha` | dirty | `map_sha256` | `world_to_map` | RTF |
| --- | --- | --- | --- | --- | --- | --- |
| `baseline_roomba_20261023T130307Z` | `sha256:ce16e2833f14392f3` | `64982d3e11eb` | True | `47d1e2ad01a41a52` | x = 0.5, y = 0.5, yaw = 0.0 | 0.99727892445462551, 0.9958041790913984, 0.99956968525049961 |
| `baseline_roomba_20261023T130436Z` | `sha256:ce16e2833f14392f3` | `64982d3e11eb` | True | `47d1e2ad01a41a52` | x = 0.5, y = 0.5, yaw = 0.0 | 1.0001930372561905, 0.99553948533595227, 0.99770378473942223 |
| `baseline_roomba_20261023T130602Z` | `sha256:ce16e2833f14392f3` | `64982d3e11eb` | True | `47d1e2ad01a41a52` | x = 0.5, y = 0.5, yaw = 0.0 | 0.99923907944100565, 1.0014867070165661, 1.00176109600678 |

## What this does and does not show

- **It is a measured spread over 3 runs, not a reproducibility guarantee.** The three runs use the
  same scenario file, the same image and the same map; what differs is
  everything the project does not control.
- **`seed:` seeds only `ral_faults`' RNG** — Phase 4.2's scan noise and
  dropout. It does not seed the simulator, the planner or the localizer, so a
  baseline run with no fault is not affected by it at all.
- **`nav2_amcl` exposes no seed.** Its particle filter draws from `drand48`,
  seeded outside our control, so the estimate differs run to run even with
  identical sensor input. Every `est_error_*` number inherits that.
- **ROS callback scheduling and the 20 Hz DWB loop are not deterministic.**
  Control decisions land on slightly different sim instants each run, which
  shows up directly in `mission_duration_s` and `path_length_m`.
- **The start HEADING is aligned before the goal is sent, and it dominates**
  **everything else.** The localization warm-up is an in-place rotation, so it
  ends wherever convergence happened to land. With that arbitrary heading the
  measured spread of `mission_duration_s` over three runs was **12.5%** (71.0 to
  80.5 s): facing away from the goal, DWB reversed at 0.1 m/s for 40 s instead
  of turning. The runner now turns back to the scenario's declared initial yaw
  first, using `/amcl_pose` — the robot's own estimate, never ground truth —
  and `run.json` records `initial_yaw_aligned` and the residual AMCL believed
  it had. The reversing run was real Nav2 behaviour, but it answered a heading
  no scenario asked for.
- **The real-time factor varies with recorder I/O and host load.** It changes
  how long a run takes in wall time, not what it measures: every duration
  above is sim time.
- **A wider spread is a result, not a failure.** If a later change widens it,
  that is evidence about the change; it is not a reason to re-run until the
  numbers agree.
