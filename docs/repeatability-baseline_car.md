# Repeatability — measured spread, not a determinism claim

Measured 2026-10-25 13:36 UTC by
`scripts/checks/repeatability_check.sh`, which runs
`scripts/run_scenario.sh baseline_car` 3 times in sequence
and reads the resulting `metrics.json` files. Every duration is sim time off
`/clock`.

## Per-run results

| Metric | run 1 | run 2 | run 3 |
| --- | --- | --- | --- |
| `mission_duration_s` | 5.690 | 10.356 | 7.030 |
| `path_length_m` | 3.987 | 5.567 | 4.928 |
| `goal_pose_error_m` | 1.517 | 0.144 | 0.589 |
| `est_error_mean_m` | 0.078 | 0.811 | 2.414 |
| `est_error_at_start_pose_m` | `null` (ArduPilot EKF3 via MAVROS (LOCAL_POSITION_NED) has no operat…) | `null` (ArduPilot EKF3 via MAVROS (LOCAL_POSITION_NED) has no operat…) | `null` (ArduPilot EKF3 via MAVROS (LOCAL_POSITION_NED) has no operat…) |
| `plan_publications` | `null` (/plan exists only in the Nav2 stack; this platform plans ins…) | `null` (/plan exists only in the Nav2 stack; this platform plans ins…) | `null` (/plan exists only in the Nav2 stack; this platform plans ins…) |
| `recoveries` | `null` (/navigate_to_pose/_action/feedback exists only in the Nav2 s…) | `null` (/navigate_to_pose/_action/feedback exists only in the Nav2 s…) | `null` (/navigate_to_pose/_action/feedback exists only in the Nav2 s…) |
| `collision_count` | 0 | 0 | 0 |
| `mission_result` | `mavros:reached_wp_radius` | `mavros:reached_wp_radius` | `mavros:reached_wp_radius` |
| `fault_to_response_s` | `null` (no FaultEvent reached state=activated; states seen: none) | `null` (no FaultEvent reached state=activated; states seen: none) | `null` (no FaultEvent reached state=activated; states seen: none) |

Why a value above is `null`:

- ArduPilot EKF3 via MAVROS (LOCAL_POSITION_NED) has no operator-triggered initialization to measure from: the estimator sets its own origin at the autopilot's first GPS fix, nothing publishes /initialpose on this platform, and its uncertainty kind is 'estimator_status' rather than a pose covariance whose crossing could open a start-pose window
- /plan exists only in the Nav2 stack; this platform plans inside the autopilot and publishes no path
- /navigate_to_pose/_action/feedback exists only in the Nav2 stack; this platform plans inside the autopilot and publishes no path
- no FaultEvent reached state=activated; states seen: none

## Spread

| Metric | min | max | max − min | max − min as % of mean |
| --- | --- | --- | --- | --- |
| `mission_duration_s` | 5.690 | 10.356 | 4.666 | 60.7% |
| `path_length_m` | 3.987 | 5.567 | 1.580 | 32.7% |

## Run provenance

| Run | Image | `git_sha` | dirty | `map_sha256` | `world_to_map` | RTF |
| --- | --- | --- | --- | --- | --- | --- |
| `baseline_car_20261025T132957Z` | `sha256:4eb135c402d493098` | `25c89d3fe455` | True | `47d1e2ad01a41a52` | x = 0.5, y = 0.5, yaw = 0.0 | 0.79288088114438082, 1.0013588439512418, 0.79369394288499018 |
| `baseline_car_20261025T133115Z` | `sha256:4eb135c402d493098` | `25c89d3fe455` | True | `47d1e2ad01a41a52` | x = 0.5, y = 0.5, yaw = 0.0 | 0.99998150034224365, 0.99557913086937455, 0.94558132212126045 |
| `baseline_car_20261025T133232Z` | `sha256:4eb135c402d493098` | `25c89d3fe455` | True | `47d1e2ad01a41a52` | x = 0.5, y = 0.5, yaw = 0.0 | 1.0095294532741816, 0.99920762835071786, 0.99983802623974916 |

## What this does and does not show

- **It is a measured spread over 3 runs, not a reproducibility guarantee.** The 3 runs use the
  same scenario file, the same image and the same map; what differs is
  everything the project does not control.
- **Provenance.** Every run above records `git_dirty` in its own `env.json`; a
  `True` there means the recorded `git_sha` is the last commit, not the code that
  ran, and `git_diff_sha256` identifies the uncommitted delta. The report refuses
  to mix runs whose `image_ref`, `git_sha` or `scenario_sha256` differ.
- **`seed:` seeds only `ral_faults`' RNG** — Phase 4.2's scan noise and
  dropout. It does not seed the simulator, the planner or the localizer, so a
  baseline run with no fault is not affected by it at all.
- **ArduPilot's EKF3 is not seedable.** The autopilot fuses its simulated
  IMU and GPS inside the firmware, and neither that fusion nor SITL's sensor
  synthesis is reproduced bit for bit run to run. Every `est_error_*` number
  above inherits that; nothing in this project can make it repeat.
- **Arrival has a resolution, and it is in the numbers.** The mission ends
  when the FCU's own `NAV_CONTROLLER_OUTPUT.wp_dist` is inside `WP_RADIUS`
  AND the robot's own estimate is inside the profile's
  `arrival_estimate_tolerance_m` of the commanded setpoint. `wp_dist` is
  uint16 METRES and truncates, so `mission_duration_s`, `path_length_m` and
  `goal_pose_error_m` all carry the spread that rule allows.
- **There is no localization warm-up and no heading alignment.** An Ackermann
  platform cannot rotate in place, so the run starts at the spawn yaw and
  `run.json` records `initial_yaw_aligned: false` with that as its reason.
  None of the spread above comes from a warm-up, because there is none.
- **MEASURED on this host: the estimate stream can stall mid-mission and
  then flush, and `est_error_*` shows it.** MAVROS delivers
  `/mavros/local_position/odom` from the MAVLink link, and that link is not
  paced by the simulation: in three consecutive `baseline_car` runs the
  largest gap inside the mission window was 0.14 s, 2.88 s and 2.59 s, the
  stalled runs ending with ~60 messages carrying header stamps within 10 ms
  of each other while their contents swept the whole mission. `est_error_*`
  pairs the estimate with truth BY HEADER STAMP, so a flush like that pairs
  early poses against late truth and reports metres of "error" the
  estimator never had. `goal_pose_error_m` is measured from truth alone and
  is unaffected, so the two columns disagreeing is the signature. The real
  consequence is not the metric: while the stream is stalled the robot has
  no fresh position of its own, and both the arrival gate and the
  no-progress supervisor close on whatever it last reported.
- **The real-time factor varies with recorder I/O and host load.** It changes
  how long a run takes in wall time, not what it measures: every duration
  above is sim time.
- **A wider spread is a result, not a failure.** If a later change widens it,
  that is evidence about the change; it is not a reason to re-run until the
  numbers agree.
