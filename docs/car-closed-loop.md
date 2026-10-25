# ArduRover SITL, MAVROS and the car closed loop — measured proof

Measured 2026-10-25 13:47 UTC by
`scripts/checks/car_nav_check.sh`, which launches
`ral_bringup/car_nav.launch.py`, puts the autopilot into `GUIDED`, arms it
and drives one real mission to `baseline_car.yaml`'s goal. Every duration is
sim time off `/clock`.

There is no Nav2, no `map_server` and no AMCL on this platform: ArduRover
plans and controls inside the autopilot, EKF3 estimates the pose from the IMU
and SITL's synthesised GPS, and MAVROS is the only link between that
autopilot and ROS. The mission therefore ends on a CONJUNCTION of two
things the robot itself reports: the FCU's own
`NAV_CONTROLLER_OUTPUT.wp_dist` inside the profile's `goal_tolerance_m`
on three consecutive messages while the mode is still `GUIDED` and the
vehicle armed, AND the robot's own estimate within
`arrival_estimate_tolerance_m` of the setpoint this run published —
the same rule `ral_eval`'s `MavrosGuidedDriver` applies in a scenario
run, re-implemented here rather than imported.

## Result: all checks passed

| # | Check | Result | Measured |
| --- | --- | --- | --- |
| 0 | bring-up | PASS | one message on each of /clock, /ground_truth/odom, /scan, /mavros/state, /mavros/local_position/odom, /frame_alignment within 240 s wall |
| 1 | autopilot link: connected, GUIDED, armed | PASS | /mavros/state connected=True, mode='GUIDED', armed=True, guided=True, manual_input=False, system_status=4 (MAV_STATE); services ready = True; set_mode: /mavros/set_mode answered mavros_msgs.srv.SetMode_Response(mode_sent=True); arming: /mavros/cmd/arming answered mavros_msgs.srv.CommandBool_Response(success=True, result=0) |
| 2 | /scan has exactly one publisher: scan_fault_relay | PASS | /scan publishers = ['scan_fault_relay'], /scan_raw publishers = ['ros_gz_bridge']; a second publisher or a different node means lidar faults would not reach every consumer |
| 3 | relay output equals relay input | PASS | 311 /scan and /scan_raw message(s) shared a header stamp over the whole run; 0 differed; 177 raw message(s) were never republished and 177 /scan message(s) had no matching raw (start-up artefacts of this harness joining between the relay's input and its output) |
| 4 | only the harness, the fault injector and the frame measurer subscribe to truth | PASS | /ground_truth/odom subscribers = ['car_nav_check', 'ekf_local_tf', 'fault_manager']; navigation offenders = []; not allowlisted = [] |
| 5 | TF topology | PASS | frames = ['base_link', 'base_link_frd', 'ekf_local', 'front_left_wheel', 'front_right_wheel', 'map', 'map_ned', 'odom', 'odom_ned', 'rear_left_wheel', 'rear_right_wheel', 'rplidar_link', 'steer_left', 'steer_right', 'world']; unexpected = [], missing = [], wrong parents (got, want) = none |
| 6 | EKF origin alignment | PASS | world -> ekf_local published at (0.9995, 3.9995, 0.0451); this harness derived (0.9995, 3.9995, 0.0451) from its own first stationary /ground_truth/odom sample (1.0000, 4.0000, -0.0000) and its own first /mavros/local_position/odom sample (0.0006, 0.0005, -0.0451); they differ by 0.0000 m and 0.0000 rad (limits 0.05 m, 0.02 rad). The comparison is this harness's own measurement, not the publisher's self-report; for reference the publisher reported residuals 0.0008 m / 0.0451 rad against the spawn pose it predicts, and its source was: /ground_truth/odom and /mavros/local_position/odom, both stationary at the spawn pose |
| 7 | mission succeeds | PASS | terminal outcome mavros:reached_wp_radius after 5.7 s sim (limit 180): /mavros/nav_controller_output/output reported wp_dist 1 m within the profile's goal_tolerance_m 1 m on 3 consecutive messages while /mavros/state reported mode GUIDED and armed, having first reported a distance beyond that tolerance, AND the robot's own estimate was 1.48 m from the commanded setpoint, within the profile's arrival_estimate_tolerance_m 1.50 |
| 8 | goal reached in ground truth | PASS | \|final truth - goal\| in world = 1.035 m (limit 2.010 = arrival_estimate_tolerance_m 1.5 + footprint radius 0.292 + this run's measured max \|estimate - truth\| 0.218); truth world = (5.469, 3.905), goal world = (6.500, 4.000) |
| 9 | measured minimum turning radius | PASS | min(speed / \|yaw rate\|) over truth = 48.556 m from 263 sample(s) faster than 0.3 m/s, against the profile's geometric 0.611 m (wheelbase / tan(steer limit)); the floor this check enforces is 0.305 m. This mission never demanded anything near full steering lock, so the number is a LOWER BOUND on what this body did here and NOT a measurement of the plant limit. Below the floor would mean the body turned tighter than its geometry allows - skidding, or not behaving as an Ackermann vehicle - and that is the only way this check fails. |
| 10 | no collisions during the mission | PASS | 0 debounced collision event(s) from 0 raw /bumper contact report(s), collapsed at 0.5 s sim per colliding pair. Contact path liveness: /bumper ROS publishers = ['ros_gz_bridge']; gz contact topic /world/ral_apartment/model/ral_car/link/base_link/sensor/bumper/contact is listed by `gz topic -l`. Both halves are required: a contact sensor is silent until something touches it, so a count of zero reports is only evidence when the path that would have carried them is shown to exist |
| 11 | estimate is an estimate, not truth | PASS | max \|EKF3 estimate - truth\| = 0.2180 m, mean = 0.0875 m over 63 paired samples (required: > 0.005 so it is not truth, < 1.0 so the vehicle is localized). The estimate is ArduPilot EKF3 via MAVROS (LOCAL_POSITION_NED), read from /mavros/local_position/odom |
| 12 | achieved speed inside the configured envelope | PASS | max achieved speed (central difference of truth) = 0.967 m/s over 310 sample(s), against 1.0 m/s configured (CRUISE_SPEED/WP_SPEED in rover_ackermann.parm, carried in the profile as plant.max_linear_m_s) x 1.2 = 1.200 m/s. This is achieved speed, not a command: ArduRover is commanded by position setpoint and its throttle output is PWM |

## Run

| Item | Value |
| --- | --- |
| World | `ral_apartment` |
| Robot | `ral_car` (profile `ral_bringup/config/robots/car.yaml`) |
| Scenario | `baseline_car` |
| Initial pose (world) | (1.0, 4.0, 0.0) |
| Goal (world) | (6.5, 4.0, 0.0) |
| Goal tolerance | `goal_tolerance_m` 1.0 m (`WP_RADIUS`) AND `arrival_estimate_tolerance_m` 1.5 m, footprint radius 0.292 m |
| Autopilot | `ardurover` SITL, `ral_bringup/config/rover_ackermann.parm` |
| `sha256(rover_ackermann.parm)` | `196f993a0a16d6a423f24c6b1859ab9ee087de43050055bb70282f559e9255a6` |
| `ARMING_CHECK` in force | `0` (read from the `.parm` this run was started with; `0` disables the pre-arm checks that can only fail for simulation artefacts — an RC transmitter and a battery monitor SITL does not have) |
| Estimate | ArduPilot EKF3 via MAVROS (LOCAL_POSITION_NED), `/mavros/local_position/odom` |
| `world_to_ekf_local` (measured this run) | x = 0.9995, y = 3.9995, yaw = 0.0451, residual = 0.0008 m |
| Alignment source | /ground_truth/odom and /mavros/local_position/odom, both stationary at the spawn pose |
| Terminal outcome | `mavros:reached_wp_radius` |
| Mission duration | 5.70 s sim |
| Sim time elapsed | 10.00 s |
| Real-time factor under lockstep (`/world/ral_apartment/stats`) | 0.34378845596181612, 0.52190255400843344, 0.35357678273413856 |
| Image | `sha256:4eb135c402d4930989bed15baf411ec98110d0e346f4ae9f7e7e3bb84b0a670e` |

## What these checks establish

- The autopilot is a real one: MAVROS completed the MAVLink handshake,
  the FCU accepted `GUIDED` and armed, and it is the FCU's own
  `wp_dist` — not this harness's arithmetic and not simulator truth —
  that decided the mission ended (checks 1, 7). Ground truth confirms
  arrival independently of what the autopilot believed (check 8).
- The estimate is an estimate: EKF3's pose differs from truth at every
  sample but stays localized (check 11). Equality would mean truth had
  leaked into localization.
- The frame the autopilot works in was MEASURED, not assumed (check 6).
  ArduPilot sets its EKF origin at its first GPS fix, so the offset
  between the gz `world` frame and `ekf_local` is a per-run
  measurement; `ekf_local_tf` makes it, publishes it on
  `/frame_alignment` with the residual against the predicted origin,
  and refuses rather than fudging it.
- The vehicle is non-holonomic in fact and not only in its SDF: the
  tightest turn its body actually made is measured from truth and
  compared with the geometry the model and `TURN_RADIUS` declare
  (check 9).
- Truth reaches the harness, the fault injector and the frame measurer
  and nobody else (check 4), and the complete TF frame set is asserted
  (check 5), so truth bridged onto `/tf` under any name — including the
  simulator's own `<world> -> <model>` names — fails as well. There is
  no `odom` frame here at all: this platform has no wheel-odometry
  integrator, and its estimate chain is `map -> ekf_local -> base_link`.
- `/scan` has exactly one publisher, `scan_fault_relay`, and its output
  was byte-identical to `/scan_raw` on every message pair of the whole
  run (checks 2, 3).

## Known limitations

- **The steering is parallel, not true Ackermann.** Both knuckles take
  `ArduPilotPlugin` channel 0 with identical gains, so the inner and
  outer wheels steer to the same angle instead of to the two angles a
  common turn centre requires. The tyres therefore scrub in tight
  turns. What the MVP relies on survives: the vehicle is non-holonomic,
  cannot rotate in place, and has a finite minimum turning radius —
  check 9 measures what that radius actually was.
- **Check 9 is a measurement over this run's samples and nothing more.**
  It is `min(speed / |yaw rate|)` over the truth samples of ONE mission.
  If this mission never demanded full steering lock, the number is a
  lower bound for this run and not the plant's limit; it is evidence
  about what the body did, not a specification of what it can do. The
  `narrow_doorway_car` expectation rests on it together with the
  geometric radius in the profile, not on it alone.
- **`wp_dist` is a `uint16` in METRES, so arrival has two halves.** The
  FCU's own distance to its target has 1 m resolution and TRUNCATES, so
  with `WP_RADIUS` 1.0 a reported 1 means anywhere in [1, 2). That
  half alone cannot say where the vehicle came to rest, so the mission
  ends only when it is satisfied AND the robot's own estimate is within
  `arrival_estimate_tolerance_m` (1.5 m) of the
  commanded setpoint — both terms the robot's own, truth never read.
  Check 8 is scored against that gate plus the footprint radius plus
  THIS run's measured maximum |estimate − truth|, so its limit is a sum
  of measurements rather than a constant.
- **The lidar is carried for evidence and consumed by nothing.**
  ArduRover navigates on EKF3 position alone; no costmap and no
  obstacle avoidance runs for this robot. `/scan` is bridged, relayed
  and recorded so the sensor gate and the evidence cover both robots
  identically — and that is exactly why `scan_degradation` is REFUSED
  for this platform (`car.yaml`'s `unsupported`) instead of being
  offered as a fault with no effect.
- **The no-progress stop is a deterministic policy, not autonomy.** It
  lives in `ral_eval/mavros_driver.py` and belongs to the scenario
  runner: ArduRover carries no obstacle avoidance and will push against
  a wall until the scenario times out, and the policy turns that into a
  safe stop with a named cause. It closes on the robot's own estimate,
  never on truth. This harness does NOT run it — a harness that stopped
  the vehicle on its own schedule would be measuring its own policy —
  so an unfinished mission here is reported as "no terminal condition",
  not as a stall.
- **This platform publishes no numeric pose uncertainty.** ArduPilot
  does not send `LOCAL_POSITION_NED_COV`, so `/mavros/local_position/*`
  carries no covariance, and `mavros_msgs/msg/EstimatorStatus` in this
  image carries the twelve EKF health FLAGS only — no
  `pos_horiz_accuracy`. There is therefore no equivalent of the
  Roomba's covariance trace, and no convergence instant to time: EKF3
  sets its origin at the autopilot's first GPS fix with no
  operator-triggered initialization.
- **Real-time factor under lockstep was 0.34378845596181612, 0.52190255400843344, 0.35357678273413856.**
  `ArduPilotPlugin`'s `<lock_step>1</lock_step>` ties the autopilot's
  clock to gz's, so the two cannot drift apart, but the pair runs only
  as fast as the slower of them. Every duration above is sim time and
  is unaffected; the wall-clock cost of a run is not.
- **One mission, no fault.** This harness runs a fault-free baseline on
  purpose: it is the control the car's fault runs are read against.
  The fault evidence is the scenario runs (`mobility_fault_car`,
  `kidnap_car`, `narrow_doorway_car`), each with its own `run.json`,
  `metrics.json` and `plot.png` under `runs/`.
