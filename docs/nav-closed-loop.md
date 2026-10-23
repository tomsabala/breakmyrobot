# Map, AMCL and Nav2 closed loop — measured proof

Measured 2026-10-23 10:33 UTC by
`scripts/checks/roomba_nav_check.sh`, which launches
`ral_bringup/roomba_nav.launch.py`, localizes the robot against the
committed static map and sends one `NavigateToPose` goal in room 3 that
is reachable only through wall B's 0.70 m doorway. Every duration is sim
time off `/clock`. This is the MVP plan's Phase 2 scope: no fault can be
injected yet, and no scenario runner or dashboard exists.

## Result: all checks passed

| # | Check | Result | Measured |
| --- | --- | --- | --- |
| 1 | static map loaded and matched to its alignment | PASS | resolution 0.05000000074505806 m, 241 x 162 cells = 12.05 x 8.10 m (>= 12.00 x 8.00 required for the apartment), sha256(apartment.pgm) = 47d1e2ad01a41a52, alignment sealed to 47d1e2ad01a41a52 |
| 2 | /scan has exactly one publisher: scan_fault_relay | PASS | /scan publishers = ['scan_fault_relay'], /scan_raw publishers = ['ros_gz_bridge']; a second publisher or a different node means lidar faults would not reach every consumer |
| 3 | relay output equals relay input | PASS | at stamp 12.100000000: 360 ranges, frame_id 'rplidar_link' -> 'rplidar_link', element-for-element identical = True |
| 4 | no navigation component subscribes to truth | PASS | /ground_truth/odom subscribers = ['roomba_nav_check']; navigation offenders = [] |
| 5 | TF topology | PASS | frames = ['base_link', 'caster_front', 'caster_rear', 'left_wheel', 'map', 'odom', 'right_wheel', 'rplidar_link', 'world']; unexpected = [], missing = [], wrong parents (got, want) = none |
| 6 | alignment residual | PASS | stationary \|AMCL - truth\| = 0.1128 m (limit 0.15), covariance trace 0.2217 after an in-place rotation at 0.6 rad/s. This is the floor below which estimate error cannot be interpreted: it is map error plus alignment error, not localization error |
| 7 | mission succeeds | PASS | NavigateToPose result status = 4 (SUCCEEDED) after 37.6 s sim (limit 180) |
| 8 | goal reached in ground truth | PASS | \|final truth - goal\| in map = 0.031 m (limit 0.25); truth world = (10.469, 6.497) |
| 9 | threaded the 0.70 m doorway | PASS | 1 crossing(s) of x = 8.0 at y = [3.045]; inside the gap (\|y - 3.0\| <= 0.35): 1 |
| 10 | no collisions during the mission | PASS | 0 contact report(s) on /bumper |
| 11 | estimate is an estimate, not truth | PASS | mean \|AMCL - truth\| = 0.0707 m, max = 0.1130 m over 103 paired samples (required: > 0.005 so it is not truth, < 0.5 so it is localized) |
| 12 | commands match the configured envelope | PASS | max \|cmd_vel.linear.x\| = 0.400 m/s (configured 0.4, required 0.380-0.4, plant 0.46), max \|cmd_vel.angular.z\| = 0.395 rad/s (plant 1.9); 37 /plan publication(s) |

## Run

| Item | Value |
| --- | --- |
| World | `ral_apartment` |
| Robot | `ral_roomba` |
| Map | `ral_bringup/maps/apartment.yaml` |
| `sha256(apartment.pgm)` | `47d1e2ad01a41a52f4a866918fe46c7d32246a8fa393b809125391d95fd48c0b` |
| `world_to_map` | x = 0.5, y = 0.5, yaw = 0.0 |
| Alignment captured | 2026-10-22T15:32:41Z |
| Alignment `git_sha` | `a4c435427a6b35127be8951a21759b05143e2a7c` |
| `inflation_radius` in force | local_costmap 0.45 m, global_costmap 0.45 m |
| Initial pose (world -> map) | (0.5, 0.5, 0.0) -> (0.000, 0.000, 0.000) |
| Goal (world -> map) | (10.5, 6.5, 1.57) -> (10.000, 6.000, 1.570) |
| Sim time elapsed | 45.85 s |
| Real-time factor (`/world/ral_apartment/stats`) | 0.99976055734651548, 0.99989051198893719, 0.99544484439206193 |
| Image | `sha256:ce16e2833f14392f3d5e4c49c812cfe4855b8829f41825119d8c3e363fcef6f6` |

## What these checks establish

- The robot localizes against a map built by `slam_toolbox` from its own
  `/scan`, not against world geometry, and against the alignment measured
  for that exact grid — check 1 compares the loaded `.pgm`'s sha256 with
  the one sealed into `apartment_frames.yaml`, so a new map beside an old
  alignment fails rather than reporting a frame offset as drift. Check 6
  measures the stationary residual, which is map error plus alignment
  error — the floor under every estimate-error number in this project.
- Nav2 planned and drove a real mission: the goal was accepted and
  reported `SUCCEEDED` (check 7), ground truth confirms arrival
  independently of what AMCL believed (check 8), and truth shows the body
  physically passing through the 0.70 m doorway (check 9).
- The estimate is an estimate: AMCL's pose differs from truth at every
  sample, but stays localized (check 11). Equality would mean truth had
  leaked into localization.
- `/scan` has exactly one publisher, `scan_fault_relay`, and its output is
  byte-identical to `/scan_raw` (checks 2, 3). Phase 4.2 injects sensor
  degradation at that node, so every lidar consumer — AMCL and both
  costmaps — is downstream of the fault by construction.
- No navigation component subscribes to `/ground_truth/odom` (check 4),
  and the complete TF frame set is asserted (check 5), so truth bridged
  onto `/tf` under any name — including the simulator's own
  `<world> -> <model>` names — fails the check.
- The commands the plant received reached the envelope Nav2 is
  configured for and did not exceed it (check 12, two-sided on linear
  speed). The lower bound is the load-bearing half: `velocity_smoother`
  publishes `/cmd_vel`, and left at its shipped TurtleBot3 `0.26 m/s` it
  would clamp DWB while still sitting inside the plant's `0.46 m/s`.
  Reaching the configured maximum is what shows the override is in force
  at runtime, not merely present in the YAML.
- The configuration is additionally enforced without a simulator by
  `ral_bringup/test/test_nav2_contract.py`, which runs in every
  `colcon test`. It also covers `behavior_server`, whose recovery
  rotations are published on `cmd_vel` **unremapped** by Nav2's
  `navigation_launch.py` and therefore bypass the velocity smoother:
  its shipped `3.2 rad/s²` exceeds this plant's `2.5 rad/s²`, so it is
  overridden too.
- The map itself was accepted by `scripts/checks/map_sanity.py`, which
  checks it against the world SDF rather than by eye.

## Known limitations

- **The map carries real SLAM error by design.** It was built from one
  mapping drive, not rasterised from `ral_apartment.sdf`. Walls are
  slightly thick and slightly skewed; that error is part of every result
  here and is intentional. The drive was scripted
  (`scripts/map_drive.py`) and steered from ground truth as the
  *operator's* view; `slam_toolbox` consumed only `/scan` and
  `odom -> base_link`.
- **The first mapping run of this world produced an unusable map** — a
  ghost copy of room 1 offset by −7.95 m, from loop closure aliasing
  across three near-identical 4 m rooms. `config/slam_roomba.yaml`
  narrows the loop search window below the room pitch; see
  `ral_bringup/maps/README.md`.
- **Check 6 is a floor, not a localization score.** The stationary
  residual mixes map error, the `world -> map` alignment measurement and
  AMCL's own error, and nothing in this phase separates them.
- **AMCL's particle cloud is not displayed.** It is a
  `nav2_msgs/msg/ParticleCloud` and RViz runs on the host, which has no
  `nav2_msgs`. `/amcl_pose` with its covariance is shown instead.
- **The `map -> world` transform is display only.** It carries a fixed,
  measured frame alignment so RViz can draw truth on the map; no
  navigation component reads it, and Phase 3's metrics must read
  `maps/apartment_frames.yaml` directly rather than listening to it.
- **No fault is exercised.** Scan degradation, obstacle spawning,
  communication loss and kidnapping are Phase 4. Nothing here shows how
  the stack behaves when a subsystem is broken.
- **One robot, one world, one goal.** The car-like platform is Phase 5,
  and the MVP's "Roomba passes, car does not" scenario depends on the
  same 0.70 m doorway measured in check 9.
