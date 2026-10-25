# Roomba platform and teleop closed loop — measured proof

Measured 2026-10-22 14:19 UTC by
`scripts/checks/roomba_teleop_check.sh`, which launches
`ral_bringup/roomba_sim.launch.py` and drives the robot from ROS. Every
duration is sim time off `/clock`. This is the MVP plan's Phase 1 scope:
the robot, the world, the bridge and the teleop loop, with no map,
localization, planner or fault injection yet.

## Result: all checks passed

| # | Check | Result | Measured |
| --- | --- | --- | --- |
| 1 | TF chain odom->base_link->rplidar_link | PASS | odom->base_link x = -0.0000 m, base_link->rplidar_link = (0.0000, 0.0000, 0.0800) m (expected (0, 0, 0.08) +/- 0.005) |
| 2 | scan frame and content | PASS | frame_id='rplidar_link', 360/360 finite ranges in [0.15, 12.0] (>= 90 % required), min=0.420 m, max=10.523 m |
| 3 | truth off the TF tree | PASS | TF frames = ['base_link', 'caster_front', 'caster_rear', 'left_wheel', 'odom', 'right_wheel', 'rplidar_link']; unexpected = [], missing = []; world->base_link_truth does not resolve |
| 4 | bumper contact while driving | PASS | contact after 14.09 s sim, collisions = ['ral_roomba::base_link::base_collision', 'wall_a_lower::link::collision'] |
| 5 | contact at the wall surface | PASS | truth x at contact = 3.7830 m, expected in [3.7325, 3.8025] (wall face 3.9500 parsed from the world SDF, robot radius 0.1675) |
| 6 | robot stopped by the wall | PASS | truth x moved 0.0005 m over 2.0 s sim while still commanding 0.3 m/s |
| 7 | dead reckoning diverges from truth | PASS | odom x = 3.8835 (advanced 0.6000 m while held), truth x - spawn = 3.2825, divergence = 0.6010 m (wheels slip while the body is held) |
| 8 | rotate in place | PASS | truth translation 0.0022 m (< 0.02) while accumulating 5.790 rad of yaw (> 3.0) over 6.0 s sim |
| 9 | IMU and joint states carry real content | PASS | imu frame_id='base_link', peak angular rate about z during the commanded 1.0 rad/s rotation = 1.093 rad/s; wheel joint left_wheel_joint turned 90.3 rad, right_wheel_joint turned 128.1 rad |

## Run

| Item | Value |
| --- | --- |
| World | `ral_apartment` |
| Robot | `ral_roomba` |
| Sim time elapsed | 23.10 s |
| Real-time factor (`/world/ral_apartment/stats`) | 1.0006504227748036, 0.99733014719595647, 1.0006494214745369 |
| Image | `sha256:ce16e2833f14392f3d5e4c49c812cfe4855b8829f41825119d8c3e363fcef6f6` |

## What these checks establish

- The robot is driven only through `/cmd_vel` -> `DiffDrive`; nothing
  teleports it and no path is scripted.
- `/odom` is integrated from wheel-joint rotation, so it diverges from
  truth exactly when the wheels slip (check 7). Equal values would mean
  simulator truth had leaked into the estimate.
- Ground truth reaches ROS on `/ground_truth/odom` only. Check 3
  asserts the complete TF frame set, so truth bridged onto `/tf` under
  any frame name - including the simulator's own `<world> -> <model>`
  names - shows up as an unexpected frame and fails the check.
- The bumper is a real contact sensor on `base_collision` (check 4), and
  the wall stops the robot through physics, not a script (checks 5, 6).
- The IMU measured the rotation the robot actually performed and the
  wheel joints turned by the amount the drive required (check 9), so
  those streams carry sensor content, not just a live topic name.
- The checks themselves have been exercised against deliberately
  broken stacks; the mutations, the dates and the checks that went red
  are recorded in the "Negative controls" section of
  `scripts/checks/roomba_teleop_check.py`. The bridge rule is also
  enforced without a simulator by `ral_bringup`'s
  `test/test_bridge_contract.py`, which runs in every `colcon test`.

## Known plant properties and limits

- `DiffDrive` has no command timeout: the last `/cmd_vel` is held
  indefinitely. A Phase 4 communication fault therefore produces a
  robot that keeps driving on its last command, not one that stops.
  That is the modelled behavior, and no watchdog exists to change it.
- `/ground_truth/odom.twist` is a finite difference of truth poses, not
  a simulator-reported velocity: forward and lateral components are
  resolved on the previous heading, and a sample with no valid
  difference is marked `twist.covariance[0] = -1`, never published as a
  zero velocity.
- The IMU is modelled without noise or bias, so its stream is cleaner
  than real hardware. The lidar carries sigma = 0.015 m.

## Not covered here

This harness runs the bare platform: no map, localization, planner or
controller is started here, and no fault is injected here. Both exist
now and are exercised elsewhere - the navigation stack by
`scripts/checks/roomba_nav_check.py`, the five faults by the
`ral_eval` scenario runs under `runs/`. Nothing in this stack consumes
`/imu` or `/joint_states`: check 9 reads them directly, and no state
estimator fuses them.
