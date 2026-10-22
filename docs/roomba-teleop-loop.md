# Roomba platform and teleop closed loop — measured proof

Measured 2026-10-22 13:40 UTC by
`scripts/_validations/roomba_teleop_check.sh`, which launches
`ral_bringup/roomba_sim.launch.py` and drives the robot from ROS. Every
duration is sim time off `/clock`. This is the MVP plan's Phase 1 scope:
the robot, the world, the bridge and the teleop loop, with no map,
localization, planner or fault injection yet.

## Result: all checks passed

| # | Check | Result | Measured |
| --- | --- | --- | --- |
| 1 | TF chain odom->rplidar_link | PASS | translation z = 0.0800 m (expected 0.08 +/- 0.005) |
| 2 | scan frame and content | PASS | frame_id='rplidar_link', 360/360 finite ranges in [0.15, 12.0], min=0.418 m |
| 3 | truth off the TF tree | PASS | world/base_link_truth absent from the tree and lookup raises LookupException; frames = ['base_link', 'caster_front', 'caster_rear', 'left_wheel', 'right_wheel', 'rplidar_link'] |
| 4 | bumper contact while driving | PASS | contact after 14.10 s sim, collisions = ['ral_roomba::base_link::base_collision', 'wall_a_lower::link::collision'] |
| 5 | no wall penetration | PASS | truth x at contact = 3.7812 m (wall face 3.95 - radius 0.1675 + 0.02 = 3.8025) |
| 6 | robot stopped by the wall | PASS | truth x moved 0.0013 m over 2.0 s sim while still commanding 0.3 m/s |
| 7 | dead reckoning diverges from truth | PASS | odom x = 3.8811, truth x - spawn = 3.2825, divergence = 0.5986 m (wheels slip while the body is held) |
| 8 | rotate in place | PASS | truth translation 0.0020 m (< 0.02) while accumulating 5.794 rad of yaw (> 3.0) over 6.0 s sim |
| 9 | IMU and joint states carry real content | PASS | imu frame_id='base_link', peak angular rate about z during the commanded 1.0 rad/s rotation = 1.093 rad/s; wheel joint left_wheel_joint turned 90.2 rad, right_wheel_joint turned 128.1 rad |

## Run

| Item | Value |
| --- | --- |
| World | `ral_apartment` |
| Robot | `ral_roomba` |
| Sim time elapsed | 23.10 s |
| Real-time factor (`/world/ral_apartment/stats`) | 0.9991851644983516, 0.99980003999200162, 0.99843395633948151 |
| Image | `sha256:ce16e2833f14392f3d5e4c49c812cfe4855b8829f41825119d8c3e363fcef6f6` |

## What these checks establish

- The robot is driven only through `/cmd_vel` -> `DiffDrive`; nothing
  teleports it and no path is scripted.
- `/odom` is integrated from wheel-joint rotation, so it diverges from
  truth exactly when the wheels slip (check 7). Equal values would mean
  simulator truth had leaked into the estimate.
- Ground truth reaches ROS on `/ground_truth/odom` only, in frames
  (`world` -> `base_link_truth`) that exist in no TF tree (check 3), so
  no localization or navigation component can consume it by accident.
- The bumper is a real contact sensor on `base_collision` (check 4), and
  the wall stops the robot through physics, not a script (checks 5, 6).
- The IMU measured the rotation the robot actually performed and the
  wheel joints turned by the amount the drive required (check 9), so
  those streams carry sensor content, not just a live topic name.

## Not covered here

No map, localization, planner or controller exists yet (Phase 2); no
fault can be injected yet (Phase 4). Nothing in Phase 1 consumes `/imu`
or `/joint_states`: check 9 reads them directly, and no state estimator
fuses them. The IMU is modelled without noise or bias, so its stream is
cleaner than real hardware.
