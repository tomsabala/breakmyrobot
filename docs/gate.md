# Phase 0 rendering / performance gate

Measured 2026-10-22 on the development workstation. Reproduce with
`scripts/gate.sh <row> <seconds>`; the world is
`ros2_ws/src/ral_description/worlds/gate_test.sdf`.

## Result: row 1 selected

| Row | Lidar samples | Lidar rate | `max_step_size` | Measured RTF | Selected |
| --- | --- | --- | --- | --- | --- |
| 1 | 360 | 10 Hz | 0.002 s | **0.999 – 1.002** | **yes** |
| 2 | 180 | 10 Hz | 0.002 s | not measured — row 1 already ≥ 0.8 | no |
| 3 | 180 | 5 Hz | 0.004 s | not measured — row 1 already ≥ 0.8 | no |

Row 1 is the preferred row and cleared the ≥ 0.8 threshold on the first
attempt, so rows 2 and 3 were never run. **These values are now the lidar and
physics defaults for every world and robot in later phases** (`ral_roomba`,
`ral_car`, `ral_apartment`).

Raw evidence, `/world/gate_test/stats` after 60 s of sim time:

```
sim_time  { sec: 60 nsec:  48000000 }
real_time { sec: 62 nsec:  77757276 }
iterations: 30024
real_time_factor: 1.0022832011321792
step_size { nsec: 2000000 }
```

Five consecutive samples ranged 0.99921 – 1.00228.

Two distinct numbers live in that block, and they answer different questions:

- `real_time_factor` as reported by gz is a **windowed average over recent
  steps** — steady-state throughput, 0.999 – 1.002 here. This is the number the
  row selection uses, because it describes the cost of the chosen parameters.
- The **cumulative** ratio is 60.048 / 62.078 = **0.967**. `real_time` counts
  from process start, so it includes ~2 s of ogre2/EGL initialisation before
  stepping begins. On a 60 s run that fixed overhead is 3 %; on a 180 s Phase 3
  scenario it is 1 %.

Neither affects results. Every node runs `use_sim_time:=true` and every metric
derives from `/clock`, so RTF sets only how long a run takes in wall-clock
time. `iterations: 30024` × `step_size` 0.002 s = 60.048 s of `sim_time`,
confirming the physics stepped exactly as configured.

## Rendering: hardware vs software

| Path | How | Measured RTF | Lidar publishes |
| --- | --- | --- | --- |
| Intel iGPU via EGL | `/dev/dri` passed through (default) | 0.999 – 1.002 | yes |
| Mesa llvmpipe (software) | `RAL_NO_GPU=1` | 1.00004 – 1.00005 | yes |

**The gate did not discriminate between the two.** This scene — six static
boxes, one 360-beam lidar at 10 Hz — is small enough that software
rasterisation keeps up. The GPU passthrough is kept as the default anyway
because Phase 1 adds an articulated robot, a second sensor (IMU) and a larger
world, where the margin is expected to matter; that expectation is **not yet
measured**. `xvfb-run` was never needed: EGL produced a working ogre2 context
directly (`RAL_XVFB=1` remains available).

## Geometry verification

The scene is dimensioned so the scan can be checked against arithmetic. Read
from the ROS side (`ros2 topic echo /scan_raw --once --full-length`) through
`ros_gz_bridge`, with 360 beams and `angle_increment` 0.017502 rad:

| Beam | Measured | Expected | Source of expectation |
| --- | --- | --- | --- |
| 0° (+x) | 1.5001 m | 1.5 m | front face of the 1 m box centred at x = 2 |
| +90° (+y) | 3.9502 m | 3.95 m | wall centred at y = 4, thickness 0.1 |
| −90° (−y) | 3.9502 m | 3.95 m | wall centred at y = −4 |
| 180° (−x) | 4.9503 m | 4.95 m | wall centred at x = −5 |

Range span 1.5001 – 6.3055 m, 141 distinct values across 360 beams. The
rendering path is dimensionally correct, not merely producing data.

## Host ↔ container graph

`ros2 topic list` **on the host** (outside the container) shows `/scan_raw` and
`/clock` published by `ros_gz_bridge` inside the container, confirming
`--network host --ipc=host` gives host tooling (rviz2, ros2 CLI) access to the
container's DDS graph.

## Four defects found and fixed while establishing this measurement

Each of these presented as "the simulator silently produces nothing", and each
has a one-line fix that later phases depend on.

1. **`colcon` froze the host twice.** Unbounded `colcon build` of `ros_gz`
   (12 parallel workers, `ros_gz_bridge` TUs at 1.5–2.5 GiB) against 15 GiB RAM
   and a 2 GiB swapfile → reclaim livelock, no OOM kill, power cycle to
   recover. BuildKit ignores `--memory` on `docker build`, so the overlay
   compile moved into a capped `docker run` (`scripts/build_image.sh`,
   `docker/build_overlay.sh`), serialized and limited to 6 GiB. Build time
   7 min 6 s, `ros_gz_bridge` alone 5 min 55 s.
2. **EGL permission denied.** `--device /dev/dri/renderD128` recreates the node
   inside the container's devfs and drops the host ACL (`user:toms:rw-`), so
   the host uid hit `EPERM` and ogre2 segfaulted after 11 attempts.
   Fix: `--group-add` the device's owning gid, read off the node
   (`scripts/dev.sh`).
3. **gz-transport discovery returned nothing.** gz-transport derives its
   default partition from `hostname:username`; `--user 1001:1001` has no
   `/etc/passwd` entry (`whoami: cannot find name for user ID 1001`), so the
   simulator and `gz topic` landed in different partitions. The sim ran,
   published, and appeared invisible. Fix: `GZ_PARTITION`/`USER` exported in
   `docker/entrypoint.sh`.
4. **Host saw an empty ROS graph.** The host exports `ROS_DOMAIN_ID=42`; the
   container defaulted to domain 0. Fix: `scripts/dev.sh` propagates
   `ROS_DOMAIN_ID` and `ROS_LOCALHOST_ONLY`.

Investigated and ruled out along the way: multicast loopback (works on both
`lo` and the LAN address), the host firewall (`ufw` disabled), discovery port
conflicts (none), Fast DDS shared-memory rendezvous (`/dev/shm` shared, same
inode — a UDP-only profile changed nothing and was discarded).

## Resolved plan assumption: bridge message prefix

`RESILIENT_AUTONOMY_LAB_MVP_PLAN.md` flagged `ignition.msgs.*` vs `gz.msgs.*`
as unverified. Confirmed: **`gz.msgs.*`**. The bridge accepted
`/scan_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan` and logged
`Creating GZ->ROS Bridge: [/scan_raw (gz.msgs.LaserScan) -> /scan_raw
(sensor_msgs/msg/LaserScan)]`. All Phase 1 bridge configs must use the
`gz.msgs.*` prefix.

## Environment

| Item | Value |
| --- | --- |
| Host | Ubuntu 22.04, kernel 6.8.0-138, i7-1255U (12 threads), 15 GiB RAM, 2 GiB swap |
| GPU | Intel iGPU `46a8`, `/dev/dri/renderD128`, Mesa 23.2.1 |
| Images | `ral:base` → `ral:overlay` → `ral:dev` (3.62 GB) |
| Gazebo | Harmonic, `gz-sim8` 8.15.0, `gz-transport13` 13.6.0, `gz-rendering8` 8.3.0, ogre2 |
| ROS 2 | Humble; `ros_gz` + `sdformat_urdf` built from source against `GZ_VERSION=harmonic` |
| Docker | 28.1.1, overlay2, cgroup v2 |
