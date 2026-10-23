# Maps

Three files here are a **matched set**:

| File | What it is |
| --- | --- |
| `apartment.pgm` | the occupancy grid, as `slam_toolbox` built it |
| `apartment.yaml` | its metadata (`resolution: 0.05`, origin, thresholds) |
| `apartment_frames.yaml` | the measured gz `world` → SLAM `map` alignment for that grid |

## Where the map comes from

One `slam_toolbox` session: the robot was driven through `/cmd_vel` while
`slam_toolbox` built the grid from `/scan`, and the result was saved with
`map_saver_cli`.

The drive was performed by **`scripts/map_drive.py`**, a scripted stand-in for a
human at `teleop_twist_keyboard`. It steers from `/ground_truth/odom` — the
*operator's* view, the equivalent of a person watching the robot in RViz — and
publishes `/cmd_vel` and nothing else. `slam_toolbox` consumes `/scan` and
`odom`→`base_link` only, so no truth reaches the map. The script exists because a
keyboard drive cannot be reproduced; `teleop_twist_keyboard` remains a valid
substitute for a human operator.

The map therefore contains **real SLAM error** relative to the world geometry in
`ral_description/worlds/ral_apartment.sdf` — slightly thick walls, small angular
skew, smearing where the robot turned fast. That is intentional. Navigating
against a map that carries the error a real mapping run produces is the point;
AMCL's residual against it is a real localization result, not a rounding artifact.

**Do not regenerate this map from world geometry.** A grid rasterised from the SDF
would be a perfect map of a world the robot never measured, and every localization
number afterwards would describe a system that cannot exist.

### Measured failure: perceptual aliasing, and the fix

The **first** mapping run of this project, with `slam_toolbox`'s shipped loop
closure settings, produced a map containing a **ghost copy of room 1 offset by
−7.95 m in x**, with wall A and the east wall missing entirely. The apartment is
three near-identical 4 m rooms in a row, so a scan registered in one room matches
its neighbours almost as well as its own, and loop closure merged the wrong ones.

Both doorways were still visible as gaps in that map, so an eyeball test passed on
a structurally destroyed grid. That is why `scripts/checks/map_sanity.py` exists
and why its checks are about **global consistency with the world SDF** rather than
local appearance.

`config/slam_roomba.yaml` now narrows `loop_search_space_dimension` to `3.0`
(±1.5 m, narrower than the 4 m room pitch, so the aliased alignment is
unreachable) and raises the loop-match response and chain-size thresholds. The
accepted map was produced with those values from the same route.

## `apartment_frames.yaml`

`slam_toolbox` registers its first scan at that scan's *odometric* pose, so it
puts the `map` frame origin on the **odom frame origin** — which, for a robot
still sitting where it spawned, is also its true pose. `world_to_map` is that
pose expressed in the gz `world` frame, sampled from `/ground_truth/odom` by
`ral_eval`'s `capture_world_to_map` node at `slam_toolbox`'s first `/map`
publication. The node verifies both halves before writing: the robot has not
moved (exit 2 otherwise) *and* `/odom` still reads as the identity (exit 4
otherwise). Stationarity alone is not enough — a robot parked anywhere after a
drive is perfectly stationary and offset from where `map` actually is.

It is consumed in two places and both need it to belong to *this* `.pgm`:

- `roomba_nav.launch.py` inverts it (`ral_eval.frames.map_to_world_tf`) into a
  static, **display-only** `map` → `world` transform so RViz can draw
  `/ground_truth/odom` on the map. It carries a fixed frame alignment, never a
  robot pose, so no truth reaches the navigation path.
- Phase 3's metrics apply it directly to express truth in `map` and difference it
  against the estimate.

A stale pair — a new `.pgm` with an old alignment — biases every estimate-error
number by a constant offset that looks exactly like localization error. That is
why the pair is **sealed**: `scripts/checks/map_sanity.py` writes the accepted
grid's SHA-256 into `apartment_frames.yaml` as `map_sha256`, and
`ral_eval.frames.load_alignment_for_map` refuses to load an alignment whose
`map_sha256` does not match the grid beside it. `roomba_nav.launch.py` therefore
will not start on a stale or unsealed pair, and check 1 of
`scripts/checks/roomba_nav_check.sh` re-derives the same comparison from the
bytes the running stack loaded.

## Regenerating (all three, together)

```bash
scripts/build.sh

# shell 1 - sim + scan relay + slam_toolbox + the alignment capture node
scripts/dev.sh bash -lc 'source /ws/ros2_ws/install/setup.bash && \
  ros2 launch ral_bringup map_build.launch.py'
# WAIT for: [capture_world_to_map] world_to_map captured: x=... y=... yaw=... -> ...
# Do not drive before that line appears: the capture node exits 2 if the robot
# moved, because the pose where `map` was defined is then unobservable.

# shell 2 (host; the container image is ros-base and has no rviz2)
source /opt/ros/humble/setup.bash && rviz2 -d ros2_ws/src/ral_bringup/rviz/ral.rviz

# shell 3 - drive. The scripted route; swap in `ros2 run teleop_twist_keyboard
# teleop_twist_keyboard` to drive by hand instead.
scripts/dev.sh bash -lc 'source /ws/ros2_ws/install/setup.bash && \
  python3 /ws/scripts/map_drive.py'

# shell 4 - save, with the stack still running
scripts/dev.sh bash -lc 'source /ws/ros2_ws/install/setup.bash && \
  ros2 run nav2_map_server map_saver_cli -t /map \
    -f /ws/ros2_ws/src/ral_bringup/maps/apartment --ros-args -p use_sim_time:=true'

# shell 5 - check and seal the pair, then install all three artifacts
python3 scripts/checks/map_sanity.py
scripts/build.sh --packages-select ral_bringup
```

`map_sanity.py` is what **seals** the pair: it writes the accepted grid's
SHA-256 into `apartment_frames.yaml`, and until it has run,
`roomba_nav.launch.py` refuses to start. Run it before the rebuild so the
sealed file is what gets installed.

Drive requirements, because the map decides everything downstream: cover all three
rooms, pass **both** doorways (0.90 m at (4.0, 4.0) and 0.70 m at (8.0, 3.0)) at
least twice in each direction, circle each of `furniture_a/b/c`, and return to the
start area so the loop closes. Keep speeds at or below 0.2 m/s — `slam_toolbox` is
configured with `minimum_travel_distance: 0.2`, and dragging a wall at full speed
smears it.

Accept the result only if **`scripts/checks/map_sanity.py` passes all seven
checks** — the four outer walls present where the SDF puts them, no occupancy
outside the shell, both interior walls present with a correctly-sized gap, the
start pose in free space, and the alignment matching this grid — and
`apartment_frames.yaml`'s `world_to_map` is within ±0.05 m / ±0.05 rad of the
spawn pose `(0.5, 0.5, 0.0)`. Otherwise redo the drive — do not patch the map,
and do not relax the checker.

A larger `world_to_map` should not be reachable: `capture_world_to_map` already
refuses to write when the robot has moved (exit 2) or is not at the odom origin
(exit 4). If it happens anyway, the capture guard is what is wrong, not the
tolerance here.

**After regenerating, re-run `scripts/checks/roomba_nav_check.sh`** and let it
rewrite `docs/nav-closed-loop.md`. The numbers in that document — the alignment
residual, the mission result, the estimate error — are all measured against this
specific map, and the document records the SHA-256 of the `.pgm` they came from.

## RViz note

`ral.rviz` does not display AMCL's particle cloud. It is a
`nav2_msgs/msg/ParticleCloud`, and RViz runs on the **host**, which has
`rviz_default_plugins` but no `nav2_msgs` or `nav2_rviz_plugins`; a config
referencing it fails to load on the machine that actually runs RViz.
`/amcl_pose` with its covariance ellipse is displayed instead. Installing
`ros-humble-nav2-msgs ros-humble-nav2-rviz-plugins` on the host would allow adding
it — that is a host change, not a repository change.
