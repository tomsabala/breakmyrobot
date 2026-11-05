# Engineering log — findings that changed the system

Each entry is a decision this project made because a measurement forced it, with
the run directories or launch logs the numbers came from. Nothing here is a
design intention: every claim is a figure some artifact under `runs/` or `logs/`
still holds.

Run directories are not committed (`runs/` is gitignored — bags are large), so
the identifiers below name artifacts on the machine that produced them. The
harness that regenerates each class of result is named with it.

## 1. The Roomba's local controller: DWB replaced by RotationShim + Regulated Pure Pursuit

**Symptom.** A measured operator mission (`runs/ui_roomba_20261026T165845Z`)
ended 3.07 m short of its goal with a valid global plan in hand.

**Cause.** DWB judges a candidate velocity by rolling it forward for a fixed
`sim_time` at the sampled speed. In a 0.65 m doorway or a 0.75 m pocket every
forward sample ends in collision at that horizon, so the controller commanded
near-zero instead of slowing down and threading the gap.

**Change.** `RotationShimController` wrapping `RegulatedPurePursuitController`:
speed is regulated by obstacle proximity and curve tightness rather than by a
fixed-horizon rollout, and the shim rotates in place when the path starts
off-heading.

**MEASURED** on the identical scenario — same three operator obstacles at the
same poses:

| Controller | Outcome | Distance from goal | Mission time (sim) | Nav2 recoveries |
| --- | --- | --- | --- | --- |
| DWB | `nav2:STATUS_CANCELED` | 3.070 m | 180 s (limit) | 10 |
| RotationShim + RPP | `nav2:STATUS_SUCCEEDED` | 0.167 m | 65 s | — |

## 2. The car got a global planner — what that changed, and what it did not

`ral_car` runs **no** AMCL and **no** Nav2 controller: ArduRover owns the
control loop and EKF3 owns the estimate. What it does run is Nav2 *global
planning only* (`map_server` + `planner_server`, `config/nav2_car.yaml`). The
driver (`ral_eval/mavros_driver.py`) asks `/compute_path_to_pose` for a route
that respects the body's rectangle and 0.611 m turning radius, follows it as
sequential GUIDED setpoints, republishes it on `/plan`, replans every 5 s sim,
and refuses a goal it cannot plan with `mavros:no_path`. Between replans the
autopilot steers straight at its current setpoint and has **no** reactive
avoidance.

**MEASURED on a straight corridor** (`baseline_car`, (1.0, 4.0) → (6.5, 4.0)):
a 7-waypoint, 5.55 m route, arrival, `goal_pose_error_m` 0.553–0.574 over three
runs, 0 collisions.

**MEASURED on a route that needs a turn** ((1.0, 1.0) → (6.5, 4.0)): the planner
returns a feasible 8-waypoint, 6.8 m route around wall A and the vehicle now
follows it — truth (1.0,1.0) → (3.1,2.2) → (3.7,2.7) → (3.9,3.9) → (4.2,4.2),
where the same mission without a planner drove into wall A and pinned at
(3.70, 2.08). It then wedges in the 0.9 m doorway and ends
`mavros:stalled_no_progress` with 2 collisions, 2.35 m from the goal.

**The limit is tracking accuracy against clearance, not the route.** There is no
local controller to correct the cross-track error the straight-line GUIDED
follower accumulates, and no reactive avoidance to recover from contact. The car
plans a route, follows it in open space, and fails visibly in tight geometry.

## 3. The pure-pursuit carrot is checked against the body's turning circle

Pure pursuit drives an arc of radius `d / (2 sin alpha)` to a candidate `d` away
at heading error `alpha`, so a candidate inside either minimum-radius circle
tangent to the nose is a command this body cannot obey.

**MEASURED before the check** (`runs/ui_car_20261026T221042Z`, the turning
route): the steering the commanded setpoint required averaged 30.8° against the
plant's ±30° of lock, while the steering actually taken averaged 3.3°. 143 of
338 odom samples carried a setpoint the body was geometrically forbidden from
reaching.

**Change.** The driver walks the carrot *past* unreachable candidates (never
past the corridor), refuses to publish one when none is admissible, re-requests
the plan from the current pose, and ends `mavros:carrot_unreachable` if that
produces nothing commandable for 15 s sim.

**MEASURED after** (`runs/ui_car_20261026T224207Z`), over the samples where the
setpoint is more than 1 m away — the range in which a carrot policy decides
anything, since the required radius goes to zero at a target the vehicle is
standing on:

| | before | after |
| --- | --- | --- |
| geometrically unreachable commands | 18 / 212 | 2 / 242 |
| p95 required steering | 26.7° | 20.5° |
| mean speed | 0.116 m/s | 0.160 m/s |

The planner then refused 9 replans from inside the doorway with "no feasible
path for this body and turning radius". The doorway remains the limit: the
vehicle still ends `mavros:stalled_no_progress` there.

## 4. Nav2 now starts on a measured condition, not a timer

`roomba_nav.launch.py` used to wait a fixed 11.5 s wall before including
`nav2_bringup`. That was both wasteful and unsafe.

- **Wasteful** — MEASURED, `logs/stack_roomba_20261026T204117Z.log`: the robot
  spawned at +0.7 s, the Nav2 container started at +12.2 s.
- **Insufficient** — MEASURED, `logs/stack_roomba_20261026T205614Z.log`: the
  include fired into an incomplete TF tree, the costmap looped on "two or more
  unconnected trees", the container ignored `SIGTERM` and was `SIGKILL`ed, and
  the stack failed with "no messages on /map within 90 s wall".

**Change.** A `ral_eval wait_for_tf` gate exits 0 only once `/clock` has carried
two messages **and** `odom -> base_link` resolves at the latest available time —
the question a costmap itself asks. Nav2 is started by an `OnProcessExit`
handler that shuts the launch down rather than starting Nav2 behind a failed
gate.

**MEASURED over five consecutive gateway starts:** `starting -> ready` in
20–22 s wall (median 21 s, against ~30 s before), zero failures, zero
"unconnected trees" lines.

The compressed bring-up exposed a second race — the runner seeded `/initialpose`
before AMCL had subscribed, so AMCL never got a pose and published nothing (3 of
5 starts). That is now gated on the subscriber existing rather than on a sleep.

## 5. A refusal that went stale, and the measurement that replaced it

`scan_degradation` used to be **refused** for the car, with the reason
"ArduRover's navigation does not consume the lidar; degrading it has no effect
on this platform". That was true when it was written and stopped being true
when the car gained a global planner: `config/nav2_car.yaml`'s global costmap
runs an `obstacle_layer` with `observation_sources: scan`, `topic: /scan` —
the fault relay's **output**, chosen there deliberately so that a degraded
lidar degrades what the planner sees. The platform was refusing a fault that
by then had a real consumer.

The fault is now offered for the car, and what it does was measured rather
than assumed.

**First attempt measured nothing, and said so.** `scan_degraded_car` with
`baseline_car`'s straight corridor and the Roomba's 15 s activation ended
`mavros:reached_wp_radius` after 7.15 s sim; `run.json` recorded the
intervention as `not_dispatched:mission_ended_first`
(`runs/scan_degraded_car_20261105T094654Z`). The scenario moved to the turning
route, where this platform actually replans, and the activation to 2 s.

**MEASURED** (`runs/scan_degraded_car_20261105T094852Z`): the degradation
activated at 57.52 s sim, and `/scan` diverged from `/scan_raw` on 370 paired
sweeps with **0 divergent before activation** — the stream really changed, and
only after the fault. The mission ended `mavros:stalled_no_progress` 3.03 m
from the goal with 1 collision after 3 plan publications, which is how the
**unfaulted** turning route ends as well (`runs/ui_car_20261026T224207Z`:
stalled, 2.49 m, 2 collisions, 3 plans). So: the fault reaches the planner's
input, and **no route change has been isolated** from this platform's dominant
failure, which is tracking accuracy in tight geometry. That sentence is
published to the operator verbatim — `car.yaml`'s new `scan_note` is appended
by `fault_manager` to the `activated` detail, the same mechanism `mobility_note`
already used, so a reader cannot mistake "the car did not react" for "the fault
did nothing".

The lesson is the failure mode, not the fault: a refusal is a claim about the
system, and it rots exactly like a comment does when the system grows a new
consumer.

## 6. The intervention that reports what it could not do

`reposition` moves the body in the simulator deliberately and
reports whether the robot's estimator could be reseeded to match:

- Roomba — publishes `/initialpose`, AMCL follows.
- Car — reports `estimate_reseeded: false`, because EKF3 exposes no seeding
  interface over MAVROS. Truth and estimate disagree by the move distance until
  the autopilot's own measurements pull them back.

## 7. Three structural properties of the dashboard

Not style choices — they are what make a browser mission evidence rather than a
demo.

1. **A browser mission is the same code path as a measured run.** It is driven
   through the same `MissionNode` and `MissionDriver` a scenario run uses, so it
   ends with the identical outcome vocabulary, and it writes its own run
   directory under `runs/ui_<robot>_<stamp>/` with `run.json`, `env.json`, the
   scenario it is equivalent to and `events.jsonl`. Ticking "Record bag +
   metrics" adds the bag, `metrics.json` and `plot.png` — so a browser mission is
   auditable by the same tools as a scenario run.
2. **The browser link is not a robotics protocol.** It is JSON over SSE + POST
   and is labelled "dashboard link" everywhere, so nobody reads it as a
   telemetry bus.
3. **One owner per ROS domain.** `scripts/dashboard.sh` takes the same `flock`
   on `/tmp/ral-run-$ROS_DOMAIN_ID.lock` that `scripts/run_scenario.sh` does, so
   a scenario run and the dashboard can never drive one stack.
