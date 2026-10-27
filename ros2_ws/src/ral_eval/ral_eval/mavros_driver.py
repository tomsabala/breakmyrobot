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
The MAVROS GUIDED mission driver: mode, arm, setpoint, and the FCU's own verdict.

ArduRover is not an action server. A GUIDED position setpoint
(SET_POSITION_TARGET_LOCAL_NED) is a stream, not a request: nothing
acknowledges it, nothing reports progress against it, and the vehicle stops if
it goes quiet for three seconds. So "the mission ended" has to be OBSERVED in
the autopilot's own telemetry rather than received as a result, and every
terminal outcome below names the topic and the value it was read from.

What ends a mission, first match wins:

  reached      BOTH halves, because neither alone is a distance. The FCU's
               own NAV_CONTROLLER_OUTPUT.wp_dist within the profile's
               goal_tolerance_m on three consecutive messages, while the mode
               is still GUIDED and the vehicle armed, AND the robot's own
               estimate within the profile's arrival_estimate_tolerance_m of
               the setpoint this run published. wp_dist is uint16 METRES and
               truncates, so on its own it cannot tell arrival from a reading
               one metre coarser than the gate; the estimate gap is a real
               distance and is what bounds where the vehicle came to rest.
               Both terms are the robot's own - truth is never read here.
  mode change  the autopilot left GUIDED by itself - an EKF failsafe, a
               thrust-loss failsafe, anything. Reported as what it is.
  disarm       the autopilot disarmed mid-mission.
  stall        the DETERMINISTIC no-progress policy below stopped the vehicle.
  cancel       the runner's timeout; the driver commanded HOLD and the FCU
               then reported SOME mode other than GUIDED - HOLD if it took
               the command, whatever it chose if it did not. The mode
               reported is named rather than asserted.

The no-progress supervisor is a PROGRAMMED POLICY, not autonomy, and is
labelled as one everywhere it appears (ENGINEERING.md section 6 forbids
presenting a programmed policy as a robot's own decision). ArduRover carries no
obstacle avoidance: against a wall it will push until the scenario times out.
The policy turns that into a safe stop with a named cause. It closes on the
robot's OWN estimate, never on truth.

Frames: the runner works in `map`, the autopilot in its EKF-local ENU frame.
The conversion uses the alignment ekf_local_tf MEASURED for this run and
published on /frame_alignment; without that message there is no way to express
a `map` goal in the autopilot's frame, so its absence is a refusal rather than
an assumption that the two frames coincide.
"""

import math
import time

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import EstimatorStatus, NavControllerOutput, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from ral_eval.frames import map_pose_to_world, world_pose_to_map, wrap_to_pi
from ral_eval.mission_driver import MissionDriver
from ral_interfaces.msg import FrameAlignment, MissionState
import rclpy
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

ALIGNMENT_WAIT_SIM = 60.0        # s sim for ekf_local_tf's measurement
MODE_WAIT_SIM = 10.0             # s sim for the FCU to report GUIDED
ARM_WAIT_SIM = 10.0              # s sim for the FCU to report armed
# How many times a mode or arming request is repeated before the run is
# failed. More than one because mavros answers success=false, result=0 when
# no COMMAND_ACK arrives in its own window, which is a transient, not a
# refusal; bounded because a vehicle that will never accept the command must
# fail the run rather than be asked for ever.
COMMAND_ATTEMPTS = 3
# ArduPilot stops a guided vehicle after 3 s of setpoint silence, so the
# republication period is a wall-clock one: it has to hold whatever the
# real-time factor is.
SETPOINT_PERIOD_WALL = 0.1       # s wall
WP_DIST_HITS = 3                 # consecutive in-tolerance wp_dist messages
# The supervisor. 0.25 m over 15 s sim is well below anything this plant does
# while driving - 1.0 m/s cruise covers 15 m in that window - so it fires only
# when the vehicle is genuinely not making progress.
STALL_MIN_PROGRESS_M = 0.25
STALL_WINDOW_SIM_S = 15.0
# WALL seconds for one planner_server round trip (accept, then result).
# SmacPlannerHybrid's own max_planning_time is 5.0 s sim in nav2_car.yaml;
# this is the wall bound on waiting for it, and it has to allow for the car's
# measured real-time factor of 0.34-0.52 under lockstep.
PLAN_WAIT_WALL = 30.0
# WALL seconds for the global planner to become available AFTER the autopilot
# link is up. Its lifecycle manager waits for /frame_alignment (+49-55 s wall,
# car.yaml:148-156) and then configures map_server and a costmap; MEASURED, a
# shared 60 s readiness budget expired mid-configuration and the gateway tore
# down a healthy stack.
PLANNER_READY_WALL = 120.0
# SIM seconds between replans FORCED by an unreachable carrot, so a corner the
# body cannot take from here cannot spin the planner at tick rate.
UNREACHABLE_REPLAN_MIN_SIM = 2.0
# SIM seconds of CONTINUOUS unreachability before the mission is ended. The
# planner is asked for a fresh route from the current pose first; if that keeps
# producing a path this body cannot be commanded onto, the mission has failed
# and says so rather than idling until the runner's timeout.
UNREACHABLE_GIVE_UP_SIM = 15.0


def _yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def carrot_unreachable(position, candidate, min_radius: float,
                       margin: float) -> bool:
    """
    Report whether this body is geometrically forbidden from reaching `candidate`.

    Pure pursuit drives the arc through the robot's current pose and the
    candidate: for a heading error `alpha` to a candidate `d` away, that arc
    has radius R = d / (2 * sin(alpha)). A non-holonomic body cannot drive an
    arc tighter than its own minimum turning radius, so the candidate is
    unreachable exactly when d < 2 * min_radius * margin * |sin(alpha)| -
    equivalently, when it lies INSIDE one of the two minimum-radius turning
    circles tangent to the body's nose.

    MEASURED, runs/ui_car_20261026T221042Z ((1.0, 1.0) -> (6.5, 4.0)): at
    t = 8.4 s sim the commanded setpoint was 0.63 m away at 65 deg off the
    nose, which needs R = 0.63 / (2 * sin 65) = 0.35 m against this car's
    0.611 m minimum - geometrically unreachable. Over that whole run the
    steering pure-pursuit geometry required averaged 30.2 deg (max 46.2)
    against the plant's +/-30 deg of lock, the steering actually taken
    averaged 3.1 deg (max 8.9), and the run ended mavros:stalled_no_progress
    3.04 m short at a mean 0.104 m/s against 1.0 m/s.

    `position` is (x, y, yaw) and `candidate` is (x, y, ...), both in `map`.
    A candidate straight ahead demands no turn and is always reachable.
    """
    dx = candidate[0] - position[0]
    dy = candidate[1] - position[1]
    distance = math.hypot(dx, dy)
    if distance < 1e-6:
        return False
    error = math.atan2(dy, dx) - position[2]
    error = math.atan2(math.sin(error), math.cos(error))
    sin_error = abs(math.sin(error))
    if sin_error < 1e-6:
        return False
    return distance < 2.0 * min_radius * margin * sin_error


class MavrosGuidedDriver(MissionDriver):
    """Drives one mission through ArduRover's GUIDED mode over MAVROS."""

    name = 'mavros_guided'

    def __init__(self, node, profile: dict, record: dict) -> None:
        self.node = node
        self.profile = profile
        self.record = record
        self.goal_tolerance = float(profile['goal_tolerance_m'])
        self.arrival_estimate_tol = float(profile['arrival_estimate_tolerance_m'])

        self.state = None            # latest mavros_msgs/State
        self.estimate = None         # latest (x, y, yaw) in ekf_local
        self.estimator_status = None
        self.wp_dist = None          # metres, uint16, from the FCU
        self.alignment = None        # FrameAlignment, world -> ekf_local

        self.setpoint = None         # PoseStamped in ekf_local
        self.goal_sent_sim = None
        self.mode_at_send = None
        self._last_setpoint_wall = 0.0
        self._wp_hits = 0
        self._target_acquired = False
        self._terminal = None
        self._cancel_reason = None
        # The supervisor's window: (sim time, estimate x, y) of its anchor.
        self._progress_anchor = None

        # --- the global planner, when this profile configures one ------------
        # Without `planner_action` the driver is what it always was: one
        # setpoint at the goal and the autopilot steering straight at it.
        command = profile['command']
        self.planner_action = command.get('planner_action')
        self.planner_id = command.get('planner_id', '')
        self.planner_tolerance = float(command.get('planner_tolerance_m', 0.25))
        self.waypoint_spacing = float(command.get('waypoint_spacing_m', 1.0))
        self.waypoint_lookahead = float(command.get('waypoint_lookahead_m', 2.0))
        self.waypoint_corridor = float(command.get('waypoint_corridor_m', 0.2))
        # The body's own geometry, so the carrot cannot command a point this
        # vehicle is physically forbidden from reaching. The margin exists
        # because min_turn_radius_m is the geometric ideal at FULL lock on a
        # PARALLEL-steered model (car.yaml plant.source records both
        # limitations): a target exactly at the limit leaves the controller no
        # authority at all. 1.15 gives this car an effective 0.70 m.
        self.min_turn_radius = float(profile['plant']['min_turn_radius_m'])
        self.carrot_radius_margin = float(
            command.get('waypoint_radius_margin', 1.15))
        self.replan_period_sim = float(command.get('replan_period_sim_s', 5.0))
        self._waypoints = []          # PoseStamped in the ekf_local frame
        self._wp_index = 0
        # Where on the path the robot is: the pure-pursuit search starts here
        # and only moves forward, so a path that doubles back cannot snap the
        # carrot onto a leg already driven.
        self._closest_index = 0
        self._final_goal_map = None   # (x, y, yaw) in `map`
        self._last_plan_sim = None
        self._replan_failures = 0
        # Continuous unreachability: when it began in SIM time, how many ticks
        # refused to publish, and how many replans that refusal forced.
        self._unreachable_since = None
        self._unreachable_ticks = 0
        self._forced_replans = 0
        self._last_forced_replan_sim = None
        self._plan_client = None
        self._plan_pub = None

        # QoS is stated for every subscription because a mismatch here receives
        # NOTHING while the topic lists and the node looks healthy. MAVROS
        # publishes local_position/* as BEST_EFFORT sensor data and the status
        # topics as latched RELIABLE.
        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        alignment_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)

        node.create_subscription(State, '/mavros/state', self._on_state, latched)
        node.create_subscription(Odometry, profile['estimate']['topic'],
                                 self._on_estimate, qos_profile_sensor_data)
        node.create_subscription(
            EstimatorStatus, profile['estimate']['uncertainty']['topic'],
            self._on_estimator_status, latched)
        node.create_subscription(NavControllerOutput,
                                 '/mavros/nav_controller_output/output',
                                 self._on_nav_controller, 10)
        node.create_subscription(FrameAlignment, '/frame_alignment',
                                 self._on_alignment, alignment_qos)

        self.set_mode = node.create_client(SetMode, '/mavros/set_mode')
        self.arming = node.create_client(CommandBool, '/mavros/cmd/arming')
        self.setpoint_pub = node.create_publisher(
            PoseStamped, profile['command']['topic'], 10)

        if self.planner_action:
            # Imported HERE, not at module scope: build_driver's rule is that
            # one platform's run must not require the other's message
            # packages, and nav2_msgs belongs to the Nav2 side.
            from nav2_msgs.action import ComputePathToPose
            from nav_msgs.msg import Path
            from rclpy.action import ActionClient
            self._plan_client = ActionClient(node, ComputePathToPose,
                                             self.planner_action)
            # Latched, so a console or a recorder that joins after the plan
            # was computed still receives it. This is the SAME /plan topic the
            # Roomba's Nav2 publishes, carrying the same message type.
            self._plan_pub = node.create_publisher(Path, '/plan', latched)

    # -- subscriptions --------------------------------------------------------
    def _on_state(self, msg) -> None:
        self.state = msg

    def _on_estimate(self, msg) -> None:
        self.estimate = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                         _yaw_of(msg.pose.pose.orientation))

    def _on_estimator_status(self, msg) -> None:
        self.estimator_status = msg

    def _on_nav_controller(self, msg) -> None:
        self.wp_dist = float(msg.wp_dist)

    def _on_alignment(self, msg) -> None:
        if msg.child_frame == self.profile['estimate']['frame']:
            self.alignment = msg

    # -- bring-up -------------------------------------------------------------
    def wait_ready(self, wall_limit: float) -> 'str | None':
        """
        Wait for the FCU link within `wall_limit`, then the planner within its own.

        TWO budgets, because the two have different natural times. The
        autopilot link is up as soon as MAVROS completes its handshake. The
        global planner cannot be ACTIVATED until the EKF origin has been
        measured (car_nav.launch.py's /frame_alignment gate), and configuring
        map_server plus a costmap after that takes longer still: MEASURED, a
        shared 60 s budget expired while the lifecycle manager was mid-way
        through "Configuring planner_server", and the gateway tore down a
        stack that was coming up correctly
        (logs/stack_car_20261026T153416Z.log).
        """
        deadline = time.monotonic() + wall_limit

        def link_ready():
            return (self.set_mode.service_is_ready() and self.arming.service_is_ready()
                    and self.state is not None and self.state.connected)

        while time.monotonic() < deadline and not link_ready():
            self.node.spin_wall(0.5)
        if not link_ready():
            missing = []
            if not self.set_mode.service_is_ready():
                missing.append('/mavros/set_mode')
            if not self.arming.service_is_ready():
                missing.append('/mavros/cmd/arming')
            if self.state is None:
                missing.append('/mavros/state (no message at all)')
            elif not self.state.connected:
                missing.append('/mavros/state.connected is false, so MAVROS never '
                               'completed the MAVLink handshake with the autopilot')
            return (f'the autopilot link did not come up within {wall_limit:.0f} s '
                    f'wall: {", ".join(missing)}; see launch.log for ardurover and '
                    'mavros')

        if self._plan_client is None:
            return None
        planner_deadline = time.monotonic() + PLANNER_READY_WALL
        while (time.monotonic() < planner_deadline
               and not self._plan_client.server_is_ready()):
            self.node.spin_wall(0.5)
        if self._plan_client.server_is_ready():
            return None
        return (f'{self.planner_action} never appeared within '
                f'{PLANNER_READY_WALL:.0f} s wall after the autopilot link came up; '
                'the global planner was not activated (its lifecycle manager waits '
                'for /frame_alignment, see car_nav.launch.py) and this platform will '
                'not drive a goal it cannot plan')

    # -- mode and arming ------------------------------------------------------
    def prepare(self, initial_map, record: dict) -> 'tuple[str, str] | None':
        refusal = self._wait_for_alignment(record)
        if refusal:
            return refusal

        refusal = self._enter_guided()
        if refusal:
            return refusal

        refusal = self._arm()
        if refusal:
            return refusal

        # Fields the Nav2 path fills from its warm-up. They are not silently
        # absent here: each says what this platform does instead.
        record['initial_yaw_aligned'] = False
        record['initial_yaw_error_rad'] = None
        record['initial_yaw_detail'] = (
            'the spawn yaw is the initial yaw: an Ackermann platform cannot rotate '
            'in place, so there is no warm-up rotation and no alignment step')
        record['localization_converged'] = None
        record['localization_converged_sim_s'] = None
        record['localization_detail'] = (
            'EKF3 sets its origin at the autopilot\'s first GPS fix; there is no '
            'operator-triggered initialization to time, so there is no convergence '
            'instant to measure')
        return None

    def _wait_for_alignment(self, record: dict) -> 'tuple[str, str] | None':
        deadline = self.node.sim_now() + ALIGNMENT_WAIT_SIM
        while rclpy.ok() and self.alignment is None:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.node.sim_now() > deadline:
                break
        if self.alignment is None:
            return ('runner:not_accepted',
                    f'no /frame_alignment within {ALIGNMENT_WAIT_SIM:.0f} s sim; the '
                    'EKF origin was never measured and the car\'s estimate cannot be '
                    'expressed in `map`. ekf_local_tf refuses rather than assuming '
                    'the frames coincide - see its log for which check failed')
        record['world_to_ekf_local'] = {
            'x': self.alignment.x, 'y': self.alignment.y, 'yaw': self.alignment.yaw,
            'residual_m': self.alignment.residual_m,
            'residual_yaw_rad': self.alignment.residual_yaw_rad,
            'source': self.alignment.source,
        }
        print(f'--- world->ekf_local measured ({self.alignment.x:.3f}, '
              f'{self.alignment.y:.3f}, {self.alignment.yaw:.3f}), residual '
              f'{self.alignment.residual_m:.3f} m / '
              f'{self.alignment.residual_yaw_rad:.3f} rad', flush=True)
        return None

    def _enter_guided(self) -> 'tuple[str, str] | None':
        """
        Ask for GUIDED until the FCU REPORTS it, or give up naming the reply.

        The service's own answer is not the evidence: what matters is
        /mavros/state.mode, because a setpoint sent outside GUIDED is ignored.
        """
        request = SetMode.Request()
        request.custom_mode = 'GUIDED'
        detail = self._insist(
            self.set_mode, request, '/mavros/set_mode',
            lambda: self.state is not None and self.state.mode == 'GUIDED',
            MODE_WAIT_SIM)
        if detail is None:
            return None
        mode = self.state.mode if self.state is not None else 'unknown'
        return ('mavros:not_accepted:set_mode',
                f'the FCU still reports mode {mode!r} after {COMMAND_ATTEMPTS} '
                f'attempt(s) of {MODE_WAIT_SIM:.0f} s sim each; last reply: {detail}')

    def _arm(self) -> 'tuple[str, str] | None':
        """
        Ask to arm until the FCU REPORTS armed, or give up naming the reply.

        Retried for a measured reason: a single request is answered with
        success=false and result=0 whenever no COMMAND_ACK arrives inside
        mavros's own window, and ArduPilot's pre-arm conditions can still be
        settling for a few seconds after "EKF3 IMU0 origin set". Observed: one
        run refused on the first attempt with no PreArm message at all, while
        the identical configuration armed when asked again. /mavros/state.armed
        is the evidence, never the acknowledgement.
        """
        request = CommandBool.Request()
        request.value = True
        detail = self._insist(
            self.arming, request, '/mavros/cmd/arming',
            lambda: self.state is not None and self.state.armed, ARM_WAIT_SIM)
        if detail is None:
            return None
        return ('mavros:not_accepted:arming',
                f'/mavros/state.armed is still false after {COMMAND_ATTEMPTS} '
                f'attempt(s) of {ARM_WAIT_SIM:.0f} s sim each; the autopilot refused '
                f'to arm and the vehicle cannot move. Last reply: {detail}')

    def _insist(self, client, request, name: str, achieved, wait_sim: float):
        """
        Call `client` until `achieved()` holds. None on success, else the reply.

        Bounded by COMMAND_ATTEMPTS, so a vehicle that will never accept the
        command fails the run instead of retrying for ever.
        """
        detail = 'no attempt was made'
        for _attempt in range(COMMAND_ATTEMPTS):
            _ok, detail = self._call(client, request, name)
            if self._wait_until(achieved, wait_sim):
                return None
        return detail

    def _call(self, client, request, name: str) -> 'tuple[bool, str]':
        """Call one MAVROS service and report its own answer, never a guess."""
        future = client.call_async(request)
        if not self._wait_until(future.done, MODE_WAIT_SIM):
            return False, f'{name} did not answer within {MODE_WAIT_SIM:.0f} s sim'
        response = future.result()
        accepted = getattr(response, 'mode_sent', None)
        if accepted is None:
            accepted = getattr(response, 'success', False)
        if not accepted:
            return False, f'{name} answered {response}'
        return True, f'{name} answered {response}'

    def _wait_until(self, predicate, sim_limit: float) -> bool:
        deadline = self.node.sim_now() + sim_limit
        wall_deadline = time.monotonic() + sim_limit * 10.0 + 30.0
        while rclpy.ok():
            if predicate():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.node.sim_now() > deadline or time.monotonic() > wall_deadline:
                return False
        return False

    # -- the mission ----------------------------------------------------------
    def send_goal(self, goal: PoseStamped, record: dict) -> 'tuple[bool, str | None, str]':
        """
        Start the mission: plan a path where a planner exists, then command it.

        With `planner_action` configured, the path comes from nav2's
        planner_server and is published whole on /plan; the driver then
        commands its waypoints ONE AT A TIME as GUIDED setpoints, because
        ArduRover accepts a single position target, not a path. Without it,
        the goal itself is the only setpoint - which is what this platform did
        before, and what it still does if the planner is removed.

        There is no acceptance to wait for either way:
        SET_POSITION_TARGET_LOCAL_NED is not acknowledged. The returned detail
        says so, so a reader of run.json cannot mistake "published" for "the
        autopilot agreed".
        """
        goal_yaw = _yaw_of(goal.pose.orientation)
        goal_map = (goal.pose.position.x, goal.pose.position.y, goal_yaw)
        self._final_goal_map = goal_map
        self._waypoints = []
        self._wp_index = 0
        self._closest_index = 0
        # Per-MISSION counters, cleared here rather than in __init__ alone,
        # because the dashboard gateway builds ONE driver per stack and runs
        # many missions through it (dashboard_api.py build_driver at stack
        # start). Without this, run.json's counts would be the stack's total.
        self._replan_failures = 0
        self._unreachable_since = None
        self._unreachable_ticks = 0
        self._forced_replans = 0
        self._last_forced_replan_sim = None
        for key in ('replan_count', 'replan_failures', 'replan_last_failure',
                    'carrot_unreachable_ticks', 'carrot_unreachable_last',
                    'carrot_forced_replans'):
            self.record.pop(key, None)
            record.pop(key, None)
        plan_detail = ''

        if self._plan_client is not None:
            poses, refusal = self._plan_path(goal_map)
            if refusal:
                return False, 'mavros:no_path', refusal
            self._waypoints = poses
            record['path_len_m'] = self._path_length(poses)
            record['waypoint_count'] = len(poses)
            plan_detail = (f'; following a {len(poses)}-waypoint path of '
                           f'{record["path_len_m"]:.2f} m from '
                           f'{self.planner_action} ({self.planner_id}), published '
                           'on /plan')

        if self._waypoints:
            # NOT waypoints[0]: the planner's first pose is the robot's own
            # current cell, and commanding it is an instruction to stop where
            # it already is. MEASURED (runs/ui_car_20261026T154045Z), the
            # setpoint oscillated back onto the start after every replan and
            # the vehicle crawled at 0.12 m/s. -1 forces _update_carrot to
            # publish rather than skip an unchanged index.
            self._wp_index = -1
            self._update_carrot()
            index = self._wp_index if self._wp_index >= 0 else len(self._waypoints) - 1
            local = self._commanded_local(self._waypoints[index])
        else:
            world = map_pose_to_world(goal_map[0], goal_map[1], goal_map[2],
                                      self.node.wtm)
            local = self._world_to_ekf_local(world)
        self._set_setpoint(local)

        self.goal_sent_sim = self.node.sim_now()
        self._last_plan_sim = self.node.sim_now()
        self.mode_at_send = self.state.mode if self.state is not None else None
        # No anchor yet: the stall window opens at the first pose this robot
        # actually REPORTS. Seeding it with a placeholder would let the stall
        # detail quote a distance measured from a pose never observed.
        self._progress_anchor = None
        final_world = map_pose_to_world(goal_map[0], goal_map[1], goal_map[2],
                                        self.node.wtm)
        final_local = self._world_to_ekf_local(final_world)
        record['goal_ekf_local'] = [final_local[0], final_local[1], final_local[2]]
        return True, None, (f'GUIDED position setpoint published at '
                            f'({local[0]:.3f}, {local[1]:.3f}) in '
                            f'{self.profile["estimate"]["frame"]}; '
                            'SET_POSITION_TARGET_LOCAL_NED is not acknowledged, so '
                            'acceptance here means published while the mode was '
                            f'GUIDED and the vehicle armed{plan_detail}')

    def _set_setpoint(self, local) -> None:
        """Command one pose, expressed in the autopilot's own frame."""
        setpoint = PoseStamped()
        setpoint.header.frame_id = self.profile['estimate']['frame']
        setpoint.pose.position.x = local[0]
        setpoint.pose.position.y = local[1]
        setpoint.pose.orientation.z = math.sin(local[2] / 2.0)
        setpoint.pose.orientation.w = math.cos(local[2] / 2.0)
        self.setpoint = setpoint
        self._publish_setpoint()

    def _commanded_local(self, pose_map):
        """One planned `map` pose, expressed in the autopilot's frame."""
        world = map_pose_to_world(pose_map[0], pose_map[1], pose_map[2],
                                  self.node.wtm)
        return self._world_to_ekf_local(world)

    @staticmethod
    def _path_length(poses) -> float:
        total = 0.0
        for first, second in zip(poses, poses[1:]):
            total += math.hypot(second[0] - first[0], second[1] - first[1])
        return total

    def _plan_path(self, goal_map) -> 'tuple[list, str | None]':
        """
        Ask planner_server for a path from the ROBOT'S OWN estimate to the goal.

        Returns (waypoints in `map`, refusal). The start is the estimate, not
        truth: planning from a pose the robot does not believe it is at would
        make the path unfollowable for the only reason that matters here.
        """
        from nav2_msgs.action import ComputePathToPose
        if self.estimate is None:
            return [], ('no estimate has arrived on '
                        f'{self.profile["estimate"]["topic"]}, so there is no start '
                        'pose to plan from')
        if self.alignment is None:
            return [], ('/frame_alignment has not been measured, so an ekf_local '
                        'estimate cannot be expressed in `map` to plan from')
        start_map = self._ekf_local_to_map(self.estimate)
        request = ComputePathToPose.Goal()
        request.use_start = True
        request.start = _pose_in_map(start_map, self.node)
        request.goal = _pose_in_map(goal_map, self.node)
        request.planner_id = self.planner_id
        send_future = self._plan_client.send_goal_async(request)
        if not self._spin_future(send_future, PLAN_WAIT_WALL):
            return [], (f'{self.planner_action} did not accept a goal within '
                        f'{PLAN_WAIT_WALL:.0f} s wall')
        handle = send_future.result()
        if not handle.accepted:
            return [], f'{self.planner_action} REJECTED the planning request'
        result_future = handle.get_result_async()
        if not self._spin_future(result_future, PLAN_WAIT_WALL):
            return [], (f'{self.planner_action} accepted the request but returned no '
                        f'result within {PLAN_WAIT_WALL:.0f} s wall')
        path = result_future.result().result.path
        poses = [(pose.pose.position.x, pose.pose.position.y,
                  _yaw_of(pose.pose.orientation)) for pose in path.poses]
        if len(poses) < 2:
            return [], (f'{self.planner_action} returned {len(poses)} pose(s) from '
                        f'map ({start_map[0]:.2f}, {start_map[1]:.2f}) to '
                        f'({goal_map[0]:.2f}, {goal_map[1]:.2f}): there is no '
                        'feasible path for this body and turning radius')
        resampled = self._resample(poses)
        path.poses = path.poses[:1]              # reuse the header the server set
        self._publish_plan(path, resampled)
        return resampled, None

    def _publish_plan(self, path_msg, poses) -> None:
        """Publish the followed path on /plan, as nav2's own planner would."""
        from geometry_msgs.msg import PoseStamped as _PoseStamped
        stamped = []
        for pose in poses:
            item = _PoseStamped()
            item.header.frame_id = 'map'
            item.header.stamp = self.node.get_clock().now().to_msg()
            item.pose.position.x = pose[0]
            item.pose.position.y = pose[1]
            item.pose.orientation.z = math.sin(pose[2] / 2.0)
            item.pose.orientation.w = math.cos(pose[2] / 2.0)
            stamped.append(item)
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.node.get_clock().now().to_msg()
        path_msg.poses = stamped
        self._plan_pub.publish(path_msg)

    def _resample(self, poses) -> list:
        """
        Thin a dense planner path to waypoint_spacing_m, keeping the final pose.

        SmacPlannerHybrid returns a pose per search step - hundreds for a
        12 m path - and commanding each one as a separate GUIDED setpoint
        would make the autopilot's own L1 controller irrelevant. The last pose
        is kept EXACTLY, because it is the goal the mission is scored against.
        """
        kept = [poses[0]]
        for pose in poses[1:-1]:
            if math.hypot(pose[0] - kept[-1][0], pose[1] - kept[-1][1]) \
                    >= self.waypoint_spacing:
                kept.append(pose)
        kept.append(poses[-1])
        return kept

    def _spin_future(self, future, wall_limit: float) -> bool:
        """Spin THIS node until a future completes; never a second spinner."""
        deadline = time.monotonic() + wall_limit
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return future.done()

    def _ekf_local_to_map(self, local) -> 'tuple[float, float, float]':
        """Invert _world_to_ekf_local, then convert world -> map."""
        cos_a = math.cos(self.alignment.yaw)
        sin_a = math.sin(self.alignment.yaw)
        world_x = self.alignment.x + local[0] * cos_a - local[1] * sin_a
        world_y = self.alignment.y + local[0] * sin_a + local[1] * cos_a
        world_yaw = wrap_to_pi(local[2] + self.alignment.yaw)
        return world_pose_to_map(world_x, world_y, world_yaw, self.node.wtm)

    def _world_to_ekf_local(self, world) -> 'tuple[float, float, float]':
        """Express a `world` pose in the autopilot's frame, using the measurement."""
        dx = world[0] - self.alignment.x
        dy = world[1] - self.alignment.y
        cos_a = math.cos(self.alignment.yaw)
        sin_a = math.sin(self.alignment.yaw)
        return (dx * cos_a + dy * sin_a,
                -dx * sin_a + dy * cos_a,
                wrap_to_pi(world[2] - self.alignment.yaw))

    def _publish_setpoint(self) -> None:
        self.setpoint.header.stamp = self.node.get_clock().now().to_msg()
        self.setpoint_pub.publish(self.setpoint)
        self._last_setpoint_wall = time.monotonic()

    def spin_once(self) -> None:
        if self.setpoint is not None and \
                time.monotonic() - self._last_setpoint_wall >= SETPOINT_PERIOD_WALL:
            self._publish_setpoint()
        if self._terminal is None and self.goal_sent_sim is not None:
            self._update_carrot()
            self._replan_if_due()
            self._terminal = self._check_terminal()

    def _update_carrot(self) -> None:
        """
        Command a carrot a LOOKAHEAD ahead ALONG THE PATH, pure-pursuit style.

        Two measurements shaped this, and both are in runs/:

        * Every GUIDED position target is a DESTINATION to ArduRover - it
          decelerates to stop at whatever point it is given. With a 0.8 m
          "advance once close" rule (baseline_car_20261026T143722Z) the
          vehicle crept at a mean 0.205 m/s against 1.0 m/s configured,
          because it was always braking for the next waypoint.
        * Choosing the carrot by STRAIGHT-LINE distance picks a waypoint on
          the far side of a wall as soon as the path turns: the car drove at
          it and pinned itself on wall A at world (3.70, 2.32)
          (ui_car_20261026T145419Z, 10-waypoint path around the doorway,
          collision_count 1). The carrot therefore advances by ARC LENGTH
          from the closest point on the path, which keeps it on the route the
          planner actually produced.
        * A carrot the BODY CANNOT TURN ONTO is a command this vehicle is
          geometrically forbidden from obeying, and the autopilot answers it
          by creeping. MEASURED (ui_car_20261026T221042Z): the heading error
          to the commanded setpoint stayed 48-69 deg for a whole run, the
          steering pure pursuit required averaged 30.2 deg (max 46.2) against
          +/-30 deg of lock, the steering taken averaged 3.1 deg, and the run
          ended mavros:stalled_no_progress 3.04 m short at 0.104 m/s. The
          walk below therefore keeps advancing past an unreachable candidate
          (see carrot_unreachable), and if none is admissible it publishes
          NOTHING and asks the planner for a fresh route instead.

        The lookahead trades speed against corner cutting: the chord between
        the robot and a carrot L ahead on a curve of radius R cuts the corner
        by about L^2 / (8R), which for L = 1.2 m and this car's 0.611 m
        minimum radius is 0.29 m - inside the 0.45 m inflation the planner
        already keeps from walls. The approach speed a 1.2 m target allows is
        sqrt(2 * ATC_DECEL_MAX * 1.2) = 1.55 m/s, still above the 1.0 m/s
        cruise, so the carrot does not limit speed.

        The FINAL waypoint is commanded as itself once it is the only one
        left: that one IS a destination, and stopping at it is arrival.

        The per-leg guards are reset with the carrot: `_target_acquired`
        exists so a wp_dist of 0 before the FCU has a target cannot end the
        mission, and `_progress_anchor` measures stall distance, which must
        not carry a previous leg's measurement into this one.
        """
        if not self._waypoints or self.estimate is None or self.alignment is None:
            return
        position = self._ekf_local_to_map(self.estimate)
        # 1. The closest waypoint, searched FORWARD only: a path that doubles
        #    back past the robot must not snap the carrot to the leg already
        #    driven.
        closest = self._closest_index
        best = None
        for index in range(self._closest_index, len(self._waypoints)):
            waypoint = self._waypoints[index]
            distance = math.hypot(waypoint[0] - position[0],
                                  waypoint[1] - position[1])
            if best is None or distance < best:
                best = distance
                closest = index
        self._closest_index = closest
        # 2. Walk forward while the candidate is EITHER under the lookahead
        #    arc length from the closest point OR unreachable from the current
        #    pose, AND the straight line the autopilot would drive to it stays
        #    inside waypoint_corridor_m of the planned path.
        #
        #    PRECEDENCE: reachability may overrun the LOOKAHEAD, never the
        #    CORRIDOR. Overrunning the lookahead only lengthens a leg;
        #    overrunning the corridor commands a straight line through the
        #    geometry the planner routed around, which is the measured wall
        #    collision below.
        #
        #    The corridor is what stops the carrot jumping across a corner:
        #    MEASURED (runs/ui_car_20261026T153031Z), the car tracked the path
        #    at 0.85 m/s until the carrot landed beyond the doorway corner, at
        #    which point the chord to it crossed wall A and the vehicle drove
        #    into the wall at world (3.70, 2.1).
        #
        # The walk starts one waypoint AHEAD of the closest and the corridor
        # is measured over the points strictly between them, because the
        # closest waypoint is usually BEHIND the robot: including it made its
        # own offset break the walk, the carrot fell back onto a point already
        # driven past, and the vehicle was commanded to reverse into its own
        # track at 0.10 m/s (MEASURED, runs/ui_car_20261026T154449Z).
        index = min(closest + 1, len(self._waypoints) - 1)
        travelled = 0.0
        while index < len(self._waypoints) - 1:
            reachable = not carrot_unreachable(
                position, self._waypoints[index], self.min_turn_radius,
                self.carrot_radius_margin)
            if travelled >= self.waypoint_lookahead and reachable:
                break
            following = self._waypoints[index + 1]
            if self._chord_departs(position, following, closest + 1, index + 1):
                break
            travelled += math.hypot(following[0] - self._waypoints[index][0],
                                    following[1] - self._waypoints[index][1])
            index += 1
        # 3. If even the furthest admissible candidate lies inside a
        #    minimum-radius turning circle, publish NOTHING. The previously
        #    commanded setpoint stands - the autopilot keeps driving the last
        #    target it was given - and a fresh plan from the current pose is
        #    requested, because the planner's own first waypoints are
        #    reachable by construction. Commanding a point the body cannot
        #    turn onto is what produced the measured 30.2 deg mean steering
        #    demand against 30 deg of lock (see carrot_unreachable).
        candidate = self._waypoints[index]
        if carrot_unreachable(position, candidate, self.min_turn_radius,
                              self.carrot_radius_margin):
            self._unreachable_ticks += 1
            if self._unreachable_since is None:
                self._unreachable_since = self.node.sim_now()
            self.record['carrot_unreachable_ticks'] = self._unreachable_ticks
            self.record['carrot_unreachable_last'] = self._unreachable_detail(
                position, candidate)
            self._force_replan()
            return
        self._unreachable_since = None
        if index == self._wp_index:
            return
        self._wp_index = index
        self._set_setpoint(self._commanded_local(self._waypoints[index]))
        self._wp_hits = 0
        self._target_acquired = False
        self._progress_anchor = None

    def _unreachable_detail(self, position, candidate) -> str:
        """Name the refused candidate in the terms that make it refusable."""
        dx = candidate[0] - position[0]
        dy = candidate[1] - position[1]
        distance = math.hypot(dx, dy)
        error = math.atan2(dy, dx) - position[2]
        error = math.atan2(math.sin(error), math.cos(error))
        sin_error = abs(math.sin(error))
        required = distance / (2.0 * sin_error) if sin_error > 1e-6 else float('inf')
        return (f'{distance:.2f} m away at {math.degrees(error):+.1f} deg, '
                f'needing a {required:.2f} m turn radius against this body\'s '
                f'{self.min_turn_radius:.3f} m minimum '
                f'(margin {self.carrot_radius_margin:.2f})')

    def _force_replan(self) -> None:
        """
        Ask for a fresh route because the current one cannot be commanded.

        Rate-limited to one forced replan per UNREACHABLE_REPLAN_MIN_SIM, so a
        corner this body cannot take from here cannot spin planner_server at
        the driver's tick rate. The replan itself is left to _replan_if_due,
        which already owns the failure counting and the carrot reset: this
        only brings its due time forward.
        """
        if self._plan_client is None:
            return
        now = self.node.sim_now()
        if self._last_forced_replan_sim is not None and \
                now - self._last_forced_replan_sim < UNREACHABLE_REPLAN_MIN_SIM:
            return
        self._last_forced_replan_sim = now
        self._forced_replans += 1
        self.record['carrot_forced_replans'] = self._forced_replans
        self._last_plan_sim = now - self.replan_period_sim

    def _chord_departs(self, start, candidate, first: int, last: int) -> bool:
        """
        Report whether the line start -> candidate leaves the planned path.

        The autopilot drives a STRAIGHT LINE to whatever position it is given,
        so a carrot beyond a corner is a command to cut that corner. This
        measures how far the intermediate path points lie from that line and
        refuses the candidate when any of them is further than
        waypoint_corridor_m: the planner routed around geometry, and the
        chord has to stay inside the corridor the planner cleared.
        """
        dx = candidate[0] - start[0]
        dy = candidate[1] - start[1]
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return False
        for index in range(first, last):
            point = self._waypoints[index]
            # Perpendicular distance from the path point to the chord.
            offset = abs(dy * (point[0] - start[0]) - dx * (point[1] - start[1])) / length
            if offset > self.waypoint_corridor:
                return True
        return False

    def _replan_if_due(self) -> None:
        """
        Re-plan every replan_period_sim_s, so a NEW obstacle changes the route.

        This is what makes a spawn_obstacle fault reach the car's navigation:
        the obstacle enters the global costmap through the obstacle_layer on
        /scan, and the next plan routes around it. Between replans the
        autopilot steers straight at its current setpoint and has no reactive
        avoidance of its own. A failed replan KEEPS the current queue and is
        counted: a path that could not be recomputed is not a reason to stop
        following the one being executed.
        """
        if self._plan_client is None or self._final_goal_map is None:
            return
        if self._last_plan_sim is None:
            return
        if self.node.sim_now() - self._last_plan_sim < self.replan_period_sim:
            return
        self._last_plan_sim = self.node.sim_now()
        poses, refusal = self._plan_path(self._final_goal_map)
        if refusal:
            self._replan_failures += 1
            self.record['replan_failures'] = self._replan_failures
            self.record['replan_last_failure'] = refusal
            return
        self._waypoints = poses
        self._closest_index = 0
        self.record['replan_count'] = self.record.get('replan_count', 0) + 1
        # A carrot, NOT poses[0]: the new plan starts at the robot's own cell
        # (see send_goal). -1 forces the publish.
        self._wp_index = -1
        self._update_carrot()
        self._wp_hits = 0
        self._target_acquired = False
        self._progress_anchor = None

    # -- what ended the mission -----------------------------------------------
    def _estimate_gap(self) -> 'float | None':
        """
        |estimate - commanded setpoint| in ekf_local, or None before either exists.

        Both terms are the robot's own: the estimate it publishes and the
        setpoint this run sent it. Truth is not read here, and this number is
        what makes `arrived` mean a distance rather than a truncated metre.
        """
        if self.estimate is None or self.setpoint is None:
            return None
        return math.hypot(self.estimate[0] - self.setpoint.pose.position.x,
                          self.estimate[1] - self.setpoint.pose.position.y)

    def _check_terminal(self) -> 'tuple[str, str, str] | None':
        state = self.state
        armed = state.armed if state is not None else False
        mode = state.mode if state is not None else None

        # The FCU must be seen NAVIGATING before "arrived" can be believed.
        # NAV_CONTROLLER_OUTPUT.wp_dist is 0 whenever the autopilot has no
        # active target, and it publishes at rest too. MEASURED failure
        # without this guard: the first three messages after arming carried
        # wp_dist 0 and the mission was declared `mavros:reached_wp_radius`
        # at 0.0 s sim, with the vehicle still standing on its spawn pose.
        # So the test waits for the autopilot's own distance to exceed the
        # tolerance at least once - which is the FCU saying "I have a target
        # and it is over there" - before any number of small readings can end
        # the mission.
        if (self.wp_dist is not None and mode == 'GUIDED' and armed
                and self.wp_dist > self.goal_tolerance):
            if not self._target_acquired:
                # That same statement is the only EXECUTION signal this
                # platform produces: the autopilot saying it has a target and
                # is driving to it. There is no feedback stream and nothing
                # acknowledges a setpoint, so `navigating_since` is set here,
                # when the FCU is OBSERVED to be executing, and never at the
                # moment the setpoint was published.
                self.navigating_since = self.node.sim_now()
            self._target_acquired = True

        gap = self._estimate_gap()
        # ARRIVAL IS ONLY ARRIVAL AT THE GOAL. With a planned path the
        # commanded setpoint is usually a CARROT, and the same conjunction
        # closes on every one of them: MEASURED
        # (runs/ui_car_20261026T153805Z), the mission was declared
        # `mavros:reached_wp_radius` after 0.98 m of travel, 5.31 m from the
        # goal, because the carrot had been reached.
        at_final = (not self._waypoints
                    or self._wp_index >= len(self._waypoints) - 1)
        if (at_final and self._target_acquired and self.wp_dist is not None
                and mode == 'GUIDED' and armed
                and self.wp_dist <= self.goal_tolerance
                and gap is not None and gap <= self.arrival_estimate_tol):
            self._wp_hits += 1
            if self._wp_hits >= WP_DIST_HITS:
                return (MissionState.STATE_SUCCEEDED, 'mavros:reached_wp_radius',
                        f'/mavros/nav_controller_output/output reported wp_dist '
                        f'{self.wp_dist:.0f} m within WP_RADIUS '
                        f'{self.goal_tolerance:.0f} m on {WP_DIST_HITS} consecutive '
                        'messages while the mode was GUIDED and the vehicle armed, '
                        'having first reported a distance beyond that tolerance, AND '
                        f'the robot\'s own estimate was {gap:.2f} m from the '
                        'commanded setpoint, within the profile\'s '
                        f'arrival_estimate_tolerance_m {self.arrival_estimate_tol:.2f}. '
                        'wp_dist is the FCU\'s own distance to its target and is '
                        'uint16 METRES, so the estimate gap is what bounds where '
                        'this vehicle came to rest.')
        elif self.wp_dist is not None:
            self._wp_hits = 0

        # Any mode but GUIDED after a cancel is the cancel landing: HOLD when
        # the autopilot took the command, something else when it did not. The
        # mode reported is named rather than asserted, because a cancel that
        # lands in HOLD and a cancel that lands anywhere else are both the end
        # of the mission and neither may fall through to "no terminal status".
        if self._cancel_reason is not None and mode is not None and mode != 'GUIDED':
            return (MissionState.STATE_CANCELED, 'mavros:canceled_timeout',
                    f'the runner cancelled ({self._cancel_reason}); HOLD was '
                    f'commanded and the FCU now reports mode {mode!r} on '
                    f'/mavros/state. Last wp_dist {self.wp_dist}')

        if mode is not None and mode != 'GUIDED' and self._cancel_reason is None:
            return (MissionState.STATE_ABORTED, f'mavros:mode_changed:{mode}',
                    f'/mavros/state reports mode {mode!r}, having been '
                    f'{self.mode_at_send!r} when the goal was sent; the autopilot '
                    'left GUIDED by itself, so the mission ended for a reason the '
                    f'FCU decided. Last wp_dist {self.wp_dist}')

        if state is not None and not armed:
            return (MissionState.STATE_ABORTED, 'mavros:disarmed',
                    '/mavros/state reports armed=false mid-mission; a disarmed rover '
                    f'cannot move. Last wp_dist {self.wp_dist}, mode {mode!r}')

        # Continuous unreachability, after the forced replans have had their
        # chance. The plan was valid and the planner kept producing one; what
        # failed is that no point on it lay outside this body's minimum
        # turning circles, so no setpoint could honestly be published. Ending
        # here rather than idling to the runner's timeout names the cause.
        if self._unreachable_since is not None and \
                self.node.sim_now() - self._unreachable_since >= UNREACHABLE_GIVE_UP_SIM:
            self._command_mode('HOLD')
            return (MissionState.STATE_ABORTED, 'mavros:carrot_unreachable',
                    f'for {self.node.sim_now() - self._unreachable_since:.1f} s '
                    'sim every candidate point on the planned path lay inside '
                    'this body\'s minimum turning circle, so this driver '
                    'published no setpoint for any of them: the furthest '
                    'admissible candidate was '
                    f'{self.record.get("carrot_unreachable_last", "unrecorded")}. '
                    f'{self._forced_replans} fresh plans were requested from the '
                    'current pose first and none produced a commandable point. '
                    'This is a DETERMINISTIC POLICY in the runner, not a '
                    'decision the autopilot made. HOLD was commanded. Last '
                    f'wp_dist {self.wp_dist}')

        return self._check_stall()

    def _check_stall(self) -> 'tuple[str, str, str] | None':
        """
        Stop the vehicle when it makes no progress, by a deterministic policy.

        This is NOT autonomy and is labelled so everywhere it appears: it is a
        policy programmed into the runner. ArduRover has no obstacle
        avoidance: against a wall it pushes until the scenario's timeout.
        This turns that into a safe stop with a named
        cause. It closes on the robot's OWN estimate - truth is never read
        here - so a run in which the estimate is wrong stalls for the reason
        the robot itself could observe. The window opens at the first pose the
        robot actually REPORTS after the goal, not at the goal instant: before
        that there is no observed position to measure progress from.
        """
        if self.estimate is None:
            return None
        if self._progress_anchor is None:
            self._progress_anchor = (self.node.sim_now(),
                                     self.estimate[0], self.estimate[1])
            return None
        now = self.node.sim_now()
        anchor_t, anchor_x, anchor_y = self._progress_anchor
        moved = math.hypot(self.estimate[0] - anchor_x, self.estimate[1] - anchor_y)
        if moved > STALL_MIN_PROGRESS_M:
            self._progress_anchor = (now, self.estimate[0], self.estimate[1])
            return None
        if now - anchor_t < STALL_WINDOW_SIM_S:
            return None
        self._command_mode('HOLD')
        return (MissionState.STATE_ABORTED, 'mavros:stalled_no_progress',
                f'the robot\'s own estimate moved {moved:.3f} m in '
                f'{now - anchor_t:.1f} s sim, less than the '
                f'{STALL_MIN_PROGRESS_M} m this driver requires over '
                f'{STALL_WINDOW_SIM_S:.0f} s. This is a DETERMINISTIC POLICY in the '
                'runner, not a decision the autopilot made: ArduRover carries no '
                'obstacle avoidance and would keep pushing until the scenario '
                f'timeout. HOLD was commanded. Last wp_dist {self.wp_dist}')

    def terminal(self) -> 'tuple[str, str, str] | None':
        return self._terminal

    def no_terminal_detail(self) -> str:
        mode = self.state.mode if self.state is not None else None
        armed = self.state.armed if self.state is not None else None
        return (f'last /mavros/state mode={mode!r} armed={armed}, last wp_dist '
                f'{self.wp_dist}, last estimate gap {self._estimate_gap()}')

    # -- stopping -------------------------------------------------------------
    def request_cancel(self, reason: str) -> None:
        self._cancel_reason = reason
        self._command_mode('HOLD')

    def _command_mode(self, mode: str) -> None:
        request = SetMode.Request()
        request.custom_mode = mode
        ok, detail = self._call(self.set_mode, request, '/mavros/set_mode')
        print(f'--- set_mode {mode}: {"ok" if ok else "FAILED"} ({detail})', flush=True)

    def stop(self) -> None:
        """HOLD, then disarm. Best effort, each failure logged with its reply."""
        if self.state is not None and self.state.mode != 'HOLD':
            self._command_mode('HOLD')
        request = CommandBool.Request()
        request.value = False
        ok, detail = self._call(self.arming, request, '/mavros/cmd/arming')
        print(f'--- disarm: {"ok" if ok else "FAILED"} ({detail})', flush=True)

    def teardown(self) -> None:
        """
        Destroy the planner action client, which holds a node reference.

        The dashboard gateway is a LONG-LIVED process that builds a driver per
        stack, so a leaked client leaves a ghost node in the graph for the
        rest of the session.
        """
        if self._plan_client is not None:
            self._plan_client.destroy()
            self._plan_client = None


def _pose_in_map(pose, node) -> PoseStamped:
    """Build a `map`-frame PoseStamped for the planner's own request fields."""
    stamped = PoseStamped()
    stamped.header.frame_id = 'map'
    stamped.header.stamp = node.get_clock().now().to_msg()
    stamped.pose.position.x = float(pose[0])
    stamped.pose.position.y = float(pose[1])
    stamped.pose.orientation.z = math.sin(float(pose[2]) / 2.0)
    stamped.pose.orientation.w = math.cos(float(pose[2]) / 2.0)
    return stamped
