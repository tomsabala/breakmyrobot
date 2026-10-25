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
from ral_eval.frames import map_pose_to_world, wrap_to_pi
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


def _yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


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
        """Both services present AND the FCU link up, or a named refusal."""
        deadline = time.monotonic() + wall_limit
        while time.monotonic() < deadline:
            if (self.set_mode.service_is_ready() and self.arming.service_is_ready()
                    and self.state is not None and self.state.connected):
                return None
            self.node.spin_wall(0.5)
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
        return (f'the autopilot link did not come up within {wall_limit:.0f} s wall: '
                f'{", ".join(missing)}; see launch.log for ardurover and mavros')

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
        Publish the first GUIDED setpoint, converted into the autopilot's frame.

        There is no acceptance to wait for: SET_POSITION_TARGET_LOCAL_NED is
        not acknowledged. The returned detail says so, so a reader of run.json
        cannot mistake "published" for "the autopilot agreed".
        """
        goal_yaw = _yaw_of(goal.pose.orientation)
        world = map_pose_to_world(goal.pose.position.x, goal.pose.position.y,
                                  goal_yaw, self.node.wtm)
        local = self._world_to_ekf_local(world)

        setpoint = PoseStamped()
        setpoint.header.frame_id = self.profile['estimate']['frame']
        setpoint.pose.position.x = local[0]
        setpoint.pose.position.y = local[1]
        setpoint.pose.orientation.z = math.sin(local[2] / 2.0)
        setpoint.pose.orientation.w = math.cos(local[2] / 2.0)
        self.setpoint = setpoint
        self._publish_setpoint()

        self.goal_sent_sim = self.node.sim_now()
        self.mode_at_send = self.state.mode if self.state is not None else None
        # No anchor yet: the stall window opens at the first pose this robot
        # actually REPORTS. Seeding it with a placeholder would let the stall
        # detail quote a distance measured from a pose never observed.
        self._progress_anchor = None
        record['goal_ekf_local'] = [local[0], local[1], local[2]]
        return True, None, (f'GUIDED position setpoint published at '
                            f'({local[0]:.3f}, {local[1]:.3f}) in '
                            f'{setpoint.header.frame_id}; '
                            'SET_POSITION_TARGET_LOCAL_NED is not acknowledged, so '
                            'acceptance here means published while the mode was '
                            'GUIDED and the vehicle armed')

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
            self._terminal = self._check_terminal()

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
        if (self._target_acquired and self.wp_dist is not None
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
