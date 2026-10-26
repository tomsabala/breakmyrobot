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
The Nav2 mission driver: AMCL warm-up, a NavigateToPose goal, a GoalStatus.

This is the path the Roomba has always taken, moved out of run_scenario.py
unchanged in behaviour - the same three /initialpose publications, the same
covariance test, the same warm-up rotation, the same yaw alignment and the
same terminal GoalStatus mapping. The constants kept their values and their
comments, because each of them was measured.

The one deliberate change is the outcome vocabulary: the terminal strings are
prefixed `nav2:`, so a reader of /mission_state can tell which autonomy stack
produced them. See ral_interfaces/msg/MissionState.msg.

Everything here closes on the robot's OWN estimate (/amcl_pose) and never on
/ground_truth/odom. Steering the body with truth would make the start pose
exact in a way no real robot's is, and would be the localization shortcut
ENGINEERING.md forbids.
"""

import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from ral_eval.frames import wrap_to_pi, yaw_of_quaternion
from ral_eval.mission_driver import MissionDriver
from ral_interfaces.msg import MissionState
import rclpy
from rclpy.action import ActionClient

AMCL_CONVERGE_SIM = 30.0         # s sim allowed for the filter to converge
AMCL_WARMUP_RATE = 0.6           # rad/s in-place rotation; AMCL updates on motion only
# AMCL publishes on filter UPDATE, and its filter updates on motion. The
# instant it converges the warm-up would otherwise stop, and the robot then
# stands still publishing nothing - measured: exactly ONE converged /amcl_pose
# before the goal, so est_error_at_start_pose_m was an average over a single
# sample. Keeping the in-place rotation going for a few more sim seconds costs
# nothing (xy is unchanged, so the samples are still taken at the start pose)
# and makes the residual a mean rather than one reading.
AMCL_FLOOR_SAMPLE_SIM = 4.0      # s sim of extra in-place rotation after convergence
AMCL_SETTLE_SIM = 2.0            # s sim stopped before reading the residual
YAW_ALIGN_SIM = 20.0             # s sim allowed to turn back to the scenario's yaw
# > amcl update_min_a (0.2 rad, nav2_roomba.yaml:79). AMCL publishes on filter
# update and its filter updates on motion, so a tolerance below that quantum
# asks the loop to close on a signal finer than the estimate it reads.
YAW_ALIGN_TOLERANCE = 0.25       # rad (~14 deg), the heading the mission starts from
YAW_ALIGN_MIN_RATE = 0.15        # rad/s floor, so the last few degrees do not stall
GOAL_ACCEPT_WAIT_SIM = 30.0      # s sim for the action server to accept or refuse

# Terminal action status -> mission state. Only these three are terminal for a
# goal this driver sent; anything else is reported as no terminal status
# rather than mapped to a guess.
TERMINAL_STATES = {
    GoalStatus.STATUS_SUCCEEDED: (MissionState.STATE_SUCCEEDED,
                                  'nav2:STATUS_SUCCEEDED'),
    GoalStatus.STATUS_CANCELED: (MissionState.STATE_CANCELED,
                                 'nav2:STATUS_CANCELED'),
    GoalStatus.STATUS_ABORTED: (MissionState.STATE_ABORTED, 'nav2:STATUS_ABORTED'),
}


class Nav2Driver(MissionDriver):
    """Drives one mission through nav2's NavigateToPose action."""

    name = 'nav2'

    def __init__(self, node, profile: dict, record: dict) -> None:
        self.node = node
        self.profile = profile
        self.record = record
        self.converged_below = float(
            profile['estimate']['uncertainty']['converged_below'])
        self.client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.handle = None
        self.result_future = None
        self.feedback = {'count': 0, 'distance_remaining': None,
                         'number_of_recoveries': None}
        self.canceled = False

    # -- bring-up -------------------------------------------------------------
    def wait_ready(self, wall_limit: float) -> 'str | None':
        deadline = time.monotonic() + wall_limit
        while time.monotonic() < deadline and not self.client.server_is_ready():
            self.node.spin_wall(0.5)
        if self.client.server_is_ready():
            return None
        return (f'navigate_to_pose never appeared within {wall_limit:.0f} s wall; '
                'Nav2 did not activate (see launch.log)')

    # -- localization ---------------------------------------------------------
    def prepare(self, initial_map, record: dict) -> 'tuple[str, str] | None':
        """
        Localize: /initialpose, a warm-up rotation, then back to the start yaw.

        AMCL updates on MOTION (update_min_d 0.1, update_min_a 0.2), so a
        robot standing still never resamples and its covariance stays at
        whatever /initialpose declared - measured: trace 0.4474 after 30 s
        sim. The in-place rotation gives the filter angular motion and returns
        the body to the same xy.

        Non-convergence is DATA, not a refusal: a run that navigates badly
        because the filter never converged is a result, and raising the
        covariance limit to make it look clean would not be.
        """
        node = self.node
        node.amcl = None
        initialpose_sim = node.sim_now()
        for _ in range(3):
            node.publish_initial_pose(*initial_map)
            node.spin_wall(0.5)

        def converged():
            if node.amcl is None:
                return False
            cov = node.amcl.pose.covariance
            return cov[0] + cov[7] + cov[35] < self.converged_below

        did_converge = node.spin_sim(AMCL_CONVERGE_SIM,
                                     cmd=_twist(angular_z=AMCL_WARMUP_RATE),
                                     until=converged)
        if did_converge:
            # Keep rotating so the converged filter publishes more than the one
            # sample est_error_at_start_pose_m would otherwise be averaged over.
            node.spin_sim(AMCL_FLOOR_SAMPLE_SIM,
                          cmd=_twist(angular_z=AMCL_WARMUP_RATE))
        node.stop()
        node.spin_sim(AMCL_SETTLE_SIM)

        # The warm-up rotation ends wherever convergence happened to land, so
        # without this the body faces an ARBITRARY direction when the goal is
        # sent - while the scenario declares an initial yaw. Measured: a run
        # that ended facing away from the goal had DWB reverse at 0.1 m/s for
        # 40 s, taking 76 s sim against 37 s for the same path driven forward.
        # That is real Nav2 behaviour, but it is a response to a heading no
        # scenario asked for, and it makes two runs of one scenario
        # incomparable.
        aligned, yaw_error = self._align_to_initial_yaw(initial_map[2])
        record['initial_yaw_aligned'] = aligned
        record['initial_yaw_error_rad'] = yaw_error
        record['initial_yaw_detail'] = (
            'turned in place until the ESTIMATED yaw matched the scenario\'s '
            'initial yaw; the loop closes on /amcl_pose and never on truth')
        if not aligned:
            print(f'--- WARNING: could not align to the scenario initial yaw within '
                  f'{YAW_ALIGN_SIM:.0f} s sim; estimated yaw error {yaw_error} rad. The '
                  'mission starts from a different heading than the scenario declares',
                  flush=True)

        record['localization_converged'] = bool(did_converge)
        # Two different quantities, under two different names. The convergence
        # time is observed in the AMCL callback at the instant the trace first
        # fell below the limit; the warm-up total also covers the extra
        # rotation, the settle and the yaw alignment that follow it.
        record['localization_converged_sim_s'] = (
            node.converged_sim - initialpose_sim if node.converged_sim is not None
            else None)
        record['warmup_total_sim_s'] = node.sim_now() - initialpose_sim
        if node.amcl is not None:
            cov = node.amcl.pose.covariance
            record['localization_cov_trace'] = cov[0] + cov[7] + cov[35]
        if not did_converge:
            print(f'--- AMCL did not converge within {AMCL_CONVERGE_SIM:.0f} s sim '
                  f'(trace {record["localization_cov_trace"]}); continuing', flush=True)
        return None

    def _align_to_initial_yaw(self, target_yaw: float) -> 'tuple[bool, float | None]':
        """
        Turn in place until the ESTIMATED yaw matches the scenario's.

        Returns (aligned, estimated yaw error in rad). The error is None when
        AMCL has published no pose at all, which is reported rather than
        treated as zero. The consequence of closing on the estimate is honest:
        if AMCL is wrong about the heading, the mission starts from the
        heading AMCL believes, and run.json records the residual it believed
        it had.
        """
        node = self.node
        deadline = node.sim_now() + YAW_ALIGN_SIM
        wall_deadline = time.monotonic() + YAW_ALIGN_SIM * 10.0 + 60.0
        error = None
        last_cmd = 0.0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if node.amcl is not None:
                error = wrap_to_pi(
                    target_yaw - yaw_of_quaternion(node.amcl.pose.pose.orientation))
                if abs(error) <= YAW_ALIGN_TOLERANCE:
                    node.stop()
                    node.spin_sim(AMCL_SETTLE_SIM)
                    return True, error
            now = time.monotonic()
            if now - last_cmd >= 0.05:
                rate = 0.0 if error is None else math.copysign(
                    min(AMCL_WARMUP_RATE, max(YAW_ALIGN_MIN_RATE, abs(error))), error)
                node.cmd_pub.publish(_twist(angular_z=rate))
                last_cmd = now
            if node.sim_now() > deadline or now > wall_deadline:
                break
        node.stop()
        node.spin_sim(AMCL_SETTLE_SIM)
        return False, error

    # -- the mission ----------------------------------------------------------
    def send_goal(self, goal: PoseStamped, record: dict) -> 'tuple[bool, str | None, str]':
        node = self.node

        def on_feedback(message):
            self.feedback['count'] += 1
            self.feedback['distance_remaining'] = message.feedback.distance_remaining
            self.feedback['number_of_recoveries'] = \
                message.feedback.number_of_recoveries
            if self.feedback['count'] == 1:
                # The runner publishes STATE_NAVIGATING off this: the first
                # feedback is the first OBSERVATION that the server is
                # executing, rather than merely having accepted.
                self.navigating_since = node.sim_now()

        request = NavigateToPose.Goal()
        request.pose = goal
        send_future = self.client.send_goal_async(request,
                                                  feedback_callback=on_feedback)
        node.spin_sim(GOAL_ACCEPT_WAIT_SIM, until=send_future.done)
        handle = send_future.result() if send_future.done() else None
        if handle is None or not handle.accepted:
            # NOT `aborted`: no GoalStatus was ever produced for a goal the
            # server refused, and `aborted` names STATUS_ABORTED (6).
            return False, 'runner:not_accepted', (
                'no response to the goal request' if handle is None
                else 'the action server refused the goal')
        self.handle = handle
        record['goal_uuid'] = bytes(handle.goal_id.uuid).hex()
        self.result_future = handle.get_result_async()
        return True, None, f'goal_id {record["goal_uuid"]}'

    def terminal(self) -> 'tuple[str, str, str] | None':
        if self.result_future is None or not self.result_future.done():
            return None
        status = int(self.result_future.result().status)
        self.record['nav2_status_code'] = status
        # A non-terminal status as the FINAL one is not an abort: it is a
        # status whose meaning for a finished mission is unknown.
        state, outcome = TERMINAL_STATES.get(
            status, (MissionState.STATE_UNKNOWN, f'nav2:unknown:status_{status}'))
        self.record['nav2_status_name'] = outcome
        detail = (f'distance_remaining={self.feedback["distance_remaining"]}, '
                  f'number_of_recoveries={self.feedback["number_of_recoveries"]}'
                  if self.feedback['count'] else 'no action feedback was received')
        return state, outcome, f'{detail}; from /navigate_to_pose status {status}'

    def no_terminal_detail(self) -> str:
        """Report what the runner says when no terminal status arrived."""
        return (f'distance_remaining={self.feedback["distance_remaining"]}, '
                f'number_of_recoveries={self.feedback["number_of_recoveries"]}'
                if self.feedback['count'] else 'no action feedback was received')

    def request_cancel(self, reason: str) -> None:
        if self.handle is not None and not self.canceled:
            self.handle.cancel_goal_async()
            self.canceled = True
            print(f'--- cancel requested ({reason}); waiting for a terminal status',
                  flush=True)

    def stop(self) -> None:
        self.node.stop()

    def teardown(self) -> None:
        """Destroy the action client, which holds its own node reference."""
        self.client.destroy()
        self.handle = None
        self.result_future = None


def _twist(linear_x: float = 0.0, angular_z: float = 0.0) -> Twist:
    """Build the warm-up rotation command; Nav2 owns /cmd_vel once a goal runs."""
    msg = Twist()
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    return msg
