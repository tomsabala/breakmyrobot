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
How one platform executes a mission, and the closed outcome vocabulary.

The scenario runner owns everything that is the same for every robot: the run
directory, the bag, /mission_state, the intervention schedule, the timeout and
the artifacts. A DRIVER owns only the part that is not: how a goal is sent to
this platform's autonomy stack, how it is monitored, and what stopping means.

The split exists because the two stacks share nothing at this level. Nav2
takes a NavigateToPose action goal and answers with a GoalStatus; ArduRover
takes a GUIDED position setpoint over MAVLink, acknowledges nothing, and is
observed through its own telemetry. Pretending those are one interface - a
"generic goal" with a "generic result" - would mean inventing a status for the
platform that does not produce one, which is exactly the kind of claim this
project refuses to make.

No driver publishes a MissionState. The runner is the single publisher of
/mission_state, so the bag has one writer and the sequence of states cannot
interleave two sources.
"""

# Every literal any driver or the runner may put in MissionState.outcome.
#
# This set is the one in ral_interfaces/msg/MissionState.msg's comment block,
# in code; test_mission_vocabulary parses that block out of the INSTALLED .msg
# and asserts the two are equal, so the message's prose and this tuple are one
# artifact and cannot drift.
#
# Two entries are TEMPLATES and appear here exactly as the message writes
# them: the emitted value substitutes the status code or the flight mode. They
# are not patterns to match against - `outcome_is_known` below is what a
# consumer uses to tell a well-formed outcome from a typo.
OUTCOMES = (
    'nav2:STATUS_SUCCEEDED',
    'nav2:STATUS_ABORTED',
    'nav2:STATUS_CANCELED',
    'nav2:unknown:status_<n>',
    'mavros:reached_wp_radius',
    'mavros:mode_changed:<MODE>',
    'mavros:disarmed',
    'mavros:stalled_no_progress',
    'mavros:canceled_timeout',
    'mavros:not_accepted:set_mode',
    'mavros:not_accepted:arming',
    'mavros:unknown:no_terminal',
    'runner:not_accepted',
    'runner:not_sent:goal_cell_occupied',
    'runner:not_sent:footprint_outside_map',
    'runner:not_sent:initial_pose_occupied',
    'runner:unknown:no_terminal_status',
    'runner:unknown:clock_stalled',
    # Written by the runner when the run ended before or around the mission
    # itself. They are outcomes like any other: run.json carries them in the
    # same field, so leaving them out of this tuple made a set that claims to
    # be exhaustive disagree with the artifacts.
    'runner:bringup_failed',
    'runner:bag_missing',
    'runner:exception',
)

# The prefix is the SOURCE: which system said the mission ended that way. Two
# autonomy stacks produce terminal outcomes of different kinds, and an
# unprefixed STATUS_ABORTED beside a mode_changed:HOLD leaves a reader unable
# to tell which of them spoke.
OUTCOME_SOURCES = ('nav2:', 'mavros:', 'runner:')


def outcome_is_known(outcome: str) -> bool:
    """
    Return True when `outcome` is a member of the closed set, templates expanded.

    A value filled from a template - `nav2:unknown:status_7`,
    `mavros:mode_changed:HOLD` - is a member; anything else is not, including
    a value that merely starts with a known prefix.
    """
    if outcome in OUTCOMES:
        return True
    if outcome.startswith('nav2:unknown:status_'):
        return outcome != 'nav2:unknown:status_'
    if outcome.startswith('mavros:mode_changed:'):
        return outcome != 'mavros:mode_changed:'
    return False


class MissionDriver:
    """
    One platform's way of executing a mission.

    The runner owns /mission_state, the bag, the timeout and the artifacts; a
    driver owns only HOW a goal is sent, monitored and stopped. Every method
    may spin the runner's node; none of them publishes a MissionState.

    Methods that can refuse return the operator-readable reason as a string
    and None when there is nothing to refuse, matching the capability gate's
    idiom: the string is what ends up in run.json and in the published detail.
    """

    #: 'nav2' | 'mavros_guided'. Recorded in metrics.json's meta.
    name = 'unnamed'

    #: Set by the driver to the sim time at which the stack was first OBSERVED
    #: to be executing the goal, or left None when this platform produces no
    #: such signal. The runner publishes STATE_NAVIGATING off it, so a driver
    #: that cannot observe execution does not claim it.
    navigating_since = None

    def wait_ready(self, wall_limit: float) -> 'str | None':
        """None when the stack can accept a goal, else the refusal detail."""
        raise NotImplementedError

    def prepare(self, initial_map, record: dict) -> 'tuple[str, str] | None':
        """
        Pre-goal work: localization warm-up, flight mode, arming.

        Returns None, or (outcome, detail) - an OUTCOMES member and the
        operator-readable reason - which aborts the run before any goal is
        sent. The outcome is the driver's, not the runner's, because only the
        driver knows which step of its own bring-up refused.

        Whatever it measured goes into `record`, so run.json carries the same
        fields for both platforms even when one of them can only state why a
        field is not applicable.
        """
        raise NotImplementedError

    def send_goal(self, goal, record: dict) -> 'tuple[bool, str | None, str]':
        """
        Send one goal; (accepted, outcome, detail).

        `accepted` False is `rejected`, and `outcome` is then the OUTCOMES
        member the runner must record - the driver names it, because only the
        driver knows which of its own refusals happened. `outcome` is None
        when `accepted` is True. The runner records what it is given rather
        than inspecting the detail string for a prefix.
        """
        raise NotImplementedError

    def spin_once(self) -> None:
        """
        Advance the driver by one mission-loop iteration.

        Setpoint republication, supervision and feedback bookkeeping live
        here. It must return promptly: the runner's intervention schedule is
        driven by the same loop.
        """

    def terminal(self) -> 'tuple[str, str, str] | None':
        """
        (mission_state, outcome, detail) once the mission has ended, else None.

        `outcome` must be a member of OUTCOMES, templates expanded.
        """
        raise NotImplementedError

    def no_terminal_detail(self) -> str:
        """
        Report what this platform last observed, for a mission with no terminal.

        The runner publishes `unknown` in that case, and this is the evidence
        it carries: the last feedback, mode, armed state or distance the
        driver actually saw, never a guess at what went wrong.
        """
        raise NotImplementedError

    def request_cancel(self, reason: str) -> None:
        """Ask the stack to stop. The driver decides what that means."""
        raise NotImplementedError

    def stop(self) -> None:
        """Leave the plant in a safe state; always called in `finally`."""


def build_driver(node, profile: dict, record: dict) -> MissionDriver:
    """
    Return the driver this robot profile asks for.

    Dispatches on `mission_driver`, and raises naming the profile file for an
    unknown value rather than defaulting to Nav2: a platform whose driver is
    misspelled would otherwise be driven by the wrong stack entirely.

    Imported lazily so that a run of one platform does not require the other's
    message packages to be installed - mavros_msgs in particular is a large
    dependency the Roomba has no use for.
    """
    kind = profile['mission_driver']
    if kind == 'nav2':
        from ral_eval.nav2_driver import Nav2Driver
        return Nav2Driver(node, profile, record)
    if kind == 'mavros_guided':
        from ral_eval.mavros_driver import MavrosGuidedDriver
        return MavrosGuidedDriver(node, profile, record)
    raise ValueError(
        f'{profile.get("profile_path", "the robot profile")}: mission_driver is '
        f'{kind!r}, which no driver implements; known drivers are nav2 and '
        'mavros_guided')
