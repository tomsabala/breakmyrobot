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
The runner, the schema and the message definitions must spell things alike.

These strings cross a process boundary: the runner publishes them, the
dashboard filters on them and metrics.py windows a run by them. A typo fails
SILENTLY - the dashboard simply never matches, and the mission looks like it
never started. Nothing at runtime can catch that, so it is caught here.
"""

import re
from pathlib import Path

from ral_eval import run_scenario
from ral_eval.metrics import GOAL_STATUS_NAMES, TERMINAL_MISSION_STATES
from ral_eval.scenario import INTERVENTION_PARAMS

from ral_interfaces.msg import FaultEvent, MissionState

import yaml


def _profiles_dir() -> Path:
    """
    ral_bringup/config/robots in the SOURCE tree.

    Read by path rather than through a package dependency: ral_bringup
    exec_depends ral_eval, and the reverse edge would be a colcon cycle.
    test_bridge_contract.py reads its own package's configs the same way.
    """
    for parent in Path(__file__).resolve().parents:
        candidate = parent / 'ral_bringup' / 'config' / 'robots'
        if candidate.is_dir():
            return candidate
    raise AssertionError('ral_bringup/config/robots was not found above '
                         f'{__file__}; the robot profiles are what the read set and '
                         'the recorded set are both derived from')


def test_the_runner_publishes_exactly_the_defined_mission_states():
    """
    Reflective, not a literal list.

    A constant added to MissionState.msg and wired nowhere would otherwise
    pass silently: the literal set would simply not mention it.
    """
    # dir(), not vars(): rosidl defines message constants as properties on the
    # METACLASS, so they are absent from the class __dict__.
    defined = {getattr(MissionState, name) for name in dir(MissionState)
               if name.startswith('STATE_')}
    assert defined, 'MissionState exposes no STATE_* constants'
    assert set(run_scenario.MISSION_STATES) == defined


def test_every_intervention_type_has_a_service_and_a_fault_kind():
    kinds = {getattr(FaultEvent, name) for name in dir(FaultEvent)
             if name.startswith('KIND_')}
    assert kinds, 'FaultEvent exposes no KIND_* constants'
    assert set(run_scenario.DISPATCH) == kinds
    assert set(INTERVENTION_PARAMS) == kinds


def test_the_fault_service_names_phase_4_must_bind_are_pinned():
    """Changing one of these silently stops every scenario's fault dispatch."""
    assert {kind: entry[0] for kind, entry in run_scenario.DISPATCH.items()} == {
        FaultEvent.KIND_SPAWN_OBSTACLE: '/faults/spawn_obstacle',
        FaultEvent.KIND_CLEAR_OBSTACLE: '/faults/clear_obstacle',
        FaultEvent.KIND_SCAN_DEGRADATION: '/faults/set_scan_degradation',
        FaultEvent.KIND_MOBILITY_FAULT: '/faults/set_mobility_fault',
        FaultEvent.KIND_KIDNAP: '/faults/kidnap',
    }


def test_injection_point_matches_the_message_definition_table():
    """FaultEvent.msg:58-62 is the authority; the runner must agree literally."""
    assert run_scenario.injection_point(
        FaultEvent.KIND_SPAWN_OBSTACLE, 'ral_apartment') \
        == '/world/ral_apartment/create (gz UserCommands)'
    assert run_scenario.injection_point(
        FaultEvent.KIND_CLEAR_OBSTACLE, 'ral_apartment') \
        == '/world/ral_apartment/remove (gz UserCommands)'
    assert run_scenario.injection_point(
        FaultEvent.KIND_SCAN_DEGRADATION, 'ral_apartment') \
        == 'scan_fault_relay (driver/transport level)'
    assert run_scenario.injection_point(
        FaultEvent.KIND_MOBILITY_FAULT, 'ral_apartment') \
        == '/world/ral_apartment/wrench/persistent (gz physics)'
    assert run_scenario.injection_point(
        FaultEvent.KIND_KIDNAP, 'ral_apartment') \
        == '/world/ral_apartment/set_pose (gz entity pose)'


def _mission_state_msg_path() -> Path:
    """Resolve the INSTALLED MissionState.msg, falling back to the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = Path(get_package_share_directory('ral_interfaces')) \
            / 'msg' / 'MissionState.msg'
        if installed.is_file():
            return installed
    except Exception:                      # noqa: BLE001 - not installed is not an error
        pass
    for parent in Path(__file__).resolve().parents:
        candidate = parent / 'ral_interfaces' / 'msg' / 'MissionState.msg'
        if candidate.is_file():
            return candidate
    raise AssertionError('MissionState.msg was found neither installed nor in the '
                         'source tree; the outcome vocabulary has no authority to '
                         'check the code against')


def _outcomes_from_the_message() -> 'tuple[set, dict]':
    """
    ({outcome}, {outcome: state}) parsed out of MissionState.msg's own prose.

    The closed-set block is the definition; this parse is what makes the
    message's prose and ral_eval.mission_driver.OUTCOMES one artifact rather
    than two lists that can drift. An entry is a comment line indented by
    exactly three spaces; anything indented further is the continuation of
    the entry above it.
    """
    text = _mission_state_msg_path().read_text().splitlines()
    start = next(index for index, line in enumerate(text)
                 if 'The closed set is exactly:' in line)
    outcomes = set()
    states = {}
    for line in text[start + 1:]:
        if 'ral_eval.mission_driver.OUTCOMES is this set' in line:
            break
        if not line.startswith('#'):
            continue
        body = line[1:]
        if not body.startswith('   ') or body.startswith('    '):
            continue
        token = body.split()[0]
        outcomes.add(token)
        # `(state unknown; a non-terminal GoalStatus ...)` is annotated the
        # same way, so the closing paren is not part of the match.
        found = re.search(r'\(state (\w+)[;)]', body)
        if found:
            states[token] = found.group(1)
    assert outcomes, 'no outcome literals parsed out of MissionState.msg'
    return outcomes, states


def test_every_outcome_names_the_system_that_produced_it():
    """
    Two stacks produce terminal outcomes, so every value carries its source.

    An unprefixed STATUS_ABORTED beside a mode_changed:HOLD leaves a reader
    unable to tell which system spoke, which is the one thing `outcome` is
    for.
    """
    from ral_eval.mission_driver import OUTCOMES
    assert OUTCOMES, 'mission_driver exports no OUTCOMES'
    for outcome in OUTCOMES:
        assert outcome.startswith(('nav2:', 'mavros:', 'runner:')), outcome


def test_the_nav2_outcomes_are_the_literal_goalstatus_names():
    """Nav2's half must stay the action's own vocabulary, never a phrase."""
    from ral_eval.mission_driver import OUTCOMES
    stripped = {outcome[len('nav2:'):] for outcome in OUTCOMES
                if outcome.startswith('nav2:')}
    assert stripped, 'no nav2: outcome in OUTCOMES'
    allowed = set(GOAL_STATUS_NAMES.values())
    for name in stripped:
        assert name in allowed or name.startswith('unknown:status_'), name


def test_the_outcome_set_in_code_equals_the_one_in_the_message():
    """The message's prose and the code are one artifact, or they drift."""
    from ral_eval.mission_driver import OUTCOMES
    assert set(OUTCOMES) == _outcomes_from_the_message()[0]


def test_metrics_closes_the_window_on_every_terminal_state_an_outcome_names():
    """
    Every state MissionState.msg attaches to an outcome must end a window.

    A terminal state metrics.py does not recognise leaves mission_duration_s
    null for a mission that demonstrably ended.
    """
    _, states = _outcomes_from_the_message()
    assert states, 'no (state ...) annotations parsed out of MissionState.msg'
    terminal = set(TERMINAL_MISSION_STATES)
    for outcome, state in states.items():
        assert state in terminal, f'{outcome} -> {state}'


def test_the_recorded_topics_cover_every_topic_metrics_reads():
    """
    Per PROFILE, because the read set is now a function of the robot.

    A topic metrics.py reads and the runner does not record is a metric that
    is null in every run of that platform, for a reason no one would look for
    in the recorder.
    """
    from ral_eval.metrics import read_topics
    profiles = sorted(_profiles_dir().glob('*.yaml'))
    assert profiles, 'no robot profiles found to check the read set against'
    for path in profiles:
        profile = yaml.safe_load(path.read_text())
        recorded = set(run_scenario.RECORD_TOPICS_COMMON) | set(profile['record_topics'])
        assert set(read_topics(profile)) <= recorded, path.name
