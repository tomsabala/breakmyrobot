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

from ral_eval import run_scenario
from ral_eval.metrics import GOAL_STATUS_NAMES, TERMINAL_MISSION_STATES
from ral_eval.scenario import INTERVENTION_PARAMS

from ral_interfaces.msg import FaultEvent, MissionState


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


def test_the_terminal_status_map_uses_the_literal_goalstatus_names():
    outcomes = {outcome for _, outcome in run_scenario.TERMINAL_STATES.values()}
    assert outcomes == {'STATUS_SUCCEEDED', 'STATUS_CANCELED', 'STATUS_ABORTED'}
    for code, (_, outcome) in run_scenario.TERMINAL_STATES.items():
        assert GOAL_STATUS_NAMES[code] == outcome


def test_metrics_closes_the_window_on_every_terminal_state_the_runner_can_publish():
    publishable = {state for state, _ in run_scenario.TERMINAL_STATES.values()}
    publishable.add(MissionState.STATE_UNREACHABLE)
    assert publishable <= set(TERMINAL_MISSION_STATES)


def test_the_recorded_topics_cover_every_topic_metrics_reads():
    from ral_eval.metrics import READ_TOPICS
    assert set(READ_TOPICS) <= set(run_scenario.RECORD_TOPICS)
