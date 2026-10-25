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
The registry's own rules, without a node.

ral_faults/lifecycle.py holds the state the fault manager mutates and the lock
that serializes the mutation. Everything here is reachable without rclpy - the
only ROS import is the MissionState TYPE, for its state constants - so these
run in any build rather than only when a simulator is up.
"""

import threading

from ral_faults.lifecycle import FaultRegistry, mission_start_is_new

from ral_interfaces.msg import MissionState


def test_a_caller_supplied_id_is_returned_unchanged():
    """The scenario runner mints its own ids and joins its run.json on them."""
    registry = FaultRegistry()
    assert registry.mint_id('kidnap', 'kidnap_1') == 'kidnap_1'
    assert registry.mint_id('kidnap', 'kidnap_1') == 'kidnap_1'


def test_concurrent_minting_never_hands_out_the_same_id_twice():
    """An unlocked read-increment-write gives two dashboard requests one id."""
    registry = FaultRegistry()
    minted = []
    guard = threading.Lock()
    start = threading.Barrier(8)

    def mint():
        start.wait()
        ours = [registry.mint_id('spawn_obstacle', '') for _ in range(20)]
        with guard:
            minted.extend(ours)

    threads = [threading.Thread(target=mint) for _ in range(8)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert len(minted) == 160
    assert len(set(minted)) == 160, 'two fault instances were given the same id'
    assert set(minted) == {f'spawn_obstacle_ui_{n}' for n in range(1, 161)}


def test_pop_if_compares_identity_not_equality():
    """A superseded singleton and its replacement carry equal-looking fields."""
    registry = FaultRegistry()
    first = {'fault_id': 'mobility_fault_1', 'reprojections': 0}
    second = {'fault_id': 'mobility_fault_1', 'reprojections': 0}
    assert first == second and first is not second

    registry.put(('mobility_fault',), first)
    assert registry.pop_if(('mobility_fault',), second) is False
    assert registry.get(('mobility_fault',)) is first
    assert registry.pop_if(('mobility_fault',), first) is True
    assert registry.get(('mobility_fault',)) is None


def test_a_snapshot_is_isolated_from_later_removals():
    """_clear_everything walks a snapshot while removing from the registry."""
    registry = FaultRegistry()
    entry = {'fault_id': 'spawn_obstacle_1'}
    registry.put(('spawn_obstacle', 'blocker_1'), entry)
    taken = registry.snapshot()
    registry.pop(('spawn_obstacle', 'blocker_1'))
    assert taken == [(('spawn_obstacle', 'blocker_1'), entry)]
    assert registry.snapshot() == []


def test_only_an_unhandled_goal_acceptance_starts_a_mission():
    """/mission_state is transient local: the backlog must not clear twice."""
    assert mission_start_is_new(MissionState.STATE_GOAL_ACCEPTED, (5, 0), None) is True
    assert mission_start_is_new(
        MissionState.STATE_GOAL_ACCEPTED, (5, 0), (5, 0)) is False
    assert mission_start_is_new(
        MissionState.STATE_GOAL_ACCEPTED, (6, 0), (5, 0)) is True
    assert mission_start_is_new(MissionState.STATE_IDLE, (6, 0), None) is False
    assert mission_start_is_new(MissionState.STATE_SUCCEEDED, (6, 0), None) is False
