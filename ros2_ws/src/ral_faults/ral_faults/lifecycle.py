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
What is in force, and the one lock that serializes changing it.

The fault manager runs on a MultiThreadedExecutor with every callback in one
ReentrantCallbackGroup, because a gz service call made from inside a service
callback needs another thread to complete its future. That arrangement
serializes NOTHING: two /faults/* requests, or a request and the re-projection
timer, genuinely run at the same time. Two failures follow from that, and both
have been measured on this plant:

  * a `set_mobility_fault` overlapping a re-projection interleaves as
    clear -> clear -> publish -> publish. gz's ApplyLinkWrench keeps a LIST of
    persistent wrenches, so two survive and the applied force is DOUBLE the
    requested one (4.76 m travelled where a single wrench gives 2.97 m).
  * a disable or a clear_all landing inside the timer's >=50 ms publish window
    sends its clear FIRST and the timer's wrench lands after. The registry is
    then empty, /faults/active is empty and `cleared` is on the wire while the
    force is still in gz - a fault claimed to be gone on the strength of a
    message having been sent, which is the one thing this project forbids.

THE LOCK CONTRACT
-----------------
The node acquires `registry.lock` at the TOP of every state-changing callback
and does not release it until the matching `_publish_active()` has returned.
`_apply_wrench`, `_clear_wrench` and every gz or relay call are made under it.
A gz call can therefore hold the lock for up to `service_wait_s`; a second
fault request waits that long, which is correct - two fault mutations in
flight at once is precisely the failure above.

The ONE caller that must not wait is the re-projection timer: it takes the
lock non-blocking and skips the tick when a request holds it. Skipping is
harmless (the next tick is 100 ms later and the yaw threshold still holds),
and it is what makes a long gz call under the lock safe.

`_on_truth` does NOT take the lock: the truth stream must never block.

The lock is an RLock because the node holds it across a whole handler while
`_publish_active()` re-acquires it.
"""

import threading

from ral_interfaces.msg import MissionState


class FaultRegistry:
    """What is in force, plus the single lock that serializes changing it."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._entries = {}
        self._counter = {}

    @property
    def lock(self) -> threading.RLock:
        """Return the mutation lock; this module's docstring is its contract."""
        return self._lock

    def mint_id(self, kind: str, given: str) -> str:
        """
        Return the caller's id when it sent one, else the next `<kind>_ui_<n>`.

        Minting takes the lock: an unlocked read-increment-write hands the
        same id to two concurrent dashboard requests, and two fault instances
        sharing an id cannot be told apart on /fault_events afterwards.
        """
        if given:
            return given
        with self._lock:
            count = self._counter.get(kind, 0) + 1
            self._counter[kind] = count
            return f'{kind}_ui_{count}'

    def get(self, key):
        """Return the entry in force under `key`, or None."""
        with self._lock:
            return self._entries.get(key)

    def put(self, key, entry: dict) -> None:
        """Record `entry` as in force under `key`, replacing any predecessor."""
        with self._lock:
            self._entries[key] = entry

    def pop(self, key):
        """Remove and return the entry under `key`, or None."""
        with self._lock:
            return self._entries.pop(key, None)

    def pop_if(self, key, entry: dict) -> bool:
        """
        Remove `key` only when the stored entry IS `entry`. True when removed.

        Identity, not equality: a superseded singleton and its replacement
        carry equal-looking fields, and `==` would let a clear belonging to
        the old instance remove the new one.
        """
        with self._lock:
            if self._entries.get(key) is entry:
                del self._entries[key]
                return True
            return False

    def snapshot(self) -> list:
        """Return a new [(key, entry)] list, safe to iterate while this changes."""
        with self._lock:
            return list(self._entries.items())


def mission_start_is_new(state: str, stamp: tuple, last: 'tuple | None') -> bool:
    """
    Return True only for a goal_accepted whose stamp has not been handled yet.

    /mission_state is transient local, so a late-starting manager receives the
    backlog - idle, then goal_accepted - and would clear every active fault
    twice for one mission without the stamp check.
    """
    if state != MissionState.STATE_GOAL_ACCEPTED:
        return False
    return stamp != last
