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
#
# Everything the fault manager currently holds IN FORCE, published on
# /faults/active.
#
# This is a SNAPSHOT, not a history. History is /fault_events, where every
# transition of every fault instance appears exactly once and nothing is ever
# retracted. A reader that wants "what is broken right now" would otherwise
# have to replay the whole event stream and reimplement the manager's own
# lifecycle rules, and two implementations of that rule would eventually
# disagree about whether a fault is still active.
#
# Every entry is the `activated` FaultEvent of a fault that is still in force,
# republished VERBATIM - same id, kind, injection_point, target and stamp -
# with only `detail` refreshed, so a reader can join an entry here against the
# event that created it. A fault that was rejected, cleared, superseded or was
# a one-shot (kidnap, clear_obstacle, which change the world once and are not
# a state that persists) never appears here.
#
# The topic is latched (RELIABLE, TRANSIENT_LOCAL, depth 1) and republished on
# EVERY registry change, including an EMPTY snapshot at manager start-up: a
# subscriber that joins late must be able to tell "no faults are active" from
# "the manager is not running", and only a latched empty message says the
# first.
#
# `stamp` is SIM time at the snapshot, like every other stamp in this project.

builtin_interfaces/Time stamp
FaultEvent[] active
