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
# One transition in the lifecycle of one injected fault, published on
# /fault_events.
#
# The states are a contract between the scenario runner (Phase 3), the fault
# manager (Phase 4) and the dashboard (Phase 6). They exist so that a reader
# can always tell a REQUEST apart from an EFFECT - the project's rule that a
# fault must never be claimed on the strength of a UI control or a sent
# message (ENGINEERING.md section 8) is enforced here, in the vocabulary:
#
#   requested  The request reached the node that owns the injection boundary
#              and has been parsed. NOTHING in the simulated world has changed
#              yet. Published BEFORE the underlying call, by the node that is
#              about to make it.
#   activated  The underlying call RETURNED SUCCESS: gz accepted the entity,
#              the wrench is being applied, or the relay's degradation
#              parameters are in force. Never published on the strength of
#              having sent a request.
#   rejected   The request was refused, or the underlying call failed or was
#              unreachable. `detail` carries the reason verbatim - service
#              unavailable, unsupported for this robot, invalid parameters, or
#              gz's own success=false message.
#   cleared    The fault was removed AND the removal call succeeded. A fault
#              that never reached `activated` cannot be `cleared`; it ends at
#              `rejected`.
#
# Who publishes what: the fault manager owns `requested`, `activated` and
# `cleared`, because only it observes the underlying call. The scenario runner
# publishes exactly one message, `rejected`, and only for the case the manager
# cannot report: its service was not there to be called at all.
#
# `stamp` is SIM time at the transition; every node that publishes this runs
# with use_sim_time true.
#
# `id` is unique per fault INSTANCE within a run and stable across its states,
# so the four transitions of one fault can be joined. The runner and the fault
# manager both build it as <kind>_<1-based index of the intervention in the
# scenario, after sorting by at_sec>.
#
# `kind` is one of the KIND_* constants below.
#
# `injection_point` is the literal gz topic/service or ROS node the request
# TARGETED, never a description of it. It is the boundary that was actually
# altered only on an `activated` or `cleared` event; on `requested` and
# `rejected` it is the boundary that would have been. The fixed strings are:
#
#   spawn_obstacle    /world/ral_apartment/create (gz UserCommands)
#   clear_obstacle    /world/ral_apartment/remove (gz UserCommands)
#   scan_degradation  scan_fault_relay (driver/transport level)
#   mobility_fault    /world/ral_apartment/wrench/persistent (gz physics)
#   kidnap            /world/ral_apartment/set_pose (gz entity pose)
#
# `target` is the entity or stream affected: the model name, the obstacle
# name, or the topic (e.g. ral_roomba, blocker_1, /scan).
#
# `detail` is free text: the parameters actually applied, or the rejection
# reason verbatim.

string STATE_REQUESTED=requested
string STATE_ACTIVATED=activated
string STATE_REJECTED=rejected
string STATE_CLEARED=cleared

string KIND_SPAWN_OBSTACLE=spawn_obstacle
string KIND_CLEAR_OBSTACLE=clear_obstacle
string KIND_SCAN_DEGRADATION=scan_degradation
string KIND_MOBILITY_FAULT=mobility_fault
string KIND_KIDNAP=kidnap

builtin_interfaces/Time stamp
string id
string kind
string injection_point
string target
string state
string detail
