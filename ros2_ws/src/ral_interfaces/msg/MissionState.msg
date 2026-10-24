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
# The scenario runner's view of ONE mission, published on /mission_state at
# every transition. The runner is the only publisher.
#
# State semantics - each is an OBSERVATION, never an inference:
#
#   idle           The runner is up and the stack passed the bring-up gate.
#                  No goal has been sent.
#   goal_accepted  The action server accepted the goal: the send_goal future
#                  returned a handle with accepted=True.
#   navigating     The first action feedback arrived, i.e. the server is
#                  executing. Published exactly once per mission.
#   succeeded      Terminal. action_msgs/GoalStatus.STATUS_SUCCEEDED (4).
#   aborted        Terminal. STATUS_ABORTED (6).
#   canceled       Terminal. STATUS_CANCELED (5), including the runner's own
#                  cancel after the scenario's timeout_sec.
#   unreachable    PRE-FLIGHT REFUSAL ONLY, and the goal is never sent: the
#                  goal cell, or a cell within robot_radius of it, is occupied
#                  in the static map, or lies outside the grid, or the
#                  scenario's own initial pose fails the same check. An
#                  in-flight failure is `aborted` - Nav2 reports a status, not
#                  a cause, and calling an abort "unreachable" would be an
#                  inference dressed as a measurement.
#   rejected       Terminal. The action server REFUSED the goal: send_goal
#                  returned a handle with accepted=False, or returned nothing.
#                  No GoalStatus was ever produced, so this is not `aborted`.
#   unknown        Terminal. The goal was accepted and executed, and no
#                  terminal GoalStatus ever arrived - after the scenario's
#                  timeout_sec and the cancel wait, or because the mission
#                  loop's deadlock guard fired. The mission's end is not
#                  known, and inventing `aborted` would claim a status the
#                  server never sent.
#
# Unknown (-1) cells do NOT make a goal unreachable: planner_server's GridBased
# planner runs with allow_unknown: true, so refusing them would make the
# runner's verdict disagree with the planner that is actually asked to plan.
# The count of unknown cells found goes into `detail`.
#
# `stamp` is SIM time at the transition.
#
# `experiment_id` is the scenario's id, i.e. the prefix of the run directory.
#
# `goal` is the goal AS SENT, frame_id 'map' (for `unreachable`, the goal that
# would have been sent). Scenarios are written in the gz `world` frame; the
# runner converts with the measured alignment before filling this in.
#
# `outcome` is the literal action_msgs/GoalStatus constant name - STATUS_
# SUCCEEDED, STATUS_ABORTED, STATUS_CANCELED - never a phrase and never
# inferred. NavigateToPose's result payload is std_msgs/Empty, so the status
# code is the only verbatim outcome the action offers. The closed set the
# runner can emit is exactly:
#
#   STATUS_SUCCEEDED, STATUS_ABORTED, STATUS_CANCELED
#   not_accepted                      (state rejected)
#   not_sent:goal_cell_occupied       (state unreachable)
#   not_sent:footprint_outside_map    (state unreachable)
#   not_sent:initial_pose_occupied    (state unreachable)
#   unknown:no_terminal_status        (state unknown)
#   unknown:clock_stalled             (state unknown)
#   unknown:status_<n>                (state unknown; a non-terminal GoalStatus
#                                      arrived as the final one)
#
# `detail` at a terminal state carries the last distance_remaining and
# number_of_recoveries seen in feedback; at `unreachable` it carries the
# occupancy values found at the goal cells and the number of unknown cells.

string STATE_IDLE=idle
string STATE_GOAL_ACCEPTED=goal_accepted
string STATE_NAVIGATING=navigating
string STATE_SUCCEEDED=succeeded
string STATE_ABORTED=aborted
string STATE_CANCELED=canceled
string STATE_UNREACHABLE=unreachable
string STATE_REJECTED=rejected
string STATE_UNKNOWN=unknown

builtin_interfaces/Time stamp
string experiment_id
string state
geometry_msgs/PoseStamped goal
string outcome
string detail
