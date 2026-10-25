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
#   succeeded      Terminal. The mission reached its goal, as reported by the
#                  autonomy stack driving it - a Nav2 GoalStatus, or the
#                  autopilot's own distance-to-waypoint. `outcome` names which.
#   aborted        Terminal, non-success, reported rather than inferred: a
#                  Nav2 STATUS_ABORTED, an autopilot that left GUIDED or
#                  disarmed by itself, or the driver's deterministic
#                  no-progress policy stopping the vehicle.
#   canceled       Terminal. The runner asked the stack to stop - after the
#                  scenario's timeout_sec - and it did.
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
# `outcome` names BOTH what happened and WHO said so. Two autonomy stacks
# produce terminal outcomes of different kinds: an unprefixed STATUS_ABORTED
# standing beside a mode_changed:HOLD would leave a reader unable to tell
# which system spoke, so every value carries its source as a prefix. `detail`
# names the deciding topic and value in every case.
#
# The `nav2:` values are the literal action_msgs/GoalStatus constant names,
# never a phrase and never inferred: NavigateToPose's result payload is
# std_msgs/Empty, so the status code is the only verbatim outcome the action
# offers. The `mavros:` values are read from the autopilot's own telemetry.
# The `runner:` values are the ones no stack produced, because nothing was
# sent or nothing answered.
#
# The closed set is exactly:
#
#   nav2:STATUS_SUCCEEDED             (state succeeded)
#   nav2:STATUS_ABORTED               (state aborted)
#   nav2:STATUS_CANCELED              (state canceled)
#   nav2:unknown:status_<n>           (state unknown; a non-terminal GoalStatus
#                                      arrived as the final one)
#   mavros:reached_wp_radius          (state succeeded) the FCU's own
#                                     NAV_CONTROLLER_OUTPUT.wp_dist fell within
#                                     WP_RADIUS while the mode was still GUIDED
#                                     and the vehicle armed
#   mavros:mode_changed:<MODE>        (state aborted) the autopilot left GUIDED
#                                     by itself
#   mavros:disarmed                   (state aborted) the autopilot disarmed
#                                     mid-mission
#   mavros:stalled_no_progress        (state aborted) the driver's
#                                     DETERMINISTIC no-progress policy stopped
#                                     the vehicle; see detail. This is a
#                                     programmed safe stop, not autonomy.
#   mavros:canceled_timeout           (state canceled) the runner's timeout;
#                                     the driver commanded HOLD
#   mavros:not_accepted:set_mode      (state rejected)
#   mavros:not_accepted:arming        (state rejected)
#   mavros:unknown:no_terminal        (state unknown)
#   runner:not_accepted               (state rejected)
#   runner:not_sent:goal_cell_occupied      (state unreachable)
#   runner:not_sent:footprint_outside_map   (state unreachable)
#   runner:not_sent:initial_pose_occupied   (state unreachable)
#   runner:unknown:no_terminal_status       (state unknown)
#   runner:unknown:clock_stalled            (state unknown)
#   runner:bringup_failed                   written to run.json only; the
#                                      stack never came up, so no mission
#                                      state was ever published for it
#   runner:bag_missing                      written to run.json only; the
#                                      mission may have run, but its bag
#                                      cannot be measured
#   runner:exception                        written to run.json only; the
#                                      runner itself raised, and the
#                                      traceback is the detail
#
# ral_eval.mission_driver.OUTCOMES is this set, in code; the vocabulary test
# parses this comment block out of the installed .msg and asserts the two are
# equal, so the prose and the code are one artifact.
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
