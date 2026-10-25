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
# One MEASURED frame alignment, published latched by the node that measured it.
#
# Phase 5 uses it for world -> ekf_local: ArduPilot's EKF origin is set by the
# autopilot at its first GPS fix, so the offset between the gz `world` frame
# and the ENU frame /mavros/local_position/* reports in is a per-run
# MEASUREMENT, never a constant. The scenario runner records it in run.json and
# metrics.py converts the car's estimate into `map` with it; a run whose
# residual exceeds the stated limit is refused rather than scored.
#
# This message carries a frame OFFSET and never a robot pose: nothing
# downstream can derive where the robot is from it. The node that publishes it
# does subscribe to simulator truth - that is how an offset is measured at all -
# but what it broadcasts afterwards is the robot's own estimate composed with
# this fixed offset, which is the same rule the display-only map -> world
# transform follows.
#
# `residual_m` is the distance between this measurement and the value predicted
# from the spawn pose, and `residual_yaw_rad` is the heading half of the same
# comparison. Together they are the numbers that say whether the measurement is
# believable, exactly as the world->map alignment residual does in Phase 2.
# Both are needed: a yaw offset leaves `residual_m` untouched while rotating
# the entire estimate track about this origin, which reads downstream as
# estimator drift that grows with distance rather than as the frame error it
# is - and an inverted magnetometer, this platform's known failure mode, is
# exactly that.
builtin_interfaces/Time stamp
string parent_frame
string child_frame
float64 x
float64 y
float64 yaw
float64 residual_m
float64 residual_yaw_rad
string source
