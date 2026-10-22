#!/usr/bin/env bash
# Sources the three overlays in dependency order, then runs the command.
#   /opt/ros/humble       ROS 2 base
#   /opt/gz_ws            source-built ros_gz bridge (Harmonic); absent in
#                         ral:base, present once build_overlay.sh has run
#   /ws/ros2_ws/install   this project, present only after scripts/build.sh
#
# GZ_PARTITION/USER are set because scripts/dev.sh runs as the host uid, which
# has no /etc/passwd entry (`whoami: cannot find name for user ID 1001`).
# gz-transport derives its default partition from hostname:username, so without
# this the simulator and `gz topic` land in different partitions and discovery
# silently returns nothing — the sim runs, publishes, and appears invisible.
# A fixed partition also isolates this project from any other gz process on the
# host. USER is exported too: colcon and the ROS 2 daemon resolve it as well.
export GZ_PARTITION="${GZ_PARTITION:-ral}"
export USER="${USER:-ral}"
set -e

source /opt/ros/humble/setup.bash

if [ -f /opt/gz_ws/install/setup.bash ]; then
  source /opt/gz_ws/install/setup.bash
fi

if [ -f /ws/ros2_ws/install/setup.bash ]; then
  source /ws/ros2_ws/install/setup.bash
fi

exec "$@"
