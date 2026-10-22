#!/usr/bin/env bash
# Build this project's ROS 2 workspace inside the toolchain image.
# Extra arguments are forwarded to colcon, e.g.
#   scripts/build.sh --packages-select ral_description
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$REPO_ROOT/scripts/dev.sh" bash -lc "cd /ws/ros2_ws && colcon build --symlink-install $*"
