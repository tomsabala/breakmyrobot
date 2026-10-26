#!/usr/bin/env bash
# Serve the operator dashboard from inside the ral:dev container.
#
#   scripts/dashboard.sh                 -> http://localhost:8771
#   RAL_DASHBOARD_PORT=9000 scripts/dashboard.sh
#
# The gateway is a LONG-LIVED process that outlives any one stack: it launches
# and stops the stack itself, so it cannot be started by a stack launch file.
# Relaunching a robot from the browser keeps this process and the browser's
# connection alive.
#
# The container runs --network host, so the port it binds is the host's.
#
# ONE owner at a time per ROS_DOMAIN_ID, exactly as scripts/run_scenario.sh:
# the stack owns fixed topic names and a single gz server, so a scenario run
# and the dashboard must never drive the same domain. The lock fd survives
# into the exec'd process and is released when it exits.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="${RAL_DASHBOARD_PORT:-8771}"

exec 9>"/tmp/ral-run-${ROS_DOMAIN_ID:-0}.lock"
flock -n 9 || {
  echo "another run or dashboard holds ROS_DOMAIN_ID ${ROS_DOMAIN_ID:-0}; two owners" \
       "in one domain drive the same robot" >&2
  exit 2
}

exec "$REPO_ROOT/scripts/dev.sh" bash -lc '
source /ws/ros2_ws/install/setup.bash
exec ros2 run ral_eval dashboard_api --port "$1" --repo-root /ws
' _ "$PORT"
