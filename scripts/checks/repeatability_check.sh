#!/usr/bin/env bash
# Measure the spread of one scenario over N identical runs and write
# docs/repeatability.md.
#
#   scripts/checks/repeatability_check.sh
#   RAL_REPEAT_COUNT=5 RAL_REPEAT_EXPERIMENT=baseline_roomba \
#     scripts/checks/repeatability_check.sh
#
# N SEQUENTIAL runs, never parallel: the stack owns fixed topic names and one
# gz server per ROS_DOMAIN_ID.
#
# This measures a spread; it does not assert determinism. `seed:` in a
# scenario seeds ral_faults' RNG and nothing else - nav2_amcl exposes no seed,
# and ROS callback scheduling is not deterministic either. The report says so
# in its own words.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
COUNT="${RAL_REPEAT_COUNT:-3}"
EXPERIMENT="${RAL_REPEAT_EXPERIMENT:-baseline_roomba}"

[ "$COUNT" -ge 2 ] 2>/dev/null || {
  echo "RAL_REPEAT_COUNT must be an integer >= 2; a spread needs at least two runs" >&2
  exit 2
}

for index in $(seq 1 "$COUNT"); do
  echo "=== repeatability run $index/$COUNT ($EXPERIMENT) ==="
  "$REPO_ROOT/scripts/run_scenario.sh" "$EXPERIMENT"
done

# argv, not interpolation: see scripts/run_scenario.sh.
exec "$REPO_ROOT/scripts/dev.sh" bash -lc '
source /ws/ros2_ws/install/setup.bash
exec python3 /ws/scripts/checks/repeatability_report.py \
  --runs-dir /ws/runs --experiment-id "$1" --count "$2" --repo-root /ws
' _ "$EXPERIMENT" "$COUNT"
