#!/usr/bin/env bash
# Export one completed run directory as the console's replay fixture.
#
#   scripts/export_replay.sh runs/scan_degraded_roomba_20261105T085234Z
#   scripts/export_replay.sh runs/ui_car_20261026T224207Z dashboard/public/replay
#
# The run directory and the output directory are given as HOST paths, relative
# to the repository root or absolute inside it, and rewritten to the /ws mount
# the container sees. Both are passed as ARGV, never interpolated into a shell
# string: a path containing a quote would otherwise break out of the quoting
# and execute inside the container.
#
# No flock, unlike run_scenario.sh: this reads a finished bag and writes JSON.
# It starts no stack, joins no ROS graph and publishes nothing, so it is safe
# beside a running simulation. It needs no image reference either - the
# provenance of a fixture is the provenance env.json recorded for the run that
# produced it, and inventing a fresh one here would re-date old evidence.
set -euo pipefail

RUN_DIR="${1:?usage: scripts/export_replay.sh <run-dir> [out-dir]}"
OUT_DIR="${2:-dashboard/public/replay}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Host path -> container path. An absolute path outside the repository is
# refused rather than passed through: /ws is the only mount, so the container
# would report "no run directory" for a path that exists on the host.
to_container() {
  local path="$1"
  case "$path" in
    /ws/*|/ws) printf '%s' "$path" ;;
    /*)
      case "$path" in
        "$REPO_ROOT"/*) printf '/ws/%s' "${path#"$REPO_ROOT"/}" ;;
        *) echo "$path is outside $REPO_ROOT, which is the only path the container" \
                "can see (mounted at /ws)" >&2; exit 2 ;;
      esac ;;
    *) printf '/ws/%s' "$path" ;;
  esac
}

RUN_DIR="$(to_container "$RUN_DIR")"
OUT_DIR="$(to_container "$OUT_DIR")"

exec "$REPO_ROOT/scripts/dev.sh" bash -lc '
source /ws/ros2_ws/install/setup.bash
exec ros2 run ral_eval replay_export --run-dir "$1" --out "$2"
' _ "$RUN_DIR" "$OUT_DIR"
