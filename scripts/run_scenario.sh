#!/usr/bin/env bash
# Run one scenario end to end inside the ral:dev container and leave a
# complete run directory behind.
#
#   scripts/run_scenario.sh baseline_roomba
#   scripts/run_scenario.sh ros2_ws/src/ral_eval/scenarios/obstacle_roomba.yaml
#
# A bare name is resolved by run_scenario against the installed
# share/ral_eval/scenarios/; a path is used as given.
#
# The image reference is resolved HERE, on the host: docker is not available
# inside the container, so the runner cannot look it up itself, and env.json
# would otherwise record `unknown` for the image that produced the evidence.
#
# ONE run at a time per ROS_DOMAIN_ID: the stack owns fixed topic names and a
# single gz server, so a second concurrent run would drive the same robot.
# Enforced below with flock, not just asserted here.
#
# The scenario and image are passed as ARGV, never interpolated into a shell
# string: a path containing a quote would otherwise break out of the quoting
# and execute inside the container.
set -euo pipefail

SCENARIO="${1:?usage: scripts/run_scenario.sh <scenario.yaml|name>}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${RAL_IMAGE:-ral:dev}"

# Prefer the pushed digest; fall back to the local image id when the image was
# never pushed (which is the normal case on this workstation).
IMAGE_REF="$(docker image inspect --format '{{index .RepoDigests 0}}' "$IMAGE" 2>/dev/null || true)"
if [ -z "$IMAGE_REF" ]; then
  IMAGE_REF="$(docker image inspect --format '{{.Id}}' "$IMAGE")"
fi

# The lock fd survives into the exec'd process, so it is held for the whole run
# and released when that process exits.
exec 9>"/tmp/ral-run-${ROS_DOMAIN_ID:-0}.lock"
flock -n 9 || {
  echo "another run holds ROS_DOMAIN_ID ${ROS_DOMAIN_ID:-0}; two runs in one domain" \
       "drive the same robot and cross-contaminate both run directories" >&2
  exit 2
}

exec "$REPO_ROOT/scripts/dev.sh" bash -lc '
source /ws/ros2_ws/install/setup.bash
exec ros2 run ral_eval run_scenario --scenario "$1" --image-ref "$2" --repo-root /ws
' _ "$SCENARIO" "$IMAGE_REF"
