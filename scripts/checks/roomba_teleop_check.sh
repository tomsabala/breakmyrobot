#!/usr/bin/env bash
# End-to-end proof of the Roomba platform and its teleop closed loop. Runs
# roomba_teleop_check.py inside the ral:dev container, which launches the whole
# stack, drives the robot and writes docs/roomba-teleop-loop.md.
#
#   scripts/checks/roomba_teleop_check.sh
#
# The image reference is resolved here, on the host: docker is not available
# inside the container, so the Python script cannot look it up itself.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IMAGE="${RAL_IMAGE:-ral:dev}"

# Prefer the pushed digest; fall back to the local image id when the image was
# never pushed (which is the normal case on this workstation).
IMAGE_REF="$(docker image inspect --format '{{index .RepoDigests 0}}' "$IMAGE" 2>/dev/null || true)"
if [ -z "$IMAGE_REF" ]; then
  IMAGE_REF="$(docker image inspect --format '{{.Id}}' "$IMAGE")"
fi

exec "$REPO_ROOT/scripts/dev.sh" bash -lc "
source /ws/ros2_ws/install/setup.bash
python3 /ws/scripts/checks/roomba_teleop_check.py --image-ref '$IMAGE_REF' --repo-root /ws
"
