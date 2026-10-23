#!/usr/bin/env bash
# Run a command inside the ral:dev toolchain image.
#
#   scripts/dev.sh                      -> interactive bash
#   scripts/dev.sh bash -lc 'ros2 ...'  -> one command
#
# --network host --ipc=host: the host has no passwordless sudo, so ROS tooling
# runs in the container while host rviz2 / ros2 CLI join the same DDS graph.
# --user $(id -u):$(id -g): colcon build artifacts stay host-owned.
# /dev/dri is passed through when present so Gazebo's ogre2 renders on the iGPU
# instead of falling back to llvmpipe software rasterisation, which would make
# the Phase 0 real-time-factor measurement meaningless. Set RAL_NO_GPU=1 to
# force the software path (used to produce the comparison in docs/gate.md).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${RAL_IMAGE:-ral:dev}"

args=(
  --rm
  --network host
  --ipc=host
  --user "$(id -u):$(id -g)"
  -v "$REPO_ROOT":/ws
  -e HOME=/ws
  -w /ws
  # The container must join the host's DDS domain, or host rviz2 / ros2 CLI see
  # an empty graph while the container's own graph looks perfectly healthy.
  # This host exports ROS_DOMAIN_ID=42; defaults match ROS 2's own defaults.
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
  -e "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}"
)

# A git worktree's .git is a FILE holding an absolute path into the main
# repository's .git directory, which lies outside $REPO_ROOT and is therefore
# not covered by the mount above. Without this, every container-side `git`
# call fails with "not a git repository", so the provenance fields that record
# which commit produced an artifact (capture_world_to_map's git_sha, the proof
# harness's run table) silently degrade to "unknown" - evidence that cannot be
# traced to a commit. Mounting the shared gitdir at its own host path is what
# makes the pointer inside .git resolve in the container. No-op for a plain
# clone, where .git is a directory already under /ws. Read-only: the only
# container-side use is `git rev-parse`, and the main repository's object
# store is not something a sandboxed build should be able to write.
if [ -f "$REPO_ROOT/.git" ]; then
  GIT_COMMON_DIR="$(git -C "$REPO_ROOT" rev-parse --path-format=absolute --git-common-dir 2>/dev/null || true)"
  if [ -n "$GIT_COMMON_DIR" ] && [ -d "$GIT_COMMON_DIR" ]; then
    args+=(-v "$GIT_COMMON_DIR":"$GIT_COMMON_DIR":ro)
  fi
fi

if [ -t 0 ] && [ -t 1 ]; then
  args+=(-it)
fi

# RAL_MEM caps the container's RAM (swap disabled, since swapping is what turns
# memory pressure into a host freeze on this 15 GiB machine). Unset by default:
# a cap on a running simulation would show up as an OOM kill mid-mission and be
# mistaken for a robotics fault. Measurement runs set it explicitly.
if [ -n "${RAL_MEM:-}" ]; then
  args+=(--memory "$RAL_MEM" --memory-swap "$RAL_MEM")
fi

# --device recreates the node inside the container's devfs, which drops the
# host ACL (`user:toms:rw-` on /dev/dri/renderD128). The container node is
# plain root:render 0660, so --user 1001:1001 gets EPERM and ogre2 dies after
# 11 failed EGL attempts. --group-add grants the owning gid as a supplementary
# group; Docker allows gids the host user is not a member of, so no host-side
# group change is needed. Read the gids off the nodes instead of hardcoding
# 109/44, which differ across distributions.
if [ "${RAL_NO_GPU:-0}" != "1" ] && [ -d /dev/dri ]; then
  for dev in /dev/dri/*; do
    [ -c "$dev" ] || continue
    args+=(--device "$dev" --group-add "$(stat -c %g "$dev")")
  done
fi

if [ "$#" -eq 0 ]; then
  set -- bash
fi

exec docker run "${args[@]}" "$IMAGE" "$@"
