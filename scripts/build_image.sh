#!/usr/bin/env bash
# Builds the ral:dev toolchain image in three bounded stages.
#
#   1. `docker build` -> ral:base      apt layers only (ROS 2 Humble, Gazebo
#      Harmonic, Nav2/MAVROS/..., Mesa). Network- and I/O-bound, safe.
#   2. `docker run` -> `docker commit` -> ral:overlay   the ros_gz source
#      overlay, compiled inside a container with an explicit memory cap.
#      Reused when it already exists; RAL_FORCE_OVERLAY=1 rebuilds it.
#   3. `docker build` -> ral:dev       entrypoint layer only, seconds to apply.
#
# Stage 2 is not a Dockerfile RUN because BuildKit ignores --memory on
# `docker build`. Compiling ros_gz unbounded froze this workstation twice
# (15 GiB RAM + 2 GiB swap vs. colcon's default 12 parallel workers, each
# ros_gz_bridge TU peaking 1.5-2.5 GiB): reclaim livelock, no OOM kill, power
# cycle to recover. With a cgroup limit the kernel kills cc1plus instead, the
# container exits non-zero, and the host stays usable.
#
# Env overrides: RAL_BUILD_MEM (default 6g), RAL_BUILD_CPUS (default 6).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_IMAGE="${RAL_BASE_IMAGE:-ral:base}"
OVERLAY_IMAGE="${RAL_OVERLAY_IMAGE:-ral:overlay}"
DEV_IMAGE="${RAL_IMAGE:-ral:dev}"
BUILD_MEM="${RAL_BUILD_MEM:-6g}"
BUILD_CPUS="${RAL_BUILD_CPUS:-6}"
CONTAINER="ral_overlay_build"

echo "== stage 1/3: $BASE_IMAGE (apt layers)"
docker build --progress=plain -t "$BASE_IMAGE" "$REPO_ROOT/docker"

# The overlay is reused when it already exists: it is the only expensive step
# and nothing above it changes without a deliberate edit to overlay.repos or
# build_overlay.sh. RAL_FORCE_OVERLAY=1 rebuilds it.
if [ "${RAL_FORCE_OVERLAY:-0}" != "1" ] && docker image inspect "$OVERLAY_IMAGE" >/dev/null 2>&1; then
  echo "== stage 2/3: $OVERLAY_IMAGE exists, reusing (RAL_FORCE_OVERLAY=1 to rebuild)"
else
  echo "== stage 2/3: ros_gz overlay, capped at $BUILD_MEM / $BUILD_CPUS CPUs"
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true

  # --memory-swap equal to --memory disables swap for the container: swapping is
  # what turns this compile into a host freeze, so the limit must be on RAM alone.
  docker run \
    --name "$CONTAINER" \
    --memory "$BUILD_MEM" \
    --memory-swap "$BUILD_MEM" \
    --cpus "$BUILD_CPUS" \
    "$BASE_IMAGE" \
    build_overlay.sh

  docker commit --change 'CMD ["bash"]' "$CONTAINER" "$OVERLAY_IMAGE"
  docker rm "$CONTAINER" >/dev/null
fi

echo "== stage 3/3: $DEV_IMAGE (entrypoint layer)"
docker build --progress=plain \
  -f "$REPO_ROOT/docker/Dockerfile.dev" \
  --build-arg "OVERLAY_IMAGE=$OVERLAY_IMAGE" \
  -t "$DEV_IMAGE" \
  "$REPO_ROOT/docker"

echo "== $DEV_IMAGE ready"
docker images "$DEV_IMAGE" --format '{{.Repository}}:{{.Tag}} {{.ID}} {{.Size}}'
