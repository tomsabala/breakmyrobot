#!/usr/bin/env bash
# Builds the source overlay listed in overlay.repos into /opt/gz_ws, then the
# ArduPilot SITL firmware listed in ardupilot.repos into /opt/ardupilot.
#
# This runs via `docker run` under an explicit cgroup memory cap rather than as
# a Dockerfile RUN layer, because BuildKit ignores --memory on `docker build`
# and an unbounded colcon run froze this workstation twice: colcon defaults to
# --parallel-workers $(nproc) (12 here) and each ros_gz_bridge translation unit
# instantiates the full gz.msgs<->ROS factory set at 1.5-2.5 GiB. Against 15 GiB
# of RAM and a 2 GiB swapfile that ends in reclaim livelock, not an OOM kill —
# the desktop dies before the kernel picks a victim.
#
# Fully serial by construction: one package at a time, one compiler process.
# Slower in wall clock, bounded in memory, and a runaway compiler now hits the
# container's limit instead of the host.
set -euo pipefail

export MAKEFLAGS="-j1"

mkdir -p /opt/gz_ws/src
cd /opt/gz_ws

vcs import --input /tmp/overlay.repos src

apt-get update
rosdep install --from-paths src --ignore-src -y

# ROS's setup.bash reads AMENT_TRACE_SETUP_FILES without a default, so nounset
# has to be off while sourcing it.
set +u
source /opt/ros/humble/setup.bash
set -u

# --packages-up-to prunes the overlay to what the MVP bridges. ros_gz_image and
# ros_gz_point_cloud serve cameras and depth sensors, which no robot in this
# project carries; ros_gz_sim_demos is sample content. ros_gz_interfaces
# (EntityWrench, Contacts, SpawnEntity/DeleteEntity/SetEntityPose) comes in
# transitively as a ros_gz_bridge dependency.
#
# ardupilot_gazebo is built from the same overlay so that its ament
# environment hooks (GZ_SIM_SYSTEM_PLUGIN_PATH, SDF_PATH, GZ_SIM_RESOURCE_PATH)
# are set by sourcing /opt/gz_ws/install/setup.bash, which is what lets a model
# say <plugin filename="ArduPilotPlugin"> and have gz find it.
colcon build \
  --merge-install \
  --executor sequential \
  --parallel-workers 1 \
  --packages-up-to ros_gz_bridge ros_gz_sim sdformat_urdf ardupilot_gazebo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

rm -rf /opt/gz_ws/build /opt/gz_ws/log

# ArduPilot SITL (Phase 5, the car). Built here rather than in a Dockerfile RUN
# for the same reason as the overlay above: this script runs under the cgroup
# memory cap, and `./waf rover` is another unbounded C++ compile.
#
# -j1, inside the MAKEFLAGS=-j1 exported at the top, is what keeps it there:
# waf defaults to one job per core and the ArduPilot tree is large enough to
# repeat the ros_gz failure mode on this host.
#
# `vcs import` does not recurse into submodules, so the update is explicit;
# ArduPilot's SITL build needs them all (mavlink, DroneCAN, waf tools).
mkdir -p /opt/ardupilot
cd /opt/ardupilot
vcs import --input /tmp/ardupilot.repos .
git -C ardupilot submodule update --init --recursive
cd ardupilot
git rev-parse HEAD > /opt/ardupilot/VERSION.txt
./waf configure --board sitl
./waf rover -j1

# The object tree is ~2 GB and nothing runs from it; the binary that matters is
# build/sitl/bin/ardurover, which entrypoint.sh puts on PATH.
rm -rf /opt/ardupilot/ardupilot/build/sitl/modules
rm -rf /var/lib/apt/lists/*
