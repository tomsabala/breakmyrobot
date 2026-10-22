# Resilient Autonomy Lab toolchain image.
#
# ROS 2 Humble + Gazebo Harmonic + a source-built ros_gz bridge.
#
# Harmonic (not Fortress) is required because ardupilot_gazebo supports
# Garden/Harmonic/Ionic/Jetty only, and Gazebo Classic 11 is EOL. Because apt's
# ros-humble-ros-gz is built against Fortress, the bridge is built from source
# with GZ_VERSION=harmonic into the /opt/gz_ws overlay.
FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Build tooling and the native deps ros_gz / ardupilot_gazebo need.
RUN apt-get update && apt-get install -y --no-install-recommends \
      wget gnupg lsb-release curl git \
      python3-vcstool python3-colcon-common-extensions python3-rosdep \
      build-essential cmake rapidjson-dev libeigen3-dev \
 && rm -rf /var/lib/apt/lists/*

# Gazebo Harmonic from the OSRF stable repository.
RUN wget -q https://packages.osrfoundation.org/gazebo.gpg \
      -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update && apt-get install -y --no-install-recommends gz-harmonic \
 && rm -rf /var/lib/apt/lists/*

# ROS packages used from Phase 2 onwards (apt, not source: integrate, do not author).
#
# The apt.conf drop-in is written here rather than earlier in the file so that
# the expensive gz-harmonic layer above stays cached. archive.ubuntu.com rotates
# mirrors mid-fetch often enough to produce "Hash Sum mismatch" (observed once
# while building this image); retries plus a disabled HTTP pipeline make an
# index fault recoverable instead of fatal to a multi-minute build.
RUN printf 'Acquire::Retries "5";\nAcquire::http::Pipeline-Depth "0";\n' \
      > /etc/apt/apt.conf.d/80-ral-apt \
 && apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-navigation2 ros-humble-nav2-bringup \
      ros-humble-slam-toolbox ros-humble-robot-localization \
      ros-humble-mavros ros-humble-mavros-extras ros-humble-ackermann-msgs \
      ros-humble-foxglove-bridge ros-humble-rosbag2-storage-mcap \
      ros-humble-tf-transformations ros-humble-xacro \
      ros-humble-robot-state-publisher ros-humble-teleop-twist-keyboard \
      python3-transforms3d python3-numpy python3-matplotlib python3-yaml python3-pytest \
 && rm -rf /var/lib/apt/lists/*

# MAVROS refuses to start without the GeographicLib geoid/gravity datasets.
RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# Mesa EGL + llvmpipe fallback + Xvfb, so `gz sim --headless-rendering` has an
# ogre2 context with or without a passed-through DRI device.
RUN apt-get update && apt-get install -y --no-install-recommends \
      libegl1-mesa libgl1-mesa-dri libglx-mesa0 libgl1-mesa-glx xvfb \
 && rm -rf /var/lib/apt/lists/*

ENV GZ_VERSION=harmonic
ENV MPLBACKEND=Agg

# osrf rosdep rules so ros_gz's gz-* keys resolve to Harmonic rather than Fortress.
RUN wget -q https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list \
      -O /etc/ros/rosdep/sources.list.d/00-gazebo.list \
 && rosdep update --rosdistro humble

# Source overlay inputs (see docker/overlay.repos). The compile itself is NOT a
# RUN layer: BuildKit ignores --memory on `docker build`, and an unbounded
# colcon run froze this host twice. scripts/build_image.sh runs
# build_overlay.sh in a memory-capped container and commits the result as
# ral:overlay, so this image stops at the toolchain.
COPY overlay.repos /tmp/overlay.repos
COPY build_overlay.sh /usr/local/bin/build_overlay.sh
RUN chmod +x /usr/local/bin/build_overlay.sh

# No ENTRYPOINT here on purpose: entrypoint.sh is applied by Dockerfile.dev as
# the last, cheap layer on top of the committed overlay, so editing it costs
# seconds instead of recompiling ros_gz. build_overlay.sh sources ROS itself.
CMD ["bash"]
