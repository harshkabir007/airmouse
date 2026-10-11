#!/usr/bin/env bash
# Fetch and build third-party ROS packages (FAST-LIO2, livox_ros_driver2).
# Used by the Dockerfile; also works on a native Ubuntu 24.04 + ROS Jazzy install.
set -eo pipefail
DEPS_DIR="${1:-/opt/airmouse_deps}"
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash

mkdir -p "$DEPS_DIR/src"
cd "$DEPS_DIR"
vcs import --recursive src < deps.repos

# livox_ros_driver2 ships separate ROS1/ROS2 manifests
cp src/livox_ros_driver2/package_ROS2.xml src/livox_ros_driver2/package.xml

# HUMBLE_ROS=humble selects the modern rosidl code path (also correct for Jazzy)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble
rm -rf build log
