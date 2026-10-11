#!/usr/bin/env bash
# Fetch and build third-party ROS packages (FAST-LIO2, livox_ros_driver2).
# Used by the Dockerfile; also works on a native Ubuntu 24.04 + ROS Jazzy install.
set -eo pipefail
DEPS_DIR="${1:-/opt/airmouse_deps}"
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash

mkdir -p "$DEPS_DIR/src"
cd "$DEPS_DIR"
vcs import --recursive src < deps.repos

# livox_ros_driver2 ships separate ROS1/ROS2 manifests and launch dirs (mirrors its build.sh)
cp src/livox_ros_driver2/package_ROS2.xml src/livox_ros_driver2/package.xml
rm -rf src/livox_ros_driver2/launch && cp -r src/livox_ros_driver2/launch_ROS2 src/livox_ros_driver2/launch

# FAST-LIO hard-codes C++14; Jazzy's rclcpp headers need C++17
sed -i -e 's/c++14/c++17/g' -e 's/CMAKE_CXX_STANDARD 14/CMAKE_CXX_STANDARD 17/' src/FAST_LIO/CMakeLists.txt

# DISTRO_ROS (humble|jazzy) selects the rosidl_get_typesupport_target code path
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=${ROS_DISTRO:-jazzy}
rm -rf build log
