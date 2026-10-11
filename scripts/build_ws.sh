#!/usr/bin/env bash
# Build the AirMouse ROS 2 workspace (inside the container, or natively on the Pi).
set -eo pipefail
WS="${WS:-/ws}"
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
[ -f /opt/airmouse_deps/install/setup.bash ] && source /opt/airmouse_deps/install/setup.bash
cd "$WS"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
echo "Build OK. In this shell run: source $WS/install/setup.bash"
