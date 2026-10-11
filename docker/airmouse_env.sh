# Sourced by the container entrypoint and by every interactive shell.
source /opt/ros/${ROS_DISTRO}/setup.bash
[ -f /opt/airmouse_deps/install/setup.bash ] && source /opt/airmouse_deps/install/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash

export AIRMOUSE_ROOT=/airmouse

# Short commands (see README)
alias am-build='bash /airmouse/scripts/build_ws.sh && source /ws/install/setup.bash'
alias am-sim='ros2 launch airmouse_bringup sim.launch.py'
alias am-robot='ros2 launch airmouse_bringup robot.launch.py'
alias am-start='ros2 topic pub --once /airmouse/cmd std_msgs/msg/String "{data: start}"'
alias am-abort='ros2 topic pub --once /airmouse/cmd std_msgs/msg/String "{data: abort}"'
