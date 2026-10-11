# syntax=docker/dockerfile:1.6
#
# One recipe, two targets:
#   base/robot : everything the drone runs (builds natively on amd64 and on the Pi 5 / arm64)
#   sim        : robot + Gazebo Harmonic + ArduPilot SITL (laptop only)
#
ARG ROS_DISTRO=jazzy

######################################################################
FROM ros:${ROS_DISTRO}-ros-base AS base
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git wget curl nano tmux geographiclib-tools \
      python3-pip python3-colcon-common-extensions python3-vcstool \
      python3-numpy python3-scipy python3-opencv \
      libpcl-dev libeigen3-dev \
      ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras \
      ros-${ROS_DISTRO}-pcl-conversions ros-${ROS_DISTRO}-pcl-ros \
      ros-${ROS_DISTRO}-sensor-msgs-py ros-${ROS_DISTRO}-geographic-msgs \
      ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-v4l2-camera \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# MAVROS needs the GeographicLib geoid datasets
RUN /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

# Livox SDK2 (needed by livox_ros_driver2, which FAST-LIO also compiles against).
# "-include cstdint" fixes the build with GCC 13 (Ubuntu 24.04).
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 \
 && cmake -S /tmp/Livox-SDK2 -B /tmp/Livox-SDK2/build -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_FLAGS="-include cstdint -Wno-error" \
 && cmake --build /tmp/Livox-SDK2/build -j"$(nproc)" \
 && cmake --install /tmp/Livox-SDK2/build \
 && rm -rf /tmp/Livox-SDK2 && ldconfig

# Person detector (CPU torch + ultralytics). numpy<2 keeps ROS/scipy apt packages working.
ARG INSTALL_YOLO=1
COPY docker/pip-constraints.txt /tmp/pip-constraints.txt
RUN if [ "$INSTALL_YOLO" = "1" ]; then \
      pip3 install --no-cache-dir -c /tmp/pip-constraints.txt torch torchvision \
        --index-url https://download.pytorch.org/whl/cpu \
      && pip3 install --no-cache-dir -c /tmp/pip-constraints.txt ultralytics \
      && mkdir -p /opt/models && cd /opt/models \
      && python3 -c "from ultralytics import YOLO; YOLO('yolo11n.pt')" ; \
    fi

# Third-party ROS workspace: FAST-LIO2 + Livox driver
COPY deps.repos /opt/airmouse_deps/deps.repos
COPY scripts/build_deps.sh /opt/airmouse_deps/build_deps.sh
RUN bash /opt/airmouse_deps/build_deps.sh /opt/airmouse_deps

# Shell environment + entrypoint
COPY docker/airmouse_env.sh /etc/airmouse_env.sh
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && echo 'source /etc/airmouse_env.sh' >> /root/.bashrc \
 && mkdir -p /ws/src
WORKDIR /ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

######################################################################
FROM base AS robot
# Nothing extra: the drone image is the base image.

######################################################################
FROM base AS sim
ARG ROS_DISTRO
# ArduPilot publishes releases as tags (Copter-x.y.z); keep in sync with the real FC firmware
ARG ARDUPILOT_REF=Copter-4.6.3
ENV GZ_VERSION=harmonic

# Gazebo Harmonic through the ROS vendor packages (official Jazzy pairing)
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-gz \
      rapidjson-dev libopencv-dev \
      libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
      gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
      python3-dev python3-lxml python3-pexpect python3-future \
      mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# ArduPilot <-> Gazebo plugin
RUN git clone --depth 1 https://github.com/ArduPilot/ardupilot_gazebo.git /opt/ardupilot_gazebo \
 && . /opt/ros/${ROS_DISTRO}/setup.sh \
 && cmake -S /opt/ardupilot_gazebo -B /opt/ardupilot_gazebo/build -DCMAKE_BUILD_TYPE=RelWithDebInfo \
 && cmake --build /opt/ardupilot_gazebo/build -j"$(nproc)"

# ArduPilot SITL (ArduCopter binary only)
RUN git clone --depth 1 --branch ${ARDUPILOT_REF} --recurse-submodules --shallow-submodules \
      https://github.com/ArduPilot/ardupilot.git /opt/ardupilot \
 && pip3 install --no-cache-dir empy==3.3.4 pymavlink dronecan \
 && cd /opt/ardupilot && ./waf configure --board sitl && ./waf copter

ENV ARDUPILOT_HOME=/opt/ardupilot \
    GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_gazebo/build \
    GZ_SIM_RESOURCE_PATH=/opt/ardupilot_gazebo/models:/opt/ardupilot_gazebo/worlds
