# Skybeta — distributable Jetson image: cuVSLAM (OAK-D-S2) + uXRCE-DDS bridge
# + MicroXRCEAgent + MAVProxy. Boot it on any Jetson Orin with the host-side
# UART DMA patch applied (see 02_uart_fix.sh) and a Pixhawk wired to TELEM2.
#
# Distribution: docker save skybeta:latest -o skybeta.tar  (then load on the
# next Jetson). aarch64-only by virtue of the NGC Isaac ROS base.
#
# Base is the exact NGC Isaac ROS Humble tag that isaac_ros_common's
# run_dev.sh pulls — the bare `aarch64-ros2_humble` tag does not exist on
# NGC, only the content-addressed variants. This one is what `isaac_ros_slam`
# already has cached locally, so the build reuses it.
FROM nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_4c0c55dddd2bbcc3e8d5f9753bee634c

ARG DEBIAN_FRONTEND=noninteractive
ARG XRCE_AGENT_TAG=v2.4.3
SHELL ["/bin/bash", "-c"]

# 1. Apt layer. Mirrors isaac_ros_slam Dockerfile.user (depthai-ros-driver-v3,
#    isaac-ros-visual-slam, NITROS imu/odometry types) plus the build deps
#    we need for the agent + workspace + tmux for the full-stack launcher.
#    Kept in one RUN so it caches as a single ~3 GB layer.
RUN rm -f /etc/apt/sources.list.d/kitware*.list \
          /etc/apt/sources.list.d/yarn*.list \
          /etc/apt/sources.list.d/nvidia-isaac-ros*.list || true \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
        build-essential cmake git wget curl ca-certificates sudo \
        python3-pip python3-dev libusb-1.0-0-dev udev usbutils tmux \
        python3-colcon-common-extensions python3-rosdep python3-vcstool \
        ros2-testing-apt-source \
 && apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-depthai-ros-driver-v3 \
        ros-humble-depthai-bridge-v3 \
        ros-humble-depthai-ros-msgs-v3 \
        ros-humble-depthai-descriptions-v3 \
        ros-humble-isaac-ros-visual-slam \
        ros-humble-isaac-ros-visual-slam-interfaces \
        ros-humble-isaac-ros-nitros-imu-type \
        ros-humble-isaac-ros-nitros-odometry-type \
 && rm -rf /var/lib/apt/lists/*

# 2. Python deps — depthai (host runtime) + MAVProxy (sanity test §3 of the
#    setup doc).
RUN pip3 install --no-cache-dir depthai MAVProxy

# 3. OAK-D udev rule baked in (also installed on host by 01_prereqs.sh).
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
    > /etc/udev/rules.d/80-movidius.rules

# 4. Micro-XRCE-DDS-Agent v2.4.3. PX4 v1.14 uxrce_dds_client is wire-incompat
#    with v3.x — pin the tag.
RUN git clone -b ${XRCE_AGENT_TAG} \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
        /opt/Micro-XRCE-DDS-Agent \
 && cmake -S /opt/Micro-XRCE-DDS-Agent -B /opt/Micro-XRCE-DDS-Agent/build \
 && make  -C /opt/Micro-XRCE-DDS-Agent/build -j"$(nproc)" \
 && make  -C /opt/Micro-XRCE-DDS-Agent/build install \
 && ldconfig /usr/local/lib/

# 5. ROS 2 workspace: oakd_vslam_bringup (copied) + px4_msgs @ release/1.14.
#    Cache busts when src/oakd_vslam_bringup/ changes — px4_msgs is pinned by
#    branch so it stays deterministic across fleet rebuilds.
RUN mkdir -p /root/skybeta_ws/src
COPY src/oakd_vslam_bringup /root/skybeta_ws/src/oakd_vslam_bringup
RUN cd /root/skybeta_ws/src \
 && git clone --depth 1 -b release/1.14 https://github.com/PX4/px4_msgs.git \
 && cd /root/skybeta_ws \
 && source /opt/ros/humble/setup.bash \
 && colcon build --symlink-install --event-handlers console_direct+

# 6. Container-side helper scripts on $PATH.
COPY container_scripts/ /usr/local/bin/
RUN chmod +x /usr/local/bin/run_agent.sh /usr/local/bin/run_slam.sh \
             /usr/local/bin/run_bridge.sh /usr/local/bin/run_full_stack.sh \
             /usr/local/bin/check_topics.sh

# 7. Auto-source ROS + workspace for interactive shells.
RUN echo 'source /opt/ros/humble/setup.bash'                       >> /root/.bashrc \
 && echo 'source /root/skybeta_ws/install/local_setup.bash'        >> /root/.bashrc

ENV XRCE_DEV=/dev/ttyTHS1 \
    XRCE_BAUD=921600

WORKDIR /root/skybeta_ws
CMD ["/bin/bash"]
