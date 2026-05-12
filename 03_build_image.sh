#!/usr/bin/env bash
# Build the skybeta docker image. First run pulls
# nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble (~8 GB), apt-installs cuVSLAM /
# NITROS / depthai-ros, builds Micro-XRCE-DDS-Agent v2.4.3, and colcon-builds
# the skybeta_ws workspace (oakd_vslam_bringup + px4_msgs @ release/1.14).
# Subsequent runs are layer-cached — no source change → seconds.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
IMAGE="${IMAGE:-skybeta:latest}"

cd "${HERE}"
echo "[03] docker build -t ${IMAGE} ."
docker build -t "${IMAGE}" .
echo "[03] Built ${IMAGE}. Next: ./04_run_container.sh"
