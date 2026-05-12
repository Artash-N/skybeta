#!/usr/bin/env bash
# Launch the skybeta container with serial passthrough so MicroXRCEAgent can
# reach Pixhawk over /dev/ttyTHS1. Uses --network host so DDS discovery works
# with any other ROS 2 nodes on the Jetson (matches isaac_ros_slam's pattern).
#
# Re-running attaches a fresh shell to the existing container if one is up;
# otherwise starts a new one.
set -euo pipefail

IMAGE="${IMAGE:-skybeta:latest}"
NAME="${NAME:-skybeta}"
DEV="${DEV:-/dev/ttyTHS1}"

# Make sure nvgetty isn't holding the UART (matches isaac_ros_slam 06_run_px4_agent.sh).
sudo systemctl stop nvgetty.service 2>/dev/null || true

if [[ -e "${DEV}" && ! -w "${DEV}" ]]; then
  echo "[04] Granting RW on ${DEV} (sudo)"
  sudo chmod 666 "${DEV}"
fi

if docker ps --format '{{.Names}}' | grep -qx "${NAME}"; then
  echo "[04] Attaching new shell to running container '${NAME}'"
  exec docker exec -it "${NAME}" /bin/bash
fi

# Remove any stopped container with the same name.
docker rm -f "${NAME}" >/dev/null 2>&1 || true

DEVICE_ARG=()
if [[ -e "${DEV}" ]]; then
  DEVICE_ARG=(--device "${DEV}:${DEV}")
else
  echo "[04] WARNING: ${DEV} not present — Pixhawk may not be wired or UART fix not applied."
fi

# X11 forwarding so rviz2 / GUI tools can render to the host display.
xhost +local:root >/dev/null 2>&1 || true
X11_ARG=()
if [[ -n "${DISPLAY:-}" ]]; then
  X11_ARG+=(-e "DISPLAY=${DISPLAY}" -v /tmp/.X11-unix:/tmp/.X11-unix)
  [[ -n "${XAUTHORITY:-}" ]] && X11_ARG+=(-v "${XAUTHORITY}:${XAUTHORITY}" -e "XAUTHORITY=${XAUTHORITY}")
fi

echo "[04] Starting container '${NAME}' from ${IMAGE}"
exec docker run -it --rm \
  --name "${NAME}" \
  --privileged \
  --runtime nvidia \
  --network host \
  --ipc host \
  -v /dev:/dev \
  -v /dev/bus/usb:/dev/bus/usb \
  "${DEVICE_ARG[@]}" \
  "${X11_ARG[@]}" \
  -e XRCE_DEV="${DEV}" \
  "${IMAGE}" \
  /bin/bash
