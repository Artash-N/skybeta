#!/usr/bin/env bash
# Host-side one-shot: bring up the entire skybeta stack — host UART + OAK-D +
# cuVSLAM + uXRCE-DDS bridge + MicroXRCEAgent — inside one container with a
# tmux session of three windows (agent, slam, bridge).
#
# Usage:
#   ./run_pipeline.sh                       # camera at body origin (FLU)
#   ./run_pipeline.sh 0.10 0.0 -0.02        # camera_x camera_y camera_z (m)
#   CAMERA_X=0.10 CAMERA_Y=0 CAMERA_Z=-0.02 ./run_pipeline.sh
#
# Stop with Ctrl-D inside the container, or `tmux kill-session -t skybeta`.
set -euo pipefail

IMAGE="${IMAGE:-skybeta:latest}"
NAME="${NAME:-skybeta-pipeline}"
DEV="${DEV:-/dev/ttyTHS1}"

CAMERA_X="${1:-${CAMERA_X:-0.0}}"
CAMERA_Y="${2:-${CAMERA_Y:-0.0}}"
CAMERA_Z="${3:-${CAMERA_Z:-0.0}}"

# Pre-flight ------------------------------------------------------------------

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "[run] image '${IMAGE}' not found — run ./03_build_image.sh first." >&2
  exit 1
fi

if ! lsusb 2>/dev/null | grep -q '03e7'; then
  echo "[run] WARNING: OAK-D (USB 03e7:*) not enumerated — cuVSLAM will fail to init."
fi

if [[ ! -e "${DEV}" ]]; then
  echo "[run] WARNING: ${DEV} not present — agent will fail. Did you run 02_uart_fix.sh + reboot?"
fi

# Free the UART from nvgetty and grant RW.
sudo systemctl stop nvgetty.service 2>/dev/null || true
[[ -e "${DEV}" && ! -w "${DEV}" ]] && sudo chmod 666 "${DEV}"

# Refuse to clash with the interactive container from 04_run_container.sh
# (would put two MicroXRCEAgents on the same UART).
if docker ps --format '{{.Names}}' | grep -qx 'skybeta'; then
  echo "[run] ERROR: container 'skybeta' already running (from 04_run_container.sh)." >&2
  echo "       Stop it first: docker rm -f skybeta" >&2
  exit 1
fi

# X11 (so rviz2 inside the SLAM window can render).
xhost +local:root >/dev/null 2>&1 || true
X11_ARG=()
if [[ -n "${DISPLAY:-}" ]]; then
  X11_ARG+=(-e "DISPLAY=${DISPLAY}" -v /tmp/.X11-unix:/tmp/.X11-unix)
  [[ -n "${XAUTHORITY:-}" ]] && X11_ARG+=(-v "${XAUTHORITY}:${XAUTHORITY}" -e "XAUTHORITY=${XAUTHORITY}")
fi

DEVICE_ARG=()
[[ -e "${DEV}" ]] && DEVICE_ARG=(--device "${DEV}:${DEV}")

# Run -------------------------------------------------------------------------

cat <<EOM
[run] Starting full stack:
        image       ${IMAGE}
        container   ${NAME}
        UART        ${DEV}
        camera FLU  x=${CAMERA_X} y=${CAMERA_Y} z=${CAMERA_Z}

      Pixhawk-side prereqs (set in QGroundControl, then reboot the Pixhawk):
        MAV_1_CONFIG=0  UXRCE_DDS_CFG=TELEM2  SER_TEL2_BAUD=921600
        EKF2_EV_CTRL: bits 0+3 (horiz pos + yaw)  EKF2_HGT_REF=Baro
        EKF2_EV_DELAY=50  EKF2_EVP_NOISE=0.1  EKF2_EVV_NOISE=0.1  EKF2_EVA_NOISE=0.05

EOM

docker rm -f "${NAME}" >/dev/null 2>&1 || true

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
  -e CAMERA_X="${CAMERA_X}" \
  -e CAMERA_Y="${CAMERA_Y}" \
  -e CAMERA_Z="${CAMERA_Z}" \
  "${IMAGE}" \
  run_full_stack.sh
