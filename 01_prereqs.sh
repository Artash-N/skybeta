#!/usr/bin/env bash
# Host prerequisites for the skybeta Pixhawk bridge container.
# Idempotent: safe to re-run.
#
# Mirrors isaac_ros_slam/01_prereqs.sh, plus the Pixhawk-UART bits skybeta
# needs (device-tree-compiler for the DMA fix, jq for the daemon.json patch).
#
# The container base is nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble — public
# on NGC, no docker login required.
set -euo pipefail

echo "[01] apt prerequisites"
# Docker itself is preinstalled on JetPack 6.x — don't install docker.io here,
# it conflicts with the JetPack-shipped containerd if download.docker.com is
# also configured as a repo on this host.
sudo apt-get update
sudo apt-get install -y \
  git curl wget jq \
  python3-pip \
  udev usbutils \
  device-tree-compiler \
  nvidia-container-toolkit

echo "[01] OAK-D udev rules (Luxonis DepthAI)"
UDEV_RULE='SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"'
if ! grep -qs "03e7" /etc/udev/rules.d/80-movidius.rules 2>/dev/null; then
  echo "${UDEV_RULE}" | sudo tee /etc/udev/rules.d/80-movidius.rules >/dev/null
  sudo udevadm control --reload-rules && sudo udevadm trigger
  echo "  -> udev rule installed; replug the OAK-D camera."
else
  echo "  -> udev rule already present."
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "[01] ERROR: docker not found. JetPack normally ships it — install manually:" >&2
  echo "       sudo apt-get install -y nvidia-docker2" >&2
  exit 1
fi

echo "[01] Docker: ensure default-runtime=nvidia (matches isaac_ros_slam setup)"
DAEMON=/etc/docker/daemon.json
TMP=$(mktemp)
if [[ -f "${DAEMON}" ]]; then
  sudo cp "${DAEMON}" "${DAEMON}.bak.$(date +%s)"
  sudo jq '. + {"default-runtime":"nvidia"} | .runtimes.nvidia //= {"path":"nvidia-container-runtime","args":[]}' "${DAEMON}" > "${TMP}"
else
  cat > "${TMP}" <<'JSON'
{
  "default-runtime": "nvidia",
  "runtimes": { "nvidia": { "path": "nvidia-container-runtime", "args": [] } }
}
JSON
fi
if ! sudo diff -q "${DAEMON}" "${TMP}" >/dev/null 2>&1; then
  sudo mv "${TMP}" "${DAEMON}"
  echo "  -> /etc/docker/daemon.json updated; restarting docker"
  sudo systemctl restart docker
else
  rm -f "${TMP}"
  echo "  -> daemon.json already correct"
fi

echo "[01] Adding $USER to the docker group (re-login required to take effect)"
sudo usermod -aG docker "$USER" || true

echo "[01] Done. Next: ./02_uart_fix.sh (REBOOTS THE JETSON), then ./03_build_image.sh"
