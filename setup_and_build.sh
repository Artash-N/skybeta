#!/usr/bin/env bash
# One-shot host setup that runs every host-side command from
# jetson_pixhawk_uxrce_ros2_setup.md and then builds the skybeta docker image
# (cuVSLAM + MicroXRCE-DDS-Agent + uXRCE-DDS bridge in one container).
#
# First build pulls ~8 GB of NGC base + apt + colcon → 30–40 min on Orin Nano.
# Re-runs are cached.
#
# Idempotent. Steps that need a reboot (UART DMA fix) are skipped if already
# applied; pass --skip-uart-fix to opt out entirely.
#
# Usage:
#   ./setup_and_build.sh              # full flow, reboots if UART patch needed
#   ./setup_and_build.sh --skip-uart-fix
#   ./setup_and_build.sh --no-reboot  # apply patch but don't reboot at the end
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
SKIP_UART_FIX=0
NO_REBOOT=0

for arg in "$@"; do
  case "$arg" in
    --skip-uart-fix) SKIP_UART_FIX=1 ;;
    --no-reboot)     NO_REBOOT=1 ;;
    *) echo "unknown flag: $arg" >&2; exit 2 ;;
  esac
done

echo "[setup] Step 1/3 — host prerequisites"
"${HERE}/01_prereqs.sh"

if [[ "${SKIP_UART_FIX}" -eq 0 ]]; then
  echo "[setup] Step 2/3 — UART DMA/IOMMU patch"
  # If the patched DTB is already in place, the script is a no-op; skip the
  # reboot in that case.
  if [[ -f /boot/dtb/uart_dma_fix.dtb ]] \
       && grep -q "uart_dma_fix.dtb" /boot/extlinux/extlinux.conf 2>/dev/null; then
    echo "[setup]   -> patched DTB already installed; skipping reboot"
  else
    chmod +x "${HERE}/jetson_uart_dma_fix.sh"
    sudo "${HERE}/jetson_uart_dma_fix.sh" apply
    if [[ "${NO_REBOOT}" -eq 0 ]]; then
      echo "[setup]   -> rebooting in 5s. Re-run this script after reboot to build the image."
      sleep 5
      sudo reboot
      exit 0
    else
      echo "[setup]   -> patch staged; reboot manually before running the container."
    fi
  fi
else
  echo "[setup] Step 2/3 — UART patch skipped (--skip-uart-fix)"
fi

echo "[setup] Step 3/3 — build docker image"
"${HERE}/03_build_image.sh"

cat <<EOM

[setup] Done.

Next steps:
  1. Wire Pixhawk TELEM2 to Jetson 40-pin (pin 8/10/6 = TX/RX/GND, no 5V).
     Pixhawk TX -> Jetson RX, Pixhawk RX -> Jetson TX.
  2. Plug the OAK-D-S2 into a Jetson USB3 port.
  3. In QGroundControl set:
       MAV_1_CONFIG    = 0
       UXRCE_DDS_CFG   = TELEM2
       SER_TEL2_BAUD   = 921600
       EKF2_EV_CTRL    = bits 0+3   (horiz pos + yaw)
       EKF2_HGT_REF    = Baro
       EKF2_EV_DELAY   = 50
       EKF2_EVP_NOISE  = 0.1   EKF2_EVV_NOISE = 0.1   EKF2_EVA_NOISE = 0.05
     and reboot the Pixhawk.
  4. Run the full stack:         ./run_pipeline.sh 0.10 0.0 -0.02   (camera FLU offsets)
     OR drop into the container: ./04_run_container.sh             (interactive)
        and inside, run any of:  run_agent.sh / run_slam.sh / run_bridge.sh / check_topics.sh
EOM
