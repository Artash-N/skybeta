#!/usr/bin/env bash
# Section 1 of jetson_pixhawk_uxrce_ros2_setup.md — patch the Jetson Orin
# UART DMA/IOMMU issue on R36.5. Wraps jetson_uart_dma_fix.sh and reboots.
#
# REBOOTS THE JETSON. Run this before plugging Pixhawk into TELEM2 the first
# time, then re-run from step 03 after the reboot.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
FIX="${HERE}/jetson_uart_dma_fix.sh"

if [[ ! -x "${FIX}" ]]; then
  chmod +x "${FIX}"
fi

echo "[02] Applying UART DMA/IOMMU patch (sudo + reboot)"
sudo "${FIX}" apply

echo "[02] Patch staged. Rebooting in 5s — Ctrl-C to abort..."
sleep 5
sudo reboot
