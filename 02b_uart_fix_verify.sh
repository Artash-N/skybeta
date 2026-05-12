#!/usr/bin/env bash
# Run AFTER the reboot from 02_uart_fix.sh. Confirms the IOMMU/DMA path is
# active for serial@3100000 — you should see "Adding to iommu group" lines.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
sudo "${HERE}/jetson_uart_dma_fix.sh" status
