#!/usr/bin/env bash
# Inside the skybeta container — start MicroXRCEAgent on the UART that the
# host passed through with --device. Default /dev/ttyTHS1 @ 921600.
set -euo pipefail

DEV="${XRCE_DEV:-/dev/ttyTHS1}"
BAUD="${XRCE_BAUD:-921600}"

if [[ ! -e "${DEV}" ]]; then
  echo "error: ${DEV} not visible inside the container." >&2
  echo "       Pass --device ${DEV} to docker run (see 04_run_container.sh)." >&2
  exit 1
fi

# ROS 2 setup scripts touch unset vars (AMENT_TRACE_SETUP_FILES etc.);
# relax nounset just for the source then re-enable.
set +u
source /opt/ros/humble/setup.bash
source /root/skybeta_ws/install/local_setup.bash
set -u

echo "[agent] MicroXRCEAgent serial --dev ${DEV} -b ${BAUD}"
exec MicroXRCEAgent serial --dev "${DEV}" -b "${BAUD}"
