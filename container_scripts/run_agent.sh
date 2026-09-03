#!/usr/bin/env bash
# Inside the skybeta container -- start MicroXRCEAgent.
#
# Two transports, selected by XRCE_TRANSPORT:
#
#   serial  (default)  the UART the host passed through with --device.
#                      XRCE_DEV=/dev/ttyTHS1  XRCE_BAUD=921600
#                      PX4 side: UXRCE_DDS_CFG=102 (TELEM2), SER_TEL2_BAUD=921600,
#                      MAV_1_CONFIG=0. One port, one owner -- see CLAUDE.md.
#
#   udp4               PX4 SITL / Isaac Sim profile: the sim's uxrce_dds_client
#                      dials in over UDP. XRCE_UDP_PORT=8888.
#
# UXRCE_DDS_DOM_ID on the FC must equal ROS_DOMAIN_ID here (both default 0).
set -euo pipefail

TRANSPORT="${XRCE_TRANSPORT:-serial}"
DEV="${XRCE_DEV:-/dev/ttyTHS1}"
BAUD="${XRCE_BAUD:-921600}"
UDP_PORT="${XRCE_UDP_PORT:-8888}"
VERBOSITY="${XRCE_VERBOSITY:-}"   # e.g. 6 for full session logging

# NB: `[[ cond ]] && cmd` as a bare statement makes the script exit 1 under
# `set -e` when the condition is false -- the same trap as bug #1. Use if.
VERB_ARG=()
if [[ -n "${VERBOSITY}" ]]; then
  VERB_ARG=(-v "${VERBOSITY}")
fi

case "${TRANSPORT}" in
  serial|udp4) ;;
  *) echo "error: XRCE_TRANSPORT='${TRANSPORT}' -- expected 'serial' or 'udp4'." >&2; exit 2 ;;
esac

if [[ "${TRANSPORT}" == "serial" && ! -e "${DEV}" ]]; then
  echo "error: ${DEV} not visible inside the container." >&2
  echo "       Pass --device ${DEV} to docker run -- host/run.sh does this for you." >&2
  echo "       For the sim profile use XRCE_TRANSPORT=udp4 instead." >&2
  exit 1
fi

# ROS 2 setup scripts touch unset vars (AMENT_TRACE_SETUP_FILES etc.);
# relax nounset just for the source then re-enable.
set +u
source /opt/ros/humble/setup.bash
source /root/skybeta_ws/install/local_setup.bash
set -u

if [[ "${TRANSPORT}" == "udp4" ]]; then
  echo "[agent] MicroXRCEAgent udp4 -p ${UDP_PORT}"
  exec MicroXRCEAgent udp4 -p "${UDP_PORT}" "${VERB_ARG[@]}"
fi

echo "[agent] MicroXRCEAgent serial --dev ${DEV} -b ${BAUD}"
exec MicroXRCEAgent serial --dev "${DEV}" -b "${BAUD}" "${VERB_ARG[@]}"
