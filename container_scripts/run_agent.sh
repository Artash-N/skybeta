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
# The agent does NOT read ROS_DOMAIN_ID: PX4's client tells it which domain to
# create its participant in, from UXRCE_DDS_DOM_ID on the FC. That parameter must
# therefore equal ROS_DOMAIN_ID here -- this drone's own domain (drone.env), not 0.
# The localhost-only Fast DDS profile (FASTRTPS_DEFAULT_PROFILES_FILE, set by
# host/run.sh) is the default participant profile, so it should bind the agent's
# participant to loopback too, keeping /fmu/* off the WiFi. Proven for rmw nodes;
# for the agent, confirm on the bench (tcpdump on wlan0 while the stack runs).
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

# Flight-critical path: the skybeta underlay only, never the team overlay, so a
# team package shadowing an underlay one cannot reach the VIO -> FC pipeline.
# skybeta_env.sh handles the nounset dance the ROS setup scripts need.
SKYBETA_UNDERLAY_ONLY=1
# shellcheck source=container_scripts/skybeta_env.sh
source skybeta_env.sh

echo "[agent] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} -- the FC's UXRCE_DDS_DOM_ID must match;" \
     "DDS profile: ${FASTRTPS_DEFAULT_PROFILES_FILE:-<none: all interfaces>}"

if [[ "${TRANSPORT}" == "udp4" ]]; then
  echo "[agent] MicroXRCEAgent udp4 -p ${UDP_PORT}"
  exec MicroXRCEAgent udp4 -p "${UDP_PORT}" "${VERB_ARG[@]}"
fi

echo "[agent] MicroXRCEAgent serial --dev ${DEV} -b ${BAUD}"
exec MicroXRCEAgent serial --dev "${DEV}" -b "${BAUD}" "${VERB_ARG[@]}"
