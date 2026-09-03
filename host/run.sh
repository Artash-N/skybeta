#!/usr/bin/env bash
# skybeta container launcher -- replaces 04_run_container.sh and run_pipeline.sh.
#
#   host/run.sh stack [cx cy cz [roll pitch yaw]]   full stack in a tmux session
#   host/run.sh shell                               interactive bash (or exec into a running one)
#   host/run.sh bag [args...]                       rosbag2 recording (container_scripts/record_bag.sh)
#   host/run.sh sim                                 Isaac Sim profile -- not yet implemented
#
# Camera pose is the base_link -> oak_parent_frame static TF: translation in
# metres, rotation in radians, FLU body frame. Positional args win over the
# CAMERA_* environment variables.
#
#   host/run.sh stack                          camera at body origin
#   host/run.sh stack 0.10 0 -0.02             translation only
#   host/run.sh stack 0.10 0 -0.02 0 0.35 0    plus a 20 deg down-pitch
#   CAMERA_X=0.10 CAMERA_PITCH=0.35 host/run.sh stack
#
# Image tag comes from the VERSION file at the repo root (falling back to
# 'latest'); override the whole reference with IMAGE=ghcr.io/.../skybeta:0.1.0.
#
# No sudo. The `sudo systemctl stop nvgetty` and `sudo chmod 666 $DEV` lines the
# old scripts carried are no-ops on this image (nvgetty.service does not exist)
# and unnecessary for a user in `dialout` -- see CLAUDE.md.
set -Eeuo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${HERE}/.." && pwd)"
DOCKERFILE="${REPO}/docker/Dockerfile.jetson"

TAG="$(cat "${REPO}/VERSION" 2>/dev/null | tr -d '[:space:]')"
[[ -n "${TAG}" ]] || TAG="latest"
IMAGE="${IMAGE:-skybeta:${TAG}}"
DEV="${DEV:-/dev/ttyTHS1}"

log()  { printf '[run] %s\n' "$*"; }
warn() { printf '[run] WARNING: %s\n' "$*" >&2; }
die()  { printf '[run] ERROR: %s\n' "$*" >&2; exit 1; }
 usage() { sed -n '2,23p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

MODE="${1:-}"
[[ $# -gt 0 ]] && shift || true
case "${MODE}" in
  stack|shell|bag|sim) ;;
  -h|--help|'') usage; exit 0 ;;
  *) usage >&2; die "unknown mode: ${MODE}" ;;
esac

# --- sim is a stub for now -------------------------------------------------
if [[ "${MODE}" == "sim" ]]; then
  cat <<'EOM'
[run] sim: not yet implemented -- see sim/

  The Isaac Sim profile (Isaac Sim 5.1.0 + Pegasus v5.1.0 + PX4 SITL v1.16.2,
  x86_64 only) is WP7. It will live in sim/ and build from docker/Dockerfile.sim.
  This Jetson cannot run it: there is no aarch64 Isaac Sim.
EOM
  exit 0
fi

# --- image -----------------------------------------------------------------
build_hint() {
  if [[ -f "${DOCKERFILE}" ]]; then
    printf 'Build it locally:  host/build_image.sh   (docker build -f %s -t %s %s)' \
      "${DOCKERFILE#"${REPO}/"}" "${IMAGE}" "${REPO}"
  else
    printf '%s is missing too -- WP0 has not landed; there is nothing to build from.' "${DOCKERFILE}"
  fi
}

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  PULLER="${HERE}/pull_image.sh"
  # pull_image.sh takes the bare tag and re-tags the result to skybeta:<tag>
  # locally, which is exactly the reference ${IMAGE} defaults to.
  PULL_TAG="${IMAGE##*:}"
  if [[ ! -f "${PULLER}" ]]; then
    die "image '${IMAGE}' not present and ${PULLER} does not exist yet (WP0 owns it).
       $(build_hint)"
  elif [[ "${PULL_TAG}" == "latest" ]]; then
    die "image '${IMAGE}' not present, and pull_image.sh refuses 'latest' on purpose -- a
       fleet of 10+ drones must run a byte-identical, pinned tag. Put a version in
       ${REPO}/VERSION, or pass IMAGE=skybeta:<tag>.
       $(build_hint)"
  else
    log "image ${IMAGE} not present locally -- pulling tag ${PULL_TAG} via host/pull_image.sh"
    bash "${PULLER}" "${PULL_TAG}" || die "pull of tag '${PULL_TAG}' failed (is REGISTRY set?).
       $(build_hint)"
  fi
  docker image inspect "${IMAGE}" >/dev/null 2>&1 || die "image '${IMAGE}' still not present after the pull"
fi

# --- one UART, one owner ---------------------------------------------------
# Two MicroXRCEAgents on the same serial port silently corrupt each other's
# framing, so refuse rather than let it happen. Checks the actual device
# passthrough of every running container, not just our own container names.
uart_holder() {
  local c devs
  for c in $(docker ps -q 2>/dev/null); do
    devs="$(docker inspect -f '{{range .HostConfig.Devices}}{{.PathOnHost}} {{end}}' "$c" 2>/dev/null || true)"
    if [[ " ${devs} " == *" ${DEV} "* ]]; then
      docker inspect -f '{{.Name}}' "$c" 2>/dev/null | sed 's|^/||'
      return 0
    fi
  done
  return 1
}

# --- shared docker run arguments ------------------------------------------
DEVICE_ARG=()
if [[ -e "${DEV}" ]]; then
  DEVICE_ARG=(--device "${DEV}:${DEV}")
else
  warn "${DEV} not present -- the uXRCE agent will have nothing to talk to. Did the UART patch reboot happen? (host/doctor.sh)"
fi

# X11 so rviz2 / GUI tools inside the container can render on the host display.
xhost +local:root >/dev/null 2>&1 || true
X11_ARG=()
if [[ -n "${DISPLAY:-}" ]]; then
  X11_ARG+=(-e "DISPLAY=${DISPLAY}" -v /tmp/.X11-unix:/tmp/.X11-unix)
  [[ -n "${XAUTHORITY:-}" ]] && X11_ARG+=(-v "${XAUTHORITY}:${XAUTHORITY}" -e "XAUTHORITY=${XAUTHORITY}")
fi

COMMON_ARG=(
  --privileged
  --runtime nvidia
  --network host
  --ipc host
  -v /dev:/dev
  -v /dev/bus/usb:/dev/bus/usb
)

# --- camera pose -----------------------------------------------------------
CAMERA_X="${1:-${CAMERA_X:-0.0}}"
CAMERA_Y="${2:-${CAMERA_Y:-0.0}}"
CAMERA_Z="${3:-${CAMERA_Z:-0.0}}"
CAMERA_ROLL="${4:-${CAMERA_ROLL:-0.0}}"
CAMERA_PITCH="${5:-${CAMERA_PITCH:-0.0}}"
CAMERA_YAW="${6:-${CAMERA_YAW:-0.0}}"

CAMERA_ENV=(
  -e "CAMERA_X=${CAMERA_X}"
  -e "CAMERA_Y=${CAMERA_Y}"
  -e "CAMERA_Z=${CAMERA_Z}"
  -e "CAMERA_ROLL=${CAMERA_ROLL}"
  -e "CAMERA_PITCH=${CAMERA_PITCH}"
  -e "CAMERA_YAW=${CAMERA_YAW}"
)

case "${MODE}" in

  # -----------------------------------------------------------------------
  shell)
    NAME="${NAME:-skybeta}"
    if docker ps --format '{{.Names}}' | grep -qx "${NAME}"; then
      log "attaching a new shell to the running container '${NAME}'"
      exec docker exec -it "${NAME}" /bin/bash
    fi
    if holder="$(uart_holder)"; then
      die "container '${holder}' already holds ${DEV}. Attach to it (host/run.sh shell) or stop it: docker rm -f ${holder}"
    fi
    docker rm -f "${NAME}" >/dev/null 2>&1 || true
    log "starting '${NAME}' from ${IMAGE} (interactive)"
    exec docker run -it --rm \
      --name "${NAME}" \
      "${COMMON_ARG[@]}" \
      "${DEVICE_ARG[@]}" \
      "${X11_ARG[@]}" \
      -e "XRCE_DEV=${DEV}" \
      "${CAMERA_ENV[@]}" \
      "${IMAGE}" \
      /bin/bash
    ;;

  # -----------------------------------------------------------------------
  stack)
    NAME="${NAME:-skybeta-stack}"
    if holder="$(uart_holder)"; then
      die "container '${holder}' already holds ${DEV} -- two MicroXRCEAgents on one UART will corrupt each other. Stop it first: docker rm -f ${holder}"
    fi
    docker rm -f "${NAME}" >/dev/null 2>&1 || true

    cat <<EOM
[run] full stack
        image        ${IMAGE}
        container    ${NAME}
        UART         ${DEV} @ ${XRCE_BAUD:-921600}
        camera FLU   xyz = ${CAMERA_X} ${CAMERA_Y} ${CAMERA_Z} m
                     rpy = ${CAMERA_ROLL} ${CAMERA_PITCH} ${CAMERA_YAW} rad

      Flight-controller prerequisites (QGroundControl, then reboot the FC):
        MAV_1_CONFIG=0   UXRCE_DDS_CFG=102 (TELEM2)   SER_TEL2_BAUD=921600
        UXRCE_DDS_DOM_ID = ROS_DOMAIN_ID on this Jetson
        EKF2_EV_CTRL=9 (horiz pos + yaw)   EKF2_HGT_REF=0 (Baro)   EKF2_BARO_CTRL=1
        EKF2_EV_DELAY = measured, not assumed   EKF2_EVP/EVV_NOISE=0.1  EKF2_EVA_NOISE=0.05
      See CLAUDE.md "Target PX4 parameter sets" -- MAV_1_CONFIG and UXRCE_DDS_CFG
      must never both point at 102.

EOM
    exec docker run -it --rm \
      --name "${NAME}" \
      "${COMMON_ARG[@]}" \
      "${DEVICE_ARG[@]}" \
      "${X11_ARG[@]}" \
      -e "XRCE_DEV=${DEV}" \
      -e "XRCE_BAUD=${XRCE_BAUD:-921600}" \
      "${CAMERA_ENV[@]}" \
      "${IMAGE}" \
      run_full_stack.sh
    ;;

  # -----------------------------------------------------------------------
  bag)
    NAME="${NAME:-skybeta-bag}"
    BAGDIR="${BAGDIR:-${REPO}/bags}"
    mkdir -p "${BAGDIR}"
    docker rm -f "${NAME}" >/dev/null 2>&1 || true
    log "recording into ${BAGDIR} (mounted at /bags)"
    # No --device: recording joins the running stack over --network host DDS,
    # so it neither needs nor may take the UART.
    exec docker run -it --rm \
      --name "${NAME}" \
      "${COMMON_ARG[@]}" \
      -v "${BAGDIR}:/bags" \
      -e "BAGDIR=/bags" \
      "${IMAGE}" \
      bash -lc 'command -v record_bag.sh >/dev/null || {
                  echo "[run] record_bag.sh is not in this image." >&2
                  echo "      container_scripts/record_bag.sh is WP6 and has not landed yet." >&2
                  echo "      Meanwhile, from inside host/run.sh shell:" >&2
                  echo "        ros2 bag record -s mcap -o /bags/\$(date +%Y%m%d-%H%M%S) \\\\" >&2
                  echo "          /oak/left/image_raw /oak/right/image_raw /oak/imu/data \\\\" >&2
                  echo "          /visual_slam/tracking/odometry /fmu/in/vehicle_visual_odometry" >&2
                  exit 127; }
                exec record_bag.sh "$@"' _ "$@"
    ;;
esac
