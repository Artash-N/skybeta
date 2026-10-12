#!/usr/bin/env bash
# Inside the skybeta container -- record the flight-debug topic set to an mcap
# bag: camera calibration, the IMU, cuVSLAM's odometry and tracking status,
# what the bridge sends PX4, every /fmu topic on the graph, and the competitor
# setpoints and state.
#
# CAMERA IMAGES ARE NOT IN THE DEFAULT SET. Recording /oak/{left,right}/image_raw
# (1024x640 bgr8 @30) writes ~36 MB/s -- measured 1.1 GB in 30 s -- and that
# write competes with the VIO pipeline: the camera fell to 19.6 Hz and cuVSLAM
# odometry to 11.3 Hz while it ran. Ask for them explicitly, and prefer mono8.
#
#   record_bag.sh                       light set, record until Ctrl-C
#   record_bag.sh --duration 60         light set, record 60 s and stop
#   record_bag.sh --images              light set + the mono8 image pair
#   record_bag.sh --images=mono         same (mono8, ~1/3 the bytes of bgr8)
#   record_bag.sh --images=raw          light set + the bgr8 image pair
#   record_bag.sh --images=both         light set + mono8 and bgr8
#   record_bag.sh --all                 every topic on the graph (images too)
#   record_bag.sh -o /bags/square-01    explicit output directory
#   record_bag.sh -- --compression-mode file --compression-format zstd
#
# Output goes to ${BAG_DIR:-/root/bags}/<timestamp> (host/run.sh bag mounts the
# repo's bags/ at /bags and sets BAGDIR, which is honoured too). Bind-mount it
# or the recording dies with the container.
set -euo pipefail

BAG_DIR="${BAG_DIR:-${BAGDIR:-/root/bags}}"
DURATION=""
RECORD_ALL=0
IMAGES="none"            # none | mono | raw | both
OUTPUT=""
EXTRA=()

# Print the comment header above, whatever length it grows to.
usage() { sed -n '2,/^set -euo/p' "${BASH_SOURCE[0]}" | grep '^#' | sed 's/^# \{0,1\}//'; }

images_mode() {
  case "$1" in
    mono|raw|both) IMAGES="$1" ;;
    *) echo "record_bag.sh: --images takes mono, raw or both (got '$1')" >&2; exit 2 ;;
  esac
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -d|--duration) DURATION="${2:-}"; shift 2 ;;
    -a|--all)      RECORD_ALL=1; shift ;;
    -o|--output)   OUTPUT="${2:-}"; shift 2 ;;
    --images=*)    images_mode "${1#--images=}"; shift ;;
    --images)
      # `--images mono` as well as a bare `--images`, which means mono8 --
      # the cheap pair. Only swallow the next argument if it is a mode.
      if [[ "${2:-}" =~ ^(mono|raw|both)$ ]]; then images_mode "$2"; shift 2
      else IMAGES="mono"; shift; fi ;;
    -h|--help)     usage; exit 0 ;;
    --)            shift; EXTRA=("$@"); break ;;
    *)             echo "record_bag.sh: unknown argument '$1'" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -n "${DURATION}" && ! "${DURATION}" =~ ^[0-9]+$ ]]; then
  echo "record_bag.sh: --duration takes whole seconds, got '${DURATION}'" >&2
  exit 2
fi

# ROS 2 setup scripts touch unset vars (AMENT_TRACE_SETUP_FILES etc.);
# relax nounset just for the source then re-enable.
set +u
source /opt/ros/humble/setup.bash
source /root/skybeta_ws/install/local_setup.bash
set -u

BAG_PATH="${OUTPUT:-${BAG_DIR}/$(date +%Y%m%d-%H%M%S)}"
mkdir -p "$(dirname "${BAG_PATH}")"

image_warning() {
  cat >&2 <<'EOM'
[bag] ###########################################################################
[bag] WARNING: this recording includes camera images.
[bag]   /oak/{left,right}/image_raw is 1024x640 bgr8 at 30 Hz. Measured on this
[bag]   rig: 1.1 GB written in 30 s (~36 MB/s), and the write competed with the
[bag]   pipeline -- camera dropped to 19.6 Hz, cuVSLAM odometry to 11.3 Hz.
[bag]   Any rate you measure from this bag is a rate under recording load.
[bag]   Use --images=mono (mono8, ~1/3 the bytes) when you can, keep the run
[bag]   short, and do not do this on a flight whose timing you care about.
[bag] ###########################################################################
EOM
}

# NB: rosbag2's own -d is --max-bag-duration (it SPLITS the bag every N
# seconds and keeps recording), so --duration is implemented below with a
# SIGINT timer instead -- SIGINT is rosbag2's clean shutdown path.
CMD=(ros2 bag record -s mcap -o "${BAG_PATH}")

if [[ ${RECORD_ALL} -eq 1 ]]; then
  CMD+=(-a)
  echo "[bag] recording ALL topics"
  image_warning
else
  # The light set: everything needed to explain a flight, minus the images.
  # cuVSLAM's odometry, its tracking status, the calibration the tracker used,
  # the IMU, and what the bridge handed PX4. Under 100 kB/s all together.
  TOPICS=(
    /oak/left/camera_info
    /oak/right/camera_info
    /oak/imu/data
    /visual_slam/tracking/odometry
    /visual_slam/status
    /fmu/in/vehicle_visual_odometry
  )

  case "${IMAGES}" in
    mono) TOPICS+=(/oak/left/image_mono /oak/right/image_mono) ;;
    raw)  TOPICS+=(/oak/left/image_raw  /oak/right/image_raw) ;;
    both) TOPICS+=(/oak/left/image_mono /oak/right/image_mono
                   /oak/left/image_raw  /oak/right/image_raw) ;;
  esac

  # Then whatever PX4 and the flight interface are actually publishing. rosbag2
  # keeps discovering, so a name that is not up yet is picked up when it
  # appears -- but it can only be *named* here if it already exists, hence the
  # fixed list above for the ones that must never be missed.
  while IFS= read -r t; do
    [[ -n "${t}" ]] || continue
    for known in "${TOPICS[@]}"; do
      if [[ "${known}" == "${t}" ]]; then continue 2; fi
    done
    TOPICS+=("${t}")
  done < <(ros2 topic list 2>/dev/null | grep -E '^/(fmu|skybeta)/' || true)

  if [[ "${IMAGES}" != "none" ]]; then image_warning; fi

  echo "[bag] recording ${#TOPICS[@]} topics (images: ${IMAGES}):"
  printf '        %s\n' "${TOPICS[@]}"
  CMD+=("${TOPICS[@]}")
fi

if [[ ${#EXTRA[@]} -gt 0 ]]; then
  CMD+=("${EXTRA[@]}")
fi

echo "[bag] ${CMD[*]}"
[[ -n "${DURATION}" ]] && echo "[bag] stopping after ${DURATION}s" || echo "[bag] Ctrl-C to stop"

# Ctrl-C is the documented way to stop an open-ended recording: catch it here
# so the summary below still prints, and do not report it as a failure. A
# trapped (not ignored) SIGINT is still delivered normally to `ros2 bag`.
trap ':' INT
rc=0
if [[ -n "${DURATION}" ]]; then
  "${CMD[@]}" &
  rec_pid=$!
  ( sleep "${DURATION}"; kill -INT "${rec_pid}" 2>/dev/null || true ) &
  timer_pid=$!
  wait "${rec_pid}" || rc=$?
  kill "${timer_pid}" 2>/dev/null || true
  wait "${timer_pid}" 2>/dev/null || true
else
  "${CMD[@]}" || rc=$?
fi
trap - INT
if [[ ${rc} -eq 130 || ${rc} -eq 2 ]]; then rc=0; fi   # SIGINT / rclpy KeyboardInterrupt

echo
if [[ -d "${BAG_PATH}" ]]; then
  echo "[bag] wrote ${BAG_PATH}  ($(du -sh "${BAG_PATH}" 2>/dev/null | cut -f1))"
  ls -1 "${BAG_PATH}" | sed 's/^/        /'
  echo "[bag] inspect with:  ros2 bag info ${BAG_PATH}"
else
  echo "[bag] no bag was written at ${BAG_PATH} (rc=${rc})" >&2
fi
exit "${rc}"
