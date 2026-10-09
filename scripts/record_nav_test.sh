#!/usr/bin/env bash
# Record a Nav2 driving test. This script never publishes a goal or velocity.

set -eo pipefail

DURATION_SEC="${1:-180}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
OUT_ROOT="${2:-${SCRIPT_DIR}}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_ROOT}/nav_test_${STAMP}"
BAG_DIR="${OUT_DIR}/bag"

if [[ ! "${DURATION_SEC}" =~ ^[1-9][0-9]*$ ]]; then
  echo "Usage: $0 [duration_seconds] [output_directory]" >&2
  exit 2
fi

source /opt/ros/jazzy/setup.bash
if [[ -f /home/ugv/colcon_ws/install/setup.bash ]]; then
  source /home/ugv/colcon_ws/install/setup.bash
fi
# Query the live DDS graph rather than a possibly stale ros2-cli daemon.
export ROS2CLI_NO_DAEMON=1

set -u
mkdir -p "${OUT_DIR}"
PIDS=()
BAG_PID=""
START_EPOCH="$(date +%s)"

log() {
  printf '[%s] %s\n' "$(date '+%F %T')" "$*" | tee -a "${OUT_DIR}/monitor.log"
}

snapshot() {
  local name="$1"
  shift
  timeout 12 "$@" >"${OUT_DIR}/${name}.log" 2>&1 || true
}

monitor_rate() {
  local topic="$1"
  local filename="$2"
  ros2 topic hz "${topic}" >"${OUT_DIR}/${filename}" 2>&1 &
  PIDS+=("$!")
}

finish() {
  local exit_code=$?
  trap - EXIT INT TERM
  log "Stopping monitor..."
  if [[ -n "${BAG_PID}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
    # ros2 bag can ignore SIGINT when it is a non-interactive background
    # child.  It gets its own session below, so terminate that whole session
    # and give MCAP a chance to flush its metadata.
    kill -TERM -- "-${BAG_PID}" 2>/dev/null || kill -TERM "${BAG_PID}" 2>/dev/null || true
    wait "${BAG_PID}" 2>/dev/null || true
  fi
  for pid in "${PIDS[@]:-}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  for pid in "${PIDS[@]:-}"; do
    wait "${pid}" 2>/dev/null || true
  done

  local end_epoch
  end_epoch="$(date +%s)"
  {
    echo "output_dir: ${OUT_DIR}"
    echo "bag_dir: ${BAG_DIR}"
    echo "start_epoch: ${START_EPOCH}"
    echo "end_epoch: ${end_epoch}"
    echo "elapsed_sec: $((end_epoch - START_EPOCH))"
    echo "requested_max_sec: ${DURATION_SEC}"
    echo "ros_domain_id: ${ROS_DOMAIN_ID:-unset}"
  } >"${OUT_DIR}/run_info.txt"
  if [[ -d "${BAG_DIR}" ]]; then
    ros2 bag info "${BAG_DIR}" >"${OUT_DIR}/bag_info.txt" 2>&1 || true
  fi
  log "Saved diagnostics to ${OUT_DIR}"
  exit "${exit_code}"
}
trap finish EXIT INT TERM

TOPICS=(
  /cmd_vel_nav
  /cmd_vel
  /bunker_odom
  /Odometry
  /tf
  /tf_static
  /navigate_to_pose/_action/goal
  /navigate_to_pose/_action/feedback
  /navigate_to_pose/_action/status
  /plan
  /local_plan
  /trajectories
  /cloud_registered_body
  /livox_costmap
  /local_costmap/costmap
  /global_costmap/costmap
  /local_costmap/published_footprint
)

log "This is record-only: it never publishes a goal or cmd_vel."
log "Output: ${OUT_DIR}"
log "Start this script, then manually send exactly one RViz navigation goal."

# Start rosbag first: parameter snapshots may take several seconds and must
# not create a blind period in which the operator can send a goal.
log "Starting MCAP rosbag recording for up to ${DURATION_SEC}s. Press Ctrl-C after the vehicle stops."
setsid ros2 bag record --storage mcap --include-hidden-topics --output "${BAG_DIR}" --topics "${TOPICS[@]}" >"${OUT_DIR}/rosbag.log" 2>&1 &
BAG_PID="$!"
sleep 1
if ! kill -0 "${BAG_PID}" 2>/dev/null; then
  log "ros2 bag recorder failed to start; inspect ${OUT_DIR}/rosbag.log"
  exit 1
fi
log "MCAP recording is active. It includes hidden NavigateToPose action topics."

snapshot node_list ros2 node list
snapshot topic_list ros2 topic list -t
snapshot action_info ros2 action info /navigate_to_pose
snapshot controller_params ros2 param dump /controller_server
snapshot velocity_smoother_params ros2 param dump /velocity_smoother
snapshot local_costmap_params ros2 param dump /local_costmap/local_costmap
snapshot global_costmap_params ros2 param dump /global_costmap/global_costmap

{
  for topic in "${TOPICS[@]}"; do
    echo "### ${topic}"
    ros2 topic info -v "${topic}" 2>&1 || true
  done
} >"${OUT_DIR}/topic_endpoints.txt"

(
  cd "${OUT_DIR}"
  timeout 8 ros2 run tf2_tools view_frames >tf_tree.log 2>&1 || true
) &
PIDS+=("$!")

monitor_rate /cmd_vel_nav cmd_vel_nav_hz.log
monitor_rate /cmd_vel cmd_vel_hz.log
monitor_rate /bunker_odom bunker_odom_hz.log
monitor_rate /Odometry fastlio_odom_hz.log
monitor_rate /cloud_registered_body cloud_registered_body_hz.log
monitor_rate /livox_costmap livox_costmap_hz.log

deadline=$((START_EPOCH + DURATION_SEC))
while kill -0 "${BAG_PID}" 2>/dev/null; do
  if (( $(date +%s) >= deadline )); then
    log "Maximum duration reached."
    break
  fi
  sleep 1
done
