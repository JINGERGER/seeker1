#!/usr/bin/env bash
set -u

# One-click on-device validation for Seeker ROS2 pipeline.
# It launches seeker, checks nodes/topics, samples hz/bw, then exits.

WORKSPACE_DEFAULT="$HOME/ros2_ws"
LAUNCH_FILE_DEFAULT="1seeker.launch.py"
CHECK_SECONDS=8

PASS_COUNT=0
WARN_COUNT=0
FAIL_COUNT=0
LAUNCH_PID=""

TOPICS_REQUIRED=(
  "/fisheye_rect/left/image_raw/compressed"
  "/fisheye_rect/right/image_raw/compressed"
  "/fisheye_rect/bright/image_raw/compressed"
  "/fisheye_rect/bleft/image_raw/compressed"
)

NODES_REQUIRED=(
  "/seeker_node"
  "/omni_undistort_node"
  "/disparity_to_depth"
)

print_usage() {
  cat <<EOF
Usage: $(basename "$0") [-w <workspace>] [-l <launch_file>] [-t <seconds>] [-- <extra launch args>]

Examples:
  $(basename "$0")
  $(basename "$0") -w ~/ros2_ws -t 10
  $(basename "$0") -- undistort_fov_scale:=2.0 use_depth:=true
EOF
}

log_info() { echo "[INFO] $*"; }
log_pass() { echo "[PASS] $*"; PASS_COUNT=$((PASS_COUNT + 1)); }
log_warn() { echo "[WARN] $*"; WARN_COUNT=$((WARN_COUNT + 1)); }
log_fail() { echo "[FAIL] $*"; FAIL_COUNT=$((FAIL_COUNT + 1)); }

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    log_info "Stopping launch process (PID=${LAUNCH_PID})..."
    kill "${LAUNCH_PID}" 2>/dev/null || true
    sleep 1
    if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      log_warn "Launch process still alive, sending SIGKILL."
      kill -9 "${LAUNCH_PID}" 2>/dev/null || true
    fi
  fi
}

require_cmd() {
  local cmd="$1"
  if command -v "${cmd}" >/dev/null 2>&1; then
    log_pass "Command found: ${cmd}"
  else
    log_fail "Missing command: ${cmd}"
  fi
}

check_topic_exists() {
  local topic="$1"
  if ros2 topic list | awk '{print $1}' | grep -Fx "${topic}" >/dev/null 2>&1; then
    log_pass "Topic exists: ${topic}"
  else
    log_fail "Topic missing: ${topic}"
  fi
}

check_node_exists() {
  local node="$1"
  if ros2 node list | awk '{print $1}' | grep -Fx "${node}" >/dev/null 2>&1; then
    log_pass "Node running: ${node}"
  else
    log_fail "Node missing: ${node}"
  fi
}

sample_topic_hz() {
  local topic="$1"
  local out
  out="$(timeout "${CHECK_SECONDS}s" ros2 topic hz "${topic}" 2>&1 || true)"
  if echo "${out}" | grep -q "average rate"; then
    local last_line
    last_line="$(echo "${out}" | awk '/average rate/ {line=$0} END{print line}')"
    log_pass "Rate sample OK for ${topic} -> ${last_line}"
  else
    log_fail "Rate sample failed for ${topic}"
  fi
}

sample_topic_bw() {
  local topic="$1"
  local out
  out="$(timeout "${CHECK_SECONDS}s" ros2 topic bw "${topic}" 2>&1 || true)"
  if echo "${out}" | grep -q "average"; then
    local last_line
    last_line="$(echo "${out}" | awk '/average/ {line=$0} END{print line}')"
    log_pass "Bandwidth sample OK for ${topic} -> ${last_line}"
  else
    log_warn "Bandwidth sample not available for ${topic} (topic may still be warming up)"
  fi
}

WORKSPACE="${WORKSPACE_DEFAULT}"
LAUNCH_FILE="${LAUNCH_FILE_DEFAULT}"
EXTRA_ARGS=()

while (($# > 0)); do
  case "$1" in
    -w|--workspace)
      WORKSPACE="${2:-}"
      shift 2
      ;;
    -l|--launch-file)
      LAUNCH_FILE="${2:-}"
      shift 2
      ;;
    -t|--time|--seconds)
      CHECK_SECONDS="${2:-}"
      shift 2
      ;;
    -h|--help)
      print_usage
      exit 0
      ;;
    --)
      shift
      EXTRA_ARGS=("$@")
      break
      ;;
    *)
      EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

trap cleanup EXIT INT TERM

log_info "==== Seeker ROS2 Hardware Self-Check ===="
log_info "Workspace: ${WORKSPACE}"
log_info "Launch file: ${LAUNCH_FILE}"
log_info "Sample seconds: ${CHECK_SECONDS}"

require_cmd "ros2"
require_cmd "timeout"
require_cmd "lsusb"

if ((FAIL_COUNT > 0)); then
  log_fail "Missing required commands, aborting."
  exit 1
fi

if [[ ! -d "${WORKSPACE}" ]]; then
  log_fail "Workspace not found: ${WORKSPACE}"
  exit 1
fi

if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  log_fail "ROS2 workspace not built: ${WORKSPACE}/install/setup.bash not found"
  exit 1
fi

# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
# shellcheck source=/dev/null
source "${WORKSPACE}/install/setup.bash"

if lsusb | grep -i "2207:0000" >/dev/null 2>&1; then
  log_pass "Seeker USB device detected (2207:0000)"
else
  log_warn "Seeker USB device ID 2207:0000 not found in lsusb"
fi

log_info "Starting launch..."
ros2 launch seeker "${LAUNCH_FILE}" "${EXTRA_ARGS[@]}" > /tmp/seeker_hw_self_check.launch.log 2>&1 &
LAUNCH_PID="$!"
sleep 6

if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  log_pass "Launch process started (PID=${LAUNCH_PID})"
else
  log_fail "Launch process exited early. Check: /tmp/seeker_hw_self_check.launch.log"
  exit 1
fi

for node in "${NODES_REQUIRED[@]}"; do
  check_node_exists "${node}"
done

for topic in "${TOPICS_REQUIRED[@]}"; do
  check_topic_exists "${topic}"
done

sample_topic_hz "/fisheye_rect/left/image_raw/compressed"
sample_topic_bw "/fisheye_rect/left/image_raw/compressed"

echo
log_info "==== Result Summary ===="
echo "PASS: ${PASS_COUNT}"
echo "WARN: ${WARN_COUNT}"
echo "FAIL: ${FAIL_COUNT}"
echo "Launch log: /tmp/seeker_hw_self_check.launch.log"

if ((FAIL_COUNT > 0)); then
  exit 1
fi

exit 0
