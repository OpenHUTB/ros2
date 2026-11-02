#!/usr/bin/env bash
set -euo pipefail

LOG_DIR="${1:-${PWD}/ros2_qos_validation_logs}"
PACKAGE="ros2_qos_compatibility_lab"
TOPIC="/qos_lab/chatter"

export PYTHONUNBUFFERED=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

mkdir -p "${LOG_DIR}"

cleanup() {
  local pid
  for pid in "${background_pids[@]:-}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  for pid in "${background_pids[@]:-}"; do
    wait "${pid}" 2>/dev/null || true
  done
  background_pids=()
}

background_pids=()
trap cleanup EXIT INT TERM

fail() {
  printf '[FAIL] %s\n' "$*" >&2
  exit 1
}

pass() {
  printf '[PASS] %s\n' "$*"
}

expect_log() {
  local pattern="$1"
  local log_file="$2"
  if ! grep -Fq "${pattern}" "${log_file}"; then
    printf '\nMissing expected text: %s\n' "${pattern}" >&2
    printf '%s\n' '--- captured log ---' >&2
    cat "${log_file}" >&2
    exit 1
  fi
}

reject_log() {
  local pattern="$1"
  local log_file="$2"
  if grep -Fq "${pattern}" "${log_file}"; then
    printf '\nUnexpected text: %s\n' "${pattern}" >&2
    printf '%s\n' '--- captured log ---' >&2
    cat "${log_file}" >&2
    exit 1
  fi
}

wait_for_log() {
  local pattern="$1"
  local log_file="$2"
  local timeout_sec="${3:-15}"
  local elapsed=0
  while (( elapsed < timeout_sec )); do
    if [[ -f "${log_file}" ]] && grep -Fq "${pattern}" "${log_file}"; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  printf '\nTimed out waiting for: %s\n' "${pattern}" >&2
  printf '%s\n' '--- captured log ---' >&2
  cat "${log_file}" >&2
  return 1
}

run_timed() {
  local log_file="$1"
  local duration_sec="$2"
  shift 2

  local status
  set +e
  timeout --signal=INT --kill-after=5s "${duration_sec}s" \
    "$@" >"${log_file}" 2>&1
  status=$?
  set -e

  if (( status != 0 && status != 124 && status != 130 )); then
    printf '\nCommand failed with status %d: %s\n' \
      "${status}" "$*" >&2
    printf '%s\n' '--- captured log ---' >&2
    cat "${log_file}" >&2
    exit "${status}"
  fi
}

command -v ros2 >/dev/null 2>&1 \
  || fail "ros2 is not on PATH; source the ROS 2 and workspace setup files"

printf 'ROS_DOMAIN_ID=%s RMW_IMPLEMENTATION=%s\n' \
  "${ROS_DOMAIN_ID}" "${RMW_IMPLEMENTATION}"

INCOMPATIBLE_LOG="${LOG_DIR}/incompatible_demo.log"
run_timed "${INCOMPATIBLE_LOG}" 15 \
  ros2 launch "${PACKAGE}" qos_demo.launch.py
expect_log "QoS INCOMPATIBLE" "${INCOMPATIBLE_LOG}"
expect_log "reliability=best_effort->reliable" "${INCOMPATIBLE_LOG}"
reject_log "Received: QoS lab message" "${INCOMPATIBLE_LOG}"
pass "incompatible Reliability demo is diagnosed and receives no messages"

COMPATIBLE_LOG="${LOG_DIR}/compatible_demo.log"
run_timed "${COMPATIBLE_LOG}" 15 \
  ros2 launch "${PACKAGE}" qos_demo.launch.py \
  publisher_reliability:=reliable \
  subscriber_reliability:=reliable
expect_log "QoS COMPATIBLE" "${COMPATIBLE_LOG}"
expect_log "Received: QoS lab message" "${COMPATIBLE_LOG}"
pass "compatible Reliability demo exchanges messages"

PUBLISHER_LOG="${LOG_DIR}/endpoint_publisher.log"
SUBSCRIBER_LOG="${LOG_DIR}/endpoint_subscriber.log"
ros2 run "${PACKAGE}" demo_publisher --ros-args \
  -p "topic:=${TOPIC}" \
  -p reliability:=best_effort \
  -p durability:=volatile >"${PUBLISHER_LOG}" 2>&1 &
background_pids+=("$!")
ros2 run "${PACKAGE}" demo_subscriber --ros-args \
  -p "topic:=${TOPIC}" \
  -p reliability:=reliable \
  -p durability:=volatile >"${SUBSCRIBER_LOG}" 2>&1 &
background_pids+=("$!")

wait_for_log "Demo publisher started" "${PUBLISHER_LOG}"
wait_for_log "Demo subscriber started" "${SUBSCRIBER_LOG}"
sleep 2

TOPIC_INFO_LOG="${LOG_DIR}/topic_info_verbose.log"
ros2 topic info "${TOPIC}" --verbose >"${TOPIC_INFO_LOG}" 2>&1
expect_log "Publisher count: 1" "${TOPIC_INFO_LOG}"
expect_log "Subscription count: 1" "${TOPIC_INFO_LOG}"
expect_log "Reliability: BEST_EFFORT" "${TOPIC_INFO_LOG}"
expect_log "Reliability: RELIABLE" "${TOPIC_INFO_LOG}"
pass "topic info reports both endpoints with their distinct QoS"

STANDALONE_LOG="${LOG_DIR}/standalone_inspector.log"
run_timed "${STANDALONE_LOG}" 12 \
  ros2 launch "${PACKAGE}" qos_inspector.launch.py \
  "topic_filter:=${TOPIC}" \
  scan_period_sec:=1.0
expect_log "QoS INCOMPATIBLE" "${STANDALONE_LOG}"
pass "standalone inspector launch accepts typed float parameters"

HZ_LOG="${LOG_DIR}/topic_hz.log"
run_timed "${HZ_LOG}" 8 ros2 topic hz "${TOPIC}"
expect_log "average rate" "${HZ_LOG}"
pass "topic frequency confirms the publisher is active"

cleanup
trap - EXIT INT TERM
printf '\nRuntime validation passed. Logs: %s\n' "${LOG_DIR}"
