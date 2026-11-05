#!/usr/bin/env bash
# Run an isolated ROS master, real turtlesim, and integration checks.
set -euo pipefail
source /opt/ros/noetic/setup.bash
PACKAGE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUN_DIR="$(mktemp -d "${TMPDIR:-/tmp}/patrol_check_XXXXXXXX")"
export PATROL_EVIDENCE_DIR="$RUN_DIR/cases"
# A separate master avoids interacting with the old launch/controller.
PORT="$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",0)); print(s.getsockname()[1]); s.close()')"
export ROS_MASTER_URI="http://127.0.0.1:$PORT"
export ROS_HOSTNAME=localhost
unset ROS_IP
export ROS_LOG_DIR="$RUN_DIR/ros_logs"
MASTER_PID=''
SIM_PID=''
cleanup() {
  code=$?
  trap - EXIT
  for pid in "$SIM_PID" "$MASTER_PID"; do
    if [[ -n "$pid" ]]; then kill -TERM -- "-$pid" 2>/dev/null || true; fi
  done
  sleep 1
  for pid in "$SIM_PID" "$MASTER_PID"; do
    if [[ -n "$pid" ]] && kill -0 -- "-$pid" 2>/dev/null; then
      kill -KILL -- "-$pid" 2>/dev/null || true
    fi
  done
  tar -czf "$RUN_DIR.tar.gz" -C "$(dirname "$RUN_DIR")" "$(basename "$RUN_DIR")"
  echo "RESULT_BUNDLE=$RUN_DIR.tar.gz"
  exit "$code"
}
trap cleanup EXIT
sha256sum "$PACKAGE_DIR/main.py" > "$RUN_DIR/source_sha256.txt"
python3 --version > "$RUN_DIR/python_version.txt" 2>&1
rosversion -d > "$RUN_DIR/ros_version.txt"
# shellcheck disable=SC2016
python3 -c 'import ast, pathlib, sys; [ast.parse(p.read_text()) for p in pathlib.Path(sys.argv[1]).rglob("*.py")]' "$PACKAGE_DIR"
setsid roscore -p "$PORT" > "$RUN_DIR/master.log" 2>&1 &
MASTER_PID=$!
for _ in $(seq 1 100); do
  if python3 -c 'import rosgraph; rosgraph.Master("/check_ready").getPid()' 2>/dev/null; then break; fi
  sleep .1
done
if [[ "${1:-}" == '--headless' ]]; then export QT_QPA_PLATFORM=offscreen; fi
setsid rosrun turtlesim turtlesim_node > "$RUN_DIR/turtlesim.log" 2>&1 &
SIM_PID=$!
# The Python test waits for /reset and is authoritative about readiness.
set +e
python3 -u "$PACKAGE_DIR/verify_turtlesim.py" 2>&1 | tee "$RUN_DIR/checks.txt"
result=${PIPESTATUS[0]}
set -e
# Plot actual observed poses even if another case fails; never infer success.
if [[ -s "$PATROL_EVIDENCE_DIR/test_01_pause_resume_completion.csv" ]]; then
  python3 "$PACKAGE_DIR/plot_trace.py" "$PATROL_EVIDENCE_DIR/test_01_pause_resume_completion.csv" \
    "$RUN_DIR/actual_patrol.png" > "$RUN_DIR/plot.txt" 2>&1 || {
      cat "$RUN_DIR/plot.txt"; result=1;
    }
fi
exit "$result"
