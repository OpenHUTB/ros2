#!/usr/bin/env bash
set -euo pipefail

VMNET8_IP="${1:?Usage: $0 <VMNET8_IP> [SimpleFlight]}"
VEHICLE="${2:-SimpleFlight}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -n "${AIRSIM_DEVEL_SETUP:-}" ]]; then
    AIRSIM_DEVEL_SETUP="$(realpath "$AIRSIM_DEVEL_SETUP")"
else
    AIRSIM_DEVEL_SETUP="$HOME/AirSim/ros/devel/setup.bash"
fi

if [[ ! -f "$AIRSIM_DEVEL_SETUP" ]]; then
    echo "AirSim ROS workspace not found: $AIRSIM_DEVEL_SETUP" >&2
    echo "Build airsim_ros_pkgs first, or set AIRSIM_DEVEL_SETUP." >&2
    exit 1
fi

source "$AIRSIM_DEVEL_SETUP"

pkill -f "main.py" 2>/dev/null || true
pkill -f "airsim_node" 2>/dev/null || true
pkill roscore 2>/dev/null || true

roslaunch "$SCRIPT_DIR/launch/teleop.launch" host:="$VMNET8_IP" output:=log &
ROSLAUNCH_PID=$!

cleanup() {
    kill "$ROSLAUNCH_PID" 2>/dev/null || true
    pkill -f "main.py" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

for _ in $(seq 1 60); do
    if rosnode list 2>/dev/null | grep -q "/airsim_node"; then
        break
    fi
    sleep 1
done

if ! rosnode list 2>/dev/null | grep -q "/airsim_node"; then
    echo "airsim_node did not start. Check the ROS launch output." >&2
    exit 1
fi

python3 "$SCRIPT_DIR/main.py" --vehicle "$VEHICLE"
