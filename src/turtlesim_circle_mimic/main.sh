#!/usr/bin/env bash
set -euo pipefail

mode="${1:-circle}"
if [[ "$#" -gt 0 ]]; then
  shift
fi

case "$mode" in
  circle) launch_file="two_turtles_circle.launch" ;;
  mimic) launch_file="turtle_mimic.launch" ;;
  -h|--help)
    printf 'Usage: bash main.sh [circle|mimic] [roslaunch arguments]\n'
    exit 0
    ;;
  *)
    printf 'Unknown mode: %s. Use circle or mimic.\n' "$mode" >&2
    exit 2
    ;;
esac

if ! command -v roslaunch >/dev/null 2>&1; then
  printf 'roslaunch is unavailable. Source your ROS1 setup.bash first.\n' >&2
  exit 1
fi

module_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$module_dir/launch/$launch_file" ]]; then
  exec roslaunch "$module_dir/launch/$launch_file" "$@"
fi
# The catkin-installed executable lives separately from the launch files.
exec roslaunch turtlesim_circle_mimic "$launch_file" "$@"
