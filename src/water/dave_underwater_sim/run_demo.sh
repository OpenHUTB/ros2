#!/bin/bash
# Launch the DAVE underwater simulation demos.
# Usage:
#   bash run_demo.sh [demo|valve|plug] [headless]
#     demo  : REXROV + Oberon7 arm in the ocean-waves world (default)
#     valve : BOP panel manipulation scene (matches the reference video)
#     plug  : underwater electrical plug & socket scene
#     headless : start gzserver only (gui:=false)
set -e
WS=${WS:-$HOME/uuv_ws}
source /opt/ros/noetic/setup.bash
if [ ! -f "$WS/devel/setup.bash" ]; then
  echo "workspace not found: $WS/devel/setup.bash"
  echo "please run install_dave.sh first"
  exit 1
fi
source "$WS/devel/setup.bash"

# hide the ROS 1 EOL popup (Noetic reached EOL on 2025-05-31)
export DISABLE_ROS1_EOL_WARNINGS=1

command -v roslaunch >/dev/null 2>&1 || {
  echo "roslaunch not found: please install/source ROS Noetic (source /opt/ros/noetic/setup.bash)"
  exit 1
}

MODE=${1:-demo}
GUI=true
[ "$2" = "headless" ] && GUI=false

case "$MODE" in
  demo)
    roslaunch dave_demo_launch dave_demo.launch gui:=$GUI
    ;;
  valve)
    roslaunch dave_demo_launch dave_valve_manipulation_demo.launch gui:=$GUI
    ;;
  plug)
    roslaunch dave_demo_launch dave_plug_and_socket_demo.launch gui:=$GUI
    ;;
  *)
    echo "unknown mode: $MODE"; exit 1
    ;;
esac
