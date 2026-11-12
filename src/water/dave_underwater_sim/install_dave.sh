#!/bin/bash
# Install Project DAVE underwater simulation (ROS Noetic + Gazebo 11) on Ubuntu 20.04.
# Usage: bash install_dave.sh
# Notes:
#   - Prerequisites: Ubuntu 20.04 + ROS Noetic desktop-full (/opt/ros/noetic) + Gazebo 11.
#     This script installs the extra apt dependencies and builds the demo packages;
#     it does NOT install ROS itself.
#   - Creates catkin workspace at ~/uuv_ws and builds the packages needed by the demos.
#   - Downloads are pinned to known-good upstream commits (not moving branches), so the
#     build stays reproducible even if upstream moves on.
set -e

WS=${WS:-$HOME/uuv_ws}
SRC=$WS/src
TAR=/tmp/dave_tars
JOBS=${JOBS:-2}

fail() { echo "ERROR: $*" >&2; exit 1; }

echo "== [0/4] prerequisites =="
[ -f /opt/ros/noetic/setup.bash ] || fail "ROS Noetic not found (/opt/ros/noetic/setup.bash missing). Install ROS Noetic desktop-full first: http://wiki.ros.org/noetic/Installation/Ubuntu"
[ -d /usr/share/gazebo-11 ] || fail "Gazebo 11 not found (/usr/share/gazebo-11 missing). Install gazebo11 (or ros-noetic-gazebo-ros-pkgs)."
command -v sudo >/dev/null 2>&1 || fail "sudo not found."

echo "== [1/4] apt dependencies =="
sudo apt-get update
sudo apt-get install -y \
  python3-pip python3-catkin-tools python3-vcstool python3-rosdep curl \
  build-essential cmake cppcheck \
  libeigen3-dev libgles2-mesa-dev pkg-config protobuf-compiler python3-venv qtbase5-dev \
  ruby wget lsb-release \
  ros-noetic-hector-gazebo-plugins ros-noetic-velodyne-gazebo-plugins \
  ros-noetic-robot-localization ros-noetic-urdfdom-py ros-noetic-joy \
  ros-noetic-joy-teleop ros-noetic-teleop-twist-joy ros-noetic-key-teleop \
  ros-noetic-image-view ros-noetic-cv-bridge ros-noetic-kdl-parser-py ros-noetic-tf2-geometry-msgs \
  ros-noetic-tf2-tools ros-noetic-teleop-tools python3-scipy python3-lxml python3-opencv

echo "== [2/4] download source tarballs (codeload, pinned commits) =="
mkdir -p "$TAR" "$SRC"
dl() {
  name=$1; url=$2
  if [ -s "$TAR/$name.tar.gz" ]; then echo "skip $name (exists)"; return 0; fi
  for i in 1 2 3 4 5; do
    curl -fL --connect-timeout 20 --max-time 900 --retry 2 --retry-delay 3 \
      -o "$TAR/$name.tar.gz" "$url" && return 0
    echo "retry $name ($i)"; sleep 5
  done
  fail "download failed: $name ($url)"
}
B=https://codeload.github.com
# Pinned to the commits verified on 2026-11-11 (upstream branches: master/nps_dev/main).
dl dave             "$B/Field-Robotics-Lab/dave/tar.gz/e54ea16bdaf7d6dbb466cf141846a03f284a44dd"
dl uuv_simulator    "$B/field-robotics-lab/uuv_simulator/tar.gz/f46ad38193ff81223912c94bbeb436bbb582b410"
dl uuv_manipulators "$B/field-robotics-lab/uuv_manipulators/tar.gz/2a22012d6471c526661a41e2bc75ae107581bd48"
dl rexrov2          "$B/uuvsimulator/rexrov2/tar.gz/1546681610ce0d3a921447bb7d176a74967eea56"
dl eca_a9           "$B/uuvsimulator/eca_a9/tar.gz/5c7e357253c0873d1441ae10910949871cfb944a"
dl ds_msgs          "$B/Field-Robotics-Lab/ds_msgs/tar.gz/179631d46871b31fad28a21f81d865e3c73446d3"
dl ds_sim           "$B/Field-Robotics-Lab/ds_sim/tar.gz/25bf60969f8ec3e815f5c34c3b7b540e8c6e7003"
dl dockwater        "$B/Field-Robotics-Lab/dockwater/tar.gz/e84c6eec0c602868a0fd6e0e2af59a3bf7e9d689"

for t in "$TAR"/*.tar.gz; do
  name=$(basename "$t" .tar.gz)
  [ -d "$SRC/$name" ] && [ -n "$(ls -A "$SRC/$name" 2>/dev/null)" ] && { echo "skip extract $name"; continue; }
  mkdir -p "$SRC/$name"
  tar xzf "$t" -C "$SRC/$name" --strip-components=1
  echo "extracted $name"
done

echo "== [3/4] rosdep =="
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update

echo "== [4/4] catkin build (subset needed by demos) =="
source /opt/ros/noetic/setup.bash
command -v catkin >/dev/null 2>&1 || fail "catkin not found after apt install (python3-catkin-tools)."
cd "$WS"
catkin init 2>/dev/null || true
catkin config --extend /opt/ros/noetic
PKGS="dave_worlds dave_object_models dave_robot_models dave_sensor_models \
dave_gazebo_model_plugins dave_gazebo_ros_plugins dave_gazebo_sensor_plugins \
dave_gazebo_world_plugins dave_gazebo_visual_plugins dave_gazebo_sensors \
rexrov_description dave_oberon7_description dave_snippets \
uuv_assistants uuv_control_cascaded_pid uuv_control_msgs uuv_control_utils \
uuv_thruster_manager uuv_trajectory_control uuv_teleop uuv_descriptions \
uuv_gazebo_ros_plugins_msgs uuv_gazebo_plugins uuv_gazebo_ros_plugins \
uuv_gazebo_worlds uuv_world_plugins uuv_world_ros_plugins uuv_world_ros_plugins_msgs \
uuv_sensor_ros_plugins_msgs uuv_gazebo plug_and_socket_description \
oberon7_description oberon7_control oberon7_gazebo"
catkin build -j"$JOBS" -p"$JOBS" --no-status $PKGS
catkin build -j"$JOBS" --no-status dave_demo_launch

echo "== done. workspace: $WS =="
echo "run: bash $(cd "$(dirname "$0")" && pwd)/run_demo.sh demo"
