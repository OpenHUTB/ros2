#!/usr/bin/env bash
# 在已经安装 ROS Noetic 的 Ubuntu 20.04 上构建；不修改系统环境。
set -e
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${UAV_WS:-$HOME/air_ws}"
mkdir -p "$WS/src"
if [ "$(readlink -f "$WS/src/uav_wind_aware_delivery" 2>/dev/null)" != "$PKG_DIR" ]; then
  rm -rf "$WS/src/uav_wind_aware_delivery"
  ln -s "$PKG_DIR" "$WS/src/uav_wind_aware_delivery"
fi
source /opt/ros/noetic/setup.bash
cd "$WS"
catkin_make -j2
chmod +x "$PKG_DIR"/scripts/nodes/*.py
echo "built: $WS/devel/setup.bash"
