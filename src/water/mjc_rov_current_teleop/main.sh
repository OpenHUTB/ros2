#!/usr/bin/env bash
# main.sh —— shell 包装：source ROS 环境后转交 main.py
# 不带参数：python3 main.py（自动跑 collect->train->test 全链路）
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 必须 source ROS Humble（rclpy 依赖）
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash 2>/dev/null || true
# 如果是在 colcon workspace 里运行，优先 source install/setup.bash
if [ -f "$SCRIPT_DIR/../../install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$SCRIPT_DIR/../../install/setup.bash"
fi

# OSMesa 离屏渲染（无 GPU 虚拟机必须）
export MUJOCO_GL=osmesa
export PYOPENGL_PLATFORM=osmesa

cd "$SCRIPT_DIR"
exec python3 -m mjc_rov_current_teleop.main "$@"
