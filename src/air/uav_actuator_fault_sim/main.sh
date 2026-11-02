#!/usr/bin/env bash
# uav_actuator_fault_sim Linux 入口（源码目录直接跑；安装后也可 ros2 run 调用）
# 提交前记得保留可执行位：git update-index --chmod=+x main.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 已 source 过 ROS2 且有 rclpy 时，--node 子命令才可用；其余子命令纯 Python
if [ "$#" -eq 0 ]; then
  exec python3 "${SCRIPT_DIR}/main.py" --demo
else
  exec python3 "${SCRIPT_DIR}/main.py" "$@"
fi
