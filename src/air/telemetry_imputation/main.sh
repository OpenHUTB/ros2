#!/usr/bin/env bash
# 统一入口：加载 ROS 与 Python 环境后转发给 main.py。
# 可用 ROS_SETUP 指定 ROS setup 文件、UAV_ENV 指定虚拟环境目录。
set -eo pipefail
cd "$(dirname "$0")"

ros_ok=0
for setup in "${ROS_SETUP:-}" /opt/ros/humble/setup.bash /opt/ros/noetic/setup.bash; do
  if [ -n "$setup" ] && [ -f "$setup" ]; then
    # shellcheck disable=SC1090
    . "$setup"
    ros_ok=1
    break
  fi
done
if [ "$ros_ok" = "0" ]; then
  echo "未找到 ROS 环境。请用 ROS_SETUP=/opt/ros/<distro>/setup.bash 指定。" >&2
  exit 2
fi

venv="${UAV_ENV:-${VIRTUAL_ENV:-$HOME/uav_prediction_env}}"
if [ -f "$venv/bin/activate" ]; then
  # shellcheck disable=SC1090
  . "$venv/bin/activate"
fi

exec python main.py "$@"
