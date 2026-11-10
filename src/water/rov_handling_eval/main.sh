#!/usr/bin/env bash
# 终端模式启动脚本（模式 A）。不需要 ROS，只要 python3 + mujoco。
#
#   ./main.sh              # 键控
#   ./main.sh --check      # 只做自由漂移自检（3 s），验证浮力/阻力符号
#   ./main.sh --current 0.3 --ballast 0.27
set -e
cd "$(dirname "$0")"

if ! python3 -c "import mujoco" 2>/dev/null; then
  echo "缺少 mujoco，先执行："
  echo "  pip3 install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple"
  exit 1
fi

exec python3 -m rov_handling_eval.main "$@"
