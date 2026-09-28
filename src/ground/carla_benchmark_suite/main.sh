#!/bin/bash
# CARLA 作业综合整合与性能评价 —— 一键运行脚本（课程 main.* 约定）
#
#   bash main.sh                              # 打印帮助
#   bash main.sh --list                       # 列出所有可调度模块
#   bash main.sh --benchmark --save_dir ~/shots --out report.json
#                                             # 运行基准评测（真实测量，无需 CARLA）
#   bash main.sh --target perception -- --headless --demo --save_dir ~/shots
#                                             # 调度作业二（"--" 之后参数原样透传）
#   bash main.sh --target end_to_end -- --mode train --epochs 60
#   bash main.sh --launch                     # 由 ROS 2 launch 启动
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"

if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi

for a in "$@"; do
    if [ "$a" = "--launch" ]; then
        exec ros2 launch carla_benchmark_suite main.launch.py "${@/--launch/}"
    fi
done

echo "=================================================="
echo "  CARLA 作业综合整合与性能评价"
echo "=================================================="

PY=python3
if command -v python3.10 >/dev/null 2>&1; then
    PY=python3.10
fi

exec "$PY" main.py "$@"
