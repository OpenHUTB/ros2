#!/usr/bin/env bash
# ============================================================
#  CARLA 无人车四次作业 · CARLA 机完整运行清单（Ubuntu）
#  在【装有 CARLA 0.9.16 + 本仓库】的机器上依次执行，验证四项任务全链路。
#
#  用法：
#    bash scripts/carla_run_checklist.sh all        # 逐项运行
#    bash scripts/carla_run_checklist.sh 1|2|3|4    # 只跑某一任务
#
#  前置：
#    1) 已装 python 依赖与 carla Python API（见 docs/setup_guide.md）
#    2) 已启动 CARLA 服务端：./CarlaUE4.sh -carla-rpc-port=2000 -quality-level=Low
# ============================================================
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

task() {
  echo
  echo "################ $1 ################"
  eval "$2"
}

run_task1() {  # 键盘控制
  task "任务1 键盘运动控制（弹出窗口，W/S/A/D 控制，ESC 退出）" \
    "python 01_control/main.py --sim_time 20"
}
run_task2() {  # 感知 + 轨迹（NN）
  task "任务2 感知/控制 NN 离线训练" \
    "python 02_perception/main.py --mode train"
  task "任务2 在线感知 + 轨迹跟踪（NN 感知与控制）" \
    "python 02_perception/main.py --mode run --model models/nn_percept.json --waypoints '40,-8 40,12 25,20' --sim_time 20"
}
run_task3() {  # 建图 + 导航（NN）
  task "任务3 规划 NN 离线训练" \
    "python 03_navigation/main.py --mode train"
  task "任务3 在线建图 + 神经网络导航避障" \
    "python 03_navigation/main.py --mode run --model models/nn_plan.json --goal '20,8' --sim_time 30"
}
run_task4() {  # 端到端 CNN
  task "任务4 采集（需 CARLA；证明数据来源）" \
    "python 04_end_to_end/main.py --mode collect --frames 200 --out_dir dataset"
  task "任务4 端到端训练：纯numpy CNN（无需 TF，本机即可）" \
    "python 04_end_to_end/main.py --mode train --backend numpy --epochs 120 --data_dir dataset --model_path models/cnn.json"
  task "任务4 端到端训练：tf.keras CNN（可选，需 TF）" \
    "python 04_end_to_end/main.py --mode train --backend tf --epochs 30 --data_dir dataset --model_path models/cnn.h5 || true"
  task "任务4 端到端 自主驾驶（用 numpy 模型）" \
    "python 04_end_to_end/main.py --mode test --backend numpy --model_path models/cnn.json --sim_time 15"
}

case "$1" in
  1) run_task1 ;;
  2) run_task2 ;;
  3) run_task3 ;;
  4) run_task4 ;;
  all) run_task1; run_task2; run_task3; run_task4 ;;
  *) echo "用法: bash $0 {1|2|3|4|all}"; exit 1 ;;
esac

echo
echo "==== CARLA 全链路运行清单完成 ===="
echo "提示：请用 ScreenToGif 录制各任务运行 GIF(<10MB) 放入 docs/assets/，"
echo "      文档引用见 docs/{01_control,02_perception,03_navigation,04_end_to_end}.md。"
