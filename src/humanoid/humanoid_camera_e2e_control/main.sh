#!/usr/bin/env bash
# 人形机器人单目图像端到端控制 —— standalone 一键流程
# 规则教师自动采集 -> 训练 CNN -> 端到端推理演示并录制素材
set -e
cd "$(dirname "$0")"

echo "==== 1/3 规则教师自动采集图像—指令数据集 ===="
python3 main.py --teacher --max-frames "${MAX_FRAMES:-3000}"

# 备选：人工键盘驾驶采集（W/S 前后、A/D 转向、空格停止、ESC 落盘）
# python3 main.py --collect --max-frames "${MAX_FRAMES:-2000}"

echo "==== 2/3 训练端到端 CNN ===="
python3 main.py --train --epochs "${EPOCHS:-50}"

echo "==== 3/3 端到端推理演示，录制 GIF 与指标 ===="
python3 main.py --run --duration "${DURATION:-25}"

echo "完成，素材位于 ./demo_assets/，权重位于 ./weights/"
