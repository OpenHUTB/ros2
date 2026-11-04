#!/usr/bin/env bash
# 在 Ubuntu 桌面上录制真实实操视频（需要已装好 ffmpeg：sudo apt install ffmpeg）
#
#   bash tools/record_desktop_demo.sh [输出文件]
#
# 录制期间请保持终端窗口在最前、不要最小化；脚本会自动依次演示
# 键盘遥控 -> 建图导航 -> 端到端 CNN，约 2 分钟。
set -euo pipefail

OUT="${1:-$HOME/humanoid_desktop_demo.mp4}"
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

command -v ffmpeg >/dev/null || { echo "请先安装 ffmpeg: sudo apt install -y ffmpeg"; exit 1; }
command -v xdpyinfo >/dev/null || { echo "请先安装 x11-utils: sudo apt install -y x11-utils"; exit 1; }

SIZE="$(xdpyinfo | awk '/dimensions:/{print $2}')"
echo "屏幕分辨率: $SIZE，输出: $OUT"

ffmpeg -y -f x11grab -framerate 25 -video_size "$SIZE" -i "${DISPLAY:-:0}" \
       -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p "$OUT" \
       >/dev/null 2>&1 &
FFPID=$!
sleep 2

set +e
bash "$PKG/tools/demo_run.sh"
set -e

sleep 2
kill -INT "$FFPID" 2>/dev/null
wait "$FFPID" 2>/dev/null
echo "已保存: $OUT ($(du -h "$OUT" | cut -f1))"
