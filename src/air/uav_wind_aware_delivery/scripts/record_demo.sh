#!/usr/bin/env bash
# 录制 Ubuntu 桌面演示：OpenCV Viewer + 桌面环境，x11grab 30 s。
export DISPLAY=:0
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
export DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=/run/user/1000/bus}"
WS="$HOME/air_ws"
OUT="$WS/media"
FF="${FFMPEG_BIN:-/home/lxy/codex_vehicle_20261021_2040/runtime/python/lib/python3.11/site-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2}"
mkdir -p "$OUT"

pkill -f 'roslaunch uav_wind_aware_delivery' 2>/dev/null || true
pkill -f 'uav_viewer' 2>/dev/null || true
pkill -f 'sim_node.py' 2>/dev/null || true
sleep 1

cd "$WS"
source /opt/ros/noetic/setup.bash
source devel/setup.bash
timeout 55 roslaunch uav_wind_aware_delivery demo.launch \
  wind_x:=3.0 wind_y:=1.0 > /tmp/demo_record.log 2>&1 &
LAUNCH_PID=$!
sleep 4
"$FF" -y -f x11grab -framerate 15 -video_size 1280x796 -i :0 \
  -t 45 -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p \
  -movflags +faststart "$OUT/ubuntu_air_demo.mp4" \
  > /tmp/ffmpeg_record.log 2>&1
kill $LAUNCH_PID 2>/dev/null || true
pkill -f 'roslaunch uav_wind_aware_delivery' 2>/dev/null || true
echo "recorded: $OUT/ubuntu_air_demo.mp4"
ls -lh "$OUT/ubuntu_air_demo.mp4"
