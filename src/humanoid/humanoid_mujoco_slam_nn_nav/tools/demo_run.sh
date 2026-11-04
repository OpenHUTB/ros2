#!/usr/bin/env bash
# 演示脚本：依次跑 键盘遥控 -> 建图导航 -> 端到端 CNN，供屏幕录制使用。
# 每段都可以单独 Ctrl-C 退出；这里用 timeout 控制时长。
set -uo pipefail

source /opt/ros/noetic/setup.bash
if [ -f "$HOME/humanoid_ws/devel/setup.bash" ]; then
  source "$HOME/humanoid_ws/devel/setup.bash"
fi

line() { printf '\n\033[1;36m$ %s\033[0m\n' "$*"; }

# ---------------------------------------------------------------- 1) 键盘遥控
line "roslaunch humanoid_nn_nav teleop.launch scene:=warehouse terminal:=false"
roslaunch humanoid_nn_nav teleop.launch scene:=warehouse terminal:=false &
LAUNCH=$!
sleep 6

echo "# 用 rostopic 模拟按键：前进 -> 左转 -> 前进"
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.22}, angular: {z: 0.0}}' >/dev/null
sleep 6
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.10}, angular: {z: 0.40}}' >/dev/null
sleep 4
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.22}, angular: {z: 0.0}}' >/dev/null
sleep 6
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null
sleep 2

line "rostopic hz /scan"
timeout 4 rostopic hz /scan || true
kill -INT $LAUNCH 2>/dev/null; wait $LAUNCH 2>/dev/null
sleep 2

# ---------------------------------------------------------------- 2) 建图导航
line "roslaunch humanoid_nn_nav slam_nav.launch scene:=warehouse"
roslaunch humanoid_nn_nav slam_nav.launch scene:=warehouse &
LAUNCH=$!
sleep 45
line "rosrun humanoid_nn_nav metrics_node.py _mode:=classical _scene:=warehouse"
kill -INT $LAUNCH 2>/dev/null; wait $LAUNCH 2>/dev/null
sleep 2
cat "$(rospack find humanoid_nn_nav)/results/ros_classical_warehouse.json" 2>/dev/null || true

# ---------------------------------------------------------------- 3) 端到端
line "roslaunch humanoid_nn_nav e2e.launch scene:=corridor"
timeout 35 roslaunch humanoid_nn_nav e2e.launch scene:=corridor || true
sleep 2

line "python3 tests/run_all.py"
cd "$(rospack find humanoid_nn_nav)" && PYTHONPATH=. python3 tests/run_all.py
