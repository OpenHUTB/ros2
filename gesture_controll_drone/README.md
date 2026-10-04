# 允许用户使用手势控制AirSim无人机

## 1. 环境要求
- Ubuntu 20.04 / ROS Noetic
- Python 3.8+
- 依赖库：`pip install -r requirements.txt`

## 2. 节点架构
- `gesture_ros_node.py`：感知节点（发布者），订阅摄像头画面，使用MediaPipe识别手势，发布至 `/gesture_command` 话题。
- `drone_control_node.py`：控制节点（订阅者），接收手势指令，转换并执行AirSim无人机飞行动作。

## 3. 运行步骤
```bash
cd ~/gesture_ws
catkin_make
source devel/setup.bash
roslaunch gesture_controll_drone gesture_control.launch
