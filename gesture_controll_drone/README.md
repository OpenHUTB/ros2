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
<img width="3072" height="4096" alt="199d7acf0b95f7b2028ecea3ed02bd36" src="https://github.com/user-attachments/assets/e3aa2a80-fed8-44a1-b763-c561d6401929" />
<img width="1706" height="1279" alt="e07fd6032b3521b082c33bda35535766" src="https://github.com/user-attachments/assets/b0da1e95-6b8f-48e8-a8ee-731f183eaa04" />
(配图说明：已成功实现ROS虚拟机与Windows端AirSim真实3D环境联调，通过底层API指令实现真实起飞)
