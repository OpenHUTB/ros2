@@ -0,0 +1,25 @@
# 无人机键盘飞行控制

## 环境
- Ubuntu 16.04
- ROS Kinetic
- AirSim 无人机仿真

## 安装依赖
pip install pynput

## 运行步骤
1. 启动无人机仿真环境
2. 编译工作空间：
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
3. 一键启动：
   roslaunch drone_keyboard_control drone_keyboard_control.launch

## 按键说明
W 前进 / S 后退 / A 左转 / D 右转 / Q 上升 / E 下降

## 演示
<img width="957" height="596" alt="1  `            drone_keyboard_demo gif`" src="https://github.com/user-attachments/assets/e7464c2a-9daf-42aa-b235-5d5999e08dc7" />


