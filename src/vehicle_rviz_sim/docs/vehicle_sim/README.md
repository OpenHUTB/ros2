
# 基于 ROS 的无人车运动仿真通过键盘控制机器人小车运动

## 任务说明
本人选择任务（1）：车辆运动仿真，通过键盘对机器人进行运动控制。

## 运行环境
- Ubuntu 20.04
- ROS Noetic
- Python 3.8

## 仿真步骤
1. 打开终端，输入 `roscore`
2. 新开终端，进入工作空间，输入 `source devel/setup.bash`
3. 输入 `roslaunch vehicle_rviz_sim vehicle_sim.launch`
4. rviz 打开后，将 Fixed Frame 设为 `map`
5. 添加 `Marker` 显示，Topic 设为 `/vehicle_marker`
6. 观察红色方块在 3D 网格中自动行驶，并用wads控制小车运动

## 运行结果
![运行效果](vehicle_sim.gif)

## 大模型声明
代码由本人参考 ROS 官方文档和课程资料编写，未使用大模型生成。
