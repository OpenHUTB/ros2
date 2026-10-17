# humanoid_sim

人形机器人键盘控制仿真 (ROS1 Noetic)。

## 运行环境

- Ubuntu 20.04
- ROS Noetic
- Python 3.8 (Noetic 自带)
- matplotlib

## 运行步骤

```bash
# 编译
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# 终端1: roscore
roscore

# 终端2: 启动仿真 + 可视化
roslaunch humanoid_sim sim.launch

# 终端3: 键盘遥控
rosrun humanoid_sim keyboard_teleop.py
```

按键：
- `w` / `↑` 前进
- `s` / `↓` 后退
- `a` / `←` 左转
- `d` / `→` 右转
- 空格 急停
- `q` 退出

## 话题

| 话题 | 类型 | 方向 | 含义 |
|------|------|------|------|
| /cmd_vel | geometry_msgs/Twist | 订阅 | linear.x=前进速度(m/s), angular.z=转向角速度(rad/s) |
| /odom | nav_msgs/Odometry | 发布 | 机器人位置 |
| /joint_states | sensor_msgs/JointState | 发布 | 腿臂关节角 |
| tf | odom -> base_footprint | 发布 | 供 rviz 使用 |
