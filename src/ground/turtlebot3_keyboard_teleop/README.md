# turtlebot3_keyboard_teleop

TurtleBot3 差速底盘键盘遥控，带超时悬停。

## 环境

- ROS 2 Humble
- TurtleBot3 Gazebo

## 构建

```bash
cd ~/ros2_ws
colcon build --packages-select turtlebot3_keyboard_teleop
source install/setup.bash
```

## 运行

先开 Gazebo：

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

再开键盘遥控（必须在前台终端里）：

```bash
ros2 launch turtlebot3_keyboard_teleop main.launch.py
```

按键：w 前进 / s 后退 / a 左转 / d 右转 / 空格停车 / q 退出。

详见 [docs/ground/turtlebot3_keyboard_teleop.md](../../../docs/ground/turtlebot3_keyboard_teleop.md)。
