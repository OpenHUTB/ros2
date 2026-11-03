# 激光雷达小车反应式避障

ROS 2 节点订阅 `/scan`，按前方、左右扇区最近测距选择直行或转向，并以 10 Hz 发布 `/cmd_vel`。首次启动只发布零速度，收到有效扫描后才运动。

## 依赖与运行

Ubuntu 22.04、ROS 2 Humble、`rclpy`、`sensor_msgs`、`geometry_msgs`。在本目录运行 `source /opt/ros/humble/setup.bash && python3 main.py`，或 `ros2 launch ./launch/main.launch.py`。

## 仿真接线

将仿真激光雷达接到 `/scan`，底盘速度控制器接收 `/cmd_vel`。先检查 `ros2 topic info /scan` 和消息角度范围，再逐步降低 `safe_distance` 或调整速度参数。节点停止时发送零速度。

## 验证记录

尚未在 ROS 2 仿真器中运行验证。接入机器人前，请在空旷、低速场景检查前方障碍、左右障碍、无障碍和急停情况。
