# 无人机三维航点跟踪

节点顺序跟踪 ROS 参数 `waypoints` 中的三维点，按当前位置误差生成限速速度指令；内部简化运动学模型积分位置并发布 `/uav/odom`，速度输出发布到 `/uav/cmd_vel`。

## 依赖与运行

Ubuntu 22.04、ROS 2 Humble、`rclpy`、`geometry_msgs`、`nav_msgs`。在本目录运行 `source /opt/ros/humble/setup.bash && python3 main.py`，或 `ros2 launch ./launch/main.launch.py`。

## 调参与验证

检查 `/uav/odom` 的位置是否依次接近参数中的航点；速度受 `max_speed` 限制。项目只演示三维位置误差到速度指令的控制链路，不连接飞控，也未模拟姿态、推力和空气动力学。

## 验证记录

尚未在目标环境运行验证。真实无人机或飞控接入前必须使用仿真和安全限速复核，不能直接将示例输出用于实机。
