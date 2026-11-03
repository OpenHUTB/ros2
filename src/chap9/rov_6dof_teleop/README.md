# 水下 ROV 六自由度遥控响应模型

订阅 `/rov/cmd_vel` 的线速度和角速度分量，用带阻尼的一阶运动学模型积分位置与姿态，发布 `nav_msgs/msg/Odometry` 到 `/rov/odom`。该模型便于观察六自由度命令和位姿之间的关系，不包含 MuJoCo、水动力、浮力或声呐。

## 依赖与运行

Ubuntu 22.04、ROS 2 Humble、`rclpy`、`geometry_msgs`、`nav_msgs`。在本目录运行 `source /opt/ros/humble/setup.bash && python3 main.py`，或 `ros2 launch ./launch/main.launch.py`。

## 验证方法

在另一终端向 `/rov/cmd_vel` 发布线速度或角速度，并观察 `/rov/odom`。停止发布后速度在阻尼作用下逐渐衰减。检查 `map` 和 `rov/base_link` 坐标系说明与四元数输出。

## 验证记录

尚未在 ROS 2 环境运行验证；这是教学用低阶模型，不是经过水池或高保真流体仿真验证的 ROV。
