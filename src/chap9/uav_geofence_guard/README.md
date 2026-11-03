# 无人机三维禁飞区守护

订阅 `geometry_msgs/msg/PointStamped` 目标 `/uav/goal_raw`，将 x/y/z 分量限制到 `min_xyz` 与 `max_xyz` 参数指定的长方体范围，并发布 `/uav/goal_safe`。若目标被修改，会记录警告。

## 依赖与运行

Ubuntu 22.04、ROS 2 Humble、`rclpy`、`geometry_msgs`。在本目录运行 `source /opt/ros/humble/setup.bash && python3 main.py`，或 `ros2 launch ./launch/main.launch.py`。

## 验证方法

用 `ros2 topic pub /uav/goal_raw geometry_msgs/msg/PointStamped ...` 发布内外各一个点，再用 `ros2 topic echo /uav/goal_safe` 检查边界限幅。该示例是轴对齐盒形边界，不是完整飞行安全系统。

## 验证记录

尚未在 ROS 2 环境运行验证；请检查坐标系、单位与上下界配置后再用于其他节点。
