# 小车纯追踪闭环示例

本节点内含一个简化的平面差速/自行车运动学模型，按纯追踪几何关系沿椭圆参考路线运动。发布 `/odom`、`/cmd_vel` 和以平面坐标对展平的 `/reference_path_xy`，便于用 `ros2 topic echo` 检查输出。

## 依赖与运行

Ubuntu 22.04、ROS 2 Humble、`rclpy`、`geometry_msgs`、`nav_msgs`、`std_msgs`。在本目录运行 `source /opt/ros/humble/setup.bash && python3 main.py`，或 `ros2 launch ./launch/main.launch.py`。

## 调参与验证

参数 `speed` 设置线速度，`lookahead` 设置前视距离。观察 `/odom` 是否连续变化，检查 `/reference_path_xy` 是否含有一组闭合椭圆采样点。该示例用于讲解控制逻辑，不包含轮胎侧滑、碰撞或 Gazebo 物理。

## 验证记录

尚未在 ROS 2 环境运行验证；不要将运动学输出当作真实车辆或物理仿真结果。
