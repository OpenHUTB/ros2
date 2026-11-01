# 人形机器人 3D 碰撞体边界检测可视化

本模块为简化人形机器人注册 12 个碰撞体（头、躯干、左右上臂、左右前臂、左右大腿、左右小腿、左右足），实时计算障碍物到各碰撞体的最短表面距离，发布最近距离、碰撞标志与可视化 MarkerArray（话题 `/distance_to_obstacle`、`/collision_detected`、`/collision_boundary`）。

## 运行环境

- Ubuntu 22.04
- ROS 2 Humble
- 依赖：`rclcpp std_msgs geometry_msgs visualization_msgs eigen`

> 本模块当前只支持静态简化碰撞体：12 个碰撞体的位姿在节点启动时确定，运行期不做 TF/URDF 跟踪。

## 构建

```bash
mkdir -p ~/ros2_ws/src
cp -r ~/ros2/src/humanoid_collision_detector ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select humanoid_collision_detector
source install/setup.bash
```

## 启动

```bash
ros2 launch humanoid_collision_detector main_launch.py
```

## 查看检测结果

```bash
ros2 topic echo /distance_to_obstacle
ros2 topic echo /collision_detected
ros2 topic list | grep collision
```

默认配置下，障碍物沿正弦轨迹在机器人前后往返，`/distance_to_obstacle` 数值随障碍物位置连续变化，最近碰撞体在头、躯干、前臂等部件之间切换；当距离低于安全边界 `safety_boundary` 时，`/collision_detected` 翻转为 `true`。

## 可视化

```bash
ros2 run rviz2 rviz2 -d install/humanoid_collision_detector/share/humanoid_collision_detector/rviz/collision_boundary.rviz
```

Fixed Frame 设为 `world`，MarkerArray 话题选择 `/collision_boundary`，即可看到半透明碰撞体、线框边界与距离指示。

## 参数

参数位于：`launch/main_launch.py`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `safety_boundary` | 0.15 | 安全边界膨胀量（m），小于该距离判为碰撞 |
| `obstacle_radius` | 0.20 | 障碍物近似半径（m） |
| `check_frequency` | 100.0 | 检测频率（Hz） |
| `origin_x` | 0.85 | 演示障碍物轨迹中心 x（m） |
| `origin_y` | 0.0 | 演示障碍物轨迹中心 y（m） |
| `origin_z` | 1.15 | 演示障碍物轨迹中心 z（m） |
| `amplitude` | 0.55 | 演示障碍物振幅（m） |
| `frequency` | 1.0 | 演示障碍物角频率（rad/s） |
| `publish_hz` | 20.0 | 障碍物位置发布频率（Hz） |

## 坐标约定

所有碰撞体位置与尺寸均以 z 轴为高度方向：`BOX` 的 `size = (x, y, z)`；`CYLINDER` 的 `size = (radius, height, 0)`，高度沿 z 轴；`SPHERE` 的 `size.x()` 为半径。

## 许可

Apache-2.0
