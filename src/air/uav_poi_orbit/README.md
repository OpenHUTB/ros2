# 无人机兴趣点(POI)环绕航拍

给定地面 POI 坐标, 无人机在指定半径和高度上绕点飞行, 偏航始终对准 POI, 适合建筑/场地环绕拍摄。

与 `uav_waypoint_tracking`(走预设离散航点)和八字轨迹(固定图形)的区别:
本模块由任意 POI 实时生成连续圆周/螺旋航线, 偏航角自动对准目标。

## 运行环境

- Ubuntu 20.04 + ROS 2 Humble
- AirSim 桥接

## 构建

```bash
cd <工作空间根目录>
colcon build --packages-select uav_poi_orbit
source install/setup.bash
```

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `poi_x` / `poi_y` | 0 / 0 | 兴趣点地面坐标 (m) |
| `orbit_radius` | 5.0 | 环绕半径 (m), 必须 > 0 |
| `orbit_height` | 8.0 | 飞行高度 (m), 必须 >= min_height |
| `omega_dps` | 15.0 | 角速度 (度/秒), 不能为 0 |
| `spiral` | false | 是否螺旋升降 |
| `spiral_height_delta` | 2.0 | 每圈高度增量 (m), 负值表示逐圈下降 |
| `waypoint_interval_s` | 0.2 | 目标点发布周期 (s), 必须 > 0 |
| `min_height` | 0.5 | 高度下限 (m), 螺旋下降时不会低于此值 |

## 话题

- 发布: `/drone/setpoint_position` (连续圆周目标, geometry_msgs/PointStamped)
- 发布: `/drone/target_yaw` (对准 POI 的偏航角, std_msgs/Float64, rad)

## 运行

```bash
ros2 run uav_poi_orbit poi_orbit_node --ros-args \
  -p poi_x:=0.0 -p poi_y:=0.0 -p orbit_radius:=5.0 -p orbit_height:=8.0
# 或
ros2 launch uav_poi_orbit uav_poi_orbit.launch.py
```

## 几何

- 圆周位置: `x = poi_x + r*cos(θ)`, `y = poi_y + r*sin(θ)`
- 偏航角: `yaw = atan2(poi_y - y, poi_x - x)`
- 螺旋: 高度按累计总转角连续变化 `z = h + (total_angle / 2π) * spiral_height_delta`,
  而不是按整圈阶跃, 避免圈边界处高度突变

## 单元测试

```bash
python3 test_orbit.py
```

覆盖角度归一化、圆周位置、POI 平移、偏航对准、角度推进回绕、圈数折算、
螺旋高度连续性、高度下限保护、参数校验共 11 项。

## 验证范围

- 节点按固定频率发布连续圆周目标点
- 偏航角随位置自动计算, 始终指向 POI
- 参数非法时节点拒绝启动 (半径/高度/角速度/步长)
- 完整飞行需配合 AirSim 位置控制桥接
