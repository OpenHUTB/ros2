# 无人机兴趣点(POI)环绕航拍

给定地面 POI 坐标, 无人机在指定半径和高度上绕点飞行, 偏航始终对准 POI, 适合建筑/场地环绕拍摄。

与 `uav_waypoint_tracking`(走预设离散航点)和八字轨迹(固定图形)的区别:
本模块由任意 POI 实时生成连续圆周/螺旋航线, 偏航角自动对准目标。

## 运行环境

- Ubuntu 20.04 + ROS 2 Humble
- AirSim 桥接

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `poi_x/y` | 0/0 | 兴趣点地面坐标 |
| `orbit_radius` | 5.0 | 环绕半径(m) |
| `orbit_height` | 8.0 | 飞行高度(m) |
| `omega_dps` | 15.0 | 角速度(度/秒) |
| `spiral` | false | 是否螺旋升降 |
| `spiral_height_delta` | 2.0 | 每圈高度增量(m) |

## 话题

- 发布: `/drone/setpoint_position` (连续圆周目标)
- 发布: `/drone/target_yaw` (对准 POI 的偏航角, rad)

## 运行

```bash
cd src/air/uav_poi_orbit
python3 main.py
# 或
ros2 launch launch/uav_poi_orbit.launch.py
```

## 几何

圆周上位置: `x = poi_x + r*cos(θ)`, `y = poi_y + r*sin(θ)`
偏航角: `yaw = atan2(poi_y - y, poi_x - x)`
开螺旋后每完成一圈高度增加 `spiral_height_delta`。

## 验证范围

- 节点按固定频率发布连续圆周目标点
- 偏航角随位置自动计算
- 完整飞行需配合 AirSim 位置控制桥接
