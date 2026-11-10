# 基于 ROS 2 的禁飞区（电子围栏）动态检测与安全预警系统

ROS 2 独立安全监控层模块。通过订阅载具实时位姿与动态下发的禁飞区多边形边界，
使用射线法（Ray Casting）实时判断载具是否侵入禁飞区，并输出 `SAFE`（安全）或
`CRITICAL`（侵入告警）两级安全状态。

详细教程见 [docs/air/geofence.md](../../../docs/air/geofence.md)。

## 目录结构

```text
src/air/
└── geofence_demo/
    ├── geofence_demo/
    │   ├── __init__.py
    │   └── geofence_node.py        # 核心检测节点（含 is_point_in_polygon）
    ├── launch/
    │   └── geofence.launch.py      # ros2 launch 启动文件
    ├── resource/
    ├── test/
    │   ├── test_geofence_logic.py  # 多边形判定单元测试
    │   ├── test_flake8.py
    │   └── test_pep257.py
    ├── package.xml
    ├── setup.cfg                   # 含 [tool:pytest] 配置
    └── setup.py
```

## 编译与运行

```bash
colcon build --packages-select geofence_demo
source install/setup.bash
ros2 launch geofence_demo geofence.launch.py
```

可通过参数覆盖期望坐标系：

```bash
ros2 launch geofence_demo geofence.launch.py expected_frame_id:=odom
```

## 坐标系要求

节点会校验 `/odom` 与 `/geofence` 话题的 `header.frame_id` 是否一致，
默认期望都为 `map`。若坐标系不一致，节点会告警并跳过检测，
需要先通过 tf2 将坐标变换到同一 frame_id 下。

## 边界与状态语义

落在围栏边上或顶点上的位置按 `SAFE` 处理，这是有意选择：把边界视为"仍在围栏内"，
避免贴边飞行时反复告警；代价是边界外侧紧邻的窄带同样不告警，如需缓冲带可改为
按到边界的距离判定。

`/geofence_status` 的消息体只有 `SAFE` / `CRITICAL` 枚举字符串，载具坐标写入日志，
便于下游直接比较；发布者使用 `TRANSIENT_LOCAL` QoS，后启动的订阅者也能拿到当前状态。

## 测试

```bash
colcon test --packages-select geofence_demo
colcon test-result --verbose --test-result-base build/geofence_demo

# 或直接运行 pytest
cd src/air/geofence_demo && python3 -m pytest test -v
```

## 话题接口

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 订阅 | `/odom` | `nav_msgs/Odometry` | 载具实时位置 |
| 订阅 | `/geofence` | `geometry_msgs/PolygonStamped` | 禁飞区多边形 |
| 发布 | `/geofence_status` | `std_msgs/String` | `SAFE` / `CRITICAL` 状态输出，`TRANSIENT_LOCAL` QoS |
