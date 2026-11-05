# reverse_parking_planner

不依赖 CARLA 的倒车入库轨迹规划器。

仓库里 `docs/ground/carla_parking_lot.md` 讲的是「怎么在 CARLA 里生成一个带车位线的
停车场」，必须在 Windows 上跑 CARLA 服务端、在 Ubuntu 22.04 + ROS 2 Humble 里跑客户端
才能看到效果。这个包把同一件事里**只跟几何有关的那一半**单独拿出来：给定车位尺寸、
车辆尺寸和转弯半径，算出一次倒车入库的完整路径，并在终端打印断言结果与字符俯视图。

因此它：

* 不需要 CARLA、不需要 GPU、不需要图形界面；
* 不需要订阅任何话题，`roslaunch` 一跑就能出结果；
* 核心几何在 `scripts/parking_core.py` 里，只用 Python 标准库，可以脱离 ROS 做回归测试。

## 目录结构

```
reverse_parking_planner/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── parking_planner.yaml     # 全部默认参数
├── launch/
│   └── main.launch              # 每个参数都有同名 arg，可直接覆盖
└── scripts/
    ├── main.py                  # 入口（launch 里 type="main.py"）
    ├── parking_planner_node.py  # ROS 侧：读参数、算路径、发消息
    └── parking_core.py          # 几何核心：Pose / Vehicle / Slot / build_plan / 断言
```

## 路径是怎么算出来的

全程倒车，因此路径只有两段，长度恒为 `R·pi/2 + entry_length`：

1. 目标位姿 `T` 在车位正中，车头朝向车道（倒车入库后车头朝外）；
2. 倒车行驶方向与车头相反，从 `T` 沿行驶方向反向退 `entry_length` 米得到 `J`，
   此时车头已经对准车位；
3. 圆心 `C` 取在 `J` 行车方向的右侧 `turn_radius` 米处，起点 `S = C - R·travel`；
4. 圆弧从 `S` 右转 90° 到 `J`，再从 `J` 沿行驶方向倒车 `entry_length` 米到 `T`。

规划器会先检查 `turn_radius` 是否小于车辆的最小转弯半径
`wheelbase / tan(max_steer)`（默认 `2.70 / tan(0.55) = 4.40 m`），小于则直接报错，
不会给出一个物理上开不出来的路径。

## 参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `slot/kind` | `perpendicular` | 车位类型，当前只实现垂直车位 |
| `slot/width` | `2.50` | 车位宽度（m） |
| `slot/length` | `5.30` | 车位长度（m） |
| `slot/x` / `slot/y` | `0.0` / `0.0` | 车位中心坐标（m） |
| `slot/yaw` | `-1.5707963` | 停放时车头朝向（rad） |
| `vehicle/wheelbase` | `2.70` | 轴距（m） |
| `vehicle/length` | `4.50` | 车长（m） |
| `vehicle/width` | `1.80` | 车宽（m） |
| `vehicle/max_steer` | `0.55` | 最大前轮转角（rad） |
| `plan/turn_radius` | `5.00` | 规划用转弯半径（m） |
| `plan/entry_length` | `3.00` | 最后一段倒车直线长度（m） |
| `plan/step` | `0.10` | 路径采样步长（m） |
| `check/tolerance` | `0.10` | 终点位置容差（m） |
| `check/clearance` | `0.00` | 与邻车间距下限（m） |
| `check/max_path` | `12.00` | 允许的最长倒车距离（m） |
| `rate/publish_hz` | `10.0` | 逐点回放频率（Hz） |
| `frame/id` | `map` | 消息坐标系 |
| `topic/plan` | `/parking/plan` | `nav_msgs/Path`（latched） |
| `topic/report` | `/parking/report` | `std_msgs/String` 报告（latched） |
| `topic/point` | `/parking/plan_point` | `geometry_msgs/PoseStamped` 逐点回放 |

## 运行

```bash
# 默认车位
roslaunch reverse_parking_planner main.launch

# 车位比车还窄（应触发「无碰撞」失败）
roslaunch reverse_parking_planner main.launch slot_width:=1.7

# 只看参数是否生效
rosparam get /plan/turn_radius
```

终端会打印这样一段报告（默认参数的真实输出）：

```
倒车入库规划报告
车辆：长 4.50 m，宽 1.80 m，轴距 2.70 m，最小转弯半径 4.40 m
车位：垂直车位 2.50 m x 5.30 m（宽 x 长），转弯半径 5.00 m，直线段 3.00 m
起点：Pose(x=5.000, y=-5.750, yaw=3.142)
终点：Pose(x=-0.000, y=2.250, yaw=-1.571)
路径：倒车 10.85 m，转角 90.0 度，采样 110 个
最小间距：0.494 m
断言：4 通过 / 0 失败（共 4 条）
  [PASS] 终点精度  偏差 0.000 m，容差 0.10 m
  [PASS] 无碰撞  与邻车最小间距 0.494 m，下限 0.00 m
  [PASS] 曲率上限  曲率 0.200 1/m，车辆上限 0.227 1/m
  [PASS] 路径长度  倒车 10.85 m，上限 12.00 m
结论：PASSED

俯视图（- 车位线  # 邻车  o 路径  @ 车身）：
...
```

## 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/parking/plan` | `nav_msgs/Path` | 完整路径，latched，可直接在 RViz 里看 |
| `/parking/report` | `std_msgs/String` | 报告全文（含断言与俯视图），latched |
| `/parking/plan_point` | `geometry_msgs/PoseStamped` | 按 `rate/publish_hz` 逐点回放 |

```bash
rostopic echo -n 3 /parking/report/data
rostopic echo -n 5 /parking/plan_point/pose/position
```

## 离线测试

`tests/test_reverse_parking_planner_local.py` 只依赖标准库，不需要 ROS、CARLA 或显示器：

```bash
# 在仓库根目录
python3 tests/test_reverse_parking_planner_local.py
```

它会检查参数默认值与读取、路径几何（起点/终点/长度/曲率）、碰撞检测、四种断言、
报告格式、俯视图、以及 `package.xml` / `CMakeLists.txt` / `main.launch` 的结构约定。

## 与 CARLA 的关系

| | 这个包 | `docs/ground/carla_parking_lot.md` 里的 CARLA 方案 |
| --- | --- | --- |
| 依赖 | 只要 ROS 1 + Python | CARLA 0.9.16 + ROS 2 Humble + Python 3.10 |
| 输出 | 终端报告 + 路径话题 | 在 Town04 里画出绿色车位线框 |
| 用途 | 规划逻辑的离线回归 | 场景可视化演示 |

两者不冲突：本包可以给 CARLA 方案提供「停得进去吗、离邻车多远」这类数值校验。

## 常见问题

* **`ParkingError: 转弯半径 3.000 m 小于车辆最小转弯半径 4.404 m`**
  说明 `plan/turn_radius` 比车能转的最小半径还小。把 `turn_radius` 调大，或者把
  `vehicle/max_steer` 调大（例如 `0.70`）。

* **`[FAIL] 无碰撞`**
  车位太窄或直线段太短，车身在切入时会碰到邻车。加大 `slot_width` 或
  `entry_length` 即可。

* **`[FAIL] 路径长度`**
  默认 `R·pi/2 + entry` 约 10.85 m，超过 `check/max_path` 就会失败；把上限调大即可。
