# trailer_kinematics：牵引车-挂车运动学与折刀判定

一个**不依赖 CARLA、不依赖 GPU、只要有 Python 就能跑**的牵引车-挂车仿真检查器。

给它一组几何参数（轴距、挂车长度、铰接点偏移）和一次操作（车速、前轮转角、前进还是倒车），
它会把两个航向状态积分出来，然后回答四个问题：

1. 铰接角离折刀还有多少余量？
2. 这份积分结果可信吗（步长减半后位置差多大）？
3. 转弯时组合体要占多宽的通道？
4. 倒车时铰接角是在收敛还是在发散？

## 为什么需要这一层

挂车不是「加长版单车」：它的航向是**另一个状态量**，靠铰接点的横向速度驱动，
所以存在一个牵引车没有的现象 —— **折刀**（trailer jackknife）。倒车时这个状态还
是不稳定的：只要方向盘保持一个角度，铰接角就会一直涨到折刀。

这类问题在实车和仿真器里都不便宜：要么开 CARLA 等着跑，要么上路试。但它的核心
其实只是两个一阶微分方程 + 一个闭式解，完全可以在离线、无图形界面、任何 Python
解释器上把「余量够不够」算清楚。

## 目录结构

```
src/ground/trailer_kinematics/
├── CMakeLists.txt
├── package.xml
├── README.md                     ← 本文件
├── config/
│   └── trailer_kinematics.yaml   ← 13 个默认参数
├── launch/
│   └── main.launch               ← 每个参数带 doc，可命令行覆盖
└── scripts/
    ├── main.py                   ← 入口（仓库约定）
    ├── trailer_core.py           ← 纯计算，不 import rospy
    └── trailer_node.py           ← ROS 收发
```

## 模型

记牵引车后轴处航向为 θ₁、挂车航向为 θ₂、铰接角 φ = θ₁ − θ₂：

```
θ₁' = v · tan(δ) / L1
θ₂' = ( v·sin(φ) − d·θ₁'·cos(φ) ) / L2
```

| 符号 | 含义 | 默认值 |
|---|---|---|
| L1 | 牵引车轴距 | 2.70 m |
| L2 | 铰接点到挂车轴 | 4.20 m |
| d | 铰接点在牵引车后轴后方的偏移（0 即「铰接在轴上」） | 0.60 m |
| δ | 前轮转角 | 25° |

稳态（φ' = 0）有闭式解，记 A = tan δ / L1：

```
sin(φ) − d·A·cos(φ) = L2·A
φ_steady = atan(dA) + asin( L2·A / sqrt(1 + (dA)²) )
```

默认参数下 φ_steady ≈ **52.09°**；把 d 设为 0 则退化成经典的
φ_steady = asin(L2·tan δ / L1) ≈ **46.50°**。这两条数字写在离线测试里，
一旦积分代码改坏，测试会立刻失败。

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `trailer/wheelbase` | 2.70 | 牵引车轴距 (m) |
| `trailer/length` | 4.20 | 铰接点到挂车轴 (m) |
| `trailer/hitch_offset` | 0.60 | 铰接点在牵引车后轴后方 (m) |
| `trailer/max_steer_deg` | 35.0 | 前轮最大转角 (度) |
| `check/jackknife_deg` | 90.0 | 折刀角上限 (度) |
| `check/channel_width` | 12.00 | 允许占用的通道宽度 (m) |
| `check/step_tolerance` | 0.01 | 步长减半后的位置差上限 (m) |
| `sim/speed` | 1.00 | 行驶速度 (m/s) |
| `sim/steer_deg` | 25.0 | 前轮转角 (度) |
| `sim/seconds` | 9.10 | 仿真时长 (s)，默认正好转 90° |
| `sim/step` | 0.02 | 积分步长 (s) |
| `sim/gear` | 1 | 1 前进、-1 倒车 |
| `rate/publish_hz` | 10.0 | 报告发布后的保活频率 (Hz) |

## 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/trailer/report` | `std_msgs/String` | 完整报告（latched） |
| `/trailer/phase` | `std_msgs/String` | 铰接角相图（latched） |
| `/trailer/articulation` | `std_msgs/Float64` | 最大铰接角（latched） |

## 运行

```bash
# 默认：前进 25°，转 90°，通道 12 m
roslaunch trailer_kinematics main.launch

# 倒车（会折刀，用来演示 FAIL）
roslaunch trailer_kinematics main.launch gear:=-1

# 通道只给 8 m
roslaunch trailer_kinematics main.launch channel_width:=8.0

# 用粗步长（积分精度会不达标）
roslaunch trailer_kinematics main.launch step:=0.5
```

默认参数的输出：

```
牵引车转角：90.05 度
铰接角：最大 43.27 度，末段 42.52 度，首段 4.75 度
占用通道宽度：10.96 m
断言：4 通过 / 0 失败（共 4 条）
[PASS] 折刀余量     最大铰接角 43.27 度，折刀角 90.00 度（余量 46.73 度）
[PASS] 积分精度     步长 0.020 与 0.010 的位置差 0.00707 m，上限 0.010 m
[PASS] 通道宽度     占用宽度 10.96 m，通道 12.00 m
[PASS] 倒车稳定性    前进：首段 4.75 度 → 末段 42.52 度，不适用倒车发散判定
结论：PASSED
```

## 离线测试

不需要 ROS、不需要 CARLA、不需要显示器：

```bash
python3 tests/test_trailer_kinematics_local.py
python3 -m pytest tests/test_trailer_kinematics_local.py -q
```

脚本会打印 `通过 N 项，失败 M 项` 并以退出码表示结果；
pytest 下每个分组是一个 `test_*`，失败时会真的抛 `AssertionError`（不是假绿）。

## 常见问题

**为什么默认跑 9.10 秒？**
按 L1 = 2.70 m、δ = 25° 算，航向角速度是 0.1727 rad/s，转 90° 正好 9.10 s。
这样一个默认场景就是「路口右转弯」，通道宽度这个指标才有意义。

**倒车为什么一定 FAIL？**
不是代码坏了，是物理如此：倒车时铰接角的不动点是不稳定的，方向盘保持一个角度，
铰接角就会单调增大到折刀。`gear:=-1` 就是为了把这条说清楚。

**为什么还要检查积分精度？**
显式欧拉在 0.02 s 步长下的误差只有 7 mm，但放大到 0.5 s 就变成 17 cm。
「步长减半位置差小于 1 cm」这条断言，是让文档里那些「误差可忽略」的说法有据可依。

**和 CARLA 的 scenario runner 是什么关系？**
CARLA 的 scenario runner 管的是「整个世界怎么演」；这个包只管「挂车这一个自由度的
运动学和稳定性」。前者需要仿真器，后者任何 Python 都能跑，用来给挂车相关模块做
回归测试的护栏。

---

所使用具体的大模型声明：使用大模型辅助代码编写、测试编写与文档整理，提交内容由本人负责。

## TF 广播（ROS 原生形态）

节点算完轨迹后会持续广播变换（`tf/enabled` 默认 `true`，频率 `tf/rate_hz` 默认 10 Hz），
因此除了看终端报告，还可以用 ROS 自带的工具核对位姿：

```
$ rosrun tf tf_echo map tractor
$ rosrun tf view_frames     # 导出一份 tf 树到 frames.pdf
```

| parent | child | 含义 |
|---|---|---|
| `map` | `map -> tractor` | 牵引车后轴在全局地图里的位姿（轨迹终点状态） |
| `tractor` | `trailer` | 挂车轴在牵引车坐标系里的位姿（与报告里的铰接角一致） |

默认参数下 `map -> tractor` 的平移是 `(5.800, 5.785, 0.000)`、yaw 约 `1.5716 rad`；
`tractor -> trailer` 的平移是 `(-2.458, 2.879, 0.000)`。两张变换的相对航向
`-0.7552 rad` 正是 `trailer_yaw - tractor_yaw`，与报告中的 `铰接角 最大 43.27°` 互相印证。

依赖：`tf2_ros`、`geometry_msgs`（`package.xml` 已声明）。关掉广播用
`roslaunch trailer_kinematics main.launch tf_enabled:=false`。
