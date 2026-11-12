# speed_profile_check

速度曲线规划：给定「这一段要开多远、想巡航多快、车能踩多狠」，规划出一条
**加速 → 匀速 → 减速** 的速度曲线，并判断它是否满足限速、加速度、距离与时间预算
四条要求。纯标准库实现，**不依赖 CARLA、不需要 GPU、不需要显示器**，一条命令在
终端出报告。

## 为什么需要它

`twist_to_control` 解决「指令 → 转角」，`braking_distance_check` 解决「从看见到停住
要几米」，中间还缺一环：**上一段路该怎么开**。手工把速度写成 0 → 40 → 0 容易忘掉
两件事：加速段要多少米、减速段要多少米；加起来超过实际距离时，曲线根本走不完。

本包把关这一点：加速段距离 `v²/(2a)` + 减速段距离 `v²/(2d)` 一旦超过总距离，就自动
退化成三角形曲线，把峰值速度降下来 —— 而不是给一条物理上跑不通的曲线。

## 目录结构

```
src/ground/speed_profile_check/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/speed_profile_check.yaml   # 12 个参数默认值
├── launch/main.launch                # 12 个带 doc 的 arg
└── scripts/
    ├── main.py                       # roslaunch 的入口
    ├── speed_core.py                 # 纯曲线计算（不 import rospy）
    └── speed_node.py                 # ROS 收发放这里
tests/test_speed_profile_check_local.py
```

## 曲线怎么算

设巡航速度 `v`（m/s）、初速 `v0`、加速度 `a`、减速度 `d`、总距离 `L`：

| 段 | 时长 | 距离 |
|---|---|---|
| 加速 | `(v - v0) / a` | `(v² - v0²) / (2a)` |
| 匀速 | `匀速距离 / v` | `L - 加速距离 - 减速距离` |
| 减速 | `v / d` | `v² / (2d)` |

若 `加速距离 + 减速距离 > L`（距离太短、来不及加到巡航速度），则退化成三角形：
令 `L = (v² - v0²)/(2a) + v²/(2d)`，反解出峰值 `v`，匀速段长度为 0。

## 四条断言

| 断言 | 判据 | 不过时的含义 |
|---|---|---|
| 限速 | 峰值速度 ≤ 路段限速 | 巡航速度定高了 |
| 加速度 | 加速/减速都是正数且在车辆能力内 | 参数写错 |
| 距离 | 规划距离与目标距离误差 ≤ 容差 | 曲线没走完全程 |
| 时间 | 总时长 ≤ 时间预算 | 这段路太慢，要赶时间 |

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `scene/distance` | 120.0 | 目标行驶距离（m） |
| `scene/cruise_speed_kmh` | 40.0 | 巡航速度（km/h） |
| `scene/start_speed_kmh` | 0.0 | 初速（km/h） |
| `vehicle/accel` | 1.2 | 加速度上限（m/s²） |
| `vehicle/decel` | 2.0 | 减速度上限（m/s²） |
| `check/speed_limit_kmh` | 50.0 | 路段限速（km/h） |
| `check/time_budget` | 30.0 | 允许用时（s） |
| `check/tolerance` | 0.5 | 距离允许误差（m） |
| `sim/dt` | 0.05 | 采样步长（s） |
| `rate/publish_hz` | 5.0 | 报告重发频率（Hz） |
| `topic/profile` | `/speed/profile` | 速度曲线话题 |
| `topic/report` | `/speed/report` | 报告话题 |

## 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/speed/profile` | `std_msgs/String`（latched） | 速度曲线的字符图 |
| `/speed/report` | `std_msgs/String`（latched） | 完整报告（分段、断言、结论） |

## 运行

```bash
cd ~/catkin_ws9
source /opt/ros/melodic/setup.bash
source devel/setup.bash
roslaunch speed_profile_check main.launch
```

默认工况（120 m、巡航 40 km/h、a = 1.2、d = 2.0）的实际输出：

```
曲线类型：梯形（含匀速段）
巡航：峰值 40.00 km/h，初速 0.00 km/h，规划距离 120.00 m
分段：加速 9.26 s / 51.44 m，匀速 3.39 s / 37.70 m，减速 5.56 s / 30.86 m
总时长 18.21 s，平均车速 23.73 km/h
断言：4 通过 / 0 失败（共 4 条）
  [PASS] 限速   峰值 40.00 km/h，限速 50.00 km/h
  [PASS] 加速度  加速 1.20 m/s^2、减速 2.00 m/s^2
  [PASS] 距离   规划 120.00 m，目标 120.00 m（容差 0.50 m）
  [PASS] 时间   用时 18.21 s，预算 30.00 s
结论：PASSED
```

后面还有一张速度曲线的字符图（横轴时间、纵轴速度）：

```
                             *************
                          ***#############**
                        **##################**
                     ***######################*
                  ***##########################**
                **###############################*
             ***##################################**
          ***#######################################**
       ***############################################*
     **################################################**
  ***####################################################**
**#########################################################*
0------------------------------------------------------18 s
```

## 两种典型情形

**距离太短 → 自动降峰值（三角形）**

```bash
roslaunch speed_profile_check main.launch distance:=60
```

峰值自动降到 **34.15 km/h**（匀速段为 0），总时长 12.65 s —— 因为 40 km/h 时加速段
要 51.44 m、减速段要 30.86 m，加起来 82.30 m 已经超过 60 m。

**巡航速度超过限速 → 只有「限速」失败**

```bash
roslaunch speed_profile_check main.launch cruise_speed_kmh:=25 speed_limit_kmh:=20
```

`[FAIL] 限速 峰值 25.00 km/h，限速 20.00 km/h`，其余三条仍通过。

**时间预算不够 → 只有「时间」失败**

```bash
roslaunch speed_profile_check main.launch time_budget:=15
```

`[FAIL] 时间 用时 18.21 s，预算 15.00 s`。

## 离线测试

```bash
cd <仓库根目录>
python3 tests/test_speed_profile_check_local.py
```

只用标准库，不需要 ROS、CARLA、显示器；同时兼容 Python 2.7 与 3.x：

```
通过 248 项，失败 0 项
全部通过
```

也可以交给 pytest（每个分组被包装成真正会失败的 `test_*`）：

```bash
python3 -m pytest tests/test_speed_profile_check_local.py -q
```

## 与已有模块的关系

| 模块 | 关注点 |
|---|---|
| `twist_to_control` | 速度指令 → 转角/轮速（执行量） |
| `braking_distance_check` | 从看见到停住要几米（纵向安全） |
| `turning_sweep_check` | 转弯时车身占多宽（横向包络） |
| `rostopic_cruise` | 手动发指令巡航 |
| 本模块 | 一段路的速度曲线怎么规划（时间-速度） |

## 方案对比

同一段距离、不同巡航速度的方案可以一起比较：

    plans = core.compare_plans(120.0, 40.0, 1.2, 2.0, 50.0)
    print("\n".join(core.format_plans(plans, 50.0, 120.0)))
    best = core.recommend_plan(plans)   # 不超限里用时最短的那条

默认场景（120 m、限速 50）会推荐 50 km/h：再往上距离不够，50、60、70 都退化成峰值 48.30 km/h 的同一个三角形。

## 常见问题

**为什么不直接上 S 型（jerk 限制）曲线？**
三角形的 jerk 是无穷大，真实控制器会自己平滑。本模块的目标是「规划层能不能走完这段
距离、时间和限速是否满足」，把 jerk 留给下层控制器是常见做法；需要时可在此基础上加
加速度斜坡。

**平均车速为什么只有 23.73 km/h，巡航不是 40 吗？**
因为起点和终点速度都是 0，加速段与减速段占了大半时间（9.26 + 5.56 = 14.81 s，共
18.21 s）。这正是「匀速段短、加减速段长」的典型短途工况。

**Melodic 上会报 `UnicodeEncodeError` 吗？**
不会。`speed_node.py` 的 `_log()` 先把消息编码成 byte str 再交给 `rospy`。

## 大模型声明

使用大模型辅助代码编写、测试编写与文档整理，提交内容由本人负责。
