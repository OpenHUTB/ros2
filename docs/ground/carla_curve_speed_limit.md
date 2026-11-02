# CARLA 弯道自适应限速（CSL）

> 从地图路点计算前方弯道曲率，按 v=√(a·R) 反解安全过弯速度，
> 滞回状态机 + 加减速斜坡使自车"入弯前提前减速、出弯平顺恢复巡航"。
> 提供离线速度跟踪曲线演示与连 CARLA 真实环路演示两条验证路径。

## 1. 概述与目标

本模块实现乘用车 **CSL（Curve Speed Limit，弯道自适应限速）** 功能的
完整闭环，在 CARLA 0.9.16 中可一键复现：

| 环节 | 实现 |
|---|---|
| 感知 | 地图 API：自车前方 30 m 路点 → 相邻三点外接圆曲率（带符号） |
| 决策 | v=√(a·R) 弯道限速 → 预瞄取最急弯 → min(巡航, 限速) 目标速度 → 滞回状态机 cruise/slowdown |
| 控制 | P 控制律 + 加减速斜坡限幅 → 油门/制动互斥输出 |

代码位于 `src/ground/carla_curve_speed_limit/`。

### 1.1 与仓库中其它地面载具模块的区分

| 模块 | 输入 | 算法 | 特点 |
|---|---|---|---|
| 自动紧急制动 AEB（`carla_aeb_braking`） | 前向 LiDAR 点云 | 规则式滞回状态机 | 前向障碍制动，需要传感器 |
| 盲区监测 BSD | 侧向 LiDAR 点云 | 三级预警状态机 | 侧向预警，只报不控 |
| **本模块** | **地图路点（零传感器）** | **v=√(a·R) 限速 + 滞回斜坡** | **舒适性件，预瞄减速 + 平顺恢复** |

CSL 是典型**舒适性/稳定性件**：弯道限速全部由显式物理参数
（横向加速度上限 `a_lat_max`）推导，决策可解释、可测试；与 AEB/BSD 依赖
LiDAR 点云不同，CSL 的数据源是**地图拓扑**——路点曲率的前瞻性由地图保证，
因此是仓库中首个零传感器依赖的决策模块。全部决策逻辑（曲率 + 限速 +
状态机 + 斜坡）均为纯离线函数，49 个单元测试不依赖 ROS 与 CARLA 即可完整覆盖。

### 1.2 为什么限速是上限而不是设定值

早期版本直接把弯道限速作为目标速度输出，实测暴露：直道限速=valid_max
（30 m/s）远高于巡航速度，控制器把 30 当成目标一路加速到 12.9 m/s
（巡航设定 8.0）。修正后的语义：

```
target_speed = min(巡航速度, 预瞄弯道限速)
```

限速只在它**低于巡航速度**时才起作用——这正是"限速"的物理含义。

## 2. 计算原理

### 2.1 路点曲率（三点外接圆法）

给定自车前方按行驶方向排序的路点序列 (x, y)，对相邻三点
p0、p1、p2 计算外接圆曲率：

```
kappa = 2·cross(p1-p0, p2-p1) / (|p1-p0|·|p2-p1|·|p2-p0|)
```

- 共线时 cross=0 → kappa=0（直道）；
- 带符号：左转为正、右转为负（取绝对值参与限速计算）；
- 路点按 2 m 步长沿车道中心线收集（`wp.next(2.0)` 循环累积，
  CARLA 0.9.16 无 `next_until_look_ahead` API）。

### 2.2 弯道限速与预瞄取严

由横向加速度物理约束 v²/R ≤ a_lat_max 反解：

```
v_max = √(a_lat_max / |kappa|) = √(a_lat_max · R)
```

- `a_lat_max = 2.5 m/s²`（舒适阈值，显式参数）；
- 直道（|kappa| < 1e-6）返回哨兵速度 valid_max=30；
- **预瞄取严**：对前瞻窗口（30 m）内全部路点逐点计算限速，
  取最小值——入弯前提前减速，而非进弯才刹。

### 2.3 滞回状态机与斜坡平滑

```
cruise ──目标低于巡航超过 slow_enter(0.5)──▶ slowdown
slowdown ──差距收窄到 slow_exit(0.2) 以内──▶ cruise
```

- 滞回带 [slow_exit, slow_enter] 消除临界抖动；
- 事件 slowdown_warn / cruise_resume 仅在真实跃迁时发布一次；
- **加减速斜坡**：指令速度向目标速度以 decel_max=2.0 / accel_max=1.5 m/s²
  限幅逼近——弯道限速阶梯变化被平滑为可执行的加减速，弯中不低于
  v_min=1.5 m/s（防弯中停车）。

### 2.4 P 控制律

```
e = commanded_speed − ego_speed
e > 0 → throttle = min(kp_throttle·e, 1.0)
e < 0 → brake    = min(kp_brake·(−e), 1.0)     （油门/制动互斥）
```

## 3. 模块结构

```
src/ground/carla_curve_speed_limit/
├── carla_curve_speed_limit/
│   ├── curve_speed_core.py     # 决策核心：曲率/限速/滞回/斜坡（纯离线）
│   ├── bridge_node.py          # CARLA 桥接：地图路点 → /csl/target_speed 等
│   ├── csl_node.py             # 速度控制节点：滞回+斜坡 → /csl/control
│   ├── carla_common.py         # CARLA 封装（复用 AEB 已验证实现）
│   └── demo.py                 # 离线/真实 CARLA 一键演示
├── launch/main.launch.py       # 桥接+控制两节点一键启动
├── config/csl_params.yaml      # 全部决策参数（与 CSLParams 一一对应）
├── tests/                      # 49 个离线单元测试（宿主机无需 ROS/CARLA）
└── main.py                     # doctor / test / build / demo 统一入口
```

## 4. ROS 接口

| 话题 | 方向 | 类型 | 含义 |
|---|---|---|---|
| `/csl/target_speed` | 桥接 → 控制 | Float32 | 预瞄目标速度 min(巡航, 限速) |
| `/csl/ego_speed` | 桥接 → 控制 | Float32 | 自车实测速度（m/s） |
| `/csl/curve_radius` | 桥接 → 外部 | Float32 | 前瞻最急弯半径（直道 999 哨兵） |
| `/csl/control` | 控制 → 桥接 | Float32MultiArray | [throttle, brake] |
| `/csl/state` | 控制 → 外部 | String | cruise / slowdown |
| `/csl/event` | 控制 → 外部 | String | slowdown_warn / cruise_resume |

## 5. 快速开始与结果复现

### 5.1 离线测试（无需 ROS 与 CARLA）

```bash
cd src/ground/carla_curve_speed_limit/
python3 main.py doctor     # 依赖体检
python3 main.py test       # 49 个离线单元测试
python3 main.py demo       # 离线演示：直道-弯道-直道 速度跟踪
```

离线场景为"直道 60 m → R=15 圆弧 → 直道 60 m"：入弯前预瞄减速
（8.0 → 6.1 m/s，√(2.5×15)），弯中稳定，出弯斜坡恢复巡航，
 slowdown 与 cruise 事件各出现两次。

### 5.2 CARLA 实测

先启动 CARLA 0.9.16（宿主机离屏渲染），VM 内执行
（`--host` 填 CARLA 所在机器 IP）：

```bash
# 方式一：源码直跑（目录：src/ground/carla_curve_speed_limit/）
python3 main.py demo --live --host <CARLA_IP> --seconds 45

# 方式二：colcon 构建 + launch（目录：ros2 工作空间根目录）
colcon build --packages-select carla_curve_speed_limit && source install/setup.bash
ros2 launch carla_curve_speed_limit main.launch.py host:=<CARLA_IP>
# 另开终端：ros2 topic echo /csl/state
```

实测现象（Town01 环路，spawn 于弯道处）：自车起步即按限速减速跟踪，
进入较大半径路段恢复巡航（实际速度全程 ≤ 7.25 m/s，未超巡航设定），
状态链 slowdown → cruise → slowdown → cruise → slowdown 完整，
每次跃迁自动截图。

## 6. 演示结果

### 6.1 事件时间线（CARLA 实测 45 s）

| 时刻 | 事件 | 目标速度 | 说明 |
|---|---|---|---|
| t=0.0s | slowdown_warn | 4.8 m/s | spawn 即处于 R≈9 m 弯道 |
| t=12.15s | cruise_resume | 8.0 m/s | R≈25 m 大半径段恢复巡航 |
| t=12.35s | slowdown_warn | 7.0 m/s | 再次进入 R≈19 m 弯道 |
| t=14.15s | cruise_resume | 8.0 m/s | 回到大半径段 |
| t=34.15s | slowdown_warn | 6.1 m/s | R≈15 m 弯道，预瞄提前减速 |

### 6.2 指标汇总

| 指标 | 离线演示 | CARLA 实测 |
|---|---|---|
| 弯道限速收敛 | v 收敛到 √(2.5×15)≈6.1 m/s（±0.5） | 目标-实际稳态差 ≤ 0.5 m/s |
| 速度上限 | 末段直道恢复 ≈8.0 | 全程 ≤ 7.25 m/s（< 巡航 8.0） |
| 状态链 | slowdown → cruise ×2 | 5 次跃迁事件完整 |
| 传感器依赖 | 无 | 无（纯地图拓扑） |

### 6.3 效果图

弯道减速瞬间与出弯恢复截图、目标/指令/实际速度三线跟踪曲线：

![弯道减速](https://raw.githubusercontent.com/82540295/ros2/feat/carla_curve_speed_limit/docs/img/ground/carla_curve_speed_limit/csl_live_slowdown.png)

![出弯恢复](https://raw.githubusercontent.com/82540295/ros2/feat/carla_curve_speed_limit/docs/img/ground/carla_curve_speed_limit/csl_live_resume.png)

![CSL 实测曲线](https://raw.githubusercontent.com/82540295/ros2/feat/carla_curve_speed_limit/docs/img/ground/carla_curve_speed_limit/csl_live_curve.png)

图片与逐拍 CSV 数据已入库 `docs/img/ground/carla_curve_speed_limit/`
（PNG 均 < 3 MB），复现命令见第 5 节。

## 7. 参数说明

全部参数集中于 `config/csl_params.yaml`，与代码 `CSLParams` 一一对应：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `cruise_speed` | 8.0 | 巡航速度（m/s） |
| `a_lat_max` | 2.5 | 横向加速度上限（m/s²），决定弯道限速 |
| `look_ahead` | 30.0 | 预瞄距离（m），越大减速越提前 |
| `v_min` | 1.5 | 弯道最低速度（m/s） |
| `decel_max` / `accel_max` | 2.0 / 1.5 | 加减速斜坡限幅（m/s²） |
| `slow_enter` / `slow_exit` | 0.5 / 0.2 | 状态机滞回带（m/s） |
| `valid_max` | 30.0 | 直道哨兵限速（m/s） |

## 8. 已知限制

- 路点曲率按车道中心线 2 m 步长差分，路口分叉处取 `next()` 首个候选，
  在多岔路口的曲率可能跳变（滞回与斜坡可吸收，但不保证最优车道选择）；
- 限速模型未考虑路面附着系数（雨雪天 a_lat_max 应下调，改参数即可）；
- 模块只控速度不控转向——演示与 launch 场景中的车道保持由桥接内嵌的
  最小航点跟随器提供，接入整车时请使用底盘自带的横向控制。
