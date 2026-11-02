# CARLA 车道偏离预警（LDW）

> 从地图路点计算自车相对车道中心线的带符号横向偏移，
> 三级滞回状态机（normal → deviation → departure）输出带方向的
> 车道偏离预警，支持纯离线测试与连 CARLA 真实偏离场景演示。

## 1. 概述与目标

本模块实现乘用车 **LDW（Lane Departure Warning，车道偏离预警）** 的
完整闭环，在 CARLA 0.9.16 中可一键复现：

| 环节 | 实现 |
|---|---|
| 感知 | 地图 API：自车位置投影到行驶车道中心线（LaneType.Driving）→ 带符号横向偏移 + 航向偏差 + 车道宽 |
| 数据质量 | 跳变剔除：单拍（0.05 s）偏移突变超 1.5 m 判定为相邻车道投影假值，限幅步进 |
| 决策 | 三级滞回状态机 normal/deviation/departure（进入/退出双阈值消除抖动） |
| 输出 | 分级状态 + 带方向跃迁事件（deviation_start_left/right、departure_start_*、deviation_clear、departure_end） |

代码位于 `src/ground/carla_lane_departure_warning/`。

### 1.1 与仓库中其它地面载具模块的区分

| 模块 | 输入 | 算法 | 特点 |
|---|---|---|---|
| 自动紧急制动 AEB（`carla_aeb_braking`） | 前向 LiDAR 点云 | 规则式滞回状态机 | 前向障碍制动，需要传感器 |
| 盲区监测 BSD（`carla_bsd_blind_spot`） | 侧向 LiDAR 点云 | 三级预警状态机 | 侧向预警，只报不控 |
| 弯道自适应限速 CSL（`carla_curve_speed_limit`） | 地图路点 | v=√(a·R) 限速 + 斜坡 | 舒适性件，只控速度 |
| **本模块** | **地图路点（零传感器）** | **横向偏移几何 + 三级滞回** | **车道保持预警，带方向事件** |

LDW 与 CSL 同为**零传感器依赖**模块（数据源是地图拓扑），但决策维度不同：
CSL 沿道路纵向看曲率（入弯减速），LDW 沿横向看偏移（出线预警）。
全部决策逻辑（偏移几何 + 跳变剔除 + 状态机）为纯离线函数，
40 个单元测试不依赖 ROS 与 CARLA 即可完整覆盖。

### 1.2 为什么需要跳变剔除

CARLA `map.get_waypoint()` 返回**最近**的可行驶车道路点。自车大幅偏离
或靠近分岔时，最近点可能跳到相邻车道/对向车道，参考中心线瞬移
（≈一个车道宽），横向偏移出现 1~2 m 的台阶假值。实测 45 s 中此类跳变
可造成状态机每 0.3 s 一次假振荡。剔除规则：0.05 s 内偏移变化超过
1.5 m（6 m/s 车速下物理不可能）时，沿用上拍方向限幅步进 1.5 m，
把投影跳变压成最大可信步长。

## 2. 计算原理

### 2.1 带符号横向偏移

道路方向单位向量 dir=(cos y, sin y)（y 为道路朝向 yaw，逆时针为正），
左侧法向 left=(−sin y, cos y)，自车 e 相对中心线参考点 w：

```
offset = (e − w) · left = −(ex−wx)·sin y + (ey−wy)·cos y
```

左偏为正、右偏为负；沿道路方向的前后错位不产生横向偏移。

### 2.2 航向偏差

`heading_err = wrap(ego_yaw − road_yaw) ∈ [−π, π]`，正值表示车头朝左偏，
作为趋势参考输出（不参与状态机判定）。

### 2.3 三级滞回状态机

```
normal ──|o| ≥ depart_enter(0.9)──▶ departure
normal ──|o| ≥ warn_enter(0.5)───▶ deviation
deviation ──|o| ≥ depart_enter───▶ departure
deviation ──|o| < warn_exit(0.3)──▶ normal
departure ──|o| < depart_exit(0.65)▶ deviation
```

双阈值滞回带 [0.3, 0.5] 与 [0.65, 0.9] 消除临界抖动；事件仅在真实
跃迁时发出一次，方向由偏移符号给出（`_left` / `_right`）。

## 3. 模块结构

```
src/ground/carla_lane_departure_warning/
├── carla_lane_departure_warning/
│   ├── lane_departure_core.py  # 决策核心：偏移几何/跳变剔除/三级滞回（纯离线）
│   ├── bridge_node.py          # CARLA 桥接：路点投影 → /ldw/lateral_offset 等
│   ├── ldw_node.py             # 预警节点：定时器推进状态机 → /ldw/state + /ldw/event
│   ├── carla_common.py         # CARLA 封装（复用 AEB/BSD/CSL 已验证实现）
│   └── demo.py                 # 离线/真实 CARLA 一键演示
├── launch/main.launch.py       # 桥接+预警两节点一键启动
├── config/ldw_params.yaml      # 全部决策参数（与 LDWParams 一一对应）
├── tests/                      # 40 个离线单元测试（宿主机无需 ROS/CARLA）
└── main.py                     # doctor / test / build / demo 统一入口
```

## 4. ROS 接口

| 话题 | 方向 | 类型 | 含义 |
|---|---|---|---|
| `/ldw/lateral_offset` | 桥接 → 预警 | Float32 | 带符号横向偏移（m，左偏正，跳变剔除后） |
| `/ldw/heading_error` | 桥接 → 外部 | Float32 | 航向偏差（rad） |
| `/ldw/lane_width` | 桥接 → 外部 | Float32 | 当前车道宽（m） |
| `/ldw/state` | 预警 → 外部 | String | normal / deviation / departure |
| `/ldw/event` | 预警 → 外部 | String | 带方向跃迁事件（仅跃迁拍发布一次） |

## 5. 快速开始与结果复现

### 5.1 离线测试（无需 ROS 与 CARLA）

```bash
cd src/ground/carla_lane_departure_warning/
python3 main.py doctor     # 依赖体检
python3 main.py test       # 40 个离线单元测试
python3 main.py demo       # 离线演示：正弦偏离轨迹
```

离线场景为自车沿直道 y=0 以 6 m/s 行驶、横向按正弦 y(t)=1.2·sin(2πt/20)
偏离：幅值跨过 warn(0.5) 与 depart(0.9) 阈值，左右两侧各触发一轮完整
deviation → departure → 恢复事件链。

### 5.2 CARLA 实测

先启动 CARLA 0.9.16（宿主机离屏渲染），VM 内执行
（`--host` 填 CARLA 所在机器 IP）：

```bash
# 方式一：源码直跑（目录：src/ground/carla_lane_departure_warning/）
python3 main.py demo --live --host <CARLA_IP> --seconds 48

# 方式二：colcon 构建 + launch（目录：ros2 工作空间根目录）
colcon build --packages-select carla_lane_departure_warning && source install/setup.bash
ros2 launch carla_lane_departure_warning main.launch.py host:=<CARLA_IP>
# 另开终端：ros2 topic echo /ldw/state
```

实测现象（Town01，16 s 周期纯追踪偏离：居中 → 左偏 1.2 m（车道内）→ 回中）：
自车起步加速后稳定 5 m/s 巡航，偏离设定值下达 4 s 内横向偏移单调逼近
1.2 m 并稳定保持——`deviation` 进入（>0.5 m）→ `departure` 压线（>0.9 m）→
回中后 `departure_end` → `deviation_clear` 恢复，全程 3 轮完整事件链、
零假振荡（跳变剔除生效）。

## 6. 演示结果

### 6.1 事件时间线（CARLA 实测 48 s）

| 轮次 | deviation_start | departure_start | departure_end | deviation_clear |
|---|---|---|---|---|
| 1 | t=5.05 s | t=6.30 s | t=11.70 s | t=12.90 s |
| 2 | t=18.45 s | t=18.70 s | t=27.05 s | t=28.25 s |
| 3 | t=36.95 s | t=38.25 s | t=43.00 s | t=44.20 s |

每轮恰好 4 个事件、顺序固定、轮间 normal 段约 4~5 s（居中段），
departure 状态合计 371/960 拍（38.7 s）。第 1 轮偏移峰值约 2.8 m 为起步
加速段纯追踪暂态，第 2/3 轮收敛至设定值 1.2 m ± 0.1 m。

### 6.2 指标汇总

| 指标 | 离线演示 | CARLA 实测 |
|---|---|---|
| 状态链 | 三级齐全、左右对称触发 | 3 轮 × 4 事件，顺序固定 |
| 设定值跟踪 | — | 收敛 1.2 m ± 0.1 m（第 2/3 轮） |
| 假振荡 | 0 | 0（跳变剔除生效，899 拍中仅 1 拍超步长） |
| 传感器依赖 | 无 | 无（纯地图拓扑） |

### 6.3 效果图

压线报警（departure）瞬间、偏离预警（deviation）瞬间、恢复（recover）
截图，以及偏移/状态/速度三行跟踪曲线：

![压线报警（departure）瞬间](https://raw.githubusercontent.com/82540295/ros2/feat/carla_lane_departure_warning/docs/img/ground/carla_lane_departure_warning/ldw_live_departure.png)

![偏离预警（deviation）瞬间](https://raw.githubusercontent.com/82540295/ros2/feat/carla_lane_departure_warning/docs/img/ground/carla_lane_departure_warning/ldw_live_deviation.png)

![LDW 实测曲线](https://raw.githubusercontent.com/82540295/ros2/feat/carla_lane_departure_warning/docs/img/ground/carla_lane_departure_warning/ldw_live_curve.png)

图片与逐拍 CSV 数据已入库 `docs/img/ground/carla_lane_departure_warning/`
（PNG 均 < 3 MB），复现命令见第 5 节。

## 7. 参数说明

全部参数集中于 `config/ldw_params.yaml`，与代码 `LDWParams` 一一对应
（阈值按默认车道宽 3.5 m 标定，即半宽 1.75 m）：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `warn_enter` / `warn_exit` | 0.5 / 0.3 | deviation 进/出阈值（m，半宽 28%~17%） |
| `depart_enter` / `depart_exit` | 0.9 / 0.65 | departure 进/出阈值（m，接近压线） |
| `lane_width` | 3.5 | 车道宽（m），bridge 按路点动态发布 |

## 8. 已知限制

- 阈值按 3.5 m 标准车道标定；窄车道（如 3 m）应等比下调 warn/depart 阈值，
  宽车道反之（改 yaml 参数即可）；
- 跳变剔除假设车速 < 6 m/s 量级；高速场景 max_step 应随速度放宽；
- 模块只预警不控车——演示中的纯追踪驾驶为演示基建，接入整车时应将
  /ldw/event 送入提示音/视觉提示，并可与车道纠偏功能（主动干预）联动；
- 路口/分岔区域的最近车道投影天然不稳定，本模块在该区域事件密度会
  升高（属已知行为，实际产品应结合车道拓扑过滤）。
