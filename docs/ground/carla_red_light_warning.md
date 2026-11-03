# CARLA 闯红灯预警（RLW）

> 交通灯状态 + 判停距离 → 三级滞回状态机
> （normal → approaching → violation_risk）→ 跃迁事件，
> 支持纯离线测试与连 CARLA 路口红绿灯场景演示。

## 1. 概述与目标

本模块实现乘用车 **RLW（Red Light Warning，闯红灯预警）** 的完整闭环，
在 CARLA 0.9.16 中可一键复现：

| 环节 | 实现 |
|---|---|
| 感知 | 交通灯 API：`vehicle.get_traffic_light()` 给出自车当前受控的灯，`get_stop_locations()`（0.9.14+）给停止线位置，计算自车到停止线距离；自车不受灯控时发布 off + 占位大距离 |
| 决策 | 判停距离 d_stop = v²/(2·a_comfort)；三级滞回状态机（黄灯只提示接近、红灯且舒适刹停不可行才判"将闯线"） |
| 输出 | 分级状态 + 跃迁事件（approach_start / violation_start / violation_clear / approach_clear） |

代码位于 `src/ground/carla_red_light_warning/`。

### 1.1 与仓库中其它地面载具模块的区分

| 模块 | 输入 | 算法 | 特点 |
|---|---|---|---|
| 自动紧急制动 AEB（`carla_aeb_braking`） | 前向 LiDAR 点云 | 规则式滞回状态机 | 前向障碍制动，需要传感器 |
| 盲区监测 BSD（`carla_bsd_blind_spot`） | 侧向 LiDAR 点云 | 三级预警状态机 | 侧向预警，只报不控 |
| 弯道自适应限速 CSL（`carla_curve_speed_limit`） | 地图路点 | v=√(a·R) 限速 + 斜坡 | 舒适性件，只控速度 |
| 车道偏离预警 LDW（`carla_lane_departure_warning`） | 地图路点 | 横向偏移 + 三级滞回 | 横向预警，只报不控 |
| **本模块（RLW）** | **交通灯 API（零传感器）** | **判停距离 + 三级滞回** | **路口纵向预警，带演示刹停基建** |

RLW 与 LDW/CSL 同为**零传感器依赖**模块，但数据源是交通灯 API 而非地图
拓扑——感知由仿真器直接给出（自车受控的灯 + 停止线），前瞻性由
"判停距离 vs 剩余距离"比较保证。全部决策逻辑（判停公式 + 状态机）为
纯离线函数，38 个单元测试不依赖 ROS 与 CARLA。

### 1.2 语义约定

- **黄灯不判闯线**：现实中黄灯越线不违规，黄灯只触发 approaching 提示；
- **violation_risk 仅在红灯且 d < d_stop 时触发**——即舒适减速已不可行，
  需要急刹或将越线的时刻；
- 模块只预警不控车：演示中的"红灯刹停"是演示基建（violation_risk 时
  按判停距离亏欠比例施加制动），接入整车时应把事件送入 HMI 提示。

## 2. 计算原理

### 2.1 判停距离

以舒适减速度 a_comfort（默认 2.5 m/s²）从速度 v 刹停所需距离：

```
d_stop = v² / (2 · a_comfort)
```

v = 8 m/s 时 d_stop ≈ 12.8 m。剩余停止线距离 d 与 d_stop 比较即"能否
舒适停下"。

### 2.2 三级滞回状态机

```
normal ──────黄/红灯且 d < warn_enter(30)──▶ approaching
approaching ──红灯且 d < d_stop────────────▶ violation_risk
approaching ──绿灯 或 d > warn_exit(35)────▶ normal
violation_risk ──绿灯─────────────────────▶ normal
violation_risk ──红灯且 d > d_stop×1.2─────▶ approaching
```

双阈值滞回（warn 30/35 m，brake 1.2×系数）消除临界抖动；事件仅在真实
跃迁时发出一次。

## 3. 模块结构

```
src/ground/carla_red_light_warning/
├── carla_red_light_warning/
│   ├── red_light_core.py   # 决策核心：判停距离/三级滞回/热身期（纯离线）
│   ├── bridge_node.py      # CARLA 桥接：交通灯感知 → /rlw/* 发布
│   ├── rlw_node.py         # 预警节点：定时器推进状态机 → /rlw/state + /rlw/event
│   ├── carla_common.py     # CARLA 封装（复用 AEB/BSD/CSL/LDW 已验证实现）
│   └── demo.py             # 离线/真实 CARLA 一键演示
├── launch/main.launch.py   # 桥接+预警两节点一键启动
├── config/rlw_params.yaml  # 全部决策参数（顶层 /** 通配，参数可生效）
├── tests/                  # 38 个离线单元测试（宿主机无需 ROS/CARLA）
└── main.py                 # doctor / test / build / demo 统一入口
```

## 4. ROS 接口

| 话题 | 方向 | 类型 | 含义 |
|---|---|---|---|
| `/rlw/light_state` | 桥接 → 预警 | String | red / yellow / green / off |
| `/rlw/dist_to_stop` | 桥接 → 预警 | Float32 | 自车到停止线距离（m；无灯时 1e4 占位） |
| `/rlw/ego_speed` | 桥接 → 预警 | Float32 | 自车速度（m/s） |
| `/rlw/state` | 预警 → 外部 | String | normal / approaching / violation_risk |
| `/rlw/event` | 预警 → 外部 | String | 跃迁事件（仅跃迁拍发布一次） |

## 5. 快速开始与结果复现

### 5.1 离线测试（无需 ROS 与 CARLA）

```bash
cd src/ground/carla_red_light_warning/
python3 main.py doctor     # 依赖体检
python3 main.py test       # 38 个离线单元测试
python3 main.py demo       # 离线演示：合成红灯接近场景
```

离线场景为自车 8 m/s 循环接近停止线（100 m → 0），灯按 25 s 周期
（10 s 红 / 15 s 绿）切换：每轮触发 approach_start → violation_start →
violation_clear 完整事件链。

### 5.2 CARLA 实测

先启动 CARLA 0.9.16（宿主机离屏渲染），执行（`--host` 填 CARLA 所在
机器 IP，Python 需可导入 carla 包，见 README 环境说明）：

```bash
# 方式一：源码直跑（目录：src/ground/carla_red_light_warning/）
python3 main.py demo --live --host <CARLA_IP> --seconds 45

# 方式二：colcon 构建 + launch（目录：ros2 工作空间根目录）
colcon build --packages-select carla_red_light_warning && source install/setup.bash
ros2 launch carla_red_light_warning main.launch.py host:=<CARLA_IP>
# 另开终端：ros2 topic echo /rlw/state
```

实测现象（Town01）：自车 6 m/s 左右巡航，遇红灯路口时 `approach_start`
（距停止线 30 m 内）→ 红灯且刹停不可行时 `violation_start` → 演示刹停
→ 变绿灯 `violation_clear` 起步通过；长直道无红绿灯段状态保持 normal。

## 6. 演示结果

### 6.1 事件时间线（CARLA 实测 45 s）

| 时刻 | 事件 | 场景 |
|---|---|---|
| t=4.15 s | approach_start | 路口 1 红灯，进入 30 m 预警区 |
| t=5.10 s | approach_clear | 灯转绿/通过，回 normal |
| t=14.00 s | approach_start | 路口 2 红灯接近 |
| t=14.35 s | violation_start | 剩余距离 < 判停距离，刹停不可行 |
| t=14.70 s | violation_clear | 演示刹停完成（v→0），灯转绿恢复 |

violation_risk 合计 7 拍（0.35 s 内完成"判定→刹停"），全程无假振荡；
长直道段（t≈15~45 s）状态保持 normal，零误报。

### 6.2 指标汇总

| 指标 | 离线演示 | CARLA 实测 |
|---|---|---|
| 状态链 | 三级齐全、多轮红灯循环 | 2 轮 approach，1 次 violation 判定+刹停 |
| 判停准确性 | — | violation_risk 后 0.35 s 内刹停（v→0） |
| 误报 | 0 | 0（长直道 30 s 全程 normal） |
| 传感器依赖 | 无 | 无（交通灯 API） |

### 6.3 效果图

接近预警（approach）、将闯线判定（violation）、刹停恢复（recover）
截图，以及停止线距离/状态/灯态三行跟踪曲线
（曲线上 dist=1e4 平台为长直道无红绿灯段的占位值）：

![接近预警瞬间](https://raw.githubusercontent.com/82540295/ros2/feat/carla_red_light_warning/docs/img/ground/carla_red_light_warning/rlw_live_approach.png)

![将闯线判定瞬间](https://raw.githubusercontent.com/82540295/ros2/feat/carla_red_light_warning/docs/img/ground/carla_red_light_warning/rlw_live_violation.png)

![RLW 实测曲线](https://raw.githubusercontent.com/82540295/ros2/feat/carla_red_light_warning/docs/img/ground/carla_red_light_warning/rlw_live_curve.png)

图片与逐拍 CSV 数据已入库 `docs/img/ground/carla_red_light_warning/`
（PNG 均 < 0.5 MB），复现命令见第 5 节。

## 7. 参数说明

全部参数集中于 `config/rlw_params.yaml`，与代码 `RLWParams` 一一对应：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `a_comfort` | 2.5 | 舒适减速上限（m/s²） |
| `warn_enter` / `warn_exit` | 30 / 35 | approaching 进/出的停止线距离（m，滞回） |
| `brake_margin` | 1.2 | violation 退出滞回系数（d > d_stop×margin 回退） |
| `period` | 0.05 | 状态机推进周期（s） |
| `warmup_ticks` | 2 | 热身拍数（冷启动灯态未稳定不输出决策） |

## 8. 已知限制

- 停止线感知依赖 `vehicle.get_traffic_light()`——仅当自车驶入灯控车道时
  仿真器才返回受控灯；路口选择由行驶路线决定（Town01 红绿灯路口密度
  有限，45 s 实测遇 2 个灯控路口）；
- 判停模型未考虑路面附着系数（雨雪天应下调 a_comfort）与坡度；
- `get_stop_locations()` 为 CARLA 0.9.14+ API，旧版本自动回退灯位近似
  （距离偏保守）；
- 模块只预警不控车，演示中的刹停为演示基建；实际产品应将 /rlw/event
  送入声光提示，并可与 AEB（紧急制动）联动形成纵深防御。
