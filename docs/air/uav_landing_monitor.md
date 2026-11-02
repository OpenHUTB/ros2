# 无人机降落过程监控（指令-运动一致性）

**一句话**：飞机"以为自己降落了"但实际悬在半空，是降落环节最危险的静默故障。这个模块只做一件事 ——
盯着 `/uav/cmd_vel` 下发的垂直速度和 `/uav/odom` 的实际高度变化，一旦"喊了不动""掉得没人管"
"超时没落地"就立刻告警并落 CSV，事后能一键出图。

![故障轮：下降指令无效](../img/air/uav_landing_monitor/stuck_run.png)

## 1. 为什么需要这个监控

在真实 UE + AirSim + `carlair_ros_bridge` 上联调时踩到一个静默故障：桥接层把每条速度指令
调成 `moveByVelocityBodyFrameAsync(..., duration=cmd_duration)`，而默认 `cmd_duration=0.1 s`。
策略侧 10 Hz 下发时，**下一条指令会在上一条生效前把它顶掉**，飞机几乎不动 —— 但上游
`rooftop_landing_nn` 只按"离屋面还差多少"判断，等它慢慢挪到容差内就宣布 `LANDED`。
从日志上看一切正常，实际上飞机停在屋面上方几十厘米的空中。

这类问题不是某个模块的 bug，而是**控制链路各层之间缺少一致性校验**。本模块补上这一层：
不改进任何控制算法、不发送任何控制指令，只做"看门狗 + 记录仪"。

## 2. 判据

| 告警码 | 级别 | 触发条件 |
| --- | --- | --- |
| `CMD_NO_EFFECT` | error | 持续下发下降指令（\|vz\| ≥ `cmd_min_vz`），但观察窗内实际下降量 < `cmd_effect_ratio` × 期望值 |
| `DESCEND_STALL` | warn | `/uav/status` 声称 `DESCEND`，但根本没有下降指令下来（上游状态机空转） |
| `FREE_FALL` | error | 高度下掉速率 > `free_fall_rate`，而速度指令远小于它（掉高 / 失控） |
| `OVERSHOOT` | error | 高度已低于目标高度超过 `overshoot_m`（过冲，会砸到屋面） |
| `LANDING_TIMEOUT` | error | 从开始下降算起超过 `timeout_s` 仍未到达目标高度 |
| `INPUT_STALE` | error | 里程计超过 `stale_input_s` 没有新数据 |
| `LANDED` | info | 高度进入目标 ± `landing_tol_m` 且速度指令已归零 |

两个细节值得一提，都是实测中逼出来的：

* **起动宽限期**（`descend_grace_s`，默认 1.5 s）：刚下发下降指令时飞控还在加速，高度几乎不变，
  不设宽限期会误报 `CMD_NO_EFFECT`。宽限期内还会清空高度历史，避免起动阶段污染后面的观察窗。
* **按实际覆盖时长算期望位移**：窗口里只有一两个样本时不参与判定，否则开监控的第一帧就会误判。

## 3. 运行

```bash
# 终端 1：桥接层
roslaunch carlair_ros_bridge main.launch host:=192.168.49.1
# 终端 2：降落模块（被监控对象）
roslaunch rooftop_landing_nn main.launch
# 终端 3：本模块
roslaunch uav_landing_monitor main.launch csv:=/tmp/landing.csv
# 等价的前台运行
rosrun uav_landing_monitor main.py
```

话题：

| 方向 | 话题 | 类型 |
| --- | --- | --- |
| 订阅 | `/uav/odom`、`/uav/cmd_vel`、`/uav/status` | `nav_msgs/Odometry`、`geometry_msgs/Twist`、`std_msgs/String` |
| 订阅（可选） | `/rooftop_nn/landing_point`、`/uav/goal` | `geometry_msgs/Point`（用来确定落地目标高度） |
| 发布 | `/uav/landing_monitor/alert` | `std_msgs/String`，一条告警一行 JSON |
| 发布 | `/uav/landing_monitor/state` | `std_msgs/String`，周期状态 JSON |

常用参数（完整列表见 `config/uav_landing_monitor.yaml`）：

```bash
roslaunch uav_landing_monitor main.launch csv:=/tmp/l.csv      # 落 CSV
roslaunch uav_landing_monitor main.launch z_up:=false          # 高度是 NED 下为正
roslaunch uav_landing_monitor main.launch target_mode:=fixed use_target_z:=true target_z:=13.9
roslaunch uav_landing_monitor main.launch timeout_s:=30 stale_input_s:=0.5
```

出图（把 CSV 变成上面那种证据图）：

```bash
python3 tools/plot_report.py /tmp/landing.csv --out landing.png --title "正常降落"
```

## 4. 实测：同一套真机链路，两种桥接层配置

环境：Windows 宿主机 UE 5.5 + Cosys-AirSim（Blocks 场景）+ 本仓库 `carlair_ros_bridge`
+ `rooftop_landing_nn` 降落模块 + 本监控模块，全程真实运行（非离线回放）。

| 轮次 | 桥接层配置 | 监控结果 |
| --- | --- | --- |
| 故障轮 | `cmd_duration:=0.1`（默认值） | `CMD_NO_EFFECT` ×7、`LANDING_TIMEOUT` ×1、`FREE_FALL` ×1、`OVERSHOOT` ×1 → 结论 **fail** |
| 正常轮 | `cmd_duration:=0.5` | 仅 `LANDED` ×1 → 结论 **ok**（零误报） |

故障轮里飞机卡在屋面上方 0.39 m（目标 13.90 m，实际 14.29 m），一直下不去，
最后被 `LANDING_TIMEOUT` 收尾：

```text
[error] CMD_NO_EFFECT    21.6s  下降指令 0.50 m/s 持续 2.0 s，期望降 0.98 m，实际只降 0.00 m
[error] CMD_NO_EFFECT    24.7s  下降指令 0.50 m/s 持续 2.0 s，期望降 0.98 m，实际只降 0.00 m
...
[error] LANDING_TIMEOUT  42.0s  下降开始已 30.1 s 仍未到达目标高度（上限 30.0 s）
告警统计: {'FREE_FALL': 1, 'OVERSHOOT': 1, 'CMD_NO_EFFECT': 7, 'LANDING_TIMEOUT': 1}
```

同一轮里前半段的 `FREE_FALL` / `OVERSHOOT` 也不是编的：重启桥接层那一刻飞机短暂失去 API 控制、
真的掉了下去（20 m → 5 m），监控如实抓到了 —— 这正是"失控"该有的样子。

正常轮（`cmd_duration:=0.5`）全程安静，只有一条落地事件：

![正常轮：降落成功且无告警](../img/air/uav_landing_monitor/normal_run.png)

## 5. 和别的模块的关系（不重复造轮子）

* `telemetry_diagnostics`：用 GRU 判**遥测数据本身**的统计异常（注入漂移/冻结/丢包），ROS2、偏数据层。
  本模块不建模数据分布，只做**控制指令与运动结果**的规则一致性校验，ROS1 桥接话题，零训练、即插即用。
* `rooftop_landing` / `rooftop_landing_nn`：负责"落到哪里"，可能因为链路问题**误以为**落地了；
  本模块不参与决策，只负责在它们出错时把问题喊出来。
* 本模块**不发任何控制指令**，因此可以挂在任何桥接层/任何降落模块旁边做旁路监控。

## 6. 已知限制

* 规则阈值是工程经验值（`config` 里都可调），极端机型的加速度差异需要重新标定；
  起动宽限期设小了会误报、设大了会漏报"很快就卡住"的故障。
* 只监听单个载具；多机需要各自带命名空间启动一份。
* 高度用单一来源 `/uav/odom`，里程计本身漂移时无法自证（需要外部真值，比如动捕/RTK）。

## 附：大模型使用声明

本模块的代码、测试与文档在编写过程中使用了大模型辅助（判据设计讨论、代码与文档起草、
绘图脚本整理）。所有结论均来自仓库内可复现的真实运行：`tests/test_uav_landing_monitor_local.py`
（33 项断言）、以及第 4 节两轮 UE + AirSim 真机运行（CSV 由节点参数 `monitor/csv` 生成，
图由 `tools/plot_report.py` 从该 CSV 绘制）。作者对提交内容的正确性与完整性负全部责任。
