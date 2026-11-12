# 多 AUV 协同围捕

## 问题

单个 AUV 追一个会逃的目标，只能形成「尾随」；要真正围住目标，需要多艘 AUV
同时从不同方向收拢。这带来两个现实约束：

- 水下没有稳定的无线链路，消息有**传播延迟**，还会**丢包**，各船对「谁去哪个位置」
  的认知会短暂不一致；
- 每艘船的**剩余电量**不同，距离近的不一定能耗得起，任务分配必须把电量算进去。

本模块用一个可以直接运行、可复现的示例演示这两点，并提供捕获判定。

## 与已有模块的区别

| 已有模块 | 关注点 |
|---|---|
| `src/water/auv_end2end` | 单 AUV 端到端感知与避障巡航 |
| `src/water/rov_depth_hold` 等 ROV 模块 | 单台 ROV 的底层控制与推力分配 |
| 本模块 | **多 AUV 的任务分配 + 通信受限条件下的协同围捕** |

差异点在于：分配结果不是全船即时共享，而是通过带延迟和丢包的水声信道广播，
收不到的 AUV 只能沿用上一次的任务——这决定了「围捕」在各种通信质量下的表现。

## 环境要求

- ROS 2 Humble（仅用于按仓库惯例打包）
- Python 3.10+、NumPy
- HoloOcean 2.x（可选，未安装时自动进入 dry-run 模式）

> HoloOcean 真实模式**未验证**：HoloOcean 的 `tick()` 观测按传感器类型做键（`LocationSensor`、
> `AcousticModem` 等），代码已为每个 agent 补上 `LocationSensor`，并在 `position()` 里同时兼容
> `Location`（dry-run）与 `LocationSensor`（HoloOcean）两种键；但作者环境没有安装 HoloOcean，
> 真实模式尚未实测，首次使用请先确认 `env.tick()` 返回里存在 `LocationSensor`。

## 文件结构

```
src/water/multi_agent_hunting/
├── resource/multi_agent_hunting
├── multi_agent_hunting/
│   ├── __init__.py
│   ├── core.py        # 信道、拍卖、控制、捕获判定
│   └── main.py        # 命令行入口
├── launch/
│   └── run_demo.sh
├── test/
│   └── test_core.py
├── scenario.json
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

## 核心设计

### 1. 水声信道真的有延迟和丢包

```
broadcast(sender, payload, now)  # 丢包在这里决定；送达时间 = now + delay
poll(now)                        # 只取 deliver_at <= now 的消息
```

`HuntingScenario` 每一步都会 `poll`，把**收到的**分配结果写进 `known_tasks`，
控制只使用 `known_tasks`。所以：

- `packet_loss = 1.0` 时，所有 AUV 都收不到任务，原地不动；
- `comm_delay_ms` 大于仿真时长时，AUV 在时限内同样收不到任务；
- 部分丢包时，个别 AUV 会短暂停留在上一轮的包抄点，之后才跟上。

### 2. 任务分配把电量算进去

代价函数：

```
cost = 距离 - energy_weight * (剩余电量 / 100)
```

电量按满电比例归一化，`energy_weight` 相当于「满电可抵扣的距离（米）」。
每艘船的耗电按自己的指令强度扣：

```
energy -= (idle_drain_rate + move_drain_rate * |cmd|) * dt
```

因此跑得远、动作大的 AUV 电量掉得快，下一轮拍卖里会被优先安排近处的任务。
`scenario.json` 里三艘船的初始电量分别是 95% / 45% / 70%，把 `energy_weight` 从 0
调到 50，左右两个包抄点的分配会互换（实测 AUV_1 与 AUV_3 对调），说明电量项真的
参与排序，而不是被同一个常数平移掩盖。

### 3. 包抄点参与控制，并有捕获判定

拍卖得到的是 `Intercept_Left` / `Intercept_Right` / `Block_Back` 三个包抄位置，
实际控制点由包抄点向逃逸目标收缩：

```
goal = 包抄点 + (逃逸目标 - 包抄点) * closing_ratio(t)
closing_ratio(t) = min(1, 起始比例 + t / closing_time)
```

一开始停在包抄点外侧完成合围，随后按 `closing_time` 逐步收紧，最终直接贴到目标。
三艘船从不同方向同时收拢，当所有 AUV 都进入 `capture_radius` 时判定捕获成功并结束。

### 4. 结果可复现

`seed` 同时控制初始位置和丢包抽样：同一个 `seed` 连跑两次结果完全一致，
也可以用命令行覆盖：

```bash
ros2 run multi_agent_hunting multi_agent_hunting --seed 7 --max-steps 300
```

## 编译与运行

```bash
cd ~/ros2_ws
colcon build --packages-select multi_agent_hunting
source install/setup.bash
ros2 run multi_agent_hunting multi_agent_hunting
```

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `seed` | 42 | 随机种子 |
| `max_steps` | 1000 | 最大步数 |
| `auction_interval_steps` | 50 | 拍卖周期（步） |
| `comm_delay_ms` | 200 | 水声通信延迟，毫秒 |
| `packet_loss` | 0.05 | 丢包率，0 ~ 1 |
| `energy_weight` | 5.0 | 电量奖励权重，满电相当于抵扣多少米 |
| `agents[].energy` | 100.0 | 各 AUV 的初始电量，示例配置为 95 / 45 / 70 |
| `closing_ratio` | 0.35 | 包抄点向目标收缩的起始比例 |
| `closing_time` | 20.0 | 收缩到贴身所需时间，秒 |
| `capture_radius` | 2.0 | 捕获判定半径，米 |
| `idle_drain_rate` | 0.02 | 待机耗电率，%/s |
| `move_drain_rate` | 0.15 | 满推力时的额外耗电率，%/s |

## 验收标准

| 检查项 | 预期 |
|---|---|
| 正常通信（`packet_loss=0`） | 每个 AUV 都拿到任务，从不同方向收拢 |
| 全丢包（`packet_loss=1`） | 所有 AUV 收不到任务，位置不变 |
| 大延迟（`comm_delay_ms` 超过仿真时长） | 时限内没有 AUV 收到任务 |
| 电量差异 | `energy_weight` 调大后，低电量的 AUV 被分配更近的包抄点 |
| 捕获判定 | 所有 AUV 进入 `capture_radius` 后结束并打印「捕获成功」 |
| 复现性 | 相同 `seed` 两次运行的初始位置与统计一致 |
| 单元测试 | `colcon test --packages-select multi_agent_hunting` 全部通过 |

## 扩展方向

- 用 HoloOcean 的 `AcousticModem` 传感器替换内部信道模型，做半实物联调；
- 给每艘 AUV 加本地估计器，在收不到广播时推断目标位置；
- 把捕获判定换成围捕难度指标（包围角覆盖、最小间距）。
