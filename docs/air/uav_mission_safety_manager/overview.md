# 无人机任务管理与安全状态机

单无人机控制模块，支持单航点与多航点队列、暂停/继续、返航降落，以及遥测超时和高度检查。原 AirSim 单航点流程在 Foxy 下验证；新增队列已在 Ubuntu 22.04 / ROS 2 Humble 下完成 demo 和 AirSim 手动复测。

- [构建与运行](usage.md)
- [任务日志与离线统计](mission_recording.md)：可选 JSONL 状态记录与 JSON/CSV 任务评估，含首次集成验证、交互手动复测及 Action 最终取消状态补充复测的任务统计。
- [测试结果与已知限制](airsim_validation.md)：按任务阶段汇总安全状态机、多航点队列、队列可靠性、Action 及日志统计验证。
- [源码](https://github.com/OpenHUTB/ros2/tree/master/src/air/uav_mission_safety_manager)

## 状态流程

```text
IDLE → CONNECTING → ARMING → TAKEOFF → EXECUTING
EXECUTING ↔ PAUSED
EXECUTING / PAUSED → RETURNING → LANDING → LANDED
异常 → FAILSAFE
```

连接时记录已落地且停稳的无人机位置作为起点。暂停保留目标和指令速度；继续时重新派发原目标。返航到起点上方后自动降落，解除解锁成功且新遥测确认落地后进入 `LANDED`。

## 实现要点

- ROS 回调异步派发命令，飞行等待与遥测使用独立工作线程。
- 替换飞行任务前取消旧任务并等待其结束，避免航点与降落命令并发。
- 返航末段低速修正位置，位置误差和速度持续满足阈值后才开始降落。
- 起飞 RPC 返回值通过 `takeoff_rpc_result` 暴露，离地状态由遥测确认。

服务返回成功仅表示请求已接受，动作是否完成应以状态话题为准。本模块是模拟器原型，安全边界见[已知限制](airsim_validation.md)。

## 多航点任务队列（ROS 2 Humble） <span id="waypoint-queue"></span>

单次发送最多 100 个 NED 航点，控制器按顺序派发，并在状态话题中报告进度。适用于预先给定的巡检路线；不包含路径规划或避障。标准 Action 接口见 [ROS 2 Action 任务接口](mission_action.md)。

### 环境与构建

本次新增功能的验证环境：Ubuntu 22.04.5 / ROS 2 Humble / Python 3.10.12。
以下演示使用本地匀速运动适配器，不需要安装 AirSim 的 Python 依赖，不连接真实载具。
它只验证任务调度和 ROS 接口，不模拟飞行动力学、碰撞、地形或网络故障。

在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/air/uav_mission_interfaces src/air/uav_mission_safety_manager --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --packages-up-to uav_mission_safety_manager
source install/setup.bash
python3 -m unittest discover -s src/air/uav_mission_safety_manager/tests -v
ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 python3 src/air/uav_mission_safety_manager/tests/ros_queue_smoke.py
```

最后一条命令自动启动独立演示节点，执行三个航点、暂停/继续、取消、返航抢占和降落，退出时清理所启动的节点。
执行时不要在同一 ROS 域运行另一个同名模块。此脚本只用于 Linux / WSL2。

### 手动启动与操作

终端一（工作区已 source）：

```bash
ros2 launch uav_mission_safety_manager main.launch.py use_demo_adapter:=true
```

也可通过已有 main 入口直接启动，二选一：

```bash
ros2 run uav_mission_safety_manager mission_manager --ros-args -p use_demo_adapter:=true
```

终端二同样加载 Humble 与工作区，先观察状态：

```bash
ros2 topic echo /uav_mission_safety_manager/status
```

另开终端，依次发送下列命令；每步等状态分别进入 `ARMING`、`TAKEOFF`、`EXECUTING` 且 `command_pending=false` 后再执行下一步：

```bash
ros2 topic pub --once /uav_mission_safety_manager/command std_msgs/msg/String '{data: "{\"event\":\"connect\"}"}'
ros2 topic pub --once /uav_mission_safety_manager/command std_msgs/msg/String '{data: "{\"event\":\"armed\"}"}'
ros2 topic pub --once /uav_mission_safety_manager/command std_msgs/msg/String '{data: "{\"event\":\"airborne\"}"}'
ros2 topic pub --once /uav_mission_safety_manager/command std_msgs/msg/String '{data: "{\"event\":\"mission\",\"waypoints\":[{\"x\":4,\"y\":0,\"z\":-2,\"speed\":2},{\"x\":8,\"y\":0,\"z\":-2,\"speed\":2},{\"x\":12,\"y\":0,\"z\":-2,\"speed\":2}]}"}'
```

服务按需调用，不必全部连续执行：

```bash
ros2 service call /uav_mission_safety_manager/pause std_srvs/srv/Trigger '{}'
ros2 service call /uav_mission_safety_manager/resume std_srvs/srv/Trigger '{}'
ros2 service call /uav_mission_safety_manager/cancel std_srvs/srv/Trigger '{}'
ros2 service call /uav_mission_safety_manager/return_home std_srvs/srv/Trigger '{}'
ros2 service call /uav_mission_safety_manager/land std_srvs/srv/Trigger '{}'
```

### 队列语义

| 操作或条件 | 队列行为 |
| --- | --- |
| `mission` | 仅在 EXECUTING 且无现存目标时接收；先校验全部航点，再派发第一点 |
| 航点到达 | 新遥测显示距离目标不超过 1 米后完成该点，再派发下一点；不要求逐点停稳 |
| 暂停 / 继续 | 保留当前索引、目标和速度；暂停期间不推进，继续时重发当前目标 |
| `cancel` | 清除当前目标，丢弃剩余任务并请求悬停；PAUSED 状态保持暂停 |
| 返航 / 降落 | 队列标记 CANCELED，交由既有返航或降落流程处理 |
| RPC 异常、高度越界或超时 | 队列标记 ABORTED，禁止继续派发后续航点 |
| 全部航点到达 | 标记 COMPLETED，清除目标并请求悬停，不自动返航或降落 |

航点必须包含数值 `x/y/z`，可选正数 `speed`（默认 3 米/秒）。拒绝布尔值、非有限值、未知字段、空列表和超过 100 点的列表。高度条件是 `home_z - max_altitude <= z < home_z`；NED 的 z 向下为正。不是 ROS ENU 坐标。

队列装载先校验 `home_z` 与 `max_altitude`：两者必须是有限的整数或浮点数，`max_altitude` 必须大于 0；布尔值、字符串、NaN 和无穷大都会被拒绝。所有航点和这两个高度参数都通过校验后才会替换当前队列，因此非法新任务不会清空或部分改写已有任务。

队列执行时拒绝新的 `goto` 和 `mission`，避免静默替换路线。先完成或取消当前队列再提交新队列。
RPC 派发尚未结束时，请求可能被拒绝，需要待 `command_pending=false` 后重试。

### 状态字段

`status` 原有字段不变，新增 `mission_queue`：

```json
{"state":"RUNNING","total":3,"completed":1,"current_index":1,"remaining":2}
```

顶层 `adapter_mode` 明确区分 `demo`（本地运动演示）、`airsim` 和 `state_only`（历史纯状态机模式）。`use_demo_adapter` 与 `use_airsim` 不能同时开启。

`current_index` 从 0 开始，`remaining` 包含当前航点。终止后索引为 `null`，可继续执行的剩余数为 0；原始总数和已完成数保留，便于区分已执行和被丢弃的航点。
状态依次为 IDLE、RUNNING / PAUSED，再到 COMPLETED、CANCELED 或 ABORTED。

终止状态的计数含义固定为：`COMPLETED` 保留全部已完成点，`CANCELED` 保留已经完成的点并将未执行点丢弃，`ABORTED` 保留故障发生前的计数。三种终止状态都将 `current_index` 置为 `null`、`remaining` 置为 0，并且不会再次派发旧航点。

服务成功仅表示请求被接受。COMPLETED 表示全部航点已进入 1 米到达容差并已请求悬停，不保证已停稳或精确到达坐标。
暂停请求被接受后，位置字段可能要到下一次遥测才更新；`command_pending=false` 也不能单独作为已停稳的依据。
取消和 FAILSAFE 不保证真实无人机已停止；尤其连接中断时无法保证悬停或自动降落。

### 实测范围

Ubuntu 22.04 / ROS 2 Humble 下构建通过，当前队列回归测试覆盖 61 项单元测试及 ROS 冒烟测试。测试包含 100 点上限、非零起点高度边界、非法装载的原子性，以及暂停、完成、取消和中止后的状态字段和不重复派发保证。demo 手动验证了队列顺序执行、暂停恢复、取消、非法队列拒绝、返航和降落抢占。

AirSim 手动验证了三航点顺序执行、执行中暂停/恢复、取消任务以及返航自动落地。非法高度输入由自动测试覆盖，未计为 AirSim 手动实测；普通航点完成采用 1 米到达容差。

已知限制：起飞 RPC 返回 `False` 的既有异常未修复；未验证真机、碰撞或避障。Humble 演示无需安装历史 Foxy/Python 3.8 的 `requirements.txt`。

## 飞行事件黑匣子

新增可选的飞行事件记录，便于复盘和提交验收材料。记录内容：

- 状态机的每次真实迁移（含 FAILSAFE 触发原因）；
- 任务里程碑：`mission_loaded`、`waypoint_reached`、`mission_completed`、`mission_canceled`；
- 外部下发的 `command` 事件。

参数与用法：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `record_events` | `true` | 关闭后与旧版本行为完全一致 |
| `record_max_events` | `500` | 环形缓冲上限，超出时丢弃最旧事件并计数 |
| `record_dir` | 空（当前目录） | 导出目录 |
| `dump_on_shutdown` | `true` | 节点退出时缓冲区非空则自动导出 |

运行中可随时导出：

```bash
ros2 service call /uav_mission_safety_manager/dump_log std_srvs/srv/Trigger '{}'
```

返回两个文件路径：

- `flight_events.csv`：`seq,time_s,kind,state,detail`，按时间排序的事件流；
- `flight_events_summary.json`：事件计数、各状态停留时长、FAILSAFE 原因、完整事件列表。

状态停留时长由相邻状态迁移的时间差累加，最后一次迁移到导出时刻的时长也计入。记录只影响可观测性，不改变控制行为。

### 统计口径（环形缓冲丢弃旧事件后）

`record_max_events` 是**有界**的：事件数超过上限后，最旧的事件被丢弃，`summary.dropped` 记录丢弃数量。因此：

- `summary.events` 是**当前仍保留**的事件数，不是整段飞行的总事件数；
- `state_durations_s` 只统计**保留区间**内的状态段——反映的是“最近一段时间的状态分布”，会短于整段飞行的真实时长；
- `seq` 是全局单调递增序号（不因丢弃而回绕），从导出的 `seq` 区间可以判断这次结果覆盖的是哪一段。

需要完整统计时，可以调大 `record_max_events`，或在飞行中定期调用 `dump_log`（每次导出当前缓冲区），再把多次导出按 `seq` 拼接。


