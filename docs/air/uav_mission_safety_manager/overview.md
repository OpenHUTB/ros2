# 无人机任务管理与安全状态机

单无人机控制模块，支持单航点与多航点队列、暂停/继续、返航降落，以及遥测超时和高度检查。原 AirSim 单航点流程在 Foxy 下验证；新增队列已在 Ubuntu 22.04 / ROS 2 Humble 下完成 demo 和 AirSim 手动复测。

- [构建与运行](usage.md)
- [测试结果与已知限制](airsim_validation.md)
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

单次发送最多 100 个 NED 航点，控制器按顺序派发，并在状态话题中报告进度。适用于预先给定的巡检路线；不包含路径规划、避障或 ROS Action。

### 环境与构建

本次新增功能的验证环境：Ubuntu 22.04.5 / ROS 2 Humble / Python 3.10.12。
以下演示使用本地匀速运动适配器，不需要安装 AirSim 的 Python 依赖，不连接真实载具。
它只验证任务调度和 ROS 接口，不模拟飞行动力学、碰撞、地形或网络故障。

在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/air/uav_mission_safety_manager --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --packages-select uav_mission_safety_manager
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

服务成功仅表示请求被接受。COMPLETED 表示全部航点已进入 1 米到达容差并已请求悬停，不保证已停稳或精确到达坐标。
暂停请求被接受后，位置字段可能要到下一次遥测才更新；`command_pending=false` 也不能单独作为已停稳的依据。
取消和 FAILSAFE 不保证真实无人机已停止；尤其连接中断时无法保证悬停或自动降落。

### 实测范围

Ubuntu 22.04 / ROS 2 Humble 下构建通过，47 项单元测试及 ROS 冒烟测试通过。demo 手动验证了队列顺序执行、暂停恢复、取消、非法队列拒绝、返航和降落抢占。

AirSim 手动验证了三航点顺序执行、执行中暂停恢复、取消任务及返航自动落地。测试记录见源码目录 `tests/logs/waypoint_queue_airsim_manual.md`。

已知限制：起飞 RPC 返回 `False` 的既有异常未修复；未验证真机、碰撞或避障。Humble 演示无需安装历史 Foxy/Python 3.8 的 `requirements.txt`。
