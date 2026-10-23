# 多航点任务队列（ROS 2 Humble）

单次发送最多 100 个 NED 航点，控制器按顺序派发，并在状态话题中报告进度。适用于预先给定的巡检路线；不包含路径规划、避障或 ROS Action。

## 环境与构建

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

## 手动启动与操作

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

## 队列语义

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

## 状态字段

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

## 实测范围

2026-10-23 在上述 Humble 环境构建，并执行单元测试、原 ROS 冒烟脚本和新增独立 launch 演示脚本。新增演示的原始输出见源码目录 `tests/logs/waypoint_queue_humble.txt`。

同日另用交互终端逐步输入上述 `ros2` 命令手动复测（ROS_DOMAIN_ID=90），验证三个航点顺序完成、暂停恢复、非法尾部整队拒绝、暂停后取消再恢复不重发、返航抢占并自动降落、reset 后再次起飞，以及显式降落抢占。最终为 LANDED，节点正常退出。采集 1930 条状态消息；保留状态/目标变化段的 52 个首尾帧和完整命令返回，分别见 `tests/logs/waypoint_queue_manual_states.jsonl`、`tests/logs/waypoint_queue_manual_cli.txt`。原始状态流另保存在复测机器 `~/queue_validation/manual/status_stream.txt`。

本次另在 Windows OpenHUTB Air 场景与 WSL2 Humble 节点间完成 AirSim 手动飞行复测：三航点顺序完成、执行中暂停并保留原目标恢复、取消队列、返航自动降落。日志确认 RETURNING → LANDING → LANDED，取消的队列未重启。环境、帧号和限制见源码目录 `tests/logs/waypoint_queue_airsim_manual.md`，变化段首尾帧见 `tests/logs/waypoint_queue_airsim_states.jsonl`。起飞 RPC 返回 False 的既有异常仍保留，未在本次修复；未验证真机、碰撞或避障。第一次贡献的 Foxy/AirSim 历史实测另见 [AirSim 实测记录](airsim_validation.md)。
原 `requirements.txt` 仍是历史 Foxy/Python 3.8 客户端依赖，本次不改动；不要将它用于上述 Humble 演示安装。
