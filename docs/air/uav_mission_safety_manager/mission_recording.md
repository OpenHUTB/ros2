# 任务日志与离线统计（ROS 2 Humble）

独立的 `mission_recorder` 节点订阅现有状态话题，将收到的完整快照保存为 JSONL；`mission_report` 离线生成 JSON 和 CSV 任务统计。适用于比较多航点任务的执行结果与观测轨迹。记录默认关闭，不改变控制器、Action 或服务接口。

这是状态观测与统计工具，不会重发飞行命令。日志中的 `adapter_mode` 区分 `demo` 和 `airsim`；demo 结果不代表真实飞行动力学或 AirSim 实测。

## 构建和启动

在仓库根目录执行，依赖和构建顺序沿用 [Action 教程](mission_action.md)：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/air/uav_mission_interfaces src/air/uav_mission_safety_manager --ignore-src --rosdistro humble -r -y
colcon build --packages-up-to uav_mission_safety_manager
source install/setup.bash
ros2 launch uav_mission_safety_manager main.launch.py \
  use_demo_adapter:=true record_missions:=true \
  recording_directory:="$HOME/uav_mission_logs"
```

`record_missions` 默认为 `false`，`recording_directory` 默认为 `~/.ros/uav_mission_logs`。每次启动使用随机标识创建新的 `mission_*.jsonl`，不会覆盖已有日志。目录自动创建，启动日志打印完整路径。日志持续增长，运行前应预留磁盘空间。

也可以给已经运行的控制节点添加记录器，**此时不要再次 launch 控制节点**：

```bash
ros2 run uav_mission_safety_manager mission_recorder --ros-args \
  -p recording_directory:="$HOME/uav_mission_logs" \
  -p status_topic:=/uav_mission_safety_manager/status
```

记录器只创建状态订阅，不创建飞行发布器、服务客户端或 Action 客户端。磁盘写入在独立进程中完成；目录无权限会使记录器启动失败，运行中写盘错误会停止记录并打印错误。两者都不会主动取消或停止飞行。观察到此类错误后，应按原控制流程处理飞机，不能把日志停止当作飞行停止。

## 完成一轮任务并导出

在第二个已 source 环境的终端中，按 [Action 教程](mission_action.md) 完成连接、解锁、起飞，然后发送航点。此处仅用于 **demo**：

```bash
ros2 action send_goal /uav_mission_safety_manager/execute_mission \
  uav_mission_interfaces/action/ExecuteMission \
  '{waypoints: [{x: 2.0, y: 0.0, z: -2.0}, {x: 4.0, y: 0.0, z: -2.0}, {x: 6.0, y: 0.0, z: -2.0}], speeds: [4.0, 4.0, 4.0]}' \
  --feedback
```

保留记录器直到观察到任务终态。若还要返航落地，应继续控制流程，确认 `LANDED` 后再按 Ctrl+C 正常停止 launch 或单独记录器。正常关闭会写入 footer；强制终止、断电或写盘失败可能留下不完整文件，报告工具会拒绝缺少 footer、截断、损坏、时序不合法的日志，并以退出码 2 报错。

将 `LOG` 改成启动信息中打印的实际文件路径。统计工具不需要启动 ROS 节点或模拟器：

```bash
LOG="$HOME/uav_mission_logs/mission_替换为实际标识.jsonl"
# 打印 JSON 报告
ros2 run uav_mission_safety_manager mission_report "$LOG"
# 导出到尚不存在的新目录，已有目录会被拒绝以防覆盖
ros2 run uav_mission_safety_manager mission_report "$LOG" \
  --output-dir "$HOME/uav_mission_report"
```

导出 `summary.json` 和 `missions.csv`，CSV 每行是一个观察到的队列任务。JSON 还包含总样本数、任务数和适配器模式。完全没有任务的有效日志导出空任务数组及仅含表头的 CSV。

## 字段语义与边界

JSONL 第一行为版本为 1 的 header（含 UTC 启动时间和订阅话题），中间每行包含 `sequence`、基于记录器单调时钟的 `elapsed_s` 和完整 `status`，最后为带样本计数的 footer。记录器逐行 flush，不承诺断电持久化。非法状态被跳过并输出警告。

| 统计字段 | 含义 |
| --- | --- |
| `mission` | 此文件内按观察顺序生成的序号，不是 Action goal UUID |
| `outcome` | 队列的 `COMPLETED` / `CANCELED` / `ABORTED`；未观察到终态为 `INCOMPLETE` |
| `total` / `completed` | 最后观察到的航点总数 / 完成数 |
| `observed_duration_s` | 从首个任务快照至最后任务快照的时间差，包含暂停 |
| `observed_pause_s` | 前一条任务样本为 `PAUSED` 的采样间隔之和 |
| `sampled_distance_m` | 任务内相邻有效 NED 位置的三维折线长度，缺失位置会断开累计 |
| `start_partial` | 记录从活动或终态开始、首样本已有进度、或观察到无终态替换时为 true |
| `adapter_mode` | 此任务首样本的 `demo` / `airsim` 等模式 |

持续发布的相同终态不会重复计为新任务。先观察到活动队列，再观察到终态，才形成通常的完整任务片段；仅看到终态仍会列一行，标为 `start_partial=true` 且时长为零。运行中正常关闭记录器，文件完整，但任务仍为 `INCOMPLETE`。

统计基于默认约 5 Hz 状态采样和接收时刻，不是精确飞行时钟、真实航程、误差评估或到达停稳证明。位置可能重复或过期；暂停边界有采样误差。未收到的终态无法恢复；同长度、相同进度的新任务如果跨越丢失的终态，可能无法区分。当前话题无任务 UUID，不宣称严格的任务审计或无损记录。`start_partial=false` 也不能证明没有丢样本。

队列 `CANCELED` 不能直接解释为 Action `CANCELED`：返航服务抢占会让队列 `CANCELED`、Action `ABORTED`，返航落地发生在队列终止以后，不计入该任务的时长与距离。详情见 [Action 结果语义](mission_action.md)。

## 自动复测

在仓库根目录、完成构建并 source 后执行。证据目录必须尚不存在：

```bash
python3 -m unittest discover -s src/air/uav_mission_safety_manager/tests -v
ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 python3 \
  src/air/uav_mission_safety_manager/tests/ros_recording_smoke.py \
  --output-dir "$HOME/uav_recording_check"
```

脚本启动真实 launch 和独立记录进程，在 demo 中验证三航点成功、暂停/恢复后客户端取消、返航抢占及最终落地；正常停止进程后实际执行安装的 `mission_report`，校验 JSONL、三个任务的终态、暂停时长、距离和 CSV 行数。原始状态和报告写入指定目录，不应提交仓库。

本功能验证范围是 Ubuntu 22.04 / ROS 2 Humble 的本地 demo。尚未验证本次记录功能在 AirSim、Foxy、真机、磁盘耗尽或进程崩溃下的完整行为；此前 Action 的 AirSim 测试不能作为本功能的实测证据。
