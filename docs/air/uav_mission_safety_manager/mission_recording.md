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

本功能已在 Ubuntu 22.04 / ROS 2 Humble 的本地 demo 和 OpenHUTB Air 场景完成验证。AirSim 覆盖三航点成功、暂停/恢复后客户端取消、返航抢占及最终落地；独立记录器生成完整 JSONL，离线 JSON/CSV 的三个任务终态与 ROS 观测一致。起飞 RPC 仍可能返回 False，离地由遥测确认；该既有异常未修复。尚未验证 Foxy、真机、磁盘耗尽或进程崩溃下的完整行为。


## 首次集成验证结果 {#integration-validation}

该轮与下文交互手动复测分别进行，不能合并样本数或混用测量值。环境为 WSL2 Ubuntu 22.04 / ROS 2 Humble / Python 3.10.12、Windows OpenHUTB Air 屋顶场景；AirSim 客户端 1.8.1、msgpack-rpc-python 0.4.1、tornado 4.5.3、numpy 1.21.5。模拟器发行构建号未确定。

两包构建、rosdep 检查、63 项单元测试（原有 54 项与新增 9 项）通过；真实 launch 的 demo 记录冒烟、原 Action 冒烟及 MkDocs 构建通过。demo 控制器和记录器在不同进程运行，导出任务如下：

| demo 任务 | 队列终态 | 完成/总数 | 观测时长 (s) | 观测暂停 (s) | 采样距离 (m) |
| --- | --- | --- | --- | --- | --- |
| 三航点 | COMPLETED | 3/3 | 1.600 | 0.000 | 5.975 |
| 暂停恢复后客户端取消 | CANCELED | 0/1 | 1.600 | 1.249 | 0.174 |
| 返航抢占 | CANCELED | 0/1 | 0.003 | 0.000 | 0.000 |

以上为本地运动适配器结果，不代表 AirSim 飞行动力学。三条均为 `demo`、`start_partial=false`，返航后的 `LANDED` 由 ROS 状态另行确认。

AirSim 该轮起点 NED `[0,0,11.196054]`，航点 `[1,0,8.196054]`、`[2,0,8.196054]`、`[2,1,8.196054]`，速度均为 0.3 m/s。三航点 Action `SUCCEEDED`；暂停恢复后客户端取消为 Action `CANCELED`；返航抢占为 Action `ABORTED`、队列 `CANCELED`，随后 `RETURNING → LANDING → LANDED`。独立 AirSim 遥测确认 Landed，最终距该轮起点约 0.03292 m；本轮未观察到 FAILSAFE。

| AirSim 任务 | 队列终态 | 完成/总数 | 观测时长 (s) | 观测暂停 (s) | 采样距离 (m) |
| --- | --- | --- | --- | --- | --- |
| 三航点 | COMPLETED | 3/3 | 10.400 | 0.000 | 2.964 |
| 暂停恢复后客户端取消 | CANCELED | 0/1 | 3.600 | 3.247 | 0.263 |
| 返航抢占 | CANCELED | 0/1 | 0.005 | 0.000 | 0.000 |

AirSim 日志共 **482** 条状态样本，footer 与样本数一致，JSON/CSV 解析导出成功；三个任务均为 `airsim`、`start_partial=false`，终态未重复计数。第三条的零距离仅表示该队列片段未观察到位置变化，后续返航不计入已终止任务。

起飞 RPC 返回 `False`，遥测确认离地；落地后 collision_info 报告屋顶对象接触，不据此宣称无碰撞或避障通过。飞行验证完成后，本地包装脚本只向 `ros2 run` 父进程发送 SIGINT，等待记录器退出超时；随后向记录器子进程及控制节点发送 SIGINT，均退出，完整 footer 和导出验证通过。这是测试收尾问题，原始失败输出保留，不将其计为飞行失败或数据丢失。数据来源为 [PR #656 的验证说明](https://github.com/OpenHUTB/ros2/pull/656)，原始日志和报告仅保留本地。

## 交互手动复测结果 {#manual-validation}

Windows OpenHUTB Air / WSL2 Ubuntu 22.04 / ROS 2 Humble。使用 [PR #656](https://github.com/OpenHUTB/ros2/pull/656) 提交 `743c6e37` 的已安装程序，独立记录器在连接和起飞前启动，落地后以 Ctrl+C 正常关闭。起点 NED 为 `[0,0,11.196103]`；三航点为 `[1,0,8.196103]`、`[2,0,8.196103]`、`[2,1,8.196103]`，速度均为 0.3 m/s。坐标仅适用于本次场景，不能直接作为其他起点的飞行指令。

| 任务 | 队列终态 | 完成/总数 | 观测时长 (s) | 观测暂停 (s) | 采样距离 (m) |
| --- | --- | --- | --- | --- | --- |
| 三航点 | COMPLETED | 3/3 | 8.400 | 0.000 | 2.164 |
| 暂停恢复后取消 | CANCELED | 0/1 | 167.086 | 158.439 | 0.896 |
| 返航抢占 | CANCELED | 0/1 | 0.263 | 0.000 | 0.008 |

日志 footer 与报告样本数均为 **6536**，模式仅为 `airsim`，三条任务均 `start_partial=false`。实际安装的 `mission_report` 成功生成 `summary.json` 和 `missions.csv`，终态持续发布没有重复形成任务。原始 JSONL 与生成报告保留本地，不提交源码仓库。

三航点 Action 明确返回 `SUCCEEDED`。暂停时飞行状态和队列均为 `PAUSED`，恢复服务返回 `Accepted: EXECUTING`；取消服务返回 `return_code=0` 且 `goals_canceling` 非空，之后状态话题与报告确认队列 `CANCELED`。本轮未取得该目标的 Action 最终 `status=5`，不能据此声称完成 Action 最终取消状态取证。

返航抢占任务返回 Action `status=6 / ABORTED`、队列 `CANCELED`；随后状态话题确认 `LANDED`、目标清空、`command_pending=false`，最终位置约 `[-0.031658,-0.005988,11.206094]`。后续返航落地不计入表中第三条已终止队列任务的时长与距离。

本地辅助脚本以 heredoc 执行 Python 后调用 `input()`，因标准输入已被脚本占用而出现 `EOFError`，任务仍保持暂停；之后手动恢复并取消。158.439 秒包含截图和排查等待，不能写成预设的 3 秒。状态查询还遇到不支持的 CLI 参数及等待无输出。以上是辅助操作问题，保留取证限制，未据此追加不存在的成功结果。

起飞 RPC 返回 `False`，位置遥测确认上升约 1.53 m 并进入 `EXECUTING`；既有异常没有修复。上述采样距离不代表精确航程、飞行误差或避障验证。本轮与首次集成验证分别记录，不沿用第四次 PR 的飞行数据。


## Action 最终取消状态补充复测 {#cancel-result-validation}

针对上一轮交互操作未取得 Action 最终取消状态的问题，使用第五次 PR 的已安装程序，在 OpenHUTB Air / WSL2 Ubuntu 22.04 / ROS 2 Humble 下重新执行集成复测。测试客户端保留 GoalHandle，在暂停、恢复后调用 `cancel_goal_async()`，再直接等待 `get_result_async()`；没有使用状态话题查询来推断结果。

实际输出：

```text
three_waypoints: SUCCEEDED, completed=3; takeoff_rpc_result=False
FINAL_CANCEL_RESULT: status=5, queue_state=CANCELED
pause_resume_client_cancel: CANCELED; final status=5 verified by get_result_async()
return_interrupt: Action ABORTED, queue CANCELED, then LANDED
PASS: separate recorder, JSONL, pause timing, terminal deduplication, JSON/CSV exports
```

本轮起点 NED `[0,0,11.164961]`，三航点相对起点的水平偏移依次为 `[1,0]`、`[2,0]`、`[2,1]`，z 为起点上方 3 m，速度 0.3 m/s。随后朝起点方向提交慢速任务，暂停约 3 秒、恢复并取消；最后新任务由返航抢占，独立 AirSim 遥测确认 Landed，最终距本轮起点约 0.05590 m。上述数值仅属于本轮场景。

控制节点与记录器分别运行，落地后均正常停止。记录器直接运行安装入口，避免只向 `ros2 run` 父进程发信号的旧收尾问题。JSONL footer 和报告样本数均为 **298**，三个任务均为 `airsim`、`start_partial=false`，实际报告入口成功导出 JSON/CSV：

| 任务 | 队列终态 | 完成/总数 | 观测时长 (s) | 观测暂停 (s) | 采样距离 (m) |
| --- | --- | --- | --- | --- | --- |
| 三航点 | COMPLETED | 3/3 | 10.599 | 0.000 | 2.989 |
| 暂停恢复后客户端取消 | CANCELED | 0/1 | 3.601 | 3.199 | 0.266 |
| 返航抢占 | CANCELED | 0/1 | 0.002 | 0.000 | 0.000 |

这轮明确补充了 Action 最终 `status=5 / CANCELED` 的直接证据，不改变上一轮查询未取得结果的事实，也不混用 482、6536 与 298 条样本的统计。起飞 RPC 仍返回 `False`，遥测确认已离地；落地后 collision_info 报告屋顶对象接触，不宣称无碰撞或避障验证。原始日志、终端输出和导出报告保留本地，不纳入仓库。
