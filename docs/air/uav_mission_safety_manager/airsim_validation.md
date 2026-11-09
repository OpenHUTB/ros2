# 测试结果与已知限制

[运行与复测命令](usage.md) · [模块概览](overview.md)

本文按实际验证阶段汇总结果。首次 Foxy 测试保留原环境和结果，后续队列、Action 与日志功能在 Humble 下分别验证；文档更新不代表重新执行全部历史测试。

## 当前功能与验证范围

当前模块支持单航点、多航点队列、ROS 2 Action、暂停/恢复、取消、返航降落，以及独立状态记录和离线统计。普通航点使用 1 米到达容差，不保证到点停稳；不包含路径规划、避障或真机验证。

| 阶段 | 已验证内容 | 详细说明 |
| --- | --- | --- |
| 安全状态机 | Foxy 单航点、暂停恢复、返航落地、RPC 传输中断检测 | 下文安全状态机阶段 |
| 多航点队列与可靠性 | Humble / AirSim 三航点、执行中暂停恢复、取消和返航落地；边界校验由自动测试覆盖 | [队列实测证据](validation/waypoint_queue_airsim_validation.md) |
| ROS 2 Action | AirSim 三航点成功、客户端取消、返航抢占及最终落地 | [Action 接口与验证范围](mission_action.md) |
| 日志与离线统计 | 首次集成验证生成 482 条状态样本；交互手动复测生成 6536 条样本；最终取消状态补充复测生成 298 条样本，各轮分别导出三个任务 | [日志教程与本轮统计](mission_recording.md#manual-validation) |

历史测试数量属于对应版本，不应解释为当前完整代码在全部环境下重新通过相同验证。新增队列、Action 和记录统计没有在 Foxy 或真机上验证。

## 安全状态机阶段（PR #472）

### 测试环境

- Windows OpenHUTB Air 场景、WSL2 Ubuntu 20.04.6、ROS2 Foxy、Python 3.8.10。
- AirSim Python 客户端 1.8.1、msgpack-rpc-python 0.4.1、numpy 1.17.4、tornado 4.5.3。
- 模拟器具体构建版本未确认；结果仅适用于本次模拟场景。

### 验证结果

实际飞行通过 ROS launch、命令话题及 Trigger 服务执行。以下为当时安全控制实现的验证结果，文档整理不代表新增飞行测试。

| 项目 | 结果 |
| --- | --- |
| Foxy 构建 | 通过 |
| 单元测试 | 29 项通过 |
| ROS 冒烟测试 | 通过；未完成 RPC 不阻塞服务、状态发布及安全检测 |
| 水平航点与暂停 | 通过；目标保留，暂停后合速度约 0.03085 m/s |
| 继续 | 通过；恢复原目标，保留 0.5 m/s 指令速度 |
| 返航自动降落 | 通过；`RETURNING → LANDING → LANDED`，最终距起点约 0.0242 m |
| 返航中显式降落 | 通过；请求前为 `RETURNING`，最终距起点约 0.1142 m；仅验证起点上方场景 |
| RPC 传输中断 | 通过；约 2.40 s 后进入 `FAILSAFE`，中断后仍收到 12 条状态消息 |

距离为本次测得的三维起点误差，不是任意场景的精度保证。断线用例保持模拟器运行，停止被测节点后独立恢复降落；不代表自动故障降落通过。

依赖清单补充后，已在独立 Python 3.8 环境按安装说明完成初始化，`pip check` 及 AirSim、NumPy、RPC、OpenCV 导入检查通过；Foxy 构建、29 项单元测试和 ROS 冒烟测试再次通过。此检查未重跑实际飞行，不代表以清单中的 OpenCV 版本完成了飞行验证。

## 多航点队列阶段（PR #508）

在 WSL2 Ubuntu 22.04.5 / ROS 2 Humble / Python 3.10.12 与 Windows OpenHUTB Air 下，手动验证三航点顺序执行、执行中暂停/恢复、执行中取消和返航自动落地。三航点使用 0.5 m/s，完成数依次为 0、1、2、3，最终 `COMPLETED`；暂停恢复用例使用 0.2 m/s，保留目标、索引与速度，恢复后完成。取消后清空目标，旧航点未再次派发。

本轮起点 `home_z=11.196125`，航点 z=8.2。原始状态流共 5742 条；记录从起飞后 `EXECUTING` 开始，没有覆盖连接起飞，也没有命令时间戳。最终位置约 `[-0.008804,-0.030834,11.205312]`。详细采样证据及限制见[队列实测记录](validation/waypoint_queue_airsim_validation.md)。这些位置仅对应本轮场景。

## 队列可靠性阶段（PR #542）

对 `home_z` 和 `max_altitude` 增加有限数值检查，要求最大高度大于零；非法参数或航点被拒绝时保持原队列不变。构建和当时的 54 项单元测试通过，队列 ROS 冒烟、服务响应与安全超时冒烟通过。回归覆盖非零起点高度、100 点上限、非法装载原子性及暂停和终态计数语义。

该阶段独立的 AirSim 手动复测覆盖执行中暂停/恢复、三航点完成、取消和返航落地。暂停时 `completed=0`、`current_index=0`、`remaining=3`，目标与 0.2 m/s 指令速度保留；完成时为 `COMPLETED`、3/3、剩余 0；取消后目标与速度清空，索引为 `null`、剩余 0。返航后最终距本轮记录起点约 0.0343 m。非法高度只由自动测试覆盖，不算 AirSim 实测。结果来源见 [PR #542](https://github.com/OpenHUTB/ros2/pull/542)。

## ROS 2 Action 阶段（PR #613）

环境为 WSL2 Ubuntu 22.04.5、ROS 2 Humble、Python 3.10.12 与 Windows OpenHUTB Air。三航点 Action 为 `SUCCEEDED / COMPLETED`，完成 3/3；GoalHandle 客户端取消为 `CANCELED`；返航服务抢占为 Action `ABORTED`、队列 `CANCELED`，随后遥测确认 `LANDED`。本轮没有手动验证 Action 暂停/恢复，该项由当时 demo 测试覆盖。

## 任务日志与离线统计阶段（PR #656）

### 首次集成验证

Humble 两包构建、依赖检查、63 项单元测试（原有 54 项与新增 9 项）、独立记录器 ROS 冒烟、原 Action 冒烟及文档构建通过。demo 通过实际 launch 启动控制器与独立记录器，覆盖完成、暂停恢复后客户端取消、返航抢占与最终落地，正常关闭后解析 JSONL 并执行报告入口。

OpenHUTB Air 集成验证记录 482 条状态样本；footer 计数一致，JSON/CSV 导出三个任务，均为 `airsim`、`start_partial=false`。三航点 Action `SUCCEEDED`，暂停恢复后客户端取消 `CANCELED`，返航抢占为 Action `ABORTED`、队列 `CANCELED`，最终由独立遥测确认 Landed。具体数值及收尾限制见[首次集成验证](mission_recording.md#integration-validation)。

### 交互手动复测

控制节点、记录器、状态观察和任务命令分别在交互终端运行。三航点成功；慢速任务暂停后恢复，通过标准 Action 取消服务接受取消并观察到队列 `CANCELED`；新任务被返航服务抢占，Action 结果为 `ABORTED`，随后状态话题确认 `LANDED`。

正常停止记录器后，JSONL footer 与报告均为 6536 条样本；JSON/CSV 成功导出三个任务，均为 `adapter_mode=airsim`、`start_partial=false`。本轮没有取得取消目标的 Action 最终 `status=5`，不将队列取消等同于 Action 最终状态验证。具体统计与辅助脚本问题见[日志教程](mission_recording.md#manual-validation)。

### Action 最终取消状态补充复测

针对交互手动复测未取得最终取消状态的问题，重新运行第五次 PR 的 AirSim 集成流程。客户端保留 GoalHandle，暂停/恢复后调用取消并直接读取 `get_result_async()`，实际输出 `FINAL_CANCEL_RESULT: status=5, queue_state=CANCELED`。三航点 `SUCCEEDED`、返航抢占 Action `ABORTED` 及独立遥测确认 Landed 均通过。

本轮独立记录器正常停止，footer 与报告均为 298 条样本，三个任务的 JSON/CSV 成功导出；所有任务模式为 `airsim`、`start_partial=false`。这补齐了最终取消状态的直接证据，不将上一轮未取到结果改写为通过。详细输出、统计及起飞和接触限制见[最终取消状态补充复测](mission_recording.md#cancel-result-validation)。

## 当前已知限制

- **起飞返回值异常**：未取消起飞任务时，RPC 仍返回 `False`，但遥测确认升高约 1.5 m。原因未定位，不能宣称起飞接口正常。
- **控制能力**：支持单航点、多航点队列与 ROS 2 Action；普通航点容差为 1 m。未实现避障、路径规划或实际电子围栏。指令速度不等于实际速度曲线。
- **记录统计**：基于状态采样与接收时刻，不是精确航程、误差或停稳证明。话题无任务 UUID，不保证丢样情况下严格划分任务；磁盘耗尽和进程崩溃尚未实测。
- **故障处理**：`FAILSAFE` 记录故障并允许显式请求降落，不保证自动悬停、降落或中止原飞行任务。
- **坐标与落地检测**：使用 AirSim 本地 NED。屋顶解除解锁兼容逻辑仅适用于记录的起降点，不能推广到任意地形或真实无人机。
- **未验证场景**：模拟器进程崩溃、远离起点的显式降落、其他地形及真机。

此前测试出现过重置后尚未停稳、返航中断窗口错过，以及已弃用调度方案导致返航超时。这些轮次不计为通过；当前脚本增加停稳预检和请求前状态检查，控制器在处理请求前收尾已完成的派发任务。

## 大模型使用声明

使用大模型辅助代码实现、问题分析及文档整理。构建、单元测试与模拟器联调由实际命令执行；上述结果以实际测试为依据，未验证能力列于已知限制。
