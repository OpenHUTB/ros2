# Humble / AirSim 手动复测证据

环境：Ubuntu 22.04.5 / ROS 2 Humble / Python 3.10.12 / WSL2；Windows OpenHUTB Air 场景；airsim 1.8.1、msgpack-rpc-python 0.4.1、tornado 4.5.3。模拟器构建版本未核实。

由作者在交互终端发送 ROS 命令，状态流使用 topic echo 保存。ROS_DOMAIN_ID=91，adapter_mode=airsim。起飞点 home_z=11.196125030517578；本轮航点 z=8.2，约高于起飞面 3 米，不能直接照搬至其他场景。

| 项目 | 原始流帧号（从 0 开始）与结果 |
| --- | --- |
| 三航点，速度 0.5 m/s | 268 → 301 → 330 → 358；completed 为 0、1、2、3，最终 COMPLETED |
| 执行中暂停/继续，速度 0.2 m/s | 1338 RUNNING → 1341 PAUSED → 1822 RUNNING；目标 [3,0,8.2]、索引 0 和速度保持；2015 COMPLETED |
| 取消执行中队列 | 2640 RUNNING → 2643 CANCELED；目标清空，后续未恢复旧航点 |
| 返航与自动降落 | 3375 RETURNING → 3440 LANDING → 3582 LANDED；队列保持 CANCELED |

最后记录位置约 [-0.008804,-0.030834,11.205312]，作者提供的场景截图显示无人机落在屋顶。

本地取证文件 waypoint_queue_airsim_states.jsonl 保留状态、队列进度、目标或速度变化段的首尾帧。原始流共 5742 条，存于复测机器 ~/queue_validation/airsim_manual/status.txt；该流从起飞后的 EXECUTING 开始，未覆盖初始连接和起飞过程，也不包含命令终端日志或时间戳。帧号表示采样顺序，不代表时间或独立测试次数。

已知异常：takeoff_rpc_result=false，last_error 保留 AirSim takeoff returned False; airborne state requires telemetry。本轮没有修复此起飞返回值问题。最初在第一轮 COMPLETED 后才暂停的操作不算执行中暂停；有效暂停来自后续 0.2 m/s 的路线。

普通航点使用 1 米到达容差，COMPLETED 不代表精确到点停稳。本轮未验证碰撞、避障、真机、断连或 AirSim 中的显式降落抢占和非法队列拒绝。后两项已有 demo/单元测试，不算本轮 AirSim 实测。测试发生于同一台机器，不替代合并前另一位同学在另一台机器的复测。

原始状态流与生成的日志仅在复测机器本地保留，不纳入源码仓库。
