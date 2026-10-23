# 无人机任务管理与安全状态机

单无人机控制模块，支持单航点与多航点队列、暂停/继续、返航降落，以及遥测超时和高度检查。原 AirSim 单航点流程在 Foxy 下验证；新增队列在 Ubuntu 22.04 / ROS 2 Humble 下使用本地运动演示适配器验证，尚未执行 AirSim 队列飞行。

- [构建与运行](usage.md)
- [多航点任务队列（Ubuntu 22.04 / ROS 2 Humble）](waypoint_queue.md)：顺序执行、进度反馈、暂停恢复和任务取消；含无需 AirSim 的演示。
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
