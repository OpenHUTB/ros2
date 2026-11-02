# 无人机任务管理与安全状态机

基于 ROS2 Foxy 和 AirSim 的单无人机控制模块，支持单航点任务、暂停/继续、返航降落，以及遥测超时和高度检查。

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
