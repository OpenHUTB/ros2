# UAV mission interfaces

`ament_cmake` 接口包，为同级 `uav_mission_safety_manager` 提供
`uav_mission_interfaces/action/ExecuteMission`。本包仅生成类型，没有独立节点或 launch。

在仓库根目录加载 ROS 2 Humble 后构建：

```bash
colcon build --packages-up-to uav_mission_safety_manager
source install/setup.bash
ros2 interface show uav_mission_interfaces/action/ExecuteMission
```

Goal 包含 1–100 个 AirSim NED 航点和等长的正数速度数组；完整约束、
反馈字段、取消与结果语义见[Action 教程](../../../docs/air/uav_mission_safety_manager/mission_action.md)。
接口使用 ROS 约定的 `ExecuteMission.action` 命名。

许可证：Apache-2.0，与任务管理器一致。
