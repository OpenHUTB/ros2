# (作业) 蒋宇鑫 2535030012 NED↔ENU 坐标转换工具库 (ned_enu_transform)

## 1. 简介

OpenHUTB/AirSim 仿真器使用 **NED**（North-East-Down，x 北 y 东 z 下，偏航顺时针为正）坐标系，而 ROS 2 遵循 **REP-103** 使用 **ENU**（x 东 y 北 z 上，偏航逆时针为正）。做仿真桥接时若不统一坐标系，无人机指令会"南辕北辙"。本模块提供一个经过单元测试的纯 Python 转换工具库 + ROS2 演示节点，是无人机定位/桥接类模块的基础组件。

转换规则：

| 量 | 规则 |
| --- | --- |
| 位置/速度 | `x_ENU = x_NED, y_ENU = -y_NED, z_ENU = -z_NED`（自反变换） |
| 偏航角 | `yaw_ENU = -yaw_NED`（归一化到 (-π, π]） |
| 四元数 | `q_ENU = (w, x, -y, -z)` |

## 2. 运行环境

- Ubuntu 22.04 + ROS 2 Humble（或任何可运行 rclpy 的环境，如 RoboStack）
- 纯 Python 核心无任何第三方依赖；测试需 pytest

## 3. 目录结构

```text
src/air/ned_enu_transform/
├── main.py              # ROS2 演示节点: NED 航点 -> ENU PoseStamped
├── core.py              # 转换核心（纯 Python, 无 ROS 依赖）
└── test/test_core.py    # 9 项单元测试
```

## 4. 运行步骤

```bash
# 单元测试（无需 ROS）
pytest src/air/ned_enu_transform/test/test_core.py -v

# ROS2 演示节点
python3 src/air/ned_enu_transform/main.py
# 另一终端验证:
ros2 topic echo /drone/pose_enu
```

## 5. 单元测试运行结果

```text
test/test_core.py::test_position_roundtrip PASSED
test/test_core.py::test_position_signs PASSED
test/test_core.py::test_yaw_signs PASSED
test/test_core.py::test_yaw_wrap PASSED
test/test_core.py::test_quaternion_identity PASSED
test/test_core.py::test_quaternion_roundtrip PASSED
test/test_core.py::test_quaternion_norm_preserved PASSED
test/test_core.py::test_yaw_quaternion_consistency PASSED
test/test_core.py::test_openhutb_takeoff_case PASSED
9 passed
```

其中 `test_openhutb_takeoff_case` 使用真实场景校验：OpenHUTB 中无人机位于 NED (50, 30, -25)（北 50m、东 30m、高 25m），转换后 ENU 应为 (50, -30, 25)。

## 6. ROS2 节点运行结果

节点将 5 个 NED 航点（方形巡检航线，高度 10m）转换为 ENU 并发布到 `/drone/pose_enu`：

```text
[ned_enu_demo]: NED->ENU 转换演示节点启动, 话题: /drone/pose_enu
[ned_enu_demo]: NED(  0.0,  0.0,-10.0, 0.000) -> ENU(  0.0,  0.0, 10.0, 0.000)
[ned_enu_demo]: NED( 50.0,  0.0,-10.0, 0.000) -> ENU( 50.0,  0.0, 10.0, 0.000)
[ned_enu_demo]: NED( 50.0, 30.0,-10.0, 1.571) -> ENU( 50.0,-30.0, 10.0,-1.571)
[ned_enu_demo]: NED(  0.0, 30.0,-10.0, 3.142) -> ENU(  0.0,-30.0, 10.0, 3.142)
[ned_enu_demo]: NED(  0.0,  0.0,-10.0, 4.712) -> ENU(  0.0,  0.0, 10.0,-1.571)
```

## 7. 设计说明

- **核心与 ROS 解耦**：`core.py` 不 import 任何 ROS 包，可在无 ROS 的机器上运行测试（CI 友好）；
- **自反变换**：NED↔ENU 位置变换的逆是其自身，往返测试保证实现一致性；
- **偏航归一化**：所有角度输出归一化到 (-π, π]，避免下游控制器收到累积角。

## 8. 参考链接

- REP-103 坐标系约定: <https://www.ros.org/reps/rep-0103.html>
- OpenHUTB 坐标系换算: <https://openhutb.github.io/air_doc/dev/Carla_Air/coordinate_systems/>

## 9. 大模型使用声明

本模块代码与文档在编写过程中使用了大模型工具（WorkBuddy）辅助生成，作者已逐行核对全部内容，理解其原理并实际运行验证，对提交内容负全部责任。
