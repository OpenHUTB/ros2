# 人形机器人轻量级 ZMP 步行控制基线

## 修改概述 / Overview

- 中文：新增 `humanoid_zmp_walking` 模块，用线性倒立摆模型（LIPM）完成
  「落脚点序列 → 平滑质心轨迹 → ZMP 反算 → 支撑多边形判定」的规划，
  再由 ROS 2 话题把 6 个关节指令送入 PyBullet 仿真。
- English: Add a lightweight ZMP walking baseline based on the linear
  inverted pendulum model (LIPM). It plans a footstep sequence, computes
  a ZMP reference from a smooth center-of-mass trajectory, verifies the
  ZMP stays inside the support polygon, and streams joint commands to a
  6-joint humanoid in PyBullet.

规划原理与限制见[教程页](../../../docs/humanoid/humanoid_zmp_walking.md)。

## 核心代码文件

| 文件 | 作用 |
|------|------|
| `humanoid_zmp_walking/lipm.py` | LIPM 正逆运算 |
| `humanoid_zmp_walking/footstep.py` | 落脚点序列与支撑多边形判定 |
| `humanoid_zmp_walking/gait.py` | 质心轨迹、ZMP 反算、关节时序 |
| `humanoid_zmp_walking/zmp_generator.py` | 轨迹与关节指令发布节点 |
| `humanoid_zmp_walking/play.py` | PyBullet 仿真节点 |
| `urdf/simple_humanoid.urdf` | 6 关节模型（含踝与脚掌） |
| `test/test_gait.py` | 单元测试 |
| `launch/main.launch.py` | 一键启动两个节点 |

## 运行环境

- 操作系统：Ubuntu 20.04
- ROS 版本：ROS 2 Foxy
- 依赖库：Python 3.8，`rclpy`，`std_msgs`，`launch`，`launch_ros`，
  `ament_index_python`，`pybullet`

## 安装与启动

```bash
colcon build --packages-select humanoid_zmp_walking
source install/setup.bash
ros2 launch humanoid_zmp_walking main.launch.py
```

无图形界面的环境可用：

```bash
ros2 run humanoid_zmp_walking play --ros-args -p use_gui:=false
```

## 参数

| 参数 | 节点 | 默认值 | 说明 |
|------|------|--------|------|
| `com_height` | `zmp_generator` | 0.65 | 质心高度 z_c（m） |
| `use_gui` | `play` | true | 是否打开 PyBullet 图形界面 |
| `sim_time_step` | `play` | 1/240 | 固定仿真步长（s） |
| `max_substeps` | `play` | 10 | 单次回调最多补跑的仿真步数 |

## 话题接口

| 话题 | 类型 | 元素含义 |
|------|------|----------|
| `/zmp_trajectory` | `Float64MultiArray` | `[t, zmp_x, zmp_y, com_x, com_y, in_support]` |
| `/joint_commands` | `Float64MultiArray` | `[左髋, 左膝, 左踝, 右髋, 右膝, 右踝]`（rad） |

`in_support` 为 1 表示该采样点的 ZMP 位于支撑多边形内。

## 测试

```bash
colcon test --packages-select humanoid_zmp_walking
colcon test-result --verbose
```

## 运行效果

![编译成功](../../../docs/img/humanoid/humanoid_zmp_walking/compile_success.png)

![演示动图](../../../docs/img/humanoid/humanoid_zmp_walking/zmp_walking_demo.gif)
