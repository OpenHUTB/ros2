# 人形机器人 - 简化双足步态动画演示 ROS 2 封装模块

## 作业概述与背景
本模块实现简化双足步态演示，在 ROS 2 环境下实现了一个步行轨迹规划可视化节点。节点模拟了人形机器人一个完整步态周期（支撑期与摆动期）的腿部轨迹，并实时绘制出机器人的运动姿态，完成双足行走动画仿真。

## 运行环境
* 操作系统：Ubuntu 24.04（Jazzy 实测）/ Ubuntu 20.04（Foxy 实测）
* ROS 2 版本：Jazzy 实测、Foxy 实测；Humble 未实测
* 依赖库：rclpy, matplotlib

## 核心代码文件清单

本模块包含以下核心文件，均已提交至仓库：

```text
src/
└── humanoid_zmp_walk_planner/
    ├── humanoid_zmp_walk_planner/        # Python 源码目录
    │   ├── __init__.py                   # 空文件，标识包
    │   ├── gait.py                       # 纯函数：周期位移、相位、摆动脚高度
    │   └── main.py                       # 简化双足步态动画可视化节点
    ├── resource/                         # 资源目录
    │   └── humanoid_zmp_walk_planner     # 无后缀空文件
    ├── launch/
    │   └── main.launch.py                # 一键启动文件
    ├── test/                             # 单元测试（15 项，不需要 ROS）
    │   ├── conftest.py
    │   └── test_gait.py
    ├── package.xml                       # ROS 2 依赖声明文件
    ├── setup.py                          # ROS 2 功能包配置文件
    ├── setup.cfg                         # 脚本安装路径配置
    └── README.md                         # 模块说明文档
```

## 运行步骤

1. 新建标准工作空间并进入 src 目录：

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. 从 ros2 仓库拷贝模块代码到工作空间：

```
cp -r ~/OpenHUTB/ros2/src/humanoid_zmp_walk_planner ~/ros2_ws/src/
cd ~/ros2_ws
```

3. 编译工作空间并部署环境：

```
colcon build --packages-select humanoid_zmp_walk_planner
source install/setup.bash
```

4. 启动简化双足步态动画演示与可视化节点：

```
ros2 run humanoid_zmp_walk_planner main        # 入口为 main.*
ros2 launch humanoid_zmp_walk_planner main.launch.py
```

> 📖 **详细原理与实验步骤请参阅：** [点击查看主文档](../../docs/humanoid/zmp_walk_planner.md)

## 步态轨迹与测试

`step_length` 的语义是**单步**长度（一只脚迈出的距离），一个完整周期
`step_time`（1.2 s）左右各一步，所以躯干每周期前进 `2 × step_length = 1.2 m`，
速度 `1.0 m/s`。原实现 `x_body = t * step_length` 每周期走 `1.2 × 0.6 = 0.72 m`，
比“每步 0.6 m”应走的 1.2 m 少 40%。

轨迹抽到 `humanoid_zmp_walk_planner/gait.py`（纯函数，不依赖 ROS）：

- `body_x(t, step_length, step_time)`：每周期前进 `2 * step_length`；
- `phase(t, step_time)`：周期内相位 `[0, 1)`；
- `foot_heights(phase, step_height)`：摆动脚高度与支撑脚（前半周期左脚、后半周期右脚）；
- `leg_targets(...)`：动画需要的量一次算齐。

测试（不需要 ROS，也不需要图形界面）：

```bash
cd src/humanoid_zmp_walk_planner
python3 -m pytest test/ -q
```

覆盖 15 项：周期位移（1.2 m/周期、半周期 0.6 m）、速度 1.0 m/s、相位归一化、
支撑脚切换、摆动脚起落与峰值、高度不越界、切换点连续性、参数非法时报错。

> 行为变化：动画里躯干前进速度从 0.6 m/s 变为 `2 × 0.6 / 1.2 = 1.0 m/s`，
> 与“每步 0.6 m、周期 1.2 s”的参数含义一致。
