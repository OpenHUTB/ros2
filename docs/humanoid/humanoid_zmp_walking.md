# humanoid_zmp_walking 使用说明

人形机器人轻量级 ZMP 步行控制基线（ROS 2 功能包）

## 修改概述 / Overview

- 中文：新增人形机器人轻量级 ZMP 步行控制基线。该模块包含轨迹生成节点与 PyBullet 仿真节点。生成节点以 100Hz 向 `/zmp_trajectory` 发布正弦参考信号；仿真节点加载 URDF 人形模型，订阅轨迹并通过位置控制驱动 4 个关节，可在 PyBullet GUI 中直观观察机器人腿部摆动过程。
- English: Add a lightweight ZMP walking baseline for humanoid robots. It includes a trajectory generator and a PyBullet simulation node. The generator publishes sinusoidal reference signals to `/zmp_trajectory` at 100Hz; the simulator loads a URDF humanoid model, subscribes to the trajectory and drives 4 joints via position control, visualizing the leg swinging in the PyBullet GUI.

## 运行环境

- 操作系统：Ubuntu 20.04
- ROS 版本：ROS 2 Foxy
- 依赖库：Python 3.8，`rclpy`，`std_msgs`，`launch`，`launch_ros`，`ament_index_python`，`pybullet`

## 核心代码文件

本模块包含以下核心文件，均已提交至仓库：

```
src/
└── humanoid/
    └── humanoid_zmp_walking/
        ├── launch/
        │   └── main.launch.py          # 节点一键启动文件
        ├── resource/
        │   └── humanoid_zmp_walking    # resource 资源标识文件
        ├── humanoid_zmp_walking/       # 核心代码子目录
        │   ├── __init__.py             # Python 包初始化文件
        │   ├── zmp_generator.py        # ZMP 参考轨迹生成节点
        │   ├── record.py               # 录制节点（占位）
        │   └── play.py                 # PyBullet 仿真执行节点
        ├── urdf/
        │   └── simple_humanoid.urdf    # 简化人形机器人模型
        ├── package.xml                 # ROS 2 功能包描述文件
        ├── setup.cfg                   # 节点安装路径配置文件
        └── setup.py                    # ROS 2 功能包构建配置文件
```

## 核心代码内容

本模块的核心代码已上传至仓库 `src/humanoid/humanoid_zmp_walking/` 目录下。

完整代码请点击下方链接查看：

* [查看 zmp_generator.py 代码](../../src/humanoid/humanoid_zmp_walking/humanoid_zmp_walking/zmp_generator.py)
* [查看 play.py 代码](../../src/humanoid/humanoid_zmp_walking/humanoid_zmp_walking/play.py)
* [查看 record.py 代码](../../src/humanoid/humanoid_zmp_walking/humanoid_zmp_walking/record.py)
* [查看 main.launch.py 代码](../../src/humanoid/humanoid_zmp_walking/launch/main.launch.py)
* [查看 simple_humanoid.urdf 代码](../../src/humanoid/humanoid_zmp_walking/urdf/simple_humanoid.urdf)
* [查看 setup.py 代码](../../src/humanoid/humanoid_zmp_walking/setup.py)

## 功能说明 / Boundary behavior

- 中文：生成节点周期性向 `/zmp_trajectory` 发布单自由度正弦参考角（幅度 0.2 rad，频率 0.5Hz 量级）；仿真节点接收数据后将其作为全部 4 个关节的目标位置，通过 `setJointMotorControl2` 位置控制驱动。机器人在重力作用下经历站立、摆动、倾倒的完整动力学过程，不倒置循环，行为由物理引擎实时解算。
- English: The generator periodically publishes a single-DOF sinusoidal reference angle (amplitude 0.2 rad) to `/zmp_trajectory`. The simulator applies it as the target position of all 4 joints via `setJointMotorControl2` position control. Under gravity the robot goes through standing, swinging and falling, all solved by the physics engine in real time without looping.

## 启动步骤

1. 进入 ROS 2 工作空间

```
cd <你的 ROS 2 工作空间>
```

2. 编译功能包

```
colcon build --packages-select humanoid_zmp_walking
source install/setup.bash
```

3. 使用 launch 文件一键启动（轨迹生成 + PyBullet 仿真节点）

```
ros2 launch humanoid_zmp_walking main.launch.py
```

4. 新开一个终端窗口，同样进入工作空间并加载环境，查看轨迹数据流

```
cd <你的 ROS 2 工作空间>
source install/setup.bash
ros2 topic echo /zmp_trajectory
```

## 运行效果

节点启动后，PyBullet GUI 将显示人形机器人模型，腿部按正弦轨迹摆动；新开终端执行 `echo` 指令后，可持续打印 `/zmp_trajectory` 话题数据。

1. **编译成功**

![编译成功](../img/humanoid/humanoid_zmp_walking/compile_success.png)

2. **完整操作流程演示动图**

![完整演示动图](../img/humanoid/humanoid_zmp_walking/zmp_walking_demo.gif)
