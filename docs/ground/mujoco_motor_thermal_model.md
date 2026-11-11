# MuJoCo 差速小车电机温升模型

本包使用 MuJoCo 模拟差速小车，并根据驱动转矩、电机电流、绕组电阻和速度相关散热计算左右电机温度。它包含独立仿真入口、ROS 2 launch 和热模型测试。

## 验证

在 Ubuntu 22.04 / ROS 2 Humble / Python 3.10 / MuJoCo 3.2 环境中执行包构建和测试；模型模块可直接按 README 命令启动。

![MuJoCo 差速小车电机热模型运行画面](../img/ground/mujoco_motor_thermal_model/mujoco_demo.png)
