#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 两轮自平衡小车的纯逻辑实现（模型、PID、级联控制、指标、批量实验）。

本包**不依赖 ROS**：只依赖 `mujoco` 与 `numpy`，GUI 可视化是可选依赖。
这样设计的原因：仓库约定要求"main 脚本能直接跑通整个模块"，而评测机/虚拟机
常常没有图形界面；把仿真与可视化解耦后，`python3 main.py` 在任何机器上都能
跑完整实验并导出证据，`--gui` 才需要 GLFW。

模块划分：

    config.py     参数集合（PID 增益、场景时长、判据阈值），集中一处便于调参
    pid.py        PID（带积分限幅与微分项可选）
    env.py        MuJoCo 模型封装：复位、步进、状态读取、施加扰动
    controller.py 双环级联控制：位置 PID 外环 → 目标倾角，倾角 PID 内环 → 力矩
    metrics.py    评价指标：稳定时间、超调、稳态误差、恢复时间、成功率
    runner.py     场景批量实验 + 指标汇总 + CSV/PNG 导出
    plotting.py   matplotlib 可选导出（缺依赖时只写 CSV，不影响主流程）
"""

__version__ = "1.0.0"

from mujoco_balance_car.config import BalanceConfig, ScenarioConfig  # noqa: F401
from mujoco_balance_car.controller import BalanceController           # noqa: F401
from mujoco_balance_car.env import BalanceEnv, MujocoUnavailable      # noqa: F401
from mujoco_balance_car.pid import PIDController                      # noqa: F401
