# -*- coding: utf-8 -*-
"""具身人形控制 —— 与仿真后端无关的纯计算层。

模块分工（每个文件只做一件事，且都能在无 MuJoCo / 无 ROS 的机器上单测）：

    geometry.py   齐次变换、二维旋转、角度归一化
    perception.py 激光测距 → 可通行性 / 障碍物 / 目标方位（纯函数）
    planning.py    代价地图 + A* 全局路径 + 纯跟踪前视点（纯函数）
    gait.py        参数化步态轨迹（相位 → 12 个关节目标角，纯函数）
    balance.py     姿态解算 + 平衡力矩 + 关节 PD（纯计算类）
    state_machine.py 任务状态机（IDLE→WALK→AVOID→REACHED/BLOCKED）
    metrics.py     误差 / 速度 / 平滑度等评价指标
"""
