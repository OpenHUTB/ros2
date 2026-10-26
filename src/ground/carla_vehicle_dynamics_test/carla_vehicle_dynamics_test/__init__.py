# -*- coding: utf-8 -*-
"""carla_vehicle_dynamics_test：CARLA 地面载具动力性测试（加速 / 制动 / 转向）。

模块结构：

* :mod:`metrics`        —— 纯数学函数（加速度、制动距离、圆拟合、理论转弯半径），可离线单测；
* :mod:`carla_utils`    —— CARLA 连接、同步步进、按车型生成/销毁车辆；
* :mod:`runner`         —— 三项测试的测量流程。

顶层 ``main.py`` 负责命令行入口与结果表格/曲线输出。
"""
