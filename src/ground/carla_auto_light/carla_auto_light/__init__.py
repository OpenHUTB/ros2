# -*- coding: utf-8 -*-
"""carla_auto_light：CARLA 地面载具灯光自动控制。

模块结构：

* :mod:`light_logic`  —— 纯决策逻辑（自动大灯/雾灯/转向灯状态机/刹车灯），可离线单测；
* :mod:`carla_utils`  —— CARLA 连接、同步步进、生成车辆、按昼夜切换天气；
* :mod:`light_node`   —— ROS 2 节点：把灯光状态发布成话题、接受手动覆盖；
* :mod:`app`          —— 命令行入口与日志输出。

顶层 ``main.py`` 只是薄壳，负责把命令行参数交给 :mod:`app`。
"""
