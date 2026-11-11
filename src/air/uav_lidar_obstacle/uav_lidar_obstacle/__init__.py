# -*- coding: utf-8 -*-
"""无人机激光雷达避障 ROS 2 包。

``node.py`` 提供 ``UavLidarObstacleNode``（rclpy 包装节点），
``drone_control.py``（位于包根目录）是不依赖 ROS 的纯 CARLA / OpenHUTB 脚本。
"""

__all__ = ["node"]
