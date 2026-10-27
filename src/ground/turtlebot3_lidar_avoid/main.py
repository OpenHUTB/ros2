#!/usr/bin/env python3
"""TurtleBot3 激光雷达自主避障 —— 模块统一入口。

仓库约定「每个模块必须支持 launch 启动，入口为 main.xxx」，
因此本文件作为模块入口，实际算法实现在
``turtlebot3_lidar_avoid/avoid.py`` 中。

三种启动方式等价：
    python3 main.py                                  # 直接运行
    ./main.sh                                        # 便捷脚本
    ros2 launch turtlebot3_lidar_avoid main.launch.py  # launch 启动
    ros2 run   turtlebot3_lidar_avoid avoid_node      # 节点启动

命令行参数（ROS 2 风格）会原样传给 rclpy，例如：
    python3 main.py --ros-args -p scan_topic:=/out/scan
"""

import os
import sys

# 保证以脚本方式运行时能导入同级的算法包
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from turtlebot3_lidar_avoid.avoid import main as avoid_main  # noqa: E402


if __name__ == '__main__':
    avoid_main()
