"""无动力被动行走（rimless wheel）作业模块。

核心只有两件事：
1. ``assets/passive_walker.xml``：一个没有执行器的轮辐式行走器 + 5° 斜坡；
2. ``passive_walker.sim.PassiveWalker``：推进 MuJoCo 仿真并统计步数/位移。

``sim`` 模块本身不依赖 ROS，可以直接 ``python -m passive_walker.sim`` 验证；
``sim_node`` 才需要 rclpy。
"""

from .sim import PassiveWalker, find_asset, format_summary, run_headless

__all__ = ["PassiveWalker", "find_asset", "format_summary", "run_headless"]
