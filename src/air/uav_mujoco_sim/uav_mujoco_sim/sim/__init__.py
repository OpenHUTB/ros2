"""仿真服务层：把 ``core`` 的 MuJoCo 仿真包装成 TCP 服务，供 ROS 侧连接。

两侧解释器无法共存（ROS 侧 rclpy 仅 py3.8 绑定，MuJoCo 3.12 要求 ≥3.10），故拆为独立进程走 TCP。
``SimServer`` 依赖 mujoco，不在本模块顶层导入，改由 ``__getattr__`` 按需加载。
"""

from .client import SimClient  # noqa: F401  —— 只依赖标准库 + numpy

__all__ = ["SimServer", "SimClient"]


def __getattr__(name):
    """``from uav_mujoco_sim.sim import SimServer`` 时才真正导入 server。"""
    if name == "SimServer":
        from .server import SimServer
        return SimServer
    raise AttributeError("module %r has no attribute %r" % (__name__, name))
