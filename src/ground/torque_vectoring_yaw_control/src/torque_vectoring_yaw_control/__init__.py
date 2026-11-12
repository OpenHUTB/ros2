"""四轮独立驱动扭矩矢量横摆控制（ROS 2，纯 Python 可离线运行）。

核心三件：

* :class:`~.vehicle.VehicleSpec` —— 整车与轮胎参数、参考横摆率；
* :func:`~.allocator.allocate` —— 附着椭圆约束下的加权最小二乘分配（解析解 + 有效集）；
* :class:`~.world.VehicleSim` —— 三自由度闭环仿真。

典型用法::

    from torque_vectoring_yaw_control import VehicleSim, SimConfig

    sim = VehicleSim(cfg=SimConfig(u0=30.0, enable_tv=True))
    res = sim.run()
    print(res.yaw_rms_error)
"""

from .allocator import AllocResult, allocate, allocate_equal
from .control import ControlOutput, ControllerConfig, YawController
from .vehicle import G, VehicleSpec, WheelState
from .world import SimConfig, SimResult, VehicleSim

__version__ = "1.0.0"

__all__ = [
    "G", "VehicleSpec", "WheelState",
    "AllocResult", "allocate", "allocate_equal",
    "ControllerConfig", "ControlOutput", "YawController",
    "SimConfig", "SimResult", "VehicleSim", "__version__",
]
