"""倾转旋翼过渡走廊与倾转角调度（ROS 2，纯 Python 可离线运行）。

三个核心对象：

* :class:`~.vehicle.VehicleSpec` —— 机型参数（质量、机翼、旋翼）；
* :class:`~.corridor.Corridor` —— 过渡走廊：(空速, 倾转角) 平面上的可行域；
* :class:`~.world.TiltrotorSim` —— 纵向动力学仿真，含执行机构饱和与掉高保护。

典型用法::

    from uav_tiltrotor_transition_corridor import Corridor, TiltrotorSim

    corridor = Corridor()
    print(corridor.summary())

    sim = TiltrotorSim()
    res = sim.run(110.0)
    print(res.summary())
"""

from .aero import G, RHO0, WingSpec, isa_density
from .control import ActuatorState, Controller, ControllerConfig
from .corridor import Corridor, LevelSolution
from .pilot import PilotConfig, TransitionPilot
from .rotor import RotorSpec
from .vehicle import VehicleSpec
from .world import SimResult, TiltrotorSim

__version__ = "1.0.0"

__all__ = [
    "G", "RHO0", "WingSpec", "RotorSpec", "VehicleSpec",
    "Corridor", "LevelSolution", "isa_density",
    "Controller", "ControllerConfig", "ActuatorState",
    "TransitionPilot", "PilotConfig",
    "TiltrotorSim", "SimResult", "__version__",
]
