"""ROS2 差速机器人卡死检测与自动脱困。

核心算法与 ROS 解耦：``detect`` / ``recovery`` / ``robot`` 只用 NumPy 和标准库，
``node`` 负责把 ROS2 话题接到这套算法上。
"""
from .detect import (  # noqa: F401
    CAUSE_BLOCKED,
    CAUSE_NONE,
    CAUSE_ONE_WHEEL,
    CAUSE_SLIP,
    CAUSE_SPIN,
    DetectorConfig,
    Sample,
    StuckDetector,
    Verdict,
)
from .recovery import (  # noqa: F401
    STATE_ABORT,
    STATE_BACK_OFF,
    STATE_CRUISE,
    STATE_PROBE,
    STATE_PULSE,
    STATE_STALE,
    STATE_TURN,
    Command,
    RecoveryConfig,
    RecoverySupervisor,
)
from .robot import Limits, Twist, clamp_twist, integrate_pose, wheel_speeds, wheels_to_twist  # noqa: F401

__version__ = "1.0.0"
