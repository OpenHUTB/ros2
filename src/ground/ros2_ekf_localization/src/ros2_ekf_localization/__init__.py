"""ros2_ekf_localization：差速机器人 EKF 多传感器融合定位。

核心算法（ekf.py、sensors.py、trajectories.py、safety.py）只依赖 NumPy，
可在无 ROS 环境下独立运行；node.py 提供 ROS2 接口，需要 rclpy 才能导入。
"""

from .ekf import EKFConfig, EKFLocalizer, normalize_angle
from .safety import FreshnessTracker, repair_covariance, sanitize_sigma
from .sensors import ImuNoise, OdometryNoise, PositionSensor, integrate_odometry
from .trajectories import TRAJECTORY_BUILDERS, Trajectory

__all__ = [
    "EKFConfig",
    "EKFLocalizer",
    "normalize_angle",
    "FreshnessTracker",
    "repair_covariance",
    "sanitize_sigma",
    "ImuNoise",
    "OdometryNoise",
    "PositionSensor",
    "integrate_odometry",
    "TRAJECTORY_BUILDERS",
    "Trajectory",
]
