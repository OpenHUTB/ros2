"""四旋翼单桨失效推力分配与姿态权限评估（与 ROS 解耦的入口）."""

from .core import QuadrotorAllocator  # noqa: F401

__all__ = ['QuadrotorAllocator']
