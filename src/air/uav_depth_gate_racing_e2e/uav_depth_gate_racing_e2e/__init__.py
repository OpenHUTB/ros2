"""深度图端到端穿越门竞速。

分层：``core/`` 只依赖 numpy 与 mujoco，``ros/`` 依赖 rclpy；
``plotting`` 只依赖 numpy 与 matplotlib。任何一层都不 import 另一层的 ROS 部分。
"""
