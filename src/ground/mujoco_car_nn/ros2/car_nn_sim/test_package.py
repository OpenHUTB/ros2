"""ROS 2 包内测试：``colcon test`` 直接跑 unittest，不依赖 pytest。

依赖 rclpy / mujoco 时执行，缺失则整体 skip，保证 CI 不红。
"""

import os
import sys
import unittest

try:  # ROS 2 运行时
    import rclpy  # noqa: F401

    HAVE_ROS = True
except Exception:  # pragma: no cover
    HAVE_ROS = False

try:  # MuJoCo 物理引擎
    import mujoco  # noqa: F401

    HAVE_MUJOCO = True
except Exception:  # pragma: no cover
    HAVE_MUJOCO = False

_MODULE_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_MODULE_ROOT, "src"))


@unittest.skipUnless(HAVE_ROS and HAVE_MUJOCO, "需要 rclpy 与 mujoco")
class TestNodeModules(unittest.TestCase):
    def test_sim_node_importable(self):
        from car_nn_sim import sim_node

        self.assertTrue(hasattr(sim_node, "CarSimNode"))

    def test_slam_node_importable(self):
        from car_nn_sim import slam_node

        self.assertTrue(hasattr(slam_node, "SlamNode"))


if __name__ == "__main__":
    unittest.main()
