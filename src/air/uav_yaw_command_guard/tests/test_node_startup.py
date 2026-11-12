"""Regression: the ROS node must be constructible without a NameError.

Requires rclpy; skipped when ROS 2 is unavailable (e.g. plain pytest CI).
"""
import unittest

try:
    import rclpy
    from uav_yaw_command_guard.nodes import YawCommandGuardNode
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False


@unittest.skipUnless(HAVE_ROS, "rclpy 不可用")
class NodeStartupTests(unittest.TestCase):
    def test_node_starts_and_sets_start_time(self):
        rclpy.init()
        node = None
        try:
            node = YawCommandGuardNode()
            self.assertGreater(node.started_at, 0.0)
        finally:
            if node is not None:
                node.csv_file.close()
                node.destroy_node()
            rclpy.shutdown()
