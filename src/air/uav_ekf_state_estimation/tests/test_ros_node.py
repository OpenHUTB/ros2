"""ROS adapter shutdown behavior."""

from unittest import mock
import unittest

try:
    import rclpy  # noqa: F401
except ImportError:
    rclpy = None

if rclpy is not None:
    from uav_ekf_state_estimation import ros_node


@unittest.skipIf(rclpy is None, "rclpy is not installed")
class RosNodeMainTest(unittest.TestCase):
    def test_interrupt_does_not_shutdown_closed_context_twice(self):
        node = mock.MagicMock()
        with mock.patch.object(ros_node.rclpy, "init"), \
             mock.patch.object(
                 ros_node,
                 "EkfStateEstimationNode",
                 return_value=node,
             ), \
             mock.patch.object(
                 ros_node.rclpy,
                 "spin",
                 side_effect=KeyboardInterrupt,
             ), \
             mock.patch.object(ros_node.rclpy, "ok", return_value=False), \
             mock.patch.object(ros_node.rclpy, "shutdown") as shutdown:
            ros_node.main()

        node.destroy_node.assert_called_once_with()
        shutdown.assert_not_called()

    def test_interrupt_shuts_down_open_context_once(self):
        node = mock.MagicMock()
        with mock.patch.object(ros_node.rclpy, "init"), \
             mock.patch.object(
                 ros_node,
                 "EkfStateEstimationNode",
                 return_value=node,
             ), \
             mock.patch.object(
                 ros_node.rclpy,
                 "spin",
                 side_effect=KeyboardInterrupt,
             ), \
             mock.patch.object(ros_node.rclpy, "ok", return_value=True), \
             mock.patch.object(ros_node.rclpy, "shutdown") as shutdown:
            ros_node.main()

        node.destroy_node.assert_called_once_with()
        shutdown.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
