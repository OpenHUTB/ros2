import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

HAS_ROS = True
try:
    from ros2_dwa_gazebo.verify_nav import VerificationState  # noqa: E402
except Exception:  # rclpy is only available in a ROS 2 environment
    HAS_ROS = False


@unittest.skipUnless(HAS_ROS, "requires rclpy (ROS 2 environment)")
class TestVerificationState(unittest.TestCase):
    def test_identity_is_not_success(self):
        state = VerificationState()
        state.update_odom(0.0, 0.0)
        state.update_scan([1.0, 1.2])
        self.assertFalse(state.result()["success"])

    def test_moving_robot_succeeds(self):
        state = VerificationState(min_displacement=0.5)
        state.update_odom(0.0, 0.0)
        state.update_odom(0.6, 0.0)
        state.update_scan([0.8, 1.0])
        state.update_cmd(0.2, 0.0)
        result = state.result()
        self.assertTrue(result["success"])
        self.assertAlmostEqual(result["displacement_m"], 0.6, places=3)

    def test_close_obstacle_fails(self):
        state = VerificationState(min_displacement=0.5, min_clearance=0.12)
        state.update_odom(0.0, 0.0)
        state.update_odom(0.6, 0.0)
        state.update_scan([0.05, 1.0])
        state.update_cmd(0.2, 0.0)
        self.assertFalse(state.result()["success"])

    def test_no_command_fails(self):
        state = VerificationState(min_displacement=0.5)
        state.update_odom(0.0, 0.0)
        state.update_odom(0.6, 0.0)
        state.update_scan([1.0])
        self.assertFalse(state.result()["success"])

    def test_wall_clock_starts_without_odom(self):
        state = VerificationState()
        self.assertIsNotNone(state.wall_start)
        self.assertGreaterEqual(state.wall_age, 0.0)

    def test_goal_reached_counts_as_success(self):
        state = VerificationState(min_displacement=10.0, goal=(2.5, 0.0))
        state.update_odom(2.4, 0.0)
        state.update_scan([1.0])
        state.update_cmd(0.2, 0.0)
        result = state.result()
        self.assertTrue(result["goal_reached"])
        self.assertTrue(result["success"])


class TestLaunchFiles(unittest.TestCase):
    def test_world_and_launch_exist(self):
        root = os.path.join(os.path.dirname(__file__), "..")
        self.assertTrue(os.path.isfile(os.path.join(root, "launch", "main_launch.py")))
        self.assertTrue(os.path.isfile(os.path.join(root, "config", "gazebo_params.yaml")))


if __name__ == "__main__":
    unittest.main()
