import unittest

from ground_wall_follower_pid.core import PIDController, WallFollower


class TestPIDController(unittest.TestCase):
    def test_proportional_term(self):
        pid = PIDController(kp=2.0)
        self.assertAlmostEqual(pid.update(1.0, 0.1), 0.2)

    def test_reset_clears_state(self):
        pid = PIDController(kp=1.0, ki=1.0)
        pid.update(1.0, 0.1)
        pid.reset()
        # After reset, derivative of a fresh error is zero.
        self.assertAlmostEqual(pid.update(1.0, 0.1), 0.1)


class TestWallFollower(unittest.TestCase):
    def test_forward_blocked_enters_avoid(self):
        wf = WallFollower(forward_safety=0.4)
        cmd = wf.compute(side_distance=0.5, forward_distance=0.2, dt=0.1)
        self.assertEqual(cmd.state, "AVOID")
        self.assertEqual(cmd.linear, 0.0)
        self.assertGreater(cmd.angular, 0.0)

    def test_too_close_to_wall_turns_away(self):
        wf = WallFollower(target_distance=0.5)
        cmd = wf.compute(side_distance=0.3, forward_distance=2.0, dt=0.1)
        self.assertEqual(cmd.state, "FOLLOW")
        # error = target - side = 0.2 -> positive angular (turn left, away from wall)
        self.assertGreater(cmd.angular, 0.0)
        self.assertGreater(cmd.linear, 0.0)

    def test_too_far_from_wall_turns_back(self):
        wf = WallFollower(target_distance=0.5)
        cmd = wf.compute(side_distance=0.9, forward_distance=2.0, dt=0.1)
        self.assertEqual(cmd.state, "FOLLOW")
        self.assertLess(cmd.angular, 0.0)

    def test_missing_side_reading_holds_setpoint(self):
        wf = WallFollower(target_distance=0.5)
        cmd = wf.compute(side_distance=None, forward_distance=2.0, dt=0.1)
        self.assertEqual(cmd.state, "FOLLOW")
        self.assertAlmostEqual(cmd.angular, 0.0, places=3)


if __name__ == "__main__":
    unittest.main()
