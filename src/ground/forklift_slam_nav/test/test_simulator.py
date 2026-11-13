import math
import unittest

from forklift_slam_nav.simulation_math import advance_planar_pose


class PlanarPoseUpdateTest(unittest.TestCase):
    def test_forward_motion_advances_position_over_multiple_steps(self):
        pose = (0.0, 0.0, 0.0)
        for _ in range(4):
            pose = advance_planar_pose(*pose, speed=0.5, yaw_rate=0.0, dt=0.1)

        self.assertAlmostEqual(pose[0], 0.2)
        self.assertAlmostEqual(pose[1], 0.0)
        self.assertAlmostEqual(pose[2], 0.0)

    def test_turning_updates_heading_and_position(self):
        pose = advance_planar_pose(0.0, 0.0, 0.0, 0.5, math.pi / 2, 1.0)

        self.assertAlmostEqual(pose[0], 0.0, places=7)
        self.assertAlmostEqual(pose[1], 0.5)
        self.assertAlmostEqual(pose[2], math.pi / 2)

    def test_rotation_in_place_preserves_position(self):
        pose = advance_planar_pose(1.0, -2.0, 0.0, 0.0, 1.0, 0.25)

        self.assertAlmostEqual(pose[0], 1.0)
        self.assertAlmostEqual(pose[1], -2.0)
        self.assertAlmostEqual(pose[2], 0.25)


if __name__ == '__main__':
    unittest.main()
