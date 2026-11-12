import math
import unittest

from forklift_slam_nav.simulation_math import yaw_from_freejoint


class FreeJointYawTest(unittest.TestCase):
    def test_yaw_quaternion_round_trip(self):
        angle = 0.7
        qpos = [0.0, 0.0, 0.22, math.cos(angle / 2), 0.0, 0.0, math.sin(angle / 2)]
        self.assertAlmostEqual(yaw_from_freejoint(qpos, 0), angle)


if __name__ == '__main__':
    unittest.main()
