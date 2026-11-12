import unittest

import numpy as np

from gvnn.sim.mujoco_backend import MuJoCoSimulation
from gvnn.sim.world import SCENES


class SimulationTest(unittest.TestCase):
    def test_step_and_sensors(self):
        sim = MuJoCoSimulation("warehouse", seed=1, noise=True)
        start = sim.truth.copy()
        sim.step(0.6, 0.0, 1.0)
        self.assertGreater(sim.truth[0] - start[0], 0.2)
        self.assertEqual(sim.collisions, 0)
        scan = sim.scan()
        self.assertEqual(scan.shape, (360,))
        self.assertTrue(np.all(scan >= 0.08))
        self.assertTrue(np.all(scan <= 9.0))
        bev = sim.bev_image()
        self.assertEqual(bev.shape, (64, 64, 3))
        rgb, depth = sim.depth_image(scan)
        self.assertEqual(rgb.shape, (48, 64, 3))
        self.assertEqual(depth.shape, (48, 64))

    def test_scene_sizes(self):
        for name in SCENES:
            sim = MuJoCoSimulation(name, seed=2, noise=False)
            self.assertEqual(sim.scene.name, name)
            self.assertGreater(sim.state()["goal_distance"], 1.0)


if __name__ == "__main__":
    unittest.main()
