"""测距几何、碰撞与控制边界验证。 / Validate geometry, contacts and control boundaries."""
import unittest
import numpy as np

from warehouse_braking.simulation import DT, WarehouseSimulation, expert_target


class SimulationTests(unittest.TestCase):
    def test_center_ray_matches_box_face(self):
        sim = WarehouseSimulation("static", obstacle_x=6)
        self.assertAlmostEqual(sim.ranges()[4], 5.7, places=5)

    def test_reference_braking_stops_without_contact(self):
        sim = WarehouseSimulation("static", speed=1.8)
        for _ in range(600):
            target, _ = expert_target(sim.ranges(), sim.data.qvel[0], 1.8)
            sim.step(target)
        self.assertFalse(sim.collision)
        self.assertLess(abs(sim.data.qvel[0]), 0.05)
        self.assertGreater(sim.ranges()[4], 0.45)

    def test_uncontrolled_run_detects_collision(self):
        sim = WarehouseSimulation("static", obstacle_x=2, speed=1.8)
        for _ in range(100):
            sim.step(1.8)
        self.assertTrue(sim.collision)

    def test_crossing_obstacle_moves(self):
        sim = WarehouseSimulation("crossing")
        for _ in range(50):
            sim.step(0)
        self.assertLess(sim.data.mocap_pos[0, 1], 2.5)

    def test_height_hold(self):
        sim = WarehouseSimulation()
        for _ in range(100):
            sim.step(0)
        self.assertLess(abs(sim.position[2] - 1), 0.01)


if __name__ == "__main__":
    unittest.main()
