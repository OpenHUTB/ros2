import unittest

import numpy as np

from gvnn.geometry import line_of_sight
from gvnn.planning.astar import GridPlanner
from gvnn.sim.world import SCENES, occupancy


class PlannerTest(unittest.TestCase):
    def test_all_scenes_have_paths(self):
        for name, scene in SCENES.items():
            occ = occupancy(scene, 0.05)
            planner = GridPlanner(scene.width, scene.height, 0.10, 0.42, 1.5)
            planner.set_map(occ)
            path = planner.plan(scene.start, scene.goal)
            self.assertIsNotNone(path, name)
            self.assertGreaterEqual(len(path), 2)
            blocked = ~planner.traversable
            for a, b in zip(path[:-1], path[1:]):
                self.assertTrue(
                    line_of_sight(blocked, a, b, 0.10, 0),
                    f"{name}: segment {a}->{b} crosses an inflated obstacle",
                )
            self.assertGreater(planner.path_cost(path), 10.0)

    def test_unknown_cost_is_traversable(self):
        occ = np.zeros((40, 40), dtype=bool)
        unknown = np.zeros_like(occ)
        unknown[:, 20:] = True
        planner = GridPlanner(4.0, 4.0, 0.10, 0.10, 2.0)
        planner.set_map(occ, unknown)
        path = planner.plan((0.3, 0.3), (3.7, 3.7))
        self.assertIsNotNone(path)


if __name__ == "__main__":
    unittest.main()
