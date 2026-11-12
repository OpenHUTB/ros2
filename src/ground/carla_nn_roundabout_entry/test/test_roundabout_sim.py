import unittest

import numpy as np

from carla_nn_roundabout_entry.end_to_end import render_state_observation
from carla_nn_roundabout_entry.roundabout_sim import (
    ObservationMapTracker,
    RoundaboutWorld,
    run_headless_with_controller,
)


class RoundaboutSimulationTests(unittest.TestCase):
    def test_rendered_observation_changes_with_traffic(self):
        ego = (0.0, 0.0, 0.05, 5.0)
        with_car = render_state_observation(ego, [(8.0, 0.5, 2.0, 0.0)])
        clear = render_state_observation(ego, [])
        np.testing.assert_array_equal(
            with_car,
            render_state_observation(ego, [(8.0, 0.5, 2.0, 0.0)]),
        )
        self.assertFalse(np.array_equal(with_car, clear))

    def test_map_tracker_integrates_scan_and_runs_astar(self):
        world = RoundaboutWorld(seed=7)
        tracker = ObservationMapTracker()
        tracker.observe(world)
        metrics = tracker.metrics()
        self.assertEqual(metrics["map_rays_integrated"], 25)
        self.assertEqual(metrics["path_search_frames"], 1)
        self.assertGreater(metrics["map_free_cells"], 0)

    def test_baseline_run_is_reproducible(self):
        first = run_headless_with_controller(7, 24, controller="baseline")
        second = run_headless_with_controller(7, 24, controller="baseline")
        self.assertEqual(first, second)
        self.assertEqual(first["map_rays_integrated"], 24 * 25)
        self.assertEqual(first["path_search_frames"], 24)


if __name__ == "__main__":
    unittest.main()
