import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_dwa_benchmark"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "ros2_dwa_avoidance"))

HAS_DEP = True
try:
    from benchmark import run_benchmark  # noqa: E402
    from metrics import summarize  # noqa: E402
    from ros2_dwa_avoidance.sim import SimResult, build_params  # noqa: E402
    from worlds import generate_world, is_feasible  # noqa: E402
except ModuleNotFoundError as exc:
    if not (exc.name or "").startswith(("ros2_dwa_avoidance", "benchmark", "metrics", "worlds")):
        raise
    HAS_DEP = False

if not HAS_DEP:
    raise RuntimeError(
        "ros2_dwa_avoidance is required for these tests; install/merge #1077 "
        "(e.g. PYTHONPATH=../ros2_dwa_avoidance). A run with all tests skipped "
        "must be treated as unverified.")


@unittest.skipUnless(HAS_DEP, "requires ros2_dwa_avoidance (merge #1077 first)")
class TestWorlds(unittest.TestCase):
    def test_generated_world_is_feasible(self):
        params = build_params()
        scenario = generate_world(1, 4, params)
        self.assertIsNotNone(scenario)
        self.assertTrue(is_feasible(scenario, params))
        self.assertEqual(len(scenario.circles), 4)

    def test_circles_stay_inside_corridor(self):
        params = build_params()
        scenario = generate_world(3, 7, params)
        self.assertIsNotNone(scenario)
        for circle in scenario.circles:
            self.assertGreaterEqual(circle.y - circle.r, -2.4)
            self.assertLessEqual(circle.y + circle.r, 2.4)
            self.assertGreater(circle.x, 0.0)
            self.assertLess(circle.x, 11.5)


@unittest.skipUnless(HAS_DEP, "requires ros2_dwa_avoidance (merge #1077 first)")
class TestMetrics(unittest.TestCase):
    def test_constant_acceleration(self):
        result = SimResult(
            scenario="t", controller="c", success=True, collision=False,
            time=0.3, path_length=0.3, min_clearance=0.5, final_distance=0.0,
            steps=3, trajectory=[[0, 0], [0.1, 0], [0.2, 0], [0.3, 0]],
            commands=[[0.1, 0.0], [0.2, 0.0], [0.3, 0.0]])
        summary = summarize(result, dt=0.1)
        self.assertAlmostEqual(summary["linear_acc_rms"], 1.0, places=4)
        self.assertAlmostEqual(summary["jerk_rms"], 0.0, places=4)

    def test_turning_metrics(self):
        result = SimResult(
            scenario="t", controller="c", success=True, collision=False,
            time=0.2, path_length=0.0, min_clearance=0.5, final_distance=0.0,
            steps=2, trajectory=[[0, 0], [0, 0], [0, 0]],
            commands=[[0.0, 0.2], [0.0, 0.4]])
        summary = summarize(result, dt=0.1)
        self.assertAlmostEqual(summary["angular_acc_rms"], 2.0, places=4)


@unittest.skipUnless(HAS_DEP, "requires ros2_dwa_avoidance (merge #1077 first)")
class TestBenchmarkSmoke(unittest.TestCase):
    def test_one_world_two_controllers(self):
        with tempfile.TemporaryDirectory() as tmp:
            # figdir="" keeps the smoke test free of matplotlib.
            payload = run_benchmark(
                os.path.join(tmp, "results.json"), "",
                densities=(2,), worlds_per_density=1)
            self.assertEqual(len(payload["rows"]), 2)
            self.assertEqual(len(payload["summary"]), 2)
            for row in payload["rows"]:
                self.assertIn("success", row)
                self.assertGreaterEqual(row["min_clearance_m"], 0.0)
            for summary in payload["summary"]:
                self.assertIn("edge_ratio", summary)
                self.assertIn("mean_time_all_s", summary)


if __name__ == "__main__":
    unittest.main()
