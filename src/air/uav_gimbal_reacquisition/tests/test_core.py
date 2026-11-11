import math
import unittest

from uav_gimbal_reacquisition.core import (
    GimbalConfig,
    GimbalController,
    Metrics,
    TargetScenario,
    bearing_from_position,
    line_of_sight_error_deg,
)


class GeometryTests(unittest.TestCase):
    def test_bearing_and_los_error(self):
        camera = (0.0, 0.0, 20.0)
        target = (30.0, 0.0, 1.5)
        yaw, pitch = bearing_from_position(camera, target)
        self.assertAlmostEqual(yaw, 0.0)
        self.assertLess(pitch, 0.0)
        self.assertAlmostEqual(line_of_sight_error_deg(camera, target, yaw, pitch), 0.0, places=6)

    def test_coincident_positions_rejected(self):
        with self.assertRaises(ValueError):
            bearing_from_position((1.0, 2.0, 3.0), (1.0, 2.0, 3.0))


class ControllerTests(unittest.TestCase):
    def _run(self, duration=45.0, dt=0.05):
        scenario = TargetScenario()
        controller = GimbalController(GimbalConfig(loss_frames=3))
        metrics = Metrics()
        observations = []
        t = 0.0
        while t <= duration:
            observation = controller.step(scenario.sample(t), dt)
            metrics.update(observation)
            observations.append(observation)
            t += dt
        return observations, metrics

    def test_initial_search_acquires_target(self):
        observations, _ = self._run(duration=3.0)
        self.assertEqual(observations[0].state, GimbalController.SEARCH)
        self.assertTrue(any(item.state == GimbalController.TRACK for item in observations))

    def test_occlusion_causes_loss_and_reacquisition(self):
        observations, metrics = self._run()
        states = {item.state for item in observations}
        self.assertIn(GimbalController.LOST, states)
        self.assertGreaterEqual(metrics.loss_events, 2)
        self.assertGreaterEqual(metrics.reacquisition_events, 2)
        self.assertTrue(all(value > 0 for value in metrics.reacquisition_times_s))

    def test_metrics_are_finite(self):
        _, metrics = self._run()
        summary = metrics.summary()
        for key in ("track_ratio", "los_rms_deg", "pixel_rms_px", "max_los_error_deg"):
            self.assertTrue(math.isfinite(summary[key]))
        self.assertGreaterEqual(summary["track_ratio"], 0.0)
        self.assertLessEqual(summary["track_ratio"], 1.0)

    def test_scenario_is_deterministic(self):
        scenario = TargetScenario()
        self.assertEqual(scenario.sample(10.0), scenario.sample(10.0))
        self.assertFalse(scenario.sample(10.0).visible)
        self.assertTrue(scenario.sample(13.0).visible)


if __name__ == "__main__":
    unittest.main()
