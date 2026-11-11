"""Unit tests for the geometry, odometry, filter and scenario layers.

The suite needs NumPy only: no ROS 2, no matplotlib and no simulator. Run it
from the package directory:

    python -m unittest discover -s tests -v
"""

import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from uwb_range_odom_fusion.ekf import RangeEkf
from uwb_range_odom_fusion.geometry import (
    as_anchors,
    gdop,
    gdop_map,
    nlos_bias,
    range_covariance,
    ranges_of,
    segment_distance,
    trilaterate,
)
from uwb_range_odom_fusion.headless import run_experiment
from uwb_range_odom_fusion.metrics import (
    chi2_quantile,
    heading_errors,
    nees_consistency,
    position_errors,
    summarize,
)
from uwb_range_odom_fusion.odometry import (
    WheelParameters,
    odometry_of,
    unicycle_step,
    wrap_angle,
)
from uwb_range_odom_fusion.runner import run_ekf, run_odometry_only
from uwb_range_odom_fusion.scenario import (
    ScenarioConfig,
    build_rounded_rectangle,
    simulate,
)

SQUARE = ((6.0, 4.0), (-6.0, 4.0), (-6.0, -4.0), (6.0, -4.0))
COLLINEAR = ((-6.0, 0.0), (-2.0, 0.0), (2.0, 0.0), (6.0, 0.0))


class GeometryTests(unittest.TestCase):
    def test_gdop_of_spread_layout_is_close_to_one(self):
        self.assertAlmostEqual(gdop((0.0, 0.0), SQUARE), 1.0833, places=3)

    def test_gdop_blows_up_on_the_anchor_line(self):
        self.assertTrue(math.isinf(gdop((0.0, 0.0), COLLINEAR)))
        self.assertGreater(gdop((0.0, 1.0), COLLINEAR), 1.5)

    def test_gdop_map_shape_and_range(self):
        xs = np.arange(-2.0, 2.01, 0.5)
        ys = np.arange(-1.0, 1.01, 0.5)
        grid = gdop_map(xs, ys, SQUARE)
        self.assertEqual(grid.shape, (len(ys), len(xs)))
        self.assertTrue(np.all(grid > 1.0))
        self.assertTrue(np.all(grid < 2.0))

    def test_exact_ranges_recover_the_position(self):
        point = np.array([1.0, -0.5])
        result = trilaterate(SQUARE, ranges_of(point, SQUARE))
        self.assertIsNotNone(result)
        self.assertLess(float(np.linalg.norm(result.position - point)), 1e-6)
        self.assertTrue(result.converged)

    def test_huber_downweights_a_gross_outlier(self):
        point = np.array([1.0, -0.5])
        measurement = ranges_of(point, SQUARE)
        measurement[2] += 3.0
        robust = trilaterate(SQUARE, measurement)
        plain = trilaterate(SQUARE, measurement, robust=False)
        self.assertLess(float(np.linalg.norm(robust.position - point)), 0.15)
        self.assertGreater(float(np.linalg.norm(plain.position - point)), 0.5)
        self.assertLess(robust.weights[2], 0.1)

    def test_trilateration_needs_three_anchors(self):
        with self.assertRaises(ValueError):
            as_anchors(((0.0, 0.0), (1.0, 0.0)))

    def test_range_covariance_grows_when_geometry_is_poor(self):
        good = range_covariance((0.0, 0.0), SQUARE, 0.04)
        poor = range_covariance((0.0, 0.3), COLLINEAR, 0.04)
        self.assertGreater(float(np.trace(poor)), 5.0 * float(np.trace(good)))

    def test_nlos_bias_only_when_a_pillar_blocks_the_ray(self):
        blocked, chord = nlos_bias((2.0, 0.0), (-6.0, 0.0), ((0.0, 0.0, 0.35),))
        clear, clear_chord = nlos_bias((2.0, 0.0), (-6.0, 0.0), ((10.0, 10.0, 0.35),))
        self.assertGreater(blocked, 0.3)
        self.assertAlmostEqual(chord, 0.7, places=6)
        self.assertEqual(clear, 0.0)
        self.assertEqual(clear_chord, 0.0)

    def test_segment_distance_of_a_point_between_two_endpoints(self):
        self.assertAlmostEqual(segment_distance((0.0, 0.0), (2.0, 0.0), (1.0, 1.0)), 1.0)
        self.assertAlmostEqual(segment_distance((0.0, 0.0), (2.0, 0.0), (5.0, 0.0)), 3.0)


class OdometryTests(unittest.TestCase):
    def setUp(self):
        self.wheel = WheelParameters()

    def test_radius_calibration_gives_two_percent_speed_bias(self):
        v_odom, _ = odometry_of(0.5, 0.0, self.wheel)
        self.assertAlmostEqual(v_odom / 0.5, self.wheel.radius_est / self.wheel.radius_true,
                               places=4)

    def test_wheelbase_calibration_gives_yaw_rate_bias(self):
        _, omega_odom = odometry_of(0.0, 0.3, self.wheel)
        ratio = omega_odom / 0.3
        self.assertGreater(ratio, 1.03)
        self.assertLess(ratio, 1.04)

    def test_left_right_mismatch_biases_straight_driving(self):
        _, omega_odom = odometry_of(0.5, 0.0, self.wheel)
        self.assertLess(omega_odom, -0.005)

    def test_unicycle_straight_line(self):
        state = unicycle_step((1.0, 2.0, 0.0), 0.5, 0.0, 2.0)
        self.assertAlmostEqual(state[0], 2.0)
        self.assertAlmostEqual(state[1], 2.0)
        self.assertAlmostEqual(state[2], 0.0)

    def test_unicycle_quarter_circle(self):
        v, omega = 0.5, math.pi / 2.0
        radius = v / omega
        state = (0.0, 0.0, 0.0)
        for _ in range(1000):
            state = unicycle_step(state, v, omega, 0.001)
        self.assertAlmostEqual(state[0], radius, places=2)
        self.assertAlmostEqual(state[1], radius, places=2)
        self.assertAlmostEqual(state[2], math.pi / 2.0, places=3)

    def test_wrap_angle(self):
        self.assertAlmostEqual(wrap_angle(3 * math.pi), -math.pi, places=9)
        self.assertAlmostEqual(wrap_angle(-3 * math.pi), -math.pi, places=9)
        self.assertAlmostEqual(wrap_angle(0.3), 0.3)


class FilterTests(unittest.TestCase):
    def _filter(self, anchors=SQUARE, use_gating=True):
        return RangeEkf(
            initial_state=(0.0, 0.0, 0.0),
            initial_covariance=np.diag([0.05**2, 0.05**2, np.deg2rad(1.0) ** 2]),
            sigma_range=0.04,
            sigma_v=0.05,
            sigma_omega=0.07,
            gate_chi2=6.635,
            use_gating=use_gating,
        )

    def test_predict_matches_the_unicycle_model(self):
        filter_ = self._filter()
        filter_.predict(0.5, 0.2, 0.05)
        expected = unicycle_step((0.0, 0.0, 0.0), 0.5, 0.2, 0.05)
        np.testing.assert_allclose(filter_.state, expected, atol=1e-12)

    def test_predict_grows_the_covariance(self):
        filter_ = self._filter()
        before = float(np.trace(filter_.covariance))
        filter_.predict(0.5, 0.2, 0.1)
        self.assertGreater(float(np.trace(filter_.covariance)), before)

    def test_gating_rejects_a_three_metre_bias(self):
        filter_ = self._filter()
        measurement = ranges_of((0.0, 0.0), SQUARE) + 3.0
        outcome = filter_.update(SQUARE, measurement)
        self.assertEqual(outcome.accepted, 0)
        self.assertEqual(outcome.rejected, 4)

    def test_gating_keeps_unbiased_ranges(self):
        filter_ = self._filter()
        rng = np.random.default_rng(0)
        rejected = 0
        total = 0
        for _ in range(150):
            filter_.predict(0.4, 0.0, 0.05)
            measurement = ranges_of(filter_.state[:2], SQUARE) + rng.normal(0.0, 0.04, 4)
            outcome = filter_.update(SQUARE, measurement)
            rejected += outcome.rejected
            total += 4
        self.assertLess(rejected / total, 0.05)

    def test_update_reduces_uncertainty(self):
        filter_ = self._filter()
        before = float(np.trace(filter_.covariance[:2, :2]))
        filter_.update(SQUARE, ranges_of((0.02, 0.01), SQUARE))
        self.assertLess(float(np.trace(filter_.covariance[:2, :2])), before)

    def test_nees_of_a_consistent_filter_is_low(self):
        filter_ = self._filter()
        filter_.update(SQUARE, ranges_of((0.0, 0.0), SQUARE))
        self.assertLess(filter_.nees((0.01, 0.0, 0.0)), 20.0)


class ScenarioTests(unittest.TestCase):
    def test_reference_path_is_closed_and_has_the_expected_length(self):
        path = build_rounded_rectangle(4.5, 2.8, 1.1, 0.02)
        self.assertAlmostEqual(float(np.linalg.norm(path.points[0] - path.points[-1])), 0.0,
                               places=6)
        expected = 4 * (4.5 - 1.1) + 4 * (2.8 - 1.1) + 2 * math.pi * 1.1
        self.assertAlmostEqual(path.length, expected, delta=0.05)

    def test_simulation_is_deterministic(self):
        config = ScenarioConfig(duration_s=8.0)
        first = simulate(config, seed=5)
        second = simulate(config, seed=5)
        np.testing.assert_array_equal(first.truth, second.truth)
        np.testing.assert_array_equal(first.odometry, second.odometry)
        np.testing.assert_allclose(first.ranges, second.ranges, equal_nan=True)

    def test_simulation_marks_nlos_only_when_ranges_are_biased(self):
        simulation = simulate(ScenarioConfig(duration_s=20.0), seed=1)
        nominal = np.linalg.norm(
            simulation.anchors[None, :, :] - simulation.truth[:, None, :2], axis=2
        )
        residual = simulation.ranges - nominal
        measured = np.isfinite(simulation.ranges)
        biased = simulation.nlos & ~simulation.outliers & measured
        self.assertTrue(np.any(biased))
        self.assertTrue(np.all(residual[biased] > 0.2))
        clean = ~simulation.nlos & ~simulation.outliers & measured
        self.assertTrue(np.all(np.abs(residual[clean]) < 0.5))

    def test_truth_stays_on_the_reference_path(self):
        simulation = simulate(ScenarioConfig(duration_s=10.0), seed=0)
        distance = np.linalg.norm(simulation.truth[:, :2] - simulation.path.points[0], axis=1)
        self.assertGreater(float(np.min(distance)), -1.0)
        self.assertTrue(np.all(np.abs(simulation.truth[:, 0]) <= 4.8))
        self.assertTrue(np.all(np.abs(simulation.truth[:, 1]) <= 3.1))

    def test_invalid_rate_ratio_is_rejected(self):
        with self.assertRaises(ValueError):
            ScenarioConfig(control_hz=30.0, truth_hz=200.0).validate()


class MetricTests(unittest.TestCase):
    def test_summarize_of_a_known_signal(self):
        summary = summarize([0.0, 3.0, 0.0, 4.0])
        self.assertAlmostEqual(summary["rmse"], math.sqrt(25.0 / 4.0), places=9)
        self.assertAlmostEqual(summary["max"], 4.0)
        self.assertEqual(summary["samples"], 4)

    def test_heading_error_wraps_across_pi(self):
        error = heading_errors([math.radians(179.0)], [math.radians(-179.0)])
        self.assertAlmostEqual(abs(float(error[0])), math.radians(2.0), places=9)

    def test_chi2_quantile_matches_the_table(self):
        self.assertAlmostEqual(chi2_quantile(3, 0.975), 9.348, delta=0.1)
        self.assertAlmostEqual(chi2_quantile(1, 0.95), 3.841, delta=0.15)

    def test_nees_consistency_interval(self):
        rng = np.random.default_rng(0)
        values = rng.chisquare(3, size=2000)
        result = nees_consistency(values)
        self.assertTrue(result["consistent"])
        self.assertAlmostEqual(result["mean"], 3.0, delta=0.3)


class IntegrationTests(unittest.TestCase):
    def test_gated_filter_beats_dead_reckoning(self):
        simulation = simulate(ScenarioConfig(duration_s=25.0), seed=3)
        gated = run_ekf(simulation)
        odometry = run_odometry_only(simulation)
        gated_error = gated.metrics(simulation)["position_m"]["rmse"]
        odometry_error = odometry.metrics(simulation)["position_m"]["rmse"]
        self.assertLess(gated_error, 0.05)
        self.assertGreater(odometry_error, 0.2)

    def test_position_error_of_a_perfect_estimate_is_zero(self):
        simulation = simulate(ScenarioConfig(duration_s=5.0), seed=0)
        errors = position_errors(simulation.truth[:, :2], simulation.truth[:, :2])
        self.assertAlmostEqual(float(np.max(errors)), 0.0)

    def test_headless_driver_writes_evidence_without_plots(self):
        with tempfile.TemporaryDirectory() as directory:
            summary = run_experiment(
                seeds=1,
                duration_s=10.0,
                out_dir=Path(directory),
                make_plots=False,
            )
            self.assertTrue((Path(directory) / "summary.json").exists())
            self.assertTrue((Path(directory) / "timeseries.csv").exists())
            stored = json.loads((Path(directory) / "summary.json").read_text(encoding="utf-8"))
            self.assertEqual(stored["module"], "uwb_range_odom_fusion")
            self.assertLess(summary["main"]["ekf_gated"]["rmse_mean_m"], 0.1)
            self.assertGreater(summary["main"]["odometry_only"]["rmse_mean_m"], 0.05)


if __name__ == "__main__":
    unittest.main()
