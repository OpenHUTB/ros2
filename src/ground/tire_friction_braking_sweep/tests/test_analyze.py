"""数据集、拟合与瓶颈判定的校验。"""

import unittest

import numpy as np

from tire_friction_braking_sweep.analyze import (
    BrakingPoint,
    SweepDataset,
    classify_points,
    elasticity,
    fit_saturation,
    summary,
)
from tire_friction_braking_sweep.model import (
    GRAVITY,
    BrakingSpec,
    braking_distance_m,
    saturated_decel_mps2,
)


def make_points(a_max: float = 6.4, mu_half: float = 0.5, frictions=(0.4, 0.8, 1.2, 1.6, 2.4), speed_kmh: float = 50.0):
    """按已知饱和模型生成一组"理想"数据点。"""
    speed = speed_kmh / 3.6
    points = []
    for mu in frictions:
        decel = float(saturated_decel_mps2(mu, a_max, mu_half))
        points.append(
            BrakingPoint(
                tire_friction=float(mu),
                entry_speed_mps=speed,
                distance_m=float(braking_distance_m(speed, decel)),
                duration_s=speed / decel,
            )
        )
    return points


class BrakingPointTest(unittest.TestCase):
    def test_derived_quantities(self):
        point = BrakingPoint(tire_friction=0.8, entry_speed_mps=13.888888, distance_m=24.5, duration_s=3.5)
        self.assertAlmostEqual(point.decel_mps2, 13.888888**2 / (2 * 24.5), places=4)
        self.assertAlmostEqual(point.ideal_distance_m, 13.888888**2 / (2 * 0.8 * GRAVITY), places=4)
        self.assertGreater(point.distance_ratio, 1.0)

    def test_zero_distance_gives_nan_decel(self):
        point = BrakingPoint(tire_friction=0.8, entry_speed_mps=10.0, distance_m=0.0, duration_s=0.0)
        self.assertNotEqual(point.decel_mps2, point.decel_mps2)  # nan


class FitTest(unittest.TestCase):
    def test_fit_recovers_known_parameters(self):
        points = make_points(a_max=6.4, mu_half=0.5)
        fit = fit_saturation(points)
        self.assertAlmostEqual(fit.a_max_mps2, 6.4, delta=0.05)
        self.assertAlmostEqual(fit.mu_half, 0.5, delta=0.05)
        self.assertGreater(fit.r2, 0.999)

    def test_fit_needs_three_points(self):
        with self.assertRaises(ValueError):
            fit_saturation(make_points()[:2])


class ElasticityTest(unittest.TestCase):
    def test_elasticity_matches_definition(self):
        points = make_points()
        slopes = elasticity(points)
        self.assertNotEqual(slopes[0], slopes[0])  # 第一个点为 nan
        previous, current = points[0], points[1]
        expected = ((current.distance_m - previous.distance_m) / previous.distance_m) / (
            (current.tire_friction - previous.tire_friction) / previous.tire_friction
        )
        self.assertAlmostEqual(slopes[1], expected, places=9)

    def test_gain_shrinks_as_friction_grows(self):
        slopes = np.array([abs(value) for value in elasticity(make_points(frictions=(0.4, 0.8, 1.2, 2.0, 4.0)))[1:]])
        self.assertTrue(np.all(np.diff(slopes) < 0.0))

    def test_classification_flags_saturation(self):
        points = make_points(a_max=6.4, mu_half=0.5, frictions=(0.4, 0.8, 1.2, 2.0, 4.0))
        labels = classify_points(points)
        self.assertEqual(labels[0], "附着受限")
        self.assertEqual(labels[-1], "制动器受限")


class SummaryTest(unittest.TestCase):
    def test_summary_structure_and_consistency(self):
        dataset = SweepDataset(
            spec=BrakingSpec(),
            points=make_points(),
            source="synthetic",
            environment={"note": "unit test"},
        )
        payload = summary(dataset)
        self.assertEqual(len(payload["points"]), 5)
        self.assertEqual(payload["source"], "synthetic")
        self.assertIn("a_max_mps2", payload["fit"])
        self.assertGreater(payload["range"]["distance_at_mu_min_m"], payload["range"]["distance_at_mu_max_m"])
        self.assertGreater(payload["range"]["distance_reduction_percent"], 0.0)
        for row in payload["points"]:
            self.assertIn("bottleneck", row)
            self.assertIn("elasticity", row)

    def test_summary_needs_three_points(self):
        with self.assertRaises(ValueError):
            summary(SweepDataset(spec=BrakingSpec(), points=make_points()[:2]))


if __name__ == "__main__":
    unittest.main()
