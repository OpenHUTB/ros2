import math
import random
import unittest

from uav_weather_risk.core.aircraft import AircraftModel, AircraftParams, AircraftState
from uav_weather_risk.core.risk import RiskThresholds, assess_risk, classify_risk
from uav_weather_risk.core.scenario import run_demo
from uav_weather_risk.core.types import WeatherSource, vnorm, vsub
from uav_weather_risk.core.weather import GustProcess, fuse_sources


class FusionTests(unittest.TestCase):
    def test_precise_source_dominates(self):
        sources = [
            WeatherSource("coarse", "nwp", (10.0, 0.0, 0.0), 3.0, 0.8),
            WeatherSource("precise", "in_situ", (4.0, 0.0, 0.0), 0.2, 0.9),
        ]
        fused = fuse_sources(sources)
        self.assertLess(fused.wind_enu[0], 5.0)
        self.assertAlmostEqual(fused.wind_enu[1], 0.0)
        self.assertGreater(fused.effective_samples, 1.0)

    def test_stale_source_is_downweighted(self):
        fixed = WeatherSource("fixed", "nwp", (0.0, 0.0, 0.0), 1.0, 1.0, 0.0)
        fresh = [fixed, WeatherSource("a", "in_situ", (10.0, 0.0, 0.0), 1.0, 1.0, 0.0)]
        stale = [fixed, WeatherSource("a", "in_situ", (10.0, 0.0, 0.0), 1.0, 1.0, 120.0)]
        self.assertLess(fuse_sources(stale).wind_enu[0], fuse_sources(fresh).wind_enu[0])

    def test_empty_sources_rejected(self):
        with self.assertRaises(ValueError):
            fuse_sources([])


class GustTests(unittest.TestCase):
    def test_reproducible(self):
        a = GustProcess(1.0, 2.0, random.Random(7))
        b = GustProcess(1.0, 2.0, random.Random(7))
        for _ in range(20):
            self.assertEqual(a.step(0.2), b.step(0.2))

    def test_zero_sigma_stays_zero(self):
        gust = GustProcess(0.0, 2.0, random.Random(1))
        for _ in range(5):
            self.assertEqual(gust.step(0.1), (0.0, 0.0, 0.0))


class AircraftTests(unittest.TestCase):
    def test_hover_converges_without_wind(self):
        model = AircraftModel(AircraftParams(), AircraftState(position_enu=(0.0, 0.0, 30.0)))
        target = (0.0, 0.0, 30.0)
        for _ in range(100):
            state = model.step(0.05, (0.0, 0.0, 0.0), target)
        self.assertLess(vnorm(vsub(state.position_enu, target)), 0.05)

    def test_wind_requires_more_control_authority(self):
        calm = AircraftModel(AircraftParams(), AircraftState(position_enu=(0.0, 0.0, 30.0)))
        windy = AircraftModel(AircraftParams(), AircraftState(position_enu=(0.0, 0.0, 30.0)))
        for _ in range(40):
            calm_state = calm.step(0.05, (0.0, 0.0, 0.0), (0.0, 0.0, 30.0))
            windy_state = windy.step(0.05, (12.0, 0.0, -2.0), (0.0, 0.0, 30.0))
        self.assertGreater(windy_state.control_saturation, calm_state.control_saturation)
        self.assertGreater(windy_state.tilt_deg, calm_state.tilt_deg)


class RiskTests(unittest.TestCase):
    def test_classification_boundaries(self):
        self.assertEqual(classify_risk(0.1, {"tilt": 0.1})[1], "NOMINAL")
        self.assertEqual(classify_risk(0.3, {"tilt": 0.3})[1], "CAUTION")
        self.assertEqual(classify_risk(0.5, {"tilt": 0.5})[1], "WARNING")
        self.assertEqual(classify_risk(0.8, {"tilt": 0.8})[1], "CRITICAL")
        self.assertEqual(classify_risk(0.1, {"tilt": 0.95})[1], "WARNING")

    def test_high_wind_has_higher_risk(self):
        thr = RiskThresholds(samples=24, horizon_s=6.0, dt_s=0.25, max_wind_mps=8.0)
        state = AircraftState(position_enu=(0.0, 0.0, 30.0))
        calm = assess_risk(state, (0.0, 0.0, 0.0), (0.0, 0.0, 30.0), thr, seed=3)
        storm = assess_risk(state, (14.0, 3.0, -3.0), (0.0, 0.0, 30.0), thr, seed=3)
        self.assertGreater(storm.score, calm.score)
        self.assertGreaterEqual(storm.level, calm.level)
        self.assertGreater(storm.probabilities["wind"], 0.5)

    def test_deterministic(self):
        thr = RiskThresholds(samples=16, horizon_s=4.0)
        state = AircraftState(position_enu=(1.0, 2.0, 30.0))
        a = assess_risk(state, (5.0, 1.0, 0.0), (10.0, 0.0, 30.0), thr, seed=11)
        b = assess_risk(state, (5.0, 1.0, 0.0), (10.0, 0.0, 30.0), thr, seed=11)
        self.assertEqual(a.as_dict(), b.as_dict())


class ScenarioTests(unittest.TestCase):
    def test_demo_is_deterministic(self):
        a = run_demo(duration_s=6.0, assessment_period_s=1.0, seed=5)
        b = run_demo(duration_s=6.0, assessment_period_s=1.0, seed=5)
        self.assertEqual(a.summary["record_sha256"], b.summary["record_sha256"])
        self.assertGreater(len(a.records), 0)

    def test_risk_levels_are_valid(self):
        result = run_demo(duration_s=8.0, assessment_period_s=1.0, seed=9)
        for record in result.records:
            self.assertIn(record["risk_level"], (0, 1, 2, 3))
            self.assertGreaterEqual(record["risk_score"], 0.0)
            self.assertLessEqual(record["risk_score"], 1.0)
            self.assertTrue(math.isfinite(record["fused_wind_speed"]))


if __name__ == "__main__":
    unittest.main()
