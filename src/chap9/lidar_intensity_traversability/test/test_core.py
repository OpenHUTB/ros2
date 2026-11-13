"""core.py 的离线单元测试：不依赖 ROS。"""

import math
import unittest

from lidar_intensity_traversability import core


def make_beam(angle_deg, rng, intensity):
    conf = core.confidence_from_intensity(intensity)
    return {
        "angle_deg": angle_deg,
        "range": rng,
        "intensity": intensity,
        "confidence": conf,
        "effective": core.effective_range(rng, conf),
    }


class TestConfidence(unittest.TestCase):
    def test_strong_return_is_full_confidence(self):
        self.assertAlmostEqual(core.confidence_from_intensity(5000.0), 1.0)

    def test_zero_intensity_hits_floor(self):
        self.assertAlmostEqual(core.confidence_from_intensity(0.0), 0.05)

    def test_glass_is_weak(self):
        self.assertLess(core.confidence_from_intensity(120.0), 0.2)

    def test_negative_ref_rejected(self):
        with self.assertRaises(ValueError):
            core.confidence_from_intensity(10.0, ref=0.0)


class TestEffectiveRange(unittest.TestCase):
    def test_confident_return_unchanged(self):
        self.assertAlmostEqual(core.effective_range(3.0, 1.0), 3.0)

    def test_weak_return_shrinks_distance(self):
        eff = core.effective_range(3.0, 0.12)
        self.assertLess(eff, 3.0)
        self.assertGreater(eff, 1.0)

    def test_range_never_negative(self):
        self.assertGreaterEqual(core.effective_range(0.2, 0.05), 0.05)


class TestBeams(unittest.TestCase):
    def test_build_beams_skips_invalid(self):
        ranges = [float("inf"), 2.0, 0.0]
        beams = core.build_beams(ranges, -0.1, 0.1, [0.0, 1400.0, 1400.0])
        self.assertEqual(len(beams), 1)
        self.assertAlmostEqual(beams[0]["angle_deg"], 0.0, places=6)

    def test_intensity_drives_confidence(self):
        beams = core.build_beams([2.0, 2.0], 0.0, 0.1, [1400.0, 100.0])
        self.assertGreater(beams[0]["confidence"], beams[1]["confidence"])


class TestChooseHeading(unittest.TestCase):
    def _corridor(self, glass_left=True):
        beams = []
        for a in range(-60, 61, 2):
            if -60 <= a < -10:
                beams.append(make_beam(a, 0.8, 100.0 if glass_left else 1400.0))
            elif -10 <= a <= 10:
                beams.append(make_beam(a, 6.0, 1400.0))
            else:
                beams.append(make_beam(a, 0.8, 1400.0))
        return beams

    def test_open_front_goes_straight(self):
        beams = [make_beam(a, 5.0, 1400.0) for a in range(-60, 61, 2)]
        cmd = core.choose_heading(beams, max_speed=0.3)
        self.assertEqual(cmd["action"], "go")
        self.assertAlmostEqual(cmd["linear"], 0.3, places=3)

    def test_weak_left_side_steers_right(self):
        beams = self._corridor(glass_left=True)
        strong = core.choose_heading(self._corridor(glass_left=False), max_speed=0.3)
        weak = core.choose_heading(beams, max_speed=0.3)
        self.assertGreater(weak["heading_deg"], strong["heading_deg"])

    def test_blocked_front_stops(self):
        beams = [make_beam(a, 0.2, 1400.0) for a in range(-60, 61, 2)]
        cmd = core.choose_heading(beams, stop_distance=0.45)
        self.assertEqual(cmd["action"], "stop")
        self.assertEqual(cmd["linear"], 0.0)

    def test_heading_within_fov(self):
        beams = self._corridor()
        cmd = core.choose_heading(beams, fov_deg=120.0)
        self.assertLessEqual(abs(cmd["heading_deg"]), 60.0 + 1e-9)


if __name__ == "__main__":
    unittest.main()
