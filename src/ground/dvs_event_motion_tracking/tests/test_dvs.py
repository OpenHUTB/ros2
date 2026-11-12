# -*- coding: utf-8 -*-
"""Collectable regression tests for the DVS event-camera module.

``python -O main.py --self-test`` no longer hides these checks: they are
``unittest.TestCase`` methods in a pytest-collectable ``tests/`` directory,
so ``python -m unittest discover -s tests`` and ``pytest tests`` both run
them.  ``main.py --self-test`` is only a thin wrapper that prints the same
result for interactive use.
"""

import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
MODULE_DIR = os.path.dirname(HERE)
if MODULE_DIR not in sys.path:
    sys.path.insert(0, MODULE_DIR)

import dvs_core  # noqa: E402


class DvsPackedRecordTest(unittest.TestCase):
    def test_stride_is_13_bytes(self):
        self.assertEqual(dvs_core.DVS_STRUCT.size, 13)

    def test_unpack_two_records(self):
        raw = (dvs_core.DVS_STRUCT.pack(12, 34, 123456789, True)
               + dvs_core.DVS_STRUCT.pack(56, 78, 123456790, False))
        self.assertEqual(
            dvs_core.unpack_raw(raw),
            [(12, 34, 123456789, True), (56, 78, 123456790, False)])

    def test_rejects_truncated_stream(self):
        with self.assertRaises(ValueError):
            dvs_core.unpack_raw(b"\x01\x02\x03")


class DvsBaselineTest(unittest.TestCase):
    def test_empty_static_frames_fail_loudly(self):
        with self.assertRaisesRegex(ValueError, "no static frames"):
            dvs_core.build_baseline([], dict(dvs_core.DEFAULTS))

    def test_single_static_frame_fails_loudly(self):
        meta, frames = dvs_core.synthetic_dataset(n_static=1, n_moving=2)
        static = [f for f in frames if f["phase"] == "static"]
        with self.assertRaisesRegex(ValueError, ">= 2 static frames"):
            dvs_core.build_baseline(static, dict(dvs_core.DEFAULTS))

    def test_two_static_frames_are_sufficient(self):
        meta, frames = dvs_core.synthetic_dataset(n_static=2, n_moving=2)
        static = [f for f in frames if f["phase"] == "static"]
        mean, std = dvs_core.build_baseline(static, dict(dvs_core.DEFAULTS))
        self.assertTrue(mean)
        self.assertTrue(all(v >= 0.0 for v in std.values()))


class DvsVelocityWindowTest(unittest.TestCase):
    def test_missed_detection_keeps_est_and_gt_windows_identical(self):
        """Dropping a detected frame must not change the fit's time span."""
        meta, frames = dvs_core.synthetic_dataset()
        cfg = dict(dvs_core.DEFAULTS)
        cfg["baseline_frames"] = 12
        records, full, _ = dvs_core.run_pipeline(meta, frames, cfg)
        self.assertGreater(full["detection_rate"], 0.9)

        # Simulate one missed detection *inside* the tail velocity window.
        dropped = [dict(r) for r in records]
        idx = len(dropped) - 3
        dropped[idx]["detected"] = False
        dropped[idx]["u"] = None
        est = dvs_core._velocity_u_series(dropped)
        self.assertIsNotNone(est[idx], "missed frame must be interpolated")
        self.assertEqual(len(est), len(dropped))
        # gt series and est series now share exactly the same usable indices.
        gt_usable = [i for i, r in enumerate(dropped) if r["gt_u"] is not None]
        est_usable = [i for i, u in enumerate(est) if u is not None]
        self.assertEqual(est_usable, gt_usable)

    def test_linear_slope_of_known_series(self):
        pts = [(0.0, 10.0), (1.0, 12.0), (2.0, 14.0)]
        self.assertAlmostEqual(dvs_core.linear_slope(pts), 2.0, places=9)


class DvsRegressionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.records, cls.stats = dvs_core.regression_stats()

    def test_no_false_alarms(self):
        self.assertEqual(self.stats["false_alarms"], 0)

    def test_moving_blob_is_tracked(self):
        self.assertGreaterEqual(self.stats["detection_rate"], 0.90)
        self.assertLessEqual(
            sum(1 for r in self.records if not r["detected"]), 4)

    def test_centroid_error_is_bounded(self):
        self.assertIsNotNone(self.stats["centroid_mae_px"])
        self.assertLess(self.stats["centroid_mae_px"], 8.0)

    def test_background_is_suppressed(self):
        self.assertGreater(self.stats["background_suppression"], 0.50)

    def test_velocity_direction(self):
        self.assertGreater(self.stats["velocity_est_px_s"], 0.0)
        self.assertGreater(self.stats["velocity_gt_px_s"], 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
