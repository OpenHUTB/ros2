import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from air_ground_detection_benchmark.evaluator import (
    evaluate_frames,
    iou_xywh,
    match_frame,
)


class DetectionBenchmarkTests(unittest.TestCase):
    def test_iou_for_identical_box_is_one(self):
        self.assertAlmostEqual(iou_xywh([10, 20, 30, 40], [10, 20, 30, 40]), 1.0)

    def test_match_counts_true_positive_and_small_object(self):
        (
            tp,
            fp,
            fn,
            small_total,
            small_hit,
            total,
            localization_error,
            matched_iou,
        ) = match_frame(
            [
                {"bbox": [0, 0, 20, 20], "small": True},
                {"bbox": [50, 50, 40, 40], "small": False},
            ],
            [
                {"bbox": [0, 0, 20, 20], "score": 0.9},
                {"bbox": [200, 200, 10, 10], "score": 0.8},
            ],
        )
        self.assertEqual((tp, fp, fn, small_total, small_hit, total), (1, 1, 1, 1, 1, 2))
        self.assertAlmostEqual(localization_error, 0.0)
        self.assertAlmostEqual(matched_iou, 1.0)

    def test_metrics_are_grouped_by_altitude_and_pitch(self):
        frames = [
            {
                "altitude_m": 30,
                "pitch_deg": -60,
                "ground_truth": [{"bbox": [0, 0, 20, 20], "small": True}],
                "predictions": [{"bbox": [0, 0, 20, 20], "score": 0.9}],
            },
            {
                "altitude_m": 80,
                "pitch_deg": -90,
                "ground_truth": [{"bbox": [0, 0, 12, 12], "small": True}],
                "predictions": [],
            },
        ]
        rows = evaluate_frames(frames)
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]["f1"], 1.0)
        self.assertEqual(rows[0]["localization_error_px"], 0.0)
        self.assertEqual(rows[0]["matched_iou"], 1.0)
        self.assertEqual(rows[1]["recall"], 0.0)
        self.assertEqual(rows[1]["small_object_recall"], 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
