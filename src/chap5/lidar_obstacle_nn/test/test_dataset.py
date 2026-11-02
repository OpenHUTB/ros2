"""Tests for balanced synthetic data and the shared labelling rule."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from lidar_obstacle_nn.generate_synthetic_dataset import (
    generate_dataset,
    save_dataset,
)
from lidar_obstacle_nn.scan_utils import label_for_min_range


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class TestDataset(unittest.TestCase):
    def test_label_rule_boundaries(self) -> None:
        cases = [
            (0.0, 0),
            (0.999, 0),
            (1.0, 1),
            (2.999, 1),
            (3.0, 2),
            (30.0, 2),
            (float('inf'), 2),
            (float('nan'), 2),
        ]
        for distance, expected in cases:
            with self.subTest(distance=distance):
                self.assertEqual(
                    label_for_min_range(distance, 1.0, 3.0),
                    expected,
                )

    def test_generated_dataset_has_expected_shape_and_balance(self) -> None:
        scans, labels = generate_dataset(
            num_samples=300,
            scan_size=360,
            noise_std=0.0,
            seed=123,
        )

        self.assertEqual(scans.shape, (300, 360))
        self.assertEqual(scans.dtype, np.float32)
        self.assertEqual(labels.shape, (300,))
        self.assertEqual(
            np.bincount(labels, minlength=3).tolist(),
            [100, 100, 100],
        )
        self.assertTrue(np.isfinite(scans).all())
        self.assertGreaterEqual(scans.min(), 0.0)
        self.assertLessEqual(scans.max(), 30.0)

        derived = np.asarray(
            [label_for_min_range(float(row.min()), 1.0, 3.0) for row in scans]
        )
        np.testing.assert_array_equal(derived, labels)

    def test_saved_dataset_contains_required_metadata(self) -> None:
        scans, labels = generate_dataset(
            num_samples=30,
            noise_std=0.01,
            seed=9,
        )
        with tempfile.TemporaryDirectory(dir=PACKAGE_ROOT) as tmp:
            output = Path(tmp) / 'dataset.npz'
            save_dataset(
                output_path=str(output),
                scans=scans,
                labels=labels,
                scan_size=360,
                danger_distance=1.0,
                far_distance=3.0,
            )

            with np.load(output) as data:
                self.assertEqual(data['scans'].shape, (30, 360))
                self.assertEqual(data['labels'].shape, (30,))
                self.assertEqual(int(data['scan_size']), 360)
                self.assertEqual(float(data['danger_distance']), 1.0)
                self.assertEqual(float(data['far_distance']), 3.0)
                self.assertEqual(
                    data['class_names'].tolist(),
                    ['near', 'far', 'free'],
                )

    def test_generator_rejects_invalid_configuration(self) -> None:
        with self.assertRaises(ValueError):
            generate_dataset(num_samples=2)
        with self.assertRaises(ValueError):
            generate_dataset(num_samples=30, scan_size=0)
        with self.assertRaises(ValueError):
            generate_dataset(
                num_samples=30,
                danger_distance=3.0,
                far_distance=1.0,
            )
