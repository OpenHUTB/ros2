#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""开集声呐分类的单元测试：不需要 PyTorch、MuJoCo 或 ROS。

A. 数据集：形状、均衡性、随机种子可复现、目标/背景可区分
B. 杂波：叠加后能量上升
C. 拒识：阈值行为与边界
D. 指标：覆盖率、被接受准确率、背景误报率的精确值

Run with: python3 test/test_open_set.py
"""
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from mjc_rov_sonar_cnn import open_set  # noqa: E402


class DatasetTests(unittest.TestCase):
    def test_shapes_and_range(self):
        images, labels = open_set.build_dataset(40, seed=7)
        self.assertEqual(images.shape, (40, 64, 64))
        self.assertEqual(images.dtype, np.uint8)
        self.assertEqual(labels.shape, (40,))
        self.assertTrue(set(np.unique(labels)).issubset({0, 1, 2, 3}))

    def test_classes_are_balanced(self):
        _, labels = open_set.build_dataset(40, seed=7)
        counts = np.bincount(labels, minlength=open_set.NUM_CLASSES)
        np.testing.assert_array_equal(counts, np.full(open_set.NUM_CLASSES, 10))

    def test_same_seed_reproduces(self):
        a, la = open_set.build_dataset(24, seed=3)
        b, lb = open_set.build_dataset(24, seed=3)
        np.testing.assert_array_equal(a, b)
        np.testing.assert_array_equal(la, lb)

    def test_different_seed_differs(self):
        a, _ = open_set.build_dataset(24, seed=3)
        b, _ = open_set.build_dataset(24, seed=4)
        self.assertFalse(np.array_equal(a, b))

    def test_target_images_brighter_than_background(self):
        rng_target = np.random.RandomState(11)
        rng_background = np.random.RandomState(11)
        target = open_set.make_sample(rng_target, 0)
        background = open_set.make_sample(rng_background, open_set.BACKGROUND)
        self.assertGreater(int(target.max()), int(background.max()))

    def test_positions_are_not_fixed(self):
        rng = np.random.RandomState(5)
        first = open_set.make_sample(rng, 1)
        second = open_set.make_sample(rng, 1)
        self.assertFalse(np.array_equal(first, second))

    def test_invalid_arguments_rejected(self):
        with self.assertRaises(ValueError):
            open_set.build_dataset(0, seed=1)
        with self.assertRaises(ValueError):
            open_set.make_sample(np.random.RandomState(1), 4)


class ClutterTests(unittest.TestCase):
    def test_clutter_increases_energy(self):
        rng = np.random.RandomState(9)
        clean = open_set.make_sample(rng, open_set.BACKGROUND, clutter=0)
        rng = np.random.RandomState(9)
        noisy = open_set.make_sample(rng, open_set.BACKGROUND, clutter=3)
        self.assertGreater(int(noisy.sum()), int(clean.sum()))

    def test_zero_clutter_is_noop(self):
        base = np.zeros((8, 8), dtype=np.uint8)
        out = open_set.add_clutter(base, np.random.RandomState(1), 0, size=8)
        np.testing.assert_array_equal(out, base)


class RejectionTests(unittest.TestCase):
    def test_confident_prediction_is_accepted(self):
        name, confidence, index = open_set.reject_from_probs([0.05, 0.9, 0.03, 0.02], 0.5)
        self.assertEqual(name, 'pipe')
        self.assertEqual(index, 1)
        self.assertAlmostEqual(confidence, 0.9)

    def test_low_confidence_is_rejected(self):
        name, confidence, _ = open_set.reject_from_probs([0.3, 0.28, 0.22, 0.2], 0.5)
        self.assertEqual(name, 'unknown')
        self.assertAlmostEqual(confidence, 0.3)

    def test_zero_threshold_never_rejects(self):
        name, _, _ = open_set.reject_from_probs([0.3, 0.28, 0.22, 0.2], 0.0)
        self.assertEqual(name, 'rock')

    def test_wrong_probability_count_rejected(self):
        with self.assertRaises(ValueError):
            open_set.reject_from_probs([0.5, 0.5], 0.5)


class MetricsTests(unittest.TestCase):
    def setUp(self):
        # 4 个样本：3 个分类正确，1 个背景被判成礁石（误报）
        self.probs = np.array([
            [0.90, 0.05, 0.03, 0.02],   # label 0，正确
            [0.10, 0.80, 0.05, 0.05],   # label 1，正确
            [0.20, 0.20, 0.30, 0.30],   # label 3（背景），被判成 2 → 误报
            [0.70, 0.10, 0.10, 0.10],   # label 3（背景），被判成 0 → 误报
        ])
        self.labels = np.array([0, 1, 3, 3])

    def test_metrics_without_rejection(self):
        row = open_set.metrics_from_probs(self.probs, self.labels, threshold=0.0)
        self.assertEqual(row['samples'], 4)
        self.assertEqual(row['accepted'], 4)
        self.assertAlmostEqual(row['coverage'], 1.0)
        self.assertAlmostEqual(row['accuracy'], 0.5)      # 2/4 正确
        self.assertAlmostEqual(row['background_fp_rate'], 1.0)  # 两个背景都误报

    def test_metrics_with_rejection(self):
        # 阈值 0.5 会拒掉第 3、4 个样本（置信度 0.3 / 0.7 → 0.7 通过）
        row = open_set.metrics_from_probs(self.probs, self.labels, threshold=0.75)
        self.assertEqual(row['accepted'], 2)              # 只有前两个 >= 0.75
        self.assertAlmostEqual(row['coverage'], 0.5)
        self.assertAlmostEqual(row['accuracy'], 1.0)      # 被接受的全对
        self.assertAlmostEqual(row['background_fp_rate'], 0.0)

    def test_sweep_coverage_is_monotonic(self):
        rows = open_set.sweep_thresholds(self.probs, self.labels)
        coverage = [row['coverage'] for row in rows]
        self.assertEqual(coverage, sorted(coverage, reverse=True))

    def test_mismatched_lengths_rejected(self):
        with self.assertRaises(ValueError):
            open_set.metrics_from_probs(self.probs, np.array([0, 1]))


if __name__ == '__main__':
    unittest.main(verbosity=2)
