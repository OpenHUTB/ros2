#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DAgger 数据聚合的单元测试：不需要 MuJoCo、PyTorch 或 ROS。

A. DaggerBuffer 空状态
B. 单轮 / 多轮累积与拼接
C. 形状与类型校验、非法输入
D. summary 报告结构

Run with: python3 test/test_dagger.py
"""
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from mjc_rov_current_teleop.dagger import DaggerBuffer  # noqa: E402


class EmptyBufferTests(unittest.TestCase):
    def test_empty_length_and_shapes(self):
        buffer = DaggerBuffer()
        self.assertEqual(len(buffer), 0)
        x, y = buffer.arrays()
        self.assertEqual(x.shape, (0, 6))
        self.assertEqual(y.shape, (0, 2))
        self.assertEqual(x.dtype, np.float32)

    def test_empty_summary(self):
        self.assertEqual(DaggerBuffer().summary(), {'rounds': [], 'total': 0})


class AccumulationTests(unittest.TestCase):
    def test_single_round(self):
        buffer = DaggerBuffer()
        added = buffer.add(np.ones((10, 6)), np.ones((10, 2)))
        self.assertEqual(added, 10)
        self.assertEqual(len(buffer), 10)
        self.assertEqual(buffer.rounds, [10])

    def test_multiple_rounds_accumulate(self):
        buffer = DaggerBuffer()
        buffer.add(np.zeros((5, 6)), np.zeros((5, 2)))
        buffer.add(np.ones((7, 6)), np.ones((7, 2)))
        self.assertEqual(len(buffer), 12)
        self.assertEqual(buffer.rounds, [5, 7])
        x, y = buffer.arrays()
        self.assertEqual(x.shape, (12, 6))
        # 前 5 条来自第一轮（全 0），后 7 条来自第二轮（全 1）
        np.testing.assert_allclose(x[:5], 0.0)
        np.testing.assert_allclose(x[5:], 1.0)

    def test_arrays_are_float32(self):
        buffer = DaggerBuffer()
        buffer.add(np.arange(12).reshape(2, 6), np.arange(4).reshape(2, 2))
        x, y = buffer.arrays()
        self.assertEqual(x.dtype, np.float32)
        self.assertEqual(y.dtype, np.float32)

    def test_summary_tracks_rounds(self):
        buffer = DaggerBuffer()
        buffer.add(np.zeros((3, 6)), np.zeros((3, 2)))
        buffer.add(np.zeros((4, 6)), np.zeros((4, 2)))
        self.assertEqual(buffer.summary(), {'rounds': [3, 4], 'total': 7})

    def test_single_sample_is_promoted_to_one_row(self):
        buffer = DaggerBuffer()
        buffer.add(np.zeros(6), np.zeros(2))
        self.assertEqual(len(buffer), 1)
        x, y = buffer.arrays()
        self.assertEqual(x.shape, (1, 6))
        self.assertEqual(y.shape, (1, 2))


class ValidationTests(unittest.TestCase):
    def test_mismatched_lengths_rejected(self):
        buffer = DaggerBuffer()
        with self.assertRaises(ValueError):
            buffer.add(np.zeros((4, 6)), np.zeros((3, 2)))
        self.assertEqual(len(buffer), 0)

    def test_empty_round_rejected(self):
        buffer = DaggerBuffer()
        with self.assertRaises(ValueError):
            buffer.add(np.zeros((0, 6)), np.zeros((0, 2)))
        self.assertEqual(len(buffer), 0)

    def test_wrong_width_rejected(self):
        buffer = DaggerBuffer()
        with self.assertRaises(ValueError):
            buffer.add(np.zeros((4, 5)), np.zeros((4, 2)))


if __name__ == '__main__':
    unittest.main(verbosity=2)
