"""传感器噪声与丢束测试（纯逻辑，不依赖 mujoco）。"""
import unittest

import numpy as np

from mujoco_car_nn_avoidance.sensor_noise import apply_sensor_noise


def _noise(ranges, sigma=0.0, dropout=0.0, seed=2026, limit=6.0):
    return apply_sensor_noise(ranges, sigma, dropout, np.random.default_rng(seed), limit)


class SensorNoiseTests(unittest.TestCase):
    def test_disabled_noise_returns_input_unchanged(self):
        ranges = np.array([1.0, 2.0, 6.0], dtype=np.float32)
        np.testing.assert_allclose(_noise(ranges), ranges)

    def test_full_dropout_saturates_to_range(self):
        ranges = np.array([1.0, 2.0, 3.0, 4.0], dtype=np.float32)
        np.testing.assert_allclose(_noise(ranges, dropout=1.0), 6.0)

    def test_gaussian_noise_stays_in_range_and_changes_values(self):
        ranges = np.full(25, 3.0, dtype=np.float32)
        out = _noise(ranges, sigma=0.10)
        self.assertTrue(np.all(out >= 0.0) and np.all(out <= 6.0))
        self.assertGreater(float(np.std(out)), 0.0)

    def test_same_seed_is_reproducible(self):
        ranges = np.full(25, 3.0, dtype=np.float32)
        first = _noise(ranges, sigma=0.05, dropout=0.1)
        second = _noise(ranges, sigma=0.05, dropout=0.1)
        np.testing.assert_allclose(first, second)

    def test_dropped_beams_stay_exactly_at_range_when_noise_enabled(self):
        """丢束语义：即使同时开噪声，丢束值也必须精确等于量程（评审指出过）"""
        ranges = np.full(25, 3.0, dtype=np.float32)
        out = _noise(ranges, sigma=0.20, dropout=1.0)
        np.testing.assert_array_equal(out, np.float32(6.0))

    def test_mixed_dropout_keeps_dropped_beams_saturated(self):
        ranges = np.full(200, 2.0, dtype=np.float32)
        out = _noise(ranges, sigma=0.20, dropout=0.5)
        at_limit = int(np.sum(out == np.float32(6.0)))
        self.assertGreater(at_limit, 0, "应至少有一束被置为量程")
        self.assertLess(at_limit, out.shape[0], "不应全部丢束")
        self.assertTrue(np.all(out <= 6.0) and np.all(out >= 0.0))
        self.assertGreater(float(np.std(out[out < np.float32(6.0)])), 0.0)

    def test_input_array_is_not_modified(self):
        """纯函数不应原地修改调用方的射线数组"""
        ranges = np.full(10, 3.0, dtype=np.float32)
        before = ranges.copy()
        _noise(ranges, sigma=0.2, dropout=0.2)
        np.testing.assert_array_equal(ranges, before)


if __name__ == '__main__':
    unittest.main()
