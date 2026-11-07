"""传感器噪声与丢束测试（逻辑层，不需要真的跑 MuJoCo 场景）。"""
import unittest
from types import SimpleNamespace

import numpy as np


class FakeEnv:
    """只带 cfg 的最小环境对象，供 _apply_sensor_noise 使用。"""

    def __init__(self, sigma=0.0, dropout=0.0, seed=2026, range_m=6.0):
        self.cfg = SimpleNamespace(lidar_noise_sigma_m=sigma,
                                   lidar_dropout_prob=dropout,
                                   sensor_seed=seed,
                                   lidar_range_m=range_m)


class SensorNoiseTests(unittest.TestCase):
    def _noise(self):
        try:
            from mujoco_car_nn_avoidance.sensors import _apply_sensor_noise
        except ImportError:  # 没装 mujoco 时跳过，保持 --logic-only 可用
            self.skipTest("需要 mujoco")
        return _apply_sensor_noise

    def test_disabled_noise_returns_input_unchanged(self):
        apply_noise = self._noise()
        ranges = np.array([1.0, 2.0, 6.0], dtype=np.float32)
        out = apply_noise(FakeEnv(), ranges)
        np.testing.assert_allclose(out, ranges)

    def test_full_dropout_saturates_to_range(self):
        apply_noise = self._noise()
        ranges = np.array([1.0, 2.0, 3.0, 4.0], dtype=np.float32)
        out = apply_noise(FakeEnv(dropout=1.0), ranges)
        np.testing.assert_allclose(out, 6.0)

    def test_gaussian_noise_stays_in_range_and_changes_values(self):
        apply_noise = self._noise()
        ranges = np.full(25, 3.0, dtype=np.float32)
        out = apply_noise(FakeEnv(sigma=0.10), ranges)
        self.assertTrue(np.all(out >= 0.0) and np.all(out <= 6.0))
        self.assertGreater(float(np.std(out)), 0.0)

    def test_same_seed_is_reproducible(self):
        apply_noise = self._noise()
        ranges = np.full(25, 3.0, dtype=np.float32)
        first = apply_noise(FakeEnv(sigma=0.05, dropout=0.1), ranges)
        second = apply_noise(FakeEnv(sigma=0.05, dropout=0.1), ranges)
        np.testing.assert_allclose(first, second)

    def test_dropped_beams_stay_exactly_at_range_when_noise_enabled(self):
        """丢束语义：即使同时开噪声，丢束值也必须精确等于量程（评审指出过）"""
        apply_noise = self._noise()
        ranges = np.full(25, 3.0, dtype=np.float32)
        out = apply_noise(FakeEnv(sigma=0.20, dropout=1.0), ranges)
        np.testing.assert_array_equal(out, np.float32(6.0))

    def test_mixed_dropout_keeps_dropped_beams_saturated(self):
        apply_noise = self._noise()
        ranges = np.full(200, 2.0, dtype=np.float32)
        out = apply_noise(FakeEnv(sigma=0.20, dropout=0.5), ranges)
        at_limit = int(np.sum(out == np.float32(6.0)))
        self.assertGreater(at_limit, 0, "应至少有一束被置为量程")
        self.assertLess(at_limit, out.shape[0], "不应全部丢束")
        # 有效回波带噪声，与量程区分开；全部仍落在 [0, 量程]
        self.assertTrue(np.all(out <= 6.0) and np.all(out >= 0.0))
        self.assertGreater(float(np.std(out[out < np.float32(6.0)])), 0.0)

    def test_env_reset_restarts_noise_sequence(self):
        apply_noise = self._noise()
        env = FakeEnv(sigma=0.05)
        ranges = np.full(25, 3.0, dtype=np.float32)
        first = apply_noise(env, ranges)
        apply_noise(env, ranges)          # 同一 env 继续推进随机序列
        env._sensor_rng = None            # 模拟 CarEnv.reset() 的重置
        again = apply_noise(env, ranges)
        np.testing.assert_allclose(first, again)


if __name__ == "__main__":
    unittest.main()
