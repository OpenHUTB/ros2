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
