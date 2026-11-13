"""粒子滤波的单元测试。"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

import pf as pfmod


class TestParticleFilter(unittest.TestCase):
    BEACONS = (2.0, 10.0, 18.0)

    def test_beats_odometry(self):
        r = pfmod.simulate()
        self.assertLess(abs(r["err_pf"][-1]), 0.25)
        odom = pfmod.rmse(r["x_odom"], r["x_true"])
        est = pfmod.rmse(r["x_pf"], r["x_true"])
        self.assertLess(est, odom / 3.0)

    def test_odometry_drifts(self):
        r = pfmod.simulate()
        # 里程计误差随时间单调增长（单向巡航 + 系统偏差）
        self.assertGreater(r["err_odom"][-1], 0.5)

    def test_weights_normalized(self):
        f = pfmod.ParticleFilter1D(n_particles=100)
        ranges = [abs(p - b) + 0.05 for p, b in
                  zip([5.0] * 3, self.BEACONS)]
        f.update(ranges, self.BEACONS)
        self.assertAlmostEqual(sum(f.weights), 1.0, places=10)
        self.assertTrue(all(w >= 0.0 for w in f.weights))

    def test_effective_n_range(self):
        f = pfmod.ParticleFilter1D(n_particles=200)
        self.assertAlmostEqual(f.effective_n(), 200.0, delta=1e-6)  # 均匀权重
        f.update([abs(5.0 - b) for b in self.BEACONS], self.BEACONS)
        self.assertLess(f.effective_n(), 200.0)
        self.assertGreaterEqual(f.effective_n(), 1.0)

    def test_resample_keeps_population(self):
        f = pfmod.ParticleFilter1D(n_particles=150)
        f.update([abs(6.0 - b) for b in self.BEACONS], self.BEACONS)
        f.resample()
        self.assertEqual(len(f.particles), 150)
        self.assertTrue(all(abs(w - 1.0 / 150) < 1e-12 for w in f.weights))

    def test_estimate_tracks_single_beacon_geometry(self):
        # 单点收敛性：粒子全部集中在 8.0 附近时，估计应接近 8.0
        f = pfmod.ParticleFilter1D(n_particles=200, x_range=(7.9, 8.1))
        ranges = [abs(8.0 - b) for b in self.BEACONS]
        for _ in range(5):
            f.update(ranges, self.BEACONS)
            est, _ = f.step(0.0, 0.05, ranges, self.BEACONS)
        self.assertAlmostEqual(est, 8.0, delta=0.05)

    def test_normal_pdf(self):
        self.assertAlmostEqual(pfmod.normal_pdf(0.0, 0.0, 1.0),
                               1.0 / math.sqrt(2 * math.pi), places=10)
        self.assertLess(pfmod.normal_pdf(3.0, 0.0, 1.0),
                        pfmod.normal_pdf(1.0, 0.0, 1.0))

    # ---------------- 权重递推（回归：不能被当前帧覆盖）----------------

    def _manual_recursive_weights(self, particles, obs_list, sigma):
        """按 w ∝ old_w × lik 手工递推，作为实现的对拍基准。"""
        w = [1.0 / len(particles)] * len(particles)
        for ranges in obs_list:
            w = [wi * math.prod(pfmod.normal_pdf(r, abs(p - b), sigma)
                                for r, b in zip(ranges, self.BEACONS))
                 for p, wi in zip(particles, w)]
            s = sum(w)
            w = [x / s for x in w]
        return w

    def test_weight_update_is_recursive(self):
        """连续多轮更新时，权重必须递推累积，而不是被当前帧似然覆盖。

        这是针对"未重采样周期丢弃历史观测"这一缺陷的回归测试：取
        sigma_range 很大（似然平缓）以保证不触发重采样，连续做 4 次
        update，结果必须与手工递推解逐元素一致。
        """
        sigma = 8.0
        f = pfmod.ParticleFilter1D(n_particles=60, x_range=(0.0, 20.0),
                                   sigma_move=0.0, sigma_range=sigma,
                                   rng=pfmod.Rng(5))
        init = list(f.particles)        # sigma_move=0 且只调 update，粒子集合不变
        obs = [[abs(9.0 - b) + d for b in self.BEACONS]
               for d in (0.5, -0.5, 0.2, 0.4)]
        for ranges in obs:
            f.update(ranges, self.BEACONS)
        expect = self._manual_recursive_weights(init, obs, sigma)
        for got, exp in zip(f.weights, expect):
            self.assertAlmostEqual(got, exp, places=10)

    def test_neff_decreases_over_updates_without_resampling(self):
        """不重采样时，每多一帧观测都在缩小后验，有效粒子数应单调下降。

        若权重被覆盖，Neff 会在第一帧之后基本不变（只剩最后一帧的信息）。
        """
        sigma = 8.0
        f = pfmod.ParticleFilter1D(n_particles=200, x_range=(0.0, 20.0),
                                   sigma_move=0.0, sigma_range=sigma,
                                   rng=pfmod.Rng(9))
        neffs = []
        for d in (0.5, -0.5, 0.2, 0.4):
            f.update([abs(9.0 - b) + d for b in self.BEACONS], self.BEACONS)
            neffs.append(f.effective_n())
        # 只调 update 不调 step，因此这 4 轮里没有发生重采样
        self.assertGreater(neffs[-1], 1.0)               # 仍有粒子多样性
        for a, b in zip(neffs, neffs[1:]):
            self.assertLess(b, a)                        # 信息累积 -> Neff 单调下降

    def test_update_after_resample_does_not_double_count(self):
        """重采样后权重被重置为均匀，历史证据已体现在粒子复制次数上。

        因此随后的 update 应等价于"仅用当前似然"，不能再乘一遍历史权重。
        """
        sigma = 4.0
        f = pfmod.ParticleFilter1D(n_particles=80, x_range=(0.0, 20.0),
                                   sigma_move=0.0, sigma_range=sigma,
                                   rng=pfmod.Rng(21))
        ranges = [abs(7.0 - b) + 0.3 for b in self.BEACONS]
        f.update(ranges, self.BEACONS)
        f.resample()
        f.update(ranges, self.BEACONS)
        expect = [math.prod(pfmod.normal_pdf(r, abs(p - b), sigma)
                            for r, b in zip(ranges, self.BEACONS))
                  for p in f.particles]
        s = sum(expect)
        expect = [x / s for x in expect]
        for got, exp in zip(f.weights, expect):
            self.assertAlmostEqual(got, exp, places=10)


if __name__ == "__main__":
    unittest.main(verbosity=2)
