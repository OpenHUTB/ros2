"""QoS 模型的单元测试（unittest，无第三方依赖）。"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

import qos_model as qm


class TestQoS(unittest.TestCase):
    PUB = 100.0
    PROC = 0.030

    def _sim(self, reliability, depth):
        return qm.simulate(qm.QoSProfile(reliability, depth), self.PUB, self.PROC)

    def test_reliable_never_drops(self):
        st = self._sim(qm.RELIABLE, 1)
        self.assertEqual(st.drops_cum[-1], 0)
        self.assertAlmostEqual(st.drop_rate, 0.0)

    def test_best_effort_drops_when_overloaded(self):
        st = self._sim(qm.BEST_EFFORT, 1)
        self.assertGreater(st.drop_rate, 0.5)

    def test_deeper_history_reduces_drops(self):
        d1 = self._sim(qm.BEST_EFFORT, 1).drop_rate
        d10 = self._sim(qm.BEST_EFFORT, 10).drop_rate
        d50 = self._sim(qm.BEST_EFFORT, 50).drop_rate
        self.assertLess(d1, 1.0)
        self.assertLess(d10, d1)       # 深度加大 -> 丢包减少
        self.assertLess(d50, d10)

    def test_deeper_history_increases_latency(self):
        # 关键教学点：深缓存用延迟换丢包
        lat1 = self._sim(qm.BEST_EFFORT, 1).mean_latency
        lat10 = self._sim(qm.BEST_EFFORT, 10).mean_latency
        self.assertGreater(lat10, lat1 * 2)

    def test_reliable_backpressure_builds_backlog(self):
        st = self._sim(qm.RELIABLE, 1)
        self.assertGreater(st.backlog[-1], 30)      # 积压远超缓存深度
        self.assertLessEqual(max(st.backlog), st.published)

    def test_throughput_limited_by_callback(self):
        # 三种配置的交付条数都约等于 处理能力 * 时长
        cap = int(1.0 / self.PROC)
        for rel in (qm.BEST_EFFORT, qm.RELIABLE):
            st = self._sim(rel, 5)
            self.assertLessEqual(st.delivered, cap + 1)
            self.assertGreaterEqual(st.delivered, cap - 3)

    def test_durability_latch(self):
        r = qm.latch_demo(pub_rate=10.0, join_time=0.5)
        self.assertEqual(r["history_sent"], 5)
        self.assertEqual(r["volatile_got"], 0)
        self.assertEqual(r["transient_got"], 1)

    # ---------- keep_last 淘汰方向：保留最新、逐出最旧 ----------
    def test_keep_last_keeps_newest_sample(self):
        """评审复现场景：depth=1、100 Hz 发布、30 ms 回调。

        keep_last 的语义是\"保留最近 N 个\"——缓存满时逐出最旧、接纳最新。
        因此每次回调拿到的都应是当时最新发布的样本：发布周期 10 ms、
        回调周期 30 ms，送达序号应为 [0, 3, 6, 9, ...]，每条延迟都
        约 30 ms（回调耗时本身），而不是旧样本的 50/70/90 ms。
        """
        st = self._sim(qm.BEST_EFFORT, 1)
        expect = list(range(0, 4 * int(1.0 / 0.030), 3))[:len(st.delivered_seqs)]
        self.assertEqual(st.delivered_seqs, expect)

    def test_keep_last_latency_bounded_by_callback(self):
        """保留最新样本时，端到端延迟上界 = 回调耗时 + 少量步长。

        旧实现（丢新保旧）会让第 2 条送达样本的延迟达到
        (0.03+0.03)-0.01 = 0.05 s 并持续增长；正确实现稳定在 0.03 s。
        """
        st = self._sim(qm.BEST_EFFORT, 1)
        for lat in st.latencies:
            self.assertLessEqual(lat, self.PROC + 0.002)
        self.assertGreaterEqual(st.mean_latency, self.PROC - 0.002)

    def test_drop_rate_capacity_limited_for_shallow_depth(self):
        """正确语义下，浅缓存的丢包率由处理能力决定：约 1 - 33/100 = 0.67。

        （深度只能吸收突发，降低不了稳态丢包；深缓存的收益是**不丢**，
        代价是延迟——见 test_deeper_history_increases_latency。）
        """
        st = self._sim(qm.BEST_EFFORT, 1)
        self.assertAlmostEqual(st.drop_rate, 0.66, delta=0.03)
        self.assertAlmostEqual(st.delivered, 34, delta=1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
