# -*- coding: utf-8 -*-
"""基准评测的单元测试（用假训练器，几毫秒跑完）。

    cd src/rl_algorithm_compare
    python3 -m unittest discover -s test
"""

from __future__ import annotations

import csv
import os
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from rl_algorithm_compare import benchmark as B  # noqa: E402


def fake_trainer(algo, layers, episodes, eval_interval=20, seed=0, tag=None):
    """固定返回：seed 越大回报越高，方便验证统计。"""
    return {
        "algo": algo,
        "layers": layers,
        "episodes": episodes,
        "seed": seed,
        "curve": [(eval_interval, 10.0 + seed),
                  (2 * eval_interval, 20.0 + seed)],
        "final_return": 20.0 + seed,
    }


class TestParse(unittest.TestCase):
    def test_algo_and_layers(self):
        self.assertEqual(B.parse_config("dqn:16"), ("dqn", "16", "dqn_mlp1"))
        self.assertEqual(B.parse_config("dqn:16,16"), ("dqn", "16,16", "dqn_mlp2"))

    def test_custom_tag(self):
        self.assertEqual(B.parse_config("reinforce:16:r1"), ("reinforce", "16", "r1"))

    def test_unknown_algo(self):
        with self.assertRaises(ValueError):
            B.parse_config("ppo:16")

    def test_bad_layers(self):
        with self.assertRaises(ValueError):
            B.parse_config("dqn:0")

    def test_default_configs(self):
        configs = B.parse_configs("")
        self.assertEqual(len(configs), 3)
        self.assertEqual(configs[0][0], "dqn")

    def test_multiple(self):
        configs = B.parse_configs("dqn:16; reinforce:16,16:r2")
        self.assertEqual(len(configs), 2)
        self.assertEqual(configs[1][2], "r2")


class TestSummarize(unittest.TestCase):
    def test_basic(self):
        st = B.summarize([1.0, 2.0, 3.0])
        self.assertEqual(st["n"], 3)
        self.assertAlmostEqual(st["mean"], 2.0)
        self.assertAlmostEqual(st["std"], 1.0)          # 样本标准差
        self.assertAlmostEqual(st["min"], 1.0)
        self.assertAlmostEqual(st["max"], 3.0)

    def test_single_value(self):
        st = B.summarize([5.0])
        self.assertEqual(st["n"], 1)
        self.assertAlmostEqual(st["mean"], 5.0)
        self.assertAlmostEqual(st["std"], 0.0)

    def test_empty_raises(self):
        with self.assertRaises(ValueError):
            B.summarize([])

    def test_sample_std_is_larger_than_population(self):
        values = [0.0, 0.0, 3.0]
        st = B.summarize(values)
        self.assertGreater(st["std"], 1.0)


class TestAggregateCurves(unittest.TestCase):
    def test_alignment(self):
        curves = [
            [(20, 10.0), (40, 20.0)],
            [(20, 14.0), (40, 24.0)],
        ]
        rows = B.aggregate_curves(curves)
        self.assertEqual([r[0] for r in rows], [20, 40])
        self.assertAlmostEqual(rows[0][1], 12.0)
        self.assertAlmostEqual(rows[1][1], 22.0)
        self.assertAlmostEqual(rows[0][2], 2.828427, places=5)

    def test_missing_points_use_available(self):
        curves = [
            [(20, 10.0), (40, 20.0)],
            [(20, 20.0)],
        ]
        rows = B.aggregate_curves(curves)
        self.assertEqual(len(rows), 2)
        self.assertAlmostEqual(rows[0][1], 15.0)
        self.assertAlmostEqual(rows[1][1], 20.0)      # 只有一个样本
        self.assertAlmostEqual(rows[1][2], 0.0)

    def test_empty(self):
        self.assertEqual(B.aggregate_curves([]), [])


class TestRunBenchmark(unittest.TestCase):
    def test_counts_and_stats(self):
        configs = [("dqn", "16", "a"), ("reinforce", "16,16", "b")]
        results = B.run_benchmark(configs, [0, 1], episodes=40,
                                  trainer=fake_trainer, verbose=False)
        self.assertEqual(len(results), 2)
        first = results[0]
        self.assertEqual(first["tag"], "a")
        self.assertEqual(first["seeds"], [0, 1])
        self.assertAlmostEqual(first["final"]["mean"], 20.5)
        self.assertAlmostEqual(first["final"]["std"], 0.707106, places=5)
        self.assertEqual(len(first["curves"]), 2)

    def test_requires_configs(self):
        with self.assertRaises(ValueError):
            B.run_benchmark([], [0], trainer=fake_trainer, verbose=False)

    def test_requires_seeds(self):
        with self.assertRaises(ValueError):
            B.run_benchmark([("dqn", "16", "a")], [], trainer=fake_trainer,
                            verbose=False)

    def test_seeds_are_passed_through(self):
        seen = []

        def trainer(algo, layers, episodes, eval_interval=20, seed=0, tag=None):
            seen.append(seed)
            return fake_trainer(algo, layers, episodes, eval_interval, seed)

        B.run_benchmark([("dqn", "16", "a")], [3, 7], trainer=trainer, verbose=False)
        self.assertEqual(seen, [3, 7])

    def test_duplicate_seeds_rejected(self):
        """重复 seed 会得到 std=0 的假"完美"结果，必须直接拒绝。"""
        with self.assertRaises(ValueError):
            B.run_benchmark([("dqn", "16", "a")], [0, 0],
                            trainer=fake_trainer, verbose=False)

    def test_tag_is_forwarded(self):
        seen = []

        def trainer(algo, layers, episodes, eval_interval=20, seed=0, tag=None):
            seen.append(tag)
            return fake_trainer(algo, layers, episodes, eval_interval, seed, tag)

        B.run_benchmark([("dqn", "16", "mytag")], [0], trainer=trainer, verbose=False)
        self.assertEqual(seen, ["mytag"])


class TestOutputs(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix="rlbench-")

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def test_summary_csv(self):
        results = B.run_benchmark([("dqn", "16", "a")], [0, 1], episodes=40,
                                  trainer=fake_trainer, verbose=False)
        summary_path, curves_path = B.write_outputs(results, self.dir)
        self.assertTrue(os.path.isfile(summary_path))
        self.assertTrue(os.path.isfile(curves_path))

        with open(summary_path, encoding="utf-8") as fh:
            rows = list(csv.reader(fh))
        self.assertEqual(rows[0][0], "tag")
        self.assertEqual(rows[1][0], "a")
        self.assertEqual(rows[1][4], "20.50")        # mean_return

        with open(curves_path, encoding="utf-8") as fh:
            rows = list(csv.reader(fh))
        self.assertEqual(rows[0][:2], ["tag", "episode"])
        self.assertEqual(len(rows), 3)               # 表头 + 2 个评估点

    def test_format_table(self):
        results = B.run_benchmark([("dqn", "16", "a")], [0], episodes=20,
                                  trainer=fake_trainer, verbose=False)
        table = B.format_table(results)
        self.assertIn("配置", table)
        self.assertIn("a", table)


class TestNetworkSeed(unittest.TestCase):
    """网络初始化必须受 seed 影响——否则多 seed 的方差会系统性偏小。"""

    def test_different_seed_different_weights(self):
        import numpy as np
        from rl_algorithm_compare.nn import MLP
        a = MLP([4, 8, 2], seed=0)
        b = MLP([4, 8, 2], seed=1)
        self.assertFalse(np.allclose(a.W[0], b.W[0]))

    def test_same_seed_same_weights(self):
        import numpy as np
        from rl_algorithm_compare.nn import MLP
        a = MLP([4, 8, 2], seed=7)
        b = MLP([4, 8, 2], seed=7)
        for wa, wb in zip(a.W, b.W):
            self.assertTrue(np.allclose(wa, wb))

    def test_default_seed_keeps_old_behaviour(self):
        import numpy as np
        from rl_algorithm_compare.nn import MLP
        a = MLP([4, 8, 2])
        b = MLP([4, 8, 2], seed=0)
        for wa, wb in zip(a.W, b.W):
            self.assertTrue(np.allclose(wa, wb))


if __name__ == "__main__":
    unittest.main()
