"""demo 模块逻辑单元测试（不依赖 ROS 与 CARLA）。"""

import csv
import sys
import tempfile
import types
import unittest
from pathlib import Path

_TMP = Path(tempfile.mkdtemp(prefix="csl_test_"))


def _ensure_ros_stubs():
    """宿主机无 ROS 时给 rclpy/std_msgs 打桩（缺啥补啥，幂等）。"""
    try:
        import rclpy  # noqa: F401
    except ImportError:
        if "rclpy" not in sys.modules:
            class _Node:
                pass

            rclpy_mod = types.ModuleType("rclpy")
            node_mod = types.ModuleType("rclpy.node")
            node_mod.Node = _Node
            rclpy_mod.node = node_mod

            def _fake(*a, **k):
                raise RuntimeError("ROS 未安装：本测试只验证演示逻辑，不应触发生命周期")

            rclpy_mod.init = _fake
            rclpy_mod.spin = _fake
            sys.modules["rclpy"] = rclpy_mod
            sys.modules["rclpy.node"] = node_mod

    msg_mod = sys.modules.setdefault(
        "std_msgs.msg", types.ModuleType("std_msgs.msg"))
    if not hasattr(msg_mod, "Float32"):
        class Float32:
            def __init__(self, data=0.0):
                self.data = data
        msg_mod.Float32 = Float32
    if not hasattr(msg_mod, "Float32MultiArray"):
        class Float32MultiArray:
            def __init__(self, data=None):
                self.data = data or []
        msg_mod.Float32MultiArray = Float32MultiArray
    if not hasattr(msg_mod, "String"):
        class String:
            def __init__(self, data=""):
                self.data = data
        msg_mod.String = String
    sys.modules["std_msgs.msg"] = msg_mod
    sys.modules.setdefault("std_msgs", types.ModuleType("std_msgs")).msg = msg_mod


_ensure_ros_stubs()

from carla_curve_speed_limit import demo  # noqa: E402


class TestOfflineScenario(unittest.TestCase):

    def test_offline_runs_and_exports(self):
        self.assertTrue(demo.run_offline(out_dir=_TMP, ticks=600))
        csv_path = _TMP / "csl_offline_data.csv"
        self.assertTrue(csv_path.exists())
        rows = list(csv.reader(open(csv_path)))
        self.assertEqual(rows[0], ["t", "v_max", "commanded", "v",
                                   "radius", "state", "event"])
        self.assertEqual(len(rows), 601)   # header + 600 拍

        states = {r[5] for r in rows[1:]}
        # 完整状态链：巡航与减速都应出现
        self.assertIn("cruise", states)
        self.assertIn("slowdown", states)
        events = [r[6] for r in rows[1:] if r[6]]
        self.assertTrue(any("slowdown_warn" in e for e in events))
        self.assertTrue(any("cruise_resume" in e for e in events))
        if (_TMP / "csl_offline_curve.png").exists():
            self.assertGreater((_TMP / "csl_offline_curve.png").stat().st_size,
                               10_000)

    def test_speed_respects_curve_limit(self):
        # 正确不变量：减速段开始时斜坡尚未收敛（v>v_max 合法），
        # 但弯道深处（限速持续低于 6 m/s 的稳定段）速度必须收敛到限速附近
        demo.run_offline(out_dir=_TMP, ticks=600)
        rows = list(csv.reader(open(_TMP / "csl_offline_data.csv")))
        # R=15 的弯道限速为 sqrt(2.5*15)≈6.12 m/s，用 6.5 筛稳定弯道段
        in_curve = [(float(r[1]), float(r[3])) for r in rows[1:]
                    if float(r[1]) < 6.5]
        self.assertGreater(len(in_curve), 20, "未见稳定弯道段")
        min_limit = min(l for l, _ in in_curve)
        min_v = min(v for _, v in in_curve)
        self.assertLessEqual(min_v, min_limit + 0.5,
                             f"弯道中速度未收敛到限速（v={min_v}, limit={min_limit}）")

    def test_straight_segment_returns_to_cruise(self):
        # 末段直道：实际速度应恢复接近巡航 8 m/s
        demo.run_offline(out_dir=_TMP, ticks=600)
        rows = list(csv.reader(open(_TMP / "csl_offline_data.csv")))
        tail = [float(r[3]) for r in rows[-40:]]
        self.assertTrue(all(v > 7.0 for v in tail),
                        f"末段直道速度未恢复巡航: {tail[-5:]}")

    def test_live_args_parsing(self):
        """--town/--spawn-index/--seconds 解析并透传给 run_live。"""
        captured = {}

        def fake(host, port, out_dir, max_seconds=60, town=None,
                 spawn_index=None):
            captured.update(host=host, town=town,
                            spawn_index=spawn_index, seconds=max_seconds)
            return True

        orig = demo.run_live
        demo.run_live = fake
        try:
            rc = demo.main(["--live", "--host", "192.168.122.1",
                            "--town", "Town02_Opt", "--spawn-index", "3",
                            "--seconds", "30"])
        finally:
            demo.run_live = orig
        self.assertEqual(rc, 0)
        self.assertEqual(captured["town"], "Town02_Opt")
        self.assertEqual(captured["spawn_index"], 3)
        self.assertEqual(captured["seconds"], 30)

    def test_live_default_args_unchanged(self):
        """不传 --town/--spawn-index 时保持原默认（None=旧回退逻辑）。"""
        captured = {}

        def fake(host, port, out_dir, max_seconds=60, town=None,
                 spawn_index=None):
            captured.update(town=town, spawn_index=spawn_index)
            return True

        orig = demo.run_live
        demo.run_live = fake
        try:
            demo.main(["--live"])
        finally:
            demo.run_live = orig
        self.assertIsNone(captured["town"])
        self.assertIsNone(captured["spawn_index"])


if __name__ == "__main__":
    unittest.main()
