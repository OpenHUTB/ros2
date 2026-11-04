"""demo 模块逻辑单元测试（不依赖 ROS 与 CARLA）。"""

import csv
import sys
import types
import unittest
from pathlib import Path


def _ensure_ros_stubs():
    """与其它测试一致的幂等桩：保证节点模块可导入。"""
    try:
        import rclpy  # noqa: F401
    except ImportError:
        if "rclpy" not in sys.modules:
            rclpy_mod = types.ModuleType("rclpy")
            node_mod = types.ModuleType("rclpy.node")

            class _Node:
                pass

            node_mod.Node = _Node
            rclpy_mod.node = node_mod
            sys.modules["rclpy"] = rclpy_mod
            sys.modules["rclpy.node"] = node_mod
    msg_mod = sys.modules.setdefault(
        "std_msgs.msg", types.ModuleType("std_msgs.msg"))
    if not hasattr(msg_mod, "Float32"):
        class Float32:
            def __init__(self, data=0.0):
                self.data = data
        msg_mod.Float32 = Float32
    if not hasattr(msg_mod, "String"):
        class String:
            def __init__(self, data=""):
                self.data = data
        msg_mod.String = String
    sys.modules["std_msgs.msg"] = msg_mod
    sys.modules.setdefault("std_msgs", types.ModuleType("std_msgs")).msg = msg_mod


_ensure_ros_stubs()

import tempfile
_TMP = Path(tempfile.mkdtemp(prefix="bsd_test_"))

from carla_bsd_blind_spot import demo  # noqa: E402


class TestOfflineScenario(unittest.TestCase):

    def test_offline_runs_and_exports(self):
        out_dir = _TMP
        self.assertTrue(demo.run_offline(out_dir=out_dir, ticks=400))
        csv_path = out_dir / "bsd_offline_data.csv"
        self.assertTrue(csv_path.exists())
        rows = list(csv.reader(open(csv_path)))
        self.assertEqual(rows[0], ["t", "left_d", "right_d",
                                   "left_level", "right_level", "event"])
        self.assertEqual(len(rows), 401)  # header + 400 拍

        levels = {r[3] for r in rows[1:]}
        # 完整状态链：接近段 approach、进窗 warn、离窗 normal 都应出现
        self.assertIn("approach", levels)
        self.assertIn("warn", levels)
        self.assertIn("normal", levels)
        events = [r[5] for r in rows[1:] if r[5]]
        self.assertTrue(any("left_warn" in e for e in events))
        self.assertTrue(any("left_clear" in e for e in events))
        # 曲线图（matplotlib 可用时）
        if (out_dir / "bsd_offline_curve.png").exists():
            self.assertGreater((out_dir / "bsd_offline_curve.png").stat().st_size,
                               10_000)

    def test_offline_never_right_warn(self):
        # 离线场景只有左侧邻车，右侧必须全程 normal
        out_dir = _TMP
        demo.run_offline(out_dir=out_dir, ticks=200)
        rows = list(csv.reader(open(out_dir / "bsd_offline_data.csv")))
        self.assertTrue(all(r[4] == "normal" for r in rows[1:]))


class TestPlotCurve(unittest.TestCase):

    def test_history_shapes_consistent(self):
        # plot 输入由 run_offline 产出，这里验证字段齐全
        out_dir = _TMP
        demo.run_offline(out_dir=out_dir, ticks=50)
        import csv as _csv
        rows = list(_csv.reader(open(out_dir / "bsd_offline_data.csv")))
        for r in rows[1:]:
            self.assertEqual(len(r), 6)


if __name__ == "__main__":
    unittest.main()
