"""demo 模块逻辑单元测试（不依赖 ROS 与 CARLA）。"""

import csv
import sys
import tempfile
import types
import unittest
from pathlib import Path

_TMP = Path(tempfile.mkdtemp(prefix="ldw_test_"))


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
    if not hasattr(msg_mod, "String"):
        class String:
            def __init__(self, data=""):
                self.data = data
        msg_mod.String = String
    sys.modules["std_msgs.msg"] = msg_mod
    sys.modules.setdefault("std_msgs", types.ModuleType("std_msgs")).msg = msg_mod


_ensure_ros_stubs()

try:
    import carla  # noqa: F401
except ImportError:
    sys.modules["carla"] = types.SimpleNamespace(
        LaneType=types.SimpleNamespace(Driving=2))

from carla_lane_departure_warning import demo  # noqa: E402


class TestOfflineScenario(unittest.TestCase):

    def test_offline_runs_and_exports(self):
        self.assertTrue(demo.run_offline(out_dir=_TMP, ticks=400))
        csv_path = _TMP / "ldw_offline_data.csv"
        self.assertTrue(csv_path.exists())
        rows = list(csv.reader(open(csv_path)))
        self.assertEqual(rows[0], ["t", "offset", "heading_err",
                                   "state", "event", "v"])
        self.assertEqual(len(rows), 401)   # header + 400 拍

        states = {r[3] for r in rows[1:]}
        # 完整状态链：三级都应出现
        self.assertIn("normal", states)
        self.assertIn("deviation", states)
        self.assertIn("departure", states)
        events = [r[4] for r in rows[1:] if r[4]]
        # 正弦偏离左右两侧各触发一轮
        self.assertTrue(any("deviation_start_right" in e for e in events))
        self.assertTrue(any("deviation_start_left" in e for e in events))
        self.assertTrue(any("departure_start" in e for e in events))
        self.assertTrue(any("deviation_clear" in e for e in events))
        if (_TMP / "ldw_offline_curve.png").exists():
            self.assertGreater((_TMP / "ldw_offline_curve.png").stat().st_size,
                               10_000)

    def test_symmetric_deviation(self):
        # 正弦偏离左右对称：departure 状态里左右两侧都应出现
        demo.run_offline(out_dir=_TMP, ticks=400)
        rows = list(csv.reader(open(_TMP / "ldw_offline_data.csv")))
        dep_offsets = [float(r[1]) for r in rows[1:] if r[3] == "departure"]
        self.assertTrue(any(o >= 0.9 for o in dep_offsets),
                        "左侧（正偏移）未触发 departure")
        self.assertTrue(any(o <= -0.9 for o in dep_offsets),
                        "右侧（负偏移）未触发 departure")


if __name__ == "__main__":
    unittest.main()
