"""demo 模块逻辑单元测试（不依赖 ROS 与 CARLA）。"""

import csv
import sys
import tempfile
import types
import unittest
from pathlib import Path

_TMP = Path(tempfile.mkdtemp(prefix="rlw_test_"))


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
    for name, default in (("Float32", 0.0), ("String", "")):
        if not hasattr(msg_mod, name):
            def _mk(default=default):
                class _M:
                    def __init__(self, data=default):
                        self.data = data
                return _M
            setattr(msg_mod, name, _mk())
    sys.modules["std_msgs.msg"] = msg_mod
    sys.modules.setdefault("std_msgs", types.ModuleType("std_msgs")).msg = msg_mod


_ensure_ros_stubs()

try:
    import carla  # noqa: F401
except ImportError:
    sys.modules["carla"] = types.SimpleNamespace(
        LaneType=types.SimpleNamespace(Driving=2))

from carla_red_light_warning import demo  # noqa: E402


class TestOfflineScenario(unittest.TestCase):

    def test_offline_runs_and_exports(self):
        self.assertTrue(demo.run_offline(out_dir=_TMP, ticks=500))
        csv_path = _TMP / "rlw_offline_data.csv"
        self.assertTrue(csv_path.exists())
        rows = list(csv.reader(open(csv_path)))
        self.assertEqual(rows[0], ["t", "dist", "light", "state",
                                   "event", "v"])
        self.assertEqual(len(rows), 501)   # header + 500 拍

        states = {r[3] for r in rows[1:]}
        # 完整状态链：三级都应出现
        self.assertIn("normal", states)
        self.assertIn("approaching", states)
        self.assertIn("violation_risk", states)
        events = [r[4] for r in rows[1:] if r[4]]
        self.assertTrue(any("approach_start" in e for e in events))
        self.assertTrue(any("violation_start" in e for e in events))
        self.assertTrue(any("violation_clear" in e for e in events))
        if (_TMP / "rlw_offline_curve.png").exists():
            self.assertGreater((_TMP / "rlw_offline_curve.png").stat().st_size,
                               10_000)

    def test_multiple_red_cycles(self):
        # 25 s 周期、500 拍 = 25 s？不——500×0.05=25 s 恰一轮；延长拍数验证多轮
        demo.run_offline(out_dir=_TMP, ticks=1000)
        rows = list(csv.reader(open(_TMP / "rlw_offline_data.csv")))
        events = [r[4] for r in rows[1:] if r[4]]
        # 50 s 内至少两轮红灯 → violation_start ≥ 2 次
        self.assertGreaterEqual(
            sum(1 for e in events if e == "violation_start"), 2)

    def test_light_phase_mapping(self):
        # 灯态合成函数：[10,20) 红、其余绿
        self.assertEqual(demo._offline_light(5.0), "green")
        self.assertEqual(demo._offline_light(15.0), "red")
        self.assertEqual(demo._offline_light(25.0), "green")
        self.assertEqual(demo._offline_light(35.0), "red")   # 第二轮


if __name__ == "__main__":
    unittest.main()
