"""离线单元测试：一键演示（纵动力学仿真 + 输出文件）。"""
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from carla_aeb_braking import demo as demo_mod  # noqa: E402


class TestSimulateScenario(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.history, cls.meta = demo_mod.simulate_aeb_scenario()

    def test_brake_triggered(self):
        self.assertIsNotNone(self.meta["brake_time"])
        self.assertIn("brake", [h[7] for h in self.history])

    def test_no_collision(self):
        # AEB 必须在撞上之前刹住：最小间隙应明显大于 0
        self.assertGreater(self.meta["min_gap"], 0.5)

    def test_stops_before_obstacle(self):
        self.assertTrue(self.meta["stopped"])

    def test_full_state_cycle_with_recovery(self):
        # 障碍车较慢（5 m/s）：刹停后间隙拉大，应走完 braking→recover→cruise
        states = {h[1] for h in self.history}
        self.assertIn("braking", states)
        self.assertIn("recover", states)
        self.assertEqual(self.meta["final_state"], "cruise")

    def test_warning_precedes_braking(self):
        events = [(h[0], h[7]) for h in self.history if h[7] is not None]
        self.assertGreaterEqual(len(events), 2)
        self.assertEqual(events[0][1], "warn")
        self.assertEqual(events[1][1], "brake")
        self.assertLess(events[0][0], events[1][0])


class TestRunOfflineOutputs(unittest.TestCase):
    def test_output_files_created(self):
        with tempfile.TemporaryDirectory() as tmp:
            rc = demo_mod.run_offline(out_dir=tmp)
            self.assertEqual(rc, 0)
            csv_path = Path(tmp) / "aeb_offline_data.csv"
            png_path = Path(tmp) / "aeb_offline_curve.png"
            self.assertTrue(csv_path.exists())
            self.assertGreater(csv_path.stat().st_size, 100)
            self.assertTrue(png_path.exists())  # 需要 matplotlib；缺省时绘图降级
            self.assertGreater(png_path.stat().st_size, 1000)


if __name__ == "__main__":
    unittest.main()
