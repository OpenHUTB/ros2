"""绘图输出测试（只验证文件生成与基本属性，不比较像素）。"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from src.config import load_config
from src.controller import PDController
from src.metrics import sine_metrics, step_metrics
from src.model import KneeModel
from src.plotting import animate_step, plot_sine, plot_step
from src.simulation import simulate
from src.trajectory import make_time_vector, sine_reference, step_reference

ROOT = Path(__file__).resolve().parents[1]


class TestPlotting(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.cfg = load_config(ROOT / "config" / "params.yaml")
        cls.model = KneeModel(cls.cfg.joint, cls.cfg.muscle)
        cls.controller = PDController(
            cls.cfg.controller,
            cls.model.max_muscle_torque,
            cls.cfg.joint.angle_min_deg,
            cls.cfg.joint.angle_max_deg,
        )
        cls.tmp = tempfile.TemporaryDirectory()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.tmp.cleanup()

    def _sine_result(self):
        cfg = self.cfg.trajectory
        t = make_time_vector(0.5, self.cfg.simulation.dt)
        result = simulate(
            self.model,
            self.controller,
            lambda x: float(
                sine_reference(x, cfg.sine_bias_deg, cfg.sine_amplitude_deg,
                               cfg.sine_frequency_hz)
            ),
            t,
            cfg.sine_bias_deg,
            self.cfg.simulation.dt,
        )
        metrics = sine_metrics(t, result.theta_deg, result.reference_deg)
        return result, metrics

    def _step_result(self):
        cfg = self.cfg.trajectory
        t = make_time_vector(2.5, self.cfg.simulation.dt)
        result = simulate(
            self.model,
            self.controller,
            lambda x: float(
                step_reference(x, cfg.step_start_deg, cfg.step_target_deg,
                               cfg.step_time)
            ),
            t,
            cfg.step_start_deg,
            self.cfg.simulation.dt,
        )
        idx = int(np.searchsorted(t, cfg.step_time - 1e-12))
        metrics = step_metrics(
            t[idx:] - t[idx],
            result.theta_deg[idx:],
            cfg.step_target_deg,
            abs(cfg.step_target_deg - cfg.step_start_deg),
        )
        return result, metrics

    def test_plot_sine_png(self) -> None:
        result, metrics = self._sine_result()
        path = Path(self.tmp.name) / "sine.png"
        plot_sine(result, metrics, path)
        self.assertTrue(path.is_file())
        self.assertGreater(path.stat().st_size, 5000)

    def test_plot_step_png(self) -> None:
        result, metrics = self._step_result()
        path = Path(self.tmp.name) / "step.png"
        plot_step(
            result,
            metrics,
            target_deg=self.cfg.trajectory.step_target_deg,
            step_time=self.cfg.trajectory.step_time,
            path=path,
        )
        self.assertTrue(path.is_file())
        self.assertGreater(path.stat().st_size, 5000)

    def test_animate_step_gif(self) -> None:
        result, _ = self._step_result()
        path = Path(self.tmp.name) / "step.gif"
        animate_step(
            result,
            self.cfg.trajectory.step_time,
            path,
            fps=5,
            dpi=50,
            max_frames=6,
        )
        self.assertTrue(path.is_file())
        self.assertGreater(path.stat().st_size, 1000)
        self.assertLess(path.stat().st_size, 3 * 1024 * 1024)


if __name__ == "__main__":
    unittest.main()
