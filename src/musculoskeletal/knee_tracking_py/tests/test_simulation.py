"""闭环仿真流程测试。"""

from __future__ import annotations

import math
import unittest
from pathlib import Path

import numpy as np

from src.config import load_config
from src.controller import PDController
from src.model import KneeModel
from src.simulation import simulate
from src.trajectory import make_time_vector, sine_reference, step_reference

ROOT = Path(__file__).resolve().parents[1]


def build(cfg):
    model = KneeModel(cfg.joint, cfg.muscle)
    controller = PDController(
        cfg.controller,
        model.max_muscle_torque,
        cfg.joint.angle_min_deg,
        cfg.joint.angle_max_deg,
    )
    return model, controller


class TestSimulation(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = load_config(ROOT / "config" / "params.yaml")
        self.model, self.controller = build(self.cfg)

    def test_sine_run_is_finite_and_complete(self) -> None:
        dt = self.cfg.simulation.dt
        t = make_time_vector(1.0, dt)
        cfg = self.cfg.trajectory
        result = simulate(
            self.model,
            self.controller,
            lambda x: float(
                sine_reference(x, cfg.sine_bias_deg, cfg.sine_amplitude_deg,
                               cfg.sine_frequency_hz)
            ),
            t,
            initial_theta_deg=cfg.sine_bias_deg,
            dt=dt,
        )
        self.assertEqual(result.t.size, t.size)
        self.assertEqual(result.theta_deg.shape, t.shape)
        for values in (
            result.theta_deg,
            result.omega_rad_s,
            result.reference_deg,
            result.flex,
            result.ext,
        ):
            self.assertTrue(np.all(np.isfinite(values)))
        self.assertTrue(np.all(result.flex >= 0.0) and np.all(result.flex <= 1.0))
        self.assertTrue(np.all(result.ext >= 0.0) and np.all(result.ext <= 1.0))
        self.assertEqual(result.invalid_count, 0)

    def test_step_tracks_upward(self) -> None:
        dt = self.cfg.simulation.dt
        t = make_time_vector(1.0, dt)
        cfg = self.cfg.trajectory
        result = simulate(
            self.model,
            self.controller,
            lambda x: float(
                step_reference(x, cfg.step_start_deg, cfg.step_target_deg,
                               cfg.step_time)
            ),
            t,
            initial_theta_deg=cfg.step_start_deg,
            dt=dt,
        )
        self.assertGreater(result.theta_deg[-1], cfg.step_start_deg)
        self.assertEqual(result.invalid_count, 0)
        self.assertLessEqual(float(np.max(result.theta_deg)), 90.0 + 1e-9)

    def test_invalid_reference_relaxes_muscles(self) -> None:
        dt = self.cfg.simulation.dt
        t = make_time_vector(0.5, dt)
        result = simulate(
            self.model,
            self.controller,
            lambda x: 100.0,  # 超出上限，非法
            t,
            initial_theta_deg=0.0,
            dt=dt,
        )
        self.assertGreater(result.invalid_count, 0)
        self.assertTrue(np.all(result.flex == 0.0))
        self.assertTrue(np.all(result.ext == 0.0))
        self.assertTrue(np.all(np.isfinite(result.theta_deg)))
        self.assertTrue(np.all(np.isfinite(result.omega_rad_s)))

    def test_upper_limit_not_exceeded(self) -> None:
        dt = self.cfg.simulation.dt
        t = make_time_vector(1.0, dt)
        result = simulate(
            self.model,
            self.controller,
            lambda x: 90.0,
            t,
            initial_theta_deg=88.0,
            dt=dt,
        )
        self.assertLessEqual(float(np.max(result.theta_deg)), 90.0 + 1e-9)

    def test_invalid_dt_rejected(self) -> None:
        t = make_time_vector(0.1, 0.005)
        with self.assertRaises(ValueError):
            simulate(
                self.model,
                self.controller,
                lambda x: 0.0,
                t,
                initial_theta_deg=0.0,
                dt=0.0,
            )

    def test_nan_reference_rejected(self) -> None:
        t = make_time_vector(0.1, 0.005)
        with self.assertRaises(ValueError):
            simulate(
                self.model,
                self.controller,
                lambda x: math.nan,
                t,
                initial_theta_deg=0.0,
                dt=0.005,
            )


if __name__ == "__main__":
    unittest.main()
