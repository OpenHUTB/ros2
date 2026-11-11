"""模型测试：MJCF 结构、执行器、限位、重力矩、激活动力学。"""

from __future__ import annotations

import math

import mujoco
import numpy as np
import pytest

from mujoco_musculoskeletal_knee_tracking.config import load_config
from mujoco_musculoskeletal_knee_tracking.model import (
    ActivationDynamics,
    KneeSim,
    PDController,
)


@pytest.fixture(scope="module")
def cfg():
    return load_config()


@pytest.fixture(scope="module")
def sim(cfg):
    return KneeSim(cfg)


def test_two_actuators_and_one_joint(sim):
    assert sim.model.nu == 2
    assert sim.model.nq == 1
    assert sim.model.actuator("flexor").id == 0
    assert sim.model.actuator("extensor").id == 1


def test_joint_range(sim):
    lower, upper = sim.model.jnt_range[sim.joint.id]
    assert math.degrees(lower) == pytest.approx(-15.0, abs=1e-3)
    assert math.degrees(upper) == pytest.approx(90.0, abs=1e-3)


def test_gravity_torque_matches_parameter(cfg, sim):
    sim.reset(50.0)
    sim.data.qvel[:] = 0.0
    mujoco.mj_forward(sim.model, sim.data)
    tau = abs(float(sim.data.qfrc_bias[sim.dof_idx]))
    # 文档口径：稳态误差 6.62° × kp 30 = 3.47 N·m = mgl·sin(50°)
    assert tau == pytest.approx(5.05 * math.sin(math.radians(50.0)), abs=0.15)


def test_timestep_from_config(cfg, sim):
    assert sim.model.opt.timestep == pytest.approx(cfg.sim.dt)


def test_pd_torque_sign(sim):
    ctrl = sim.controller
    tau = ctrl.torque(0.0, 0.0, math.radians(30.0))
    assert tau > 0  # 目标在当前位置屈曲方向 -> 正力矩
    tau_back = ctrl.torque(math.radians(30.0), 0.0, 0.0)
    assert tau_back < 0


def test_activation_mapping_saturates(cfg):
    ctrl = PDController(cfg)
    u_flex, u_ext, tau = ctrl.activations(0.0, 0.0, math.radians(90.0))
    assert tau > cfg.ctrl.max_torque
    assert u_flex == pytest.approx(1.0)
    assert u_ext == 0.0


def test_activation_dynamics_time_constants():
    act = ActivationDynamics(0.02, 0.05)
    a, _ = act.update(1.0, 0.0, 0.01)  # 上升：Δ = (1-0)*0.01/0.02 = 0.5
    assert a == pytest.approx(0.5, abs=1e-9)
    act.flex, act.ext = 1.0, 1.0
    a2, _ = act.update(0.0, 0.0, 0.01)  # 下降：Δ = -1*0.01/0.05 = -0.2
    assert a2 == pytest.approx(0.8, abs=1e-9)


def test_activation_clipped():
    act = ActivationDynamics(0.01, 0.01)
    a, _ = act.update(1.0, 0.0, 1.0)
    assert a == pytest.approx(1.0)


def test_state_torque_from_activations(sim):
    sim.reset(30.0)
    sim.activation.flex, sim.activation.ext = 0.5, 0.1
    st = sim.state()
    assert st.tau_muscle == pytest.approx(17.5 * 0.4, rel=1e-9)


def test_step_is_finite(sim):
    sim.reset(30.0)
    for _ in range(200):
        sim.step(math.radians(30.0))
    st = sim.state()
    assert np.isfinite(st.theta_rad) and np.isfinite(st.omega_rad_s)
