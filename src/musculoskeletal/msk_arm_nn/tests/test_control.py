#!/usr/bin/env python3
"""Control tests for the MSK-Arm muscle controller.

Run: ``/home/lxy/roscx_runtime/venv/bin/python tests/test_control.py``

Acceptance: ``MuscleController`` drives a real ``MskArmSim`` through the
``reach``, ``shelf`` and ``draw`` targets with 0 collisions; metrics printed per
scene: time, final end-effector error, peak joint torque, effort, re-plans.
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import numpy as np

from msk_arm_nn import config as C
from msk_arm_nn import control
from msk_arm_nn.control import (MissionController, MuscleController, PID,
                                SafetyFilter)
from msk_arm_nn.scenes import get_scene
from msk_arm_nn.sim import MskArmSim


def test_dynamics_model() -> None:
    sim = MskArmSim("reach", seed=1, noise=False)
    q = np.array([0.30, 0.90])
    dq = np.array([0.40, -0.30])
    sim.q = q.copy()
    sim.dq = dq.copy()
    assert np.allclose(control.inertia_matrix(q), sim.inertia_matrix())
    assert np.allclose(control.coriolis(q, dq), sim.coriolis())
    assert np.allclose(control.gravity_torque(q), sim.gravity_torque())
    assert np.allclose(control.activation_map(q), sim.activation_map())
    assert np.allclose(control.forward_kinematics(q), sim.ee())
    ik = control.inverse_kinematics(*sim.ee())
    assert ik is not None and np.allclose(ik, q, atol=1e-6), \
        "IK/FK round-trip failed"
    assert MissionController is MuscleController
    print("[model] M(q), C(q,dq), G(q), B(q), FK/IK match the simulator OK")


def test_torque_mapping() -> None:
    sim = MskArmSim("reach", seed=2, noise=False)
    sim.q = np.array([0.20, 0.80])
    b_mat = sim.activation_map()
    tau = np.array([1.20, -0.35])
    activation = control.torque_to_activation(tau, b_mat, co_activation=0.02)
    assert activation.shape == (4,)
    assert float(activation.min()) >= 0.0 and float(activation.max()) <= 1.0
    achieved = b_mat @ activation
    # the balanced co-contraction cancels, so the torque is reproduced exactly
    assert np.allclose(achieved, tau, atol=1e-9), \
        "torque mapping error %s" % np.round(achieved - tau, 6)
    saturated = control.torque_to_activation(np.array([50.0, 50.0]), b_mat)
    assert np.allclose(saturated[[0, 2]], 1.0), "flexors should saturate at 1"
    print("[torque] tau -> activation exact (%.2f, %.2f) N*m, saturation OK"
          % (tau[0], tau[1]))


def test_gravity_hold() -> None:
    sim = MskArmSim("reach", seed=3, noise=False)
    q0 = sim.q.copy()
    ee0 = sim.ee().copy()
    kp, kd = 60.0, 14.0
    for _ in range(int(1.5 / 0.02)):
        q = sim.q.copy()
        dq = sim.dq.copy()
        b_mat = sim.activation_map()
        tau = control.inertia_matrix(q) @ (kp * (q0 - q) - kd * dq) \
            + control.gravity_torque(q) + control.damping_torque(dq)
        for j in (0, 1):
            limit = 0.95 * max(abs(b_mat[j, 2 * j]), abs(b_mat[j, 2 * j + 1]))
            tau[j] = float(np.clip(tau[j], -limit, limit))
        sim.step(control.torque_to_activation(tau, b_mat), 0.02)
    q_err = float(np.max(np.abs(sim.q - q0)))
    ee_err = float(np.linalg.norm(sim.ee() - ee0))
    print("[gravity] PD + gravity hold 1.5 s: joint drift %.4f rad, "
          "EE drift %.4f m, collisions %d" % (q_err, ee_err, sim.collisions))
    assert q_err < 0.06, "gravity compensation drifted %.3f rad" % q_err
    assert ee_err < 0.02, "end effector drifted %.3f m" % ee_err
    assert sim.collisions == 0


def test_pid_shim() -> None:
    pid = PID(kp=2.0, ki=1.0, kd=0.0, limit=3.0, i_limit=10.0)
    assert abs(pid.update(1.0, 0.1) - 2.1) < 1e-9
    for _ in range(100):
        out = pid.update(1.0, 0.1)
    assert abs(out - 3.0) < 1e-9
    safety = SafetyFilter()
    cmd = safety.apply([1.2, -0.3, 0.5, 0.9], None, None)
    assert np.allclose(cmd, [1.0, 0.0, 0.5, 0.9])
    print("[pid] PID / SafetyFilter template shims OK")


def run_mission(name: str, max_time: float = 90.0) -> dict:
    """Run one mission scene; helper (not a pytest case -- no ``test_`` prefix)."""
    scene = get_scene(name)
    sim = MskArmSim(name, seed=C.DEFAULT_SEED, noise=True)
    controller = MuscleController(scene)
    controller.reset(sim.pose())
    dt = 0.02
    t = 0.0
    ee_prev = sim.ee().copy()
    ee_path = 0.0
    success_at = None
    while t < max_time:
        command = controller.update(sim.pose(), sim.goal(), None, None, dt)
        assert command.shape == (4,)
        assert float(command.min()) >= 0.0 and float(command.max()) <= 1.0
        sim.step(command, dt)
        t += dt
        ee = sim.ee()
        ee_path += float(np.linalg.norm(ee - ee_prev))
        ee_prev = ee.copy()
        if sim.success:
            if success_at is None:
                success_at = t
            elif t - success_at >= 0.2:
                break
    final_err = float(np.linalg.norm(sim.ee() - np.array(scene.targets[-1])))
    result = dict(scene=name, success=sim.success, time=t,
                  reached=sim.reached, final=final_err,
                  ee_path=ee_path, collisions=sim.collisions,
                  max_torque=sim.max_torque, effort=sim.effort,
                  replans=controller.replans,
                  ctrl_reached=controller.reached,
                  ctrl_success=controller.success)
    print("[mission] %-6s success=%-5s t=%6.2f s  targets=%d/%d  "
          "final=%.4f m  ee_path=%.3f m  peak_tau=%.2f N*m  effort=%.2f  "
          "replans=%d  collisions=%d"
          % (name, sim.success, t, sim.reached, len(scene.targets),
             final_err, ee_path, sim.max_torque, sim.effort,
             controller.replans, sim.collisions))
    assert sim.collisions == 0, "%s: arm collided %d times" % (
        name, sim.collisions)
    assert sim.success, "%s: mission did not finish (reached %d/%d)" % (
        name, sim.reached, len(scene.targets))
    assert sim.reached == len(scene.targets), \
        "%s: reached %d/%d targets" % (name, sim.reached, len(scene.targets))
    assert final_err <= C.TARGET_TOLERANCE + 0.02, \
        "%s: final EE error %.4f m too large" % (name, final_err)
    assert controller.reached >= 1, "controller telemetry not updating"
    return result


def main() -> int:
    print("== test_control ==")
    test_dynamics_model()
    test_torque_mapping()
    test_gravity_hold()
    test_pid_shim()
    results = [run_mission(name) for name in C.SCENES]
    print("[mission] summary: success %d/%d, collisions %d, "
          "targets reached %d/%d, max effort %.2f"
          % (sum(1 for r in results if r["success"]), len(results),
             sum(r["collisions"] for r in results),
             sum(r["reached"] for r in results),
             sum(len(get_scene(r["scene"]).targets) for r in results),
             max(r["effort"] for r in results)))
    print("[control] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
