"""控制器与单次飞行的集成测试。

需要 MuJoCo（底座）与 torch；缺任一时相关用例跳过。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pytest

from uav_neural_fault_tolerant_control import allocation as A
from uav_neural_fault_tolerant_control import controller as C
from uav_neural_fault_tolerant_control import faults as FL


@dataclass
class FakeParams:
    mass: float = 0.95
    arm: float = 0.106
    k_m: float = 0.015
    max_thrust: float = 6.0
    hover_thrust: float = 2.33


PARAMS = FakeParams()
B = np.array([
    [1.0, 1.0, 1.0, 1.0],
    [-0.106, 0.106, 0.106, -0.106],
    [-0.106, 0.106, -0.106, 0.106],
    [-0.015, -0.015, 0.015, 0.015],
])
DESIRED = np.array([9.32, 0.0, 0.0, 0.0])


def _mixer(T: float, tau, p) -> np.ndarray:
    """与底座 `mixer` 同构的简化版（测试里不引入 MuJoCo）。"""
    u, v, w = tau[0] / p.arm, tau[1] / p.arm, tau[2] / p.k_m
    budget = min(4.0 * p.max_thrust - T, T)
    spread = abs(u) + abs(v) + abs(w)
    if spread > budget and spread > 0.0:
        k = budget / spread
        u, v, w = u * k, v * k, w * k
    return np.clip(np.array([(T + u + v + w) / 4.0, (T - u - v - w) / 4.0,
                             (T + u - v - w) / 4.0, (T - u + v + w) / 4.0]),
                   0.0, p.max_thrust)


def _controller(mode: str, gains=None, net=None) -> C.FaultTolerantController:
    return C.FaultTolerantController(PARAMS, _mixer, None, B, mode=mode,
                                     net=net, gains=gains)


def test_row_weights_switch_when_rotor_lost():
    ctrl = _controller("ls", gains=np.ones(4))
    assert ctrl.row_weights()[0] == pytest.approx(A.THRUST_WEIGHT)
    assert ctrl.row_weights()[3] == pytest.approx(1.0)

    ctrl.set_health(np.array([0.0, 1.0, 1.0, 1.0]))
    # 少一个桨就放弃偏航，否则力矩互相抢预算
    assert ctrl.row_weights()[3] == 0.0


@pytest.mark.parametrize("mode", ["geometric", "naive", "ls"])
def test_allocate_returns_four_bounded_thrusts(mode):
    ctrl = _controller(mode, gains=np.array([0.0, 1.0, 1.0, 1.0]))
    u = ctrl.allocate(DESIRED)
    assert u.shape == (4,)
    assert np.all(u >= 0.0) and np.all(u <= PARAMS.max_thrust + 1e-9)


def test_ls_mode_without_weights():
    """退化桨存在时，ls 分配不应给坏桨下指令。"""
    ctrl = _controller("ls", gains=np.array([0.0, 1.0, 1.0, 1.0]))
    u = ctrl.allocate(DESIRED)
    assert u[0] == 0.0
    assert np.isclose(A.effective_thrust(u, np.array([0.0, 1.0, 1.0, 1.0])).sum(),
                      DESIRED[0], atol=0.5)


def test_untrained_neural_equals_ls():
    """输出层零初始化 ⇒ 未训练的 neural 与 ls 完全一致。"""
    M = pytest.importorskip("uav_neural_fault_tolerant_control.model")
    net = M.build_model()
    gains = np.array([0.6, 1.0, 1.0, 1.0])
    u_ls = _controller("ls", gains=gains).allocate(DESIRED)
    u_nn = _controller("neural", gains=gains, net=net).allocate(DESIRED)
    assert np.allclose(u_ls, u_nn, atol=1e-6)


def test_neural_residual_is_clamped():
    """残差限幅：即使网络给出很大的修正，指令也只是偏离 ls 一定量。"""
    M = pytest.importorskip("uav_neural_fault_tolerant_control.model")
    net = M.build_model()
    last = net.net[-1]
    last.weight.data.fill_(5.0)       # 故意给一个很大的输出层
    last.bias.data.fill_(0.0)
    gains = np.array([0.6, 1.0, 1.0, 1.0])
    u_ls = _controller("ls", gains=gains).allocate(DESIRED)
    u_nn = _controller("neural", gains=gains, net=net).allocate(DESIRED)
    limit = C.MAX_RESIDUAL * PARAMS.max_thrust + 1e-6
    assert np.all(np.abs(u_nn - u_ls) <= limit)


# -- 单次飞行（需要 MuJoCo 底座） -------------------------------------------

def _api():
    pytest.importorskip("mujoco", reason="需要 MuJoCo")
    from uav_neural_fault_tolerant_control import _compat as COMPAT

    try:
        return COMPAT.load_base()
    except ImportError as exc:      # pragma: no cover - 取决于本机仓库布局
        pytest.skip(f"找不到底座 uav_mujoco_sim：{exc}")


def test_healthy_hover_does_not_crash():
    from uav_neural_fault_tolerant_control import evaluate as E

    api = _api()
    row = E.run_mode(api, "ls", FL.FaultPlan(kind="healthy"), "hover", seconds=3.0)
    assert not row.crashed
    assert row.min_altitude == pytest.approx(1.5, abs=0.05)


def test_degraded_rotor_crash_differs_between_modes():
    """退化桨：朴素补偿会掉高度，加权分配不会 —— 这是本模块的核心结论。"""
    from uav_neural_fault_tolerant_control import evaluate as E

    api = _api()
    plan = FL.FaultPlan(kind="degradation", rotor=1, onset=1.0, severity=0.6, ramp=0.0)
    naive = E.run_mode(api, "naive", plan, "hover", seconds=6.0)
    ls = E.run_mode(api, "ls", plan, "hover", seconds=6.0)
    assert ls.min_altitude > naive.min_altitude
    assert ls.min_altitude > 1.4
