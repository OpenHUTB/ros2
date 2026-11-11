"""最小验证：无执行器的轮辐式被动行走器能靠重力自己滚下斜坡。"""

import os
import sys
import math

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

pytest.importorskip("mujoco")   # 干净机器上没装 mujoco 时整个文件跳过，而不是报错

from passive_walker.sim import PassiveWalker  # noqa: E402


def test_no_actuator():
    """模型必须完全没有执行器，否则就不是“被动”行走。"""
    sim = PassiveWalker()
    assert sim.model.nu == 0
    assert sim.n_spoke >= 6


def test_walks_downhill():
    """跑 10 s，应该有明显位移、转过若干根辐条，并且轮心始终贴着坡面。"""
    sim = PassiveWalker(push_speed=0.3)
    sim.run(10.0)
    info = sim.summary()
    assert info["distance_m"] > 5.0, info
    assert info["steps"] >= 20, info
    assert 0.40 < info["hub_height_m"] < 0.50, info


def test_run_uses_requested_duration():
    """A final partial physics step prevents run() from overshooting its duration."""
    sim = PassiveWalker()
    requested = sim.dt * 0.5
    sim.run(requested)
    assert math.isclose(sim.data.time, requested, rel_tol=1e-9, abs_tol=1e-12)


def test_step_rejects_fractional_and_negative_counts():
    sim = PassiveWalker()
    for count in (-1, 1.5, float("nan")):
        with pytest.raises(ValueError, match="non-negative integer"):
            sim.step(count)


if __name__ == "__main__":
    test_no_actuator()
    test_walks_downhill()
    print("[OK] test_sim 全部通过")
