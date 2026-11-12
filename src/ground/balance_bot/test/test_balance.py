#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""balance_bot 的 pytest 回归测试（无 ROS，只需要 mujoco + numpy）.

对应 ``tools/check_balance.py`` 里的 A/B/D 三项自检，但改成 pytest 断言，
这样 ``pytest src/ground/balance_bot`` 能直接收集，CI/本地回归不用手看终端输出。
"""
import math
import sys
from pathlib import Path

import pytest

PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT / "src"))

from balance_bot import config  # noqa: E402
from balance_bot.control import BalanceController  # noqa: E402
from balance_bot.env import BalanceBotEnv  # noqa: E402


@pytest.fixture(scope="module")
def env():
    e = BalanceBotEnv(config.resolve_scene_path(None), verbose=False)
    yield e
    e.close()


def test_uncontrolled_tips_over(env):
    """A. 无控制：倒立摆必须在 2 s 内倒下，证明仿真不是“站着不动”。"""
    env.reset(tilt=config.SPAWN_TILT)
    t_fall = None
    for _ in range(int(2.0 * config.CONTROL_HZ)):
        st = env.step()
        if not env.is_upright():
            t_fall = st.t
            break
    assert t_fall is not None, "无控制时 2 s 内没有倒下，倒立摆模型可能不对"
    assert t_fall < 2.0


def test_pd_controller_stands(env):
    """B. 倾角 PD：5 s 站立不倒也基本不漂。"""
    ctrl = BalanceController()
    env.reset(tilt=config.SPAWN_TILT)
    worst = 0.0
    for _ in range(int(5.0 * config.CONTROL_HZ)):
        tl, tr = ctrl(env.state(), 0.0, 0.0)
        env.set_torque(tl, tr)
        st = env.step()
        worst = max(worst, abs(st.theta))
        assert env.is_upright(), "5 s 内倒下，t=%.2f s" % st.t
    assert worst < math.radians(5.0), "最大倾角 %.2f°" % math.degrees(worst)
    assert abs(env.state().x) < 0.10, "站立漂移 %.3f m" % env.state().x


def test_speed_tracking(env):
    """D. 速度跟踪：指令 0.30 m/s，末速度要跟上且真的往前走了。"""
    ctrl = BalanceController()
    env.reset(tilt=0.0)
    st = None
    for _ in range(int(6.0 * config.CONTROL_HZ)):
        tl, tr = ctrl(env.state(), 0.30, 0.0)
        env.set_torque(tl, tr)
        st = env.step()
    assert st is not None
    assert abs(st.v - 0.30) < 0.10, "末速度 %.3f m/s" % st.v
    assert st.x > 0.30, "6 s 只前进 %.3f m" % st.x
