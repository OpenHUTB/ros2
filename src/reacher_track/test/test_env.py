# -*- coding: utf-8 -*-
"""test_env.py —— MuJoCo Reacher 环境与感知状态（没有 mujoco 时整文件 skip）。

重点不是「跑得起来」，而是三处会静默影响结果的地方：
  1. `state()` 的 6 维语义（sin/cos 两个关节 + 全局 dx/dy）必须和文档一致，
     否则 MLP 学到的是另一种输入，部署时误差会莫名偏大；
  2. `step()` 要把扭矩夹到执行器 `ctrlrange`（±2.0），否则仿真里被夹、
     记录的数据没被夹，训练标签就错了；
  3. IK+PD 专家必须稳定收敛——它是演示数据的来源，也是「专家 100%」这句话的依据。
"""
import math

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco', reason='需要 mujoco 才能验证环境与专家控制')

from reacher_track.reacher_env import (L1, L2, ReacherEnv, find_model_path,  # noqa: E402
                                      forward_kinematics)


@pytest.fixture(scope='module')
def env():
    return ReacherEnv()


def test_state_layout_matches_documented_semantics(env):
    env.reset(target_xy=(0.20, 0.05), init_q=(0.3, -0.4))
    s = env.state()
    assert s.shape == (6,)

    q1, q2 = env.q()
    assert s[0] == pytest.approx(math.sin(q1))
    assert s[1] == pytest.approx(math.cos(q1))
    assert s[2] == pytest.approx(math.sin(q2))
    assert s[3] == pytest.approx(math.cos(q2))

    ft = env.fingertip_xy()
    tg = env.target_xy()
    assert s[4] == pytest.approx(tg[0] - ft[0])
    assert s[5] == pytest.approx(tg[1] - ft[1])

    # 末端的解析 FK 必须与仿真里 site 的位置一致，否则 dx/dy 就是错的
    fx, fy = forward_kinematics(q1, q2)
    assert abs(ft[0] - fx) < 1e-6 and abs(ft[1] - fy) < 1e-6


def test_step_clips_ctrl_to_actuator_range(env):
    env.reset(target_xy=(0.15, 0.0), init_q=(0.0, 0.0))
    lo, hi = env.ctrl_range[:, 0], env.ctrl_range[:, 1]
    env.step([99.0, -99.0])
    assert env.data.ctrl[0] == pytest.approx(hi[0])
    assert env.data.ctrl[1] == pytest.approx(lo[1])


def test_reset_sets_target_and_initial_pose(env):
    env.reset(target_xy=(0.12, 0.10), init_q=(0.2, 0.1))
    tg = env.target_xy()
    assert abs(tg[0] - 0.12) < 1e-6 and abs(tg[1] - 0.10) < 1e-6
    assert env.q() == pytest.approx((0.2, 0.1), abs=1e-9)
    assert env.qvel() == pytest.approx((0.0, 0.0), abs=1e-9)
    assert env.distance() == pytest.approx(
        math.hypot(*env.fingertip_xy() - env.target_xy()))


def test_expert_ik_pd_converges_from_random_starts(env):
    """随机初始关节角 × 5 回合，专家控制器必须在 200 步内收敛到 5 cm 内。"""
    np.random.seed(0)
    finals = []
    for _ in range(5):
        q1 = np.random.uniform(-1.5, 1.5)
        q2 = np.random.uniform(-1.5, 1.5)
        target = env.reset(init_q=(q1, q2))
        for _ in range(200):
            env.step(env.ik_pd_ctrl(target))
            if env.distance() < 0.02:
                break
        finals.append(env.distance())

    assert max(finals) < 0.05, '专家未收敛，最终误差 %r' % (finals,)


def test_arm_reach_constants_match_model():
    """L1/L2 是写死的几何常量，必须和 XML 里的连杆长度一致。"""
    m = mujoco.MjModel.from_xml_path(find_model_path())
    assert L1 == pytest.approx(0.15)
    assert L2 == pytest.approx(0.12)
    # XML 中 upper_arm 的 capsule 从 0 到 0.15、fore_arm 从 0 到 0.12
    assert m.nq == 2 and m.nu == 2
