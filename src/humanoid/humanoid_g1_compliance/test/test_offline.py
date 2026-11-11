"""离线单元测试（不需要 ROS、不需要显卡）。

    python -m pytest test/test_offline.py -q
"""

import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from humanoid_g1_compliance.compliance import ComplianceRunner, Perturbation  # noqa: E402
from humanoid_g1_compliance.lac.action_map import (                          # noqa: E402
    action_to_q_target, make_scale_offset_tensors,
)
from humanoid_g1_compliance.lac.constants import (                            # noqa: E402
    ACTION_SCALE_23, ART_TO_J23, DAMPING_23, DEFAULT_JOINT_POS_23,
    DEFAULT_JOINT_POS_ART, JOINT_23_TO_ART, JOINT_23_TO_SDK, JOINT_NAMES_23DOF,
    NUM_JOINTS, STIFFNESS_23,
)
from humanoid_g1_compliance.policy import (                                   # noqa: E402
    Command, LacPolicy, StartupFSM, STIFF_BAND, STIFF_NAMES,
)
from humanoid_g1_compliance.sim_core import G1Sim, INACTIVE_SDK_SLOTS         # noqa: E402


@pytest.fixture(scope='module')
def sim():
    return G1Sim()


@pytest.fixture(scope='module')
def policy():
    return LacPolicy(verbose=False)


@pytest.fixture(scope='module')
def runner(sim, policy):
    return ComplianceRunner(sim, policy, StartupFSM(direct=True))


# ---------------------------------------------------------------- 常量一致性
def test_joint_table_consistency():
    assert len(JOINT_NAMES_23DOF) == NUM_JOINTS == 23
    assert len(ACTION_SCALE_23) == len(DEFAULT_JOINT_POS_23) == NUM_JOINTS
    assert len(STIFFNESS_23) == len(DAMPING_23) == NUM_JOINTS
    assert len(JOINT_23_TO_SDK) == NUM_JOINTS
    assert len(set(JOINT_23_TO_SDK)) == NUM_JOINTS, 'SDK 槽位有重复'
    assert max(JOINT_23_TO_SDK) <= 26


def test_articulation_permutation():
    """JOINT_23_TO_ART 必须是 0..22 的置换，且 ART_TO_J23 是它的逆。"""
    assert sorted(JOINT_23_TO_ART) == list(range(NUM_JOINTS))
    for i23, art in enumerate(JOINT_23_TO_ART):
        assert ART_TO_J23[art] == i23
    # 默认角在两套顺序下必须指同一个物理姿态
    for i23, art in enumerate(JOINT_23_TO_ART):
        assert abs(DEFAULT_JOINT_POS_23[i23] - DEFAULT_JOINT_POS_ART[art]) < 1e-9


def test_inactive_slots():
    """23DOF 用不到的 6 个 SDK 槽位：腰 roll/pitch + 左右腕 pitch/yaw。"""
    assert set(INACTIVE_SDK_SLOTS) == {13, 14, 20, 21, 27, 28}
    assert len(JOINT_23_TO_SDK) + len(INACTIVE_SDK_SLOTS) == 29


# ---------------------------------------------------------------- 仿真模型
def test_model_shape(sim):
    assert sim.nu == 29, 'G1 23DOF 的 MuJoCo 模型应有 29 个执行器'
    assert 30.0 < sim.total_mass() < 40.0
    assert abs(sim.pelvis_pos()[2] - 0.74) < 0.02


def test_reset_state(sim):
    sim.reset()
    st = sim.read()
    assert st.q.shape == (23,) and st.dq.shape == (23,)
    assert st.quat.shape == (4,) and st.gyro.shape == (3,)
    assert np.allclose(st.q, DEFAULT_JOINT_POS_23, atol=1e-6)
    assert np.allclose(st.dq, 0.0, atol=1e-9)
    # 直立时重力投影 ≈ (0, 0, -1)
    pg = sim.projected_gravity(st.quat)
    assert abs(pg[2] + 1.0) < 1e-6
    assert abs(pg[0]) < 1e-6 and abs(pg[1]) < 1e-6


def test_projected_gravity_rotation(sim):
    """绕 z 轴转 90° 后重力投影不变；绕 y 轴转 90° 后应指到 x。"""
    pg = sim.projected_gravity(np.array([np.cos(np.pi / 4), 0, 0, np.sin(np.pi / 4)]))
    assert np.allclose(pg, [0, 0, -1], atol=1e-6)
    qy = np.array([np.cos(np.pi / 4), 0, np.sin(np.pi / 4), 0])
    pg2 = sim.projected_gravity(qy)
    assert abs(abs(pg2[0]) - 1.0) < 1e-6


def test_pd_zero_when_on_target(sim):
    """q_target 等于当前角且 dq=0 时，力矩应为 0。"""
    sim.reset()
    st = sim.read()
    sim.apply_pd(st.q, np.full(23, 40.0), np.full(23, 2.0))
    assert np.allclose(sim.data.ctrl, 0.0, atol=1e-9)


def test_pd_inactive_slots_zero(sim):
    sim.reset()
    sim.apply_pd(np.full(23, 0.5), np.full(23, 100.0), np.full(23, 5.0))
    for slot in INACTIVE_SDK_SLOTS:
        assert abs(sim.data.ctrl[slot]) < 1e-12


# ---------------------------------------------------------------- 策略
def test_policy_dims(policy):
    assert policy.info['num_policy_obs'] == 950
    assert policy.info['num_policy_hist_obs'] == 6080
    assert policy.info['policy_frame_dim'] == 95


def test_policy_deterministic(sim, policy):
    """同一初始状态、同一历史深度，两次独立 reset 后的首帧动作必须一致。

    注意只能比「每轮第一次推理」：从第二次起 ``last_action`` 反馈已进入观测，
    动作本来就会变——那是策略的正常依赖，不是随机性。
    """
    cmd = Command()
    first = []
    for _ in range(2):
        sim.reset()
        policy.reset()
        for k in range(3):
            a = policy.infer(sim.read(), cmd).copy()
            if k == 0:
                first.append(a)
    assert np.allclose(first[0], first[1], atol=1e-6)
    # 相邻两帧必须不同，否则说明 last_action 没进观测
    assert not np.allclose(first[0], a, atol=1e-6)


def test_action_to_q_target():
    scale, offset = make_scale_offset_tensors(device='cpu')
    q = action_to_q_target(__import__('torch').zeros(23), scale, offset)
    assert np.allclose(q.cpu().numpy(), DEFAULT_JOINT_POS_23, atol=1e-6)
    # 单位动作时 q = scale + default
    q1 = action_to_q_target(__import__('torch').ones(23), scale, offset)
    expect = np.asarray(ACTION_SCALE_23) + np.asarray(DEFAULT_JOINT_POS_23)
    assert np.allclose(q1.cpu().numpy(), expect, atol=1e-5)


def test_command_clamp():
    cmd = Command()
    cmd.base_height = 2.0
    cmd.clamp()
    assert 0.56 <= cmd.base_height <= 0.78
    cmd.stiffness = np.array([-5.0, 1e6, 250.0, 1.0, 500.0])
    cmd.clamp()
    assert cmd.stiffness.min() >= 1.0


def test_stiffness_band_definition():
    assert set(STIFF_NAMES) == {'K_lin_la', 'K_lin_ra', 'K_lin_to',
                                'K_ang_la', 'K_ang_ra'}
    assert STIFF_BAND['K_lin'][0] < STIFF_BAND['K_lin'][1]
    assert STIFF_BAND['K_ang'][0] < STIFF_BAND['K_ang'][1]


# ---------------------------------------------------------------- 闭环
def test_stand_stable(runner):
    log = runner.run(4.0, cmd=Command())
    assert not log.fell, '零指令站立时摔倒'
    pel = np.asarray(log.pelvis)
    assert pel[:, 2].min() > 0.6
    assert np.asarray(log.pg_z)[-10:].mean() < -0.98


def test_policy_period(runner):
    """策略周期必须正好 20 ms（4 × 5 ms），否则观测历史的时间尺度就错了。"""
    assert abs(runner.dt - 0.02) < 1e-12


def test_compliance_low_vs_high(runner):
    """同一外力下，低刚度必须让位更多。"""
    pert = Perturbation(force=6.0, t_start=3.0, duration=2.0)
    res = {}
    for k in (30.0, 250.0):
        cmd = Command()
        cmd.stiffness[0] = k
        log = runner.run(5.5, cmd=cmd, perturb=pert)
        res[k] = runner.metrics(log, pert, k_lin_la=k)
    assert not res[30.0].fell and not res[250.0].fell
    assert res[30.0].steady_mm > res[250.0].steady_mm * 1.5, (
        f'低刚度 {res[30.0].steady_mm:.1f} mm 未显著大于高刚度 '
        f'{res[250.0].steady_mm:.1f} mm')


def test_metrics_recover(runner):
    """撤销外力后末端应回落：残留必须小于峰值。"""
    pert = Perturbation(force=6.0, t_start=3.0, duration=2.0)
    log = runner.run(7.0, cmd=Command(), perturb=pert)
    m = runner.metrics(log, pert)
    assert m.peak_mm > 5.0
    assert m.settle_mm < m.peak_mm
    assert m.tau_peak > 0.0
    assert m.effective_stiffness() > 0.0
