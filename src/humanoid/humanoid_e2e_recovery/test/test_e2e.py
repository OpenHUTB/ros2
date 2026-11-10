"""test_e2e.py —— 端到端视觉平衡控制模块单元测试。

覆盖：模型加载、专家控制器站姿判据、MLP 前向/反传、权重判别力、推理量纲。
"""
from __future__ import annotations
import os
import sys
import tempfile

import numpy as np
import pytest

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)

from humanoid_e2e_recovery.sim import E2ESim, DEFAULT_STAND_TARGET
from humanoid_e2e_recovery.expert import expert_action
from humanoid_e2e_recovery.e2e_nn import E2EMLP
from humanoid_e2e_recovery.predictor import E2EPredictor

MODEL = os.path.join(HERE, "models", "humanoid_camera.xml")
WEIGHTS = os.path.join(HERE, "models_trained", "e2e_policy.npz")

pytest.importorskip("mujoco")


def test_模型加载():
    sim = E2ESim(MODEL)
    assert sim.model.nq > 0
    assert sim.model.nu == 15
    sim.close()


def test_专家控制器站姿稳10秒():
    """teacher 必须自己站得住——这是评审阻塞项②的验收条件。"""
    sim = E2ESim(MODEL)
    sim.reset()
    fell_at = None
    for _ in range(int(10.0 / 0.002)):
        sim.step(target=DEFAULT_STAND_TARGET, torque_extra=expert_action(sim))
        if sim.fallen():
            fell_at = sim.data.time
            break
    sim.close()
    assert fell_at is None, f"专家控制器在 {fell_at:.2f}s 跌倒"


def test_专家控制器抗推恢复():
    sim = E2ESim(MODEL)
    sim.reset()
    for _ in range(300):
        sim.step(target=DEFAULT_STAND_TARGET, torque_extra=expert_action(sim))
    sim.apply_push((50, 0, 0), steps=40, target=DEFAULT_STAND_TARGET,
                   balance=expert_action)
    fell_at = None
    for _ in range(int(8.0 / 0.002)):
        sim.step(target=DEFAULT_STAND_TARGET, torque_extra=expert_action(sim))
        if sim.fallen():
            fell_at = sim.data.time
            break
    sim.close()
    assert fell_at is None, f"50 N 推扰后 {fell_at:.2f}s 跌倒"


def test_mlp前向反传():
    rng = np.random.default_rng(0)
    X = rng.normal(size=(32, 256))
    Y = rng.normal(size=(32, 15))
    m = E2EMLP(256, 128, 64, 15)
    loss0 = m.train_step(X, Y, lr=0.01)
    for _ in range(30):
        loss1 = m.train_step(X, Y, lr=0.01)
    assert loss1 < loss0, "训练 30 步损失没有下降"


def test_权重文件可加载且非全零():
    if not os.path.exists(WEIGHTS):
        pytest.skip("权重文件尚未生成")
    sim = E2ESim(MODEL)
    sim.reset()
    img = sim.render_front(gray=True)
    sim.close()
    p = E2EPredictor(WEIGHTS)
    out = p.predict_from_image(img)
    assert out.shape == (15,)
    assert np.std(out) > 1e-6, "网络输出全零，权重未真正加载"
    assert np.all(np.abs(out) < 600), f"输出量纲异常: max={np.max(np.abs(out)):.1f}"


def test_零权重被判为失稳():
    """判别力回归测试：全零权重必须站不住，否则说明站姿判据无效。"""
    if not os.path.exists(WEIGHTS):
        pytest.skip("权重文件尚未生成")
    tmpdir = tempfile.mkdtemp(prefix="e2e_zero_")
    d = dict(np.load(WEIGHTS))
    for k in ("W1", "W2", "W3", "b1", "b2", "b3"):
        d[k] = np.zeros_like(d[k])
    zero_w = os.path.join(tmpdir, "zero.npz")
    np.savez(zero_w, **d)
    p = E2EPredictor(zero_w)
    sim = E2ESim(MODEL)
    sim.reset()
    fell_at = None
    for i in range(int(10.0 / 0.002)):
        if i % 2 == 0:
            u = p.predict_from_image(sim.render_front(gray=True))
        sim.step(target=DEFAULT_STAND_TARGET, torque_extra=u)
        if sim.fallen():
            fell_at = sim.data.time
            break
    sim.close()
    assert fell_at is not None, "全零权重竟然站满 10 s，说明自检无判别力"
