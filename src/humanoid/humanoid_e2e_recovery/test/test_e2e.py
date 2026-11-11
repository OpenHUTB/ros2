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
DATASET = os.path.join(HERE, "data", "e2e_dataset.npz")

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


def test_权重文件可加载且对样本有响应():
    """跨**样本**判别：不能只看同一帧 15 维之间的离散度。

    同一帧内踝 pitch 是大数、其余维为 0，np.std(out) 天然很大（实测 18.7），
    即使第一层 ReLU 全死、网络退化成常数函数也能通过，故改为用数据集里的
    真样本前向，检查输出在样本之间是否真的变化，以及隐藏层是否整片死亡。
    """
    if not os.path.exists(WEIGHTS):
        pytest.skip("权重文件尚未生成")
    p = E2EPredictor(WEIGHTS)

    sim = E2ESim(MODEL)
    sim.reset()
    img = sim.render_front(gray=True)
    sim.close()
    out = p.predict_from_image(img)
    assert out.shape == (15,)
    assert np.all(np.abs(out) < 600), f"输出量纲异常: max={np.max(np.abs(out)):.1f}"

    if not os.path.exists(DATASET):
        pytest.skip("数据集不存在，无法做跨样本判别")
    ds = np.load(DATASET)
    Xs = ds["X"][:3000].astype(np.float64)
    outs = p.predict_vecs(Xs)
    per_dim = outs.std(axis=0)
    assert per_dim.max() > 1e-3, (
        f"网络输出在 {len(Xs)} 条真样本间几乎不变（最大逐维 std={per_dim.max():.6f}），"
        f"相当于对图像无响应")
    st = p.hidden_stats(Xs)
    assert st["relu1_alive_neurons"] >= st["relu1_total_neurons"] * 0.5, (
        f"第一层 ReLU 大片死亡：存活 {st['relu1_alive_neurons']}/"
        f"{st['relu1_total_neurons']}，激活率 {st['relu1_activation_rate']*100:.2f}%")


def test_零权重被判为失稳():
    """判别力回归测试：全零权重必须站不住，否则说明站姿判据无效。"""
    if not os.path.exists(WEIGHTS):
        pytest.skip("权重文件尚未生成")
    tmpdir = tempfile.mkdtemp(prefix="e2e_zero_")
    d = dict(np.load(WEIGHTS))
    for k in ("W1", "W2", "W3", "b1", "b2", "b3"):
        d[k] = np.zeros_like(d[k])
    # 输出中心化项一并清零，保证零权重网络真的输出零力矩
    if "Y_mean" in d:
        d["Y_mean"] = np.zeros_like(d["Y_mean"])
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
