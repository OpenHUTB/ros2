"""test_fall_warning.py —— 多模态跌倒预警模块的回归测试

覆盖历史上真出过问题的路径：
  * 站姿：必须能真站满 3 s。早期站姿配平不成立（ankle ≠ -(hip+knee)），
    纯 PD 在 1.4 s 内前倾到 89.8° 直接倒地，而自检只跑 0.6 s 看不出来；
  * 采集口径：输入必须取自"扰动之后"。原实现取扰动之前，2000 个样本的图像特征
    与 IMU 完全相同、标签全 0，数据集彻底退化；
  * 标签窗口：horizon 必须覆盖到真实倒地时间。原 20 步（0.04 s）永远判不出正例；
  * 网络：一步更新必须让损失下降；权重与标准化参数存取往返一致；
  * 推理：真实站立输入不得触发预警。旧权重对站立输入输出 p_fall=0.754。
"""
import os
import sys
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, PKG)

MODEL = os.path.join(PKG, "models", "humanoid_warning.xml")
WEIGHTS = os.path.join(PKG, "models_trained", "fall_warning.npz")

try:
    import mujoco  # noqa: F401
    HAS_MUJOCO = True
except (ImportError, OSError, RuntimeError):
    HAS_MUJOCO = False

HAS_DISPLAY = bool(os.environ.get("DISPLAY"))


@pytest.mark.skipif(not HAS_MUJOCO, reason="mujoco 不可用")
def test_standing_pose_survives_three_seconds():
    """站姿必须撑满 3 s（1500 步）：倾角 < 5°、root_z > 1.0、不跌倒。"""
    from humanoid_fall_warning.sim import FallWarningSim
    sim = FallWarningSim(MODEL)
    sim.reset()
    max_tilt = 0.0
    for _ in range(1500):
        sim.step()
        p, r = sim.torso_pose()
        max_tilt = max(max_tilt, abs(np.degrees(p)), abs(np.degrees(r)))
    assert not sim.fallen(), "3 s 内跌倒"
    assert max_tilt < 5.0, f"3 s 最大倾角 {max_tilt:.1f}° 超限"
    assert sim.root_z > 1.0, f"根高度漂移到 {sim.root_z:.3f}"
    sim.close()


@pytest.mark.skipif(not HAS_MUJOCO, reason="mujoco 不可用")
def test_balance_layer_is_effective():
    """关掉平衡层必须明显变差 —— 证明平衡层不是摆样子。"""
    from humanoid_fall_warning.sim import FallWarningSim
    sim = FallWarningSim(MODEL)
    sim.reset()
    for _ in range(1500):
        sim.step(balance=False)
    tilt = abs(np.degrees(sim.torso_pose()[0]))
    assert tilt > 20.0, f"关掉平衡层后倾角只有 {tilt:.1f}°，平衡层可能没生效"
    sim.close()


@pytest.mark.skipif(not HAS_MUJOCO, reason="mujoco 不可用")
def test_push_calibration_still_separable():
    """推力标定仍然可分离：150 N 以前不倒、150 N 起会倒（数据集正负例的前提）。"""
    from humanoid_fall_warning.sim import FallWarningSim
    sim = FallWarningSim(MODEL)

    def falls(force, settle=300, duration=40, horizon=600):
        sim.reset()
        for _ in range(settle):
            sim.step()
        sim.apply_push((force, 0.0, 0.0), duration_steps=duration)
        for _ in range(horizon):
            sim.step()
            if sim.fallen():
                return True
        return False

    assert not falls(80.0), "80 N 不应把机器人推倒"
    assert falls(200.0), "200 N 应能把机器人推倒"
    sim.close()


@pytest.mark.skipif(not HAS_DISPLAY, reason="需要 DISPLAY 才能渲染")
def test_image_feature_shape_and_variance():
    """图像特征固定 256 维，且不同姿态下必须真的不同。"""
    from humanoid_fall_warning.sim import FallWarningSim, SQUAT_TARGET
    from humanoid_fall_warning.dataset import image_feature
    sim = FallWarningSim(MODEL)
    sim.reset()
    for _ in range(300):
        sim.step()
    f_stand = image_feature(sim)
    assert f_stand.shape == (256,)
    for _ in range(400):
        sim.step(target=SQUAT_TARGET)
    f_squat = image_feature(sim)
    assert np.linalg.norm(f_stand - f_squat) > 0.1, "站立与下蹲的图像特征几乎相同，采集没起作用"
    sim.close()


@pytest.mark.skipif(not (HAS_MUJOCO and HAS_DISPLAY), reason="需要 mujoco 与 DISPLAY")
def test_collect_dataset_is_not_degenerate():
    """小样本采集：正例率不能是 0 或 1，图像特征不能全同。"""
    from humanoid_fall_warning.dataset import collect
    out = os.path.join("/tmp", "fw_test_dataset.npz")
    collect(MODEL, out, n_samples=12, seed=7, verbose=False)
    d = np.load(out)
    X, y = d["X"], d["y"]
    assert X.shape == (12, 258)
    assert 0.1 < float(y.mean()) < 0.9, f"正例率 {float(y.mean()):.2f}，数据集退化"
    uniq = len(np.unique(X[:, :256].round(4), axis=0))
    assert uniq > 3, f"只有 {uniq} 种图像特征，输入几乎是常量"


def test_network_one_step_reduces_loss():
    """一步梯度更新必须让损失下降（Adam + 标准化）。"""
    from humanoid_fall_warning.nn import FallWarningMLP
    rng = np.random.default_rng(0)
    X = rng.normal(0, 1, (64, 258))
    y = (X[:, 0] + X[:, 256] > 0).astype(int)
    model = FallWarningMLP(seed=0)
    model.fit_normalizer(X)
    l0 = -np.mean(np.log(model.predict(X)[np.arange(64), y] + 1e-8))
    for _ in range(5):
        model.train_step(X, y, lr=0.01)
    l1 = -np.mean(np.log(model.predict(X)[np.arange(64), y] + 1e-8))
    assert l1 < l0, f"损失没有下降：{l0:.3f} → {l1:.3f}"


def test_weights_roundtrip_keeps_normalizer():
    """权重存取必须带上标准化参数，否则推理会把原始特征当已标准化输入。"""
    from humanoid_fall_warning.nn import FallWarningMLP
    rng = np.random.default_rng(1)
    X = rng.normal(3.0, 2.0, (32, 258))
    model = FallWarningMLP(seed=3)
    model.fit_normalizer(X)
    path = os.path.join("/tmp", "fw_test_weights.npz")
    model.save(path)
    loaded = FallWarningMLP(seed=9).load(path)
    assert np.allclose(loaded.x_mean, model.x_mean)
    assert np.allclose(loaded.x_std, model.x_std)
    assert np.allclose(loaded.predict(X), model.predict(X))


@pytest.mark.skipif(not os.path.isfile(WEIGHTS), reason="未训练权重")
@pytest.mark.skipif(not (HAS_MUJOCO and HAS_DISPLAY), reason="需要 mujoco 与 DISPLAY")
def test_predictor_does_not_warn_on_standing():
    """真实站立输入不得触发预警：旧权重在这里输出 p_fall=0.754，等于永远喊跌倒。"""
    from humanoid_fall_warning.sim import FallWarningSim
    from humanoid_fall_warning.dataset import image_feature
    from humanoid_fall_warning.predictor import FallWarningPredictor
    sim = FallWarningSim(MODEL)
    sim.reset()
    for _ in range(1500):
        sim.step()
    pred = FallWarningPredictor(WEIGHTS)
    p_normal, p_fall = pred.predict(image_feature(sim), *sim.torso_pose())
    assert abs(p_normal + p_fall - 1.0) < 1e-3
    assert p_fall <= 0.6, f"站立输入被判跌倒 p_fall={p_fall:.3f}"
    sim.close()
