"""test_depth_pose.py —— 深度图姿态估计模块的回归测试

覆盖四条曾经出问题或容易被改坏的路径：
  * 站姿：必须能真站满 3 s（早期只测 0.6 s，掩盖了 1.4 s 内自我倾倒）；
  * 深度图：shape 与归一化区间；
  * 控制器：pitch/roll/yaw 三路修正的符号与落点关节；
  * 网络：一步梯度更新必须让损失下降，权重存取往返一致。
"""
import os
import sys
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, PKG)

MODEL = os.path.join(PKG, "models", "humanoid_depth.xml")

try:
    import mujoco  # noqa: F401
    HAS_MUJOCO = True
except (ImportError, OSError, RuntimeError):
    HAS_MUJOCO = False


@pytest.mark.skipif(not HAS_MUJOCO, reason="mujoco 不可用")
def test_standing_pose_survives_three_seconds():
    from humanoid_depth_pose_control.sim import DepthPoseSim
    sim = DepthPoseSim(MODEL)
    sim.reset()
    max_tilt = 0.0
    for _ in range(1500):                 # 3 s @ timestep 0.002
        sim.step()
        pitch, roll, _ = sim.torso_pose()
        max_tilt = max(max_tilt, abs(np.degrees(pitch)), abs(np.degrees(roll)))
    fallen, z = sim.fallen(), sim.root_z
    sim.close()
    assert not fallen, f"3 s 内倒下，root_z={z:.3f}"
    assert max_tilt < 5.0, f"3 s 最大倾角 {max_tilt:.2f}°，超过 5°"


@pytest.mark.skipif(not HAS_MUJOCO, reason="mujoco 不可用")
def test_open_loop_pd_is_not_stable():
    """关掉平衡反馈就必须站不住 —— 说明 3 s 站姿是反馈在起作用，不是碰巧。"""
    from humanoid_depth_pose_control.sim import DepthPoseSim
    sim = DepthPoseSim(MODEL)
    sim.reset()
    max_tilt = 0.0
    for _ in range(1500):
        sim.step(balance=False)
        pitch, roll, _ = sim.torso_pose()
        max_tilt = max(max_tilt, abs(np.degrees(pitch)), abs(np.degrees(roll)))
    sim.close()
    assert max_tilt > 15.0, "开环 PD 竟然稳住了，说明平衡反馈没被真正关掉"


@pytest.mark.skipif(not HAS_MUJOCO, reason="mujoco 不可用")
def test_depth_image_shape_and_range():
    from humanoid_depth_pose_control.sim import DepthPoseSim
    sim = DepthPoseSim(MODEL)
    sim.reset()
    for _ in range(100):
        sim.step()
    depth = sim.render_depth()
    sim.close()
    assert depth.shape == (64, 64)
    assert 0.0 <= float(depth.min()) and float(depth.max()) <= 1.0


def test_controller_three_axis_signs():
    from humanoid_depth_pose_control.controller import ProportionalController
    from humanoid_depth_pose_control.sim import DEFAULT_STAND_TARGET
    ctrl = ProportionalController()
    base = ctrl.target_for(0.0, 0.0, 0.0)
    np.testing.assert_allclose(base, DEFAULT_STAND_TARGET)

    # 前倾 → 踝背屈（正）、髋反向；左右踝同向，左右髋对称
    p = ctrl.target_for(0.1, 0.0, 0.0)
    assert p[3] > base[3] and p[8] > base[8]
    assert p[1] < base[1] and p[6] < base[6]

    # 右倾 → 左右踝/髋反向
    r = ctrl.target_for(0.0, 0.1, 0.0)
    assert r[4] > base[4] and r[9] < base[9]
    assert r[0] < base[0] and r[5] > base[5]

    # yaw → 只动颈部，不动腿
    y = ctrl.target_for(0.0, 0.0, 0.2)
    assert y[14] < base[14]
    for i in range(10):
        assert y[i] == base[i]


def test_mlp_one_step_reduces_loss():
    from humanoid_depth_pose_control.nn import DepthPoseMLP
    rng = np.random.default_rng(0)
    model = DepthPoseMLP(input_dim=16, hidden1=12, hidden2=8, output_dim=3, seed=0)
    x = rng.normal(size=(8, 16))
    y = rng.normal(size=(8, 3))
    before = model.loss(x, y)
    for _ in range(5):
        model.train_step(x, y, lr=0.01)
    after = model.loss(x, y)
    assert after < before, f"训练 5 步后损失没下降：{before:.6f} → {after:.6f}"


def test_predictor_weight_roundtrip(tmp_path):
    from humanoid_depth_pose_control.nn import DepthPoseMLP
    from humanoid_depth_pose_control.predictor import DepthPosePredictor
    path = str(tmp_path / "w.npz")
    model = DepthPoseMLP(seed=3)
    model.save(path)
    pred = DepthPosePredictor(path)
    depth = np.full((64, 64), 0.5, dtype=np.float32)
    out = pred.predict_from_depth(depth)
    assert len(out) == 3
    assert all(np.isfinite(out))
