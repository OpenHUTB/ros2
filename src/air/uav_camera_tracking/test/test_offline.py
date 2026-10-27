# -*- coding: utf-8 -*-
"""离线单元测试：不需要 ROS，直接 `pytest test/` 或 `colcon test` 即可运行。

覆盖三处"错了会静默失效"的地方：相机投影方向、热图标签对齐、专家指令符号。
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from uav_camera_tracking import params as P                     # noqa: E402
from uav_camera_tracking.camera import Camera                   # noqa: E402
from uav_camera_tracking.controllers import expert_command       # noqa: E402
from uav_camera_tracking.detector import DetectorNet            # noqa: E402
from uav_camera_tracking.nn import MLP                          # noqa: E402
from uav_camera_tracking.quadrotor import Quadrotor             # noqa: E402


def test_projection_center_aligned():
    """光轴正下方的地面点应投影到画面中心（下俯角方向的符号验证）。"""
    cam = Camera()
    h = 20.0
    # 光轴与地面交点：水平距离 = h / tan(PITCH)
    d = h / np.tan(np.deg2rad(P.PITCH_DEG))
    uv, depth = cam.project(np.array([0.0, 0.0, h]), 0.0,
                            np.array([[d, 0.0, 0.0]]))
    assert depth[0] > 0
    assert abs(uv[0, 0] - (P.IMG_W - 1) / 2.0) < 1.5
    assert abs(uv[0, 1] - (P.IMG_H - 1) / 2.0) < 1.5


def test_target_farther_is_higher_in_image():
    """地面目标越远，成像越靠画面上方（v 更小）——距离环用它当误差量。"""
    cam = Camera()
    cam_pos = np.array([0.0, 0.0, P.H_DES])
    near, _ = cam.project(cam_pos, 0.0, np.array([[9.0, 0.0, 0.0]]))
    far, _ = cam.project(cam_pos, 0.0, np.array([[16.0, 0.0, 0.0]]))
    assert far[0, 1] < near[0, 1]


def test_wall_occludes_target():
    """立柱挡在相机与目标之间时，可见性应为 False。"""
    cam = Camera()
    cam_pos = np.array([0.0, 0.0, P.H_DES])
    # 高度必须挡住视线：相机 9 m 高、目标在 14 m 外，视线在 x=7 m 处高约 4.5 m
    wall = (7.0, 0.0, 0.6, 0.6, 3.0)          # 6 m 高、1.2 m 见方
    hit = cam.observe(cam_pos, 0.0, (14.0, 0.0), (wall,))
    clear = cam.observe(cam_pos, 0.0, (14.0, 0.0), ())
    assert clear['visible'] and clear['in_fov']
    assert hit['occluded'] and not hit['visible']


def test_heat_label_and_decode_are_consistent():
    """软标签与 detect() 的软质心解码必须用同一套格子约定，否则定位整体偏移。"""
    det = DetectorNet()
    idx = np.arange(P.DS)
    gx, gy = np.meshgrid(idx, idx)
    for uc, vc in ((0.1, 0.1), (0.5, 0.5), (0.9, 0.8), (0.73, 0.42)):
        q = det._heat_target(uc, vc)
        assert abs(float(q.sum()) - 1.0) < 1e-5        # 归一化软标签
        u = (q * (gx.reshape(-1) + 0.5)).sum() / P.DS * P.IMG_W
        v = (q * (gy.reshape(-1) + 0.5)).sum() / P.DS * P.IMG_H
        assert abs(u - uc * P.IMG_W) < 2.5             # 误差小于半个格子
        assert abs(v - vc * P.IMG_H) < 2.5


def test_expert_command_signs():
    """专家指令符号：目标偏右->右移；目标偏下(太近)->减速后退。"""
    feat = np.zeros(7)
    feat[0], feat[1], feat[4], feat[6] = 0.4, 0.3, 0.0, 0.0
    cmd, lost = expert_command(feat, True)
    assert not lost
    assert cmd[1] > 0.0        # ex > 0 目标在右侧 -> vy > 0
    assert cmd[0] < 0.0        # ey > 0 目标偏下(太近) -> vx < 0

    hidden, _ = expert_command(np.zeros(7), False)
    assert np.allclose(hidden, 0.0)               # 丢失时悬停等待


def test_quadrotor_tracks_speed_command():
    """速度指令给足时间后，实际速度应收敛到指令附近（上限受 V_MAX 约束）。"""
    q = Quadrotor((0.0, 0.0, P.H_DES), 0.0)
    q.set_cmd((1.0, 0.0), 0.0, 0.0)
    for _ in range(int(3.0 / P.DT)):
        q.step(np.zeros(3))
    assert abs(q.v[0] - 1.0) < 0.15
    assert abs(q.p[2] - P.H_DES) < 0.5            # 高度保持


def test_mlp_fit_linear_function():
    """MLP 基本拟合能力（回归掩码与标准化路径一起走一遍）。"""
    rng = np.random.default_rng(0)
    X = rng.normal(size=(400, 3))
    Y = np.stack([X[:, 0] - 0.5 * X[:, 1], X[:, 2] * 0.5], axis=1)
    net = MLP([3, 16, 2], seed=0)
    hist = net.fit(X, Y, epochs=60, batch=64, lr=3e-3)
    assert hist['val_loss'][-1] < 0.05
    pred = net.predict(X[:10])
    assert np.max(np.abs(pred - Y[:10])) < 0.5
