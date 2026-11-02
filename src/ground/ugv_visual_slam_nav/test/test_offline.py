# -*- coding: utf-8 -*-
"""离线单元测试：几何、检测、SLAM、建图、规划、控制、网络梯度。

全部不依赖 ROS，任何装了 numpy 的机器都能跑：
    python -m pytest test/test_offline.py -q
"""

import math

import numpy as np
import pytest

from ugv_visual_slam_nav import camera, detector, params as P
from ugv_visual_slam_nav.cnn import IN_CH, Adam, CNN
from ugv_visual_slam_nav.controllers import (avoid_from_depth, goal_reached,
                                            pure_pursuit)
from ugv_visual_slam_nav.gridmap import GridMap, truth_grid
from ugv_visual_slam_nav.kinematics import DiffDrive, integrate, wrap
from ugv_visual_slam_nav.planner import astar, inflate, proximity_cost
from ugv_visual_slam_nav.policies import (E2EPolicy, STOP_TOL, goal_channel,
                                          make_policy, mark_circle)
from ugv_visual_slam_nav.slam import EKFSLAM
from ugv_visual_slam_nav.sim import stack_dataset
from ugv_visual_slam_nav.world import make_scene


# ------------------------------------------------------------------ 运动学
def test_integrate_straight_line():
    pose = integrate(np.array([1.0, 2.0, 0.0]), 0.5, 0.0, 1.0)
    assert pose[0] == pytest.approx(1.5, abs=1e-9)
    assert pose[1] == pytest.approx(2.0, abs=1e-9)
    assert pose[2] == pytest.approx(0.0, abs=1e-9)


def test_integrate_arc_radius():
    """圆弧半径应等于 v/w，转 90° 后终点在 (r, r)。"""
    v, w = 0.4, 1.0
    r = v / w
    pose = integrate(np.array([0.0, 0.0, 0.0]), v, w, math.pi / 2.0 / w)
    assert pose[0] == pytest.approx(r, abs=1e-9)
    assert pose[1] == pytest.approx(r, abs=1e-9)
    assert pose[2] == pytest.approx(math.pi / 2, abs=1e-9)


def test_wrap_range():
    for a in (-7.0, -math.pi, 0.0, 3.0, 9.0):
        assert -math.pi - 1e-9 <= wrap(a) <= math.pi + 1e-9


def test_odometry_drifts_but_tracks():
    rng = np.random.default_rng(0)
    drive = DiffDrive([2.0, 2.0, 0.0])
    for _ in range(50):
        drive.step(0.4, 0.0, P.DT, rng)
    err = math.hypot(drive.odom[0] - drive.pose[0], drive.odom[1] - drive.pose[1])
    assert 0.0 < err < 0.6, '里程计应有漂移但不能离谱，实际 %.3f' % err


# ------------------------------------------------------------------ 光线投射
def test_raycast_hits_wall_ahead():
    w = make_scene('corridor', 0)
    dist, hid = w.raycast((2.0, 2.0), np.array([0.0]))
    # 正前方是 x=6.85 的隔墙
    assert hid[0] > 0
    assert dist[0] == pytest.approx(6.85 - 2.0, abs=0.02)


def test_raycast_hits_landmark_before_wall():
    w = make_scene('corridor', 0)
    lx, ly = w.landmarks[1][:2]
    origin = (lx - 1.0, ly)
    dist, hid = w.raycast(origin, np.array([0.0]))
    n_rect = len(w.rects)
    assert n_rect < hid[0] <= n_rect + len(w.landmarks), '应命中路标而不是墙'
    assert dist[0] == pytest.approx(1.0 - P.LM_RADIUS, abs=0.02)


def test_raycast_returns_max_range_when_clear():
    w = make_scene('corridor', 0)
    dist, hid = w.raycast((2.0, 2.0), np.array([math.pi * 0.75]), max_range=3.0)
    assert dist[0] <= 3.0 + 1e-9


# ------------------------------------------------------------------ 相机
def test_render_shapes_and_depth_range():
    w = make_scene('corridor', 0)
    rgb, depth = camera.render(w, w.start, np.random.default_rng(0))
    assert rgb.shape == (P.CAM_H, P.CAM_W, 3) and rgb.dtype == np.uint8
    assert depth.shape == (P.CAM_H, P.CAM_W)
    assert depth.min() > 0.0 and depth.max() <= P.CAM_RANGE_MAX + 1e-3


def test_render_night_is_darker():
    w = make_scene('corridor', 0)
    day, _ = camera.render(w, w.start, np.random.default_rng(1))
    night, _ = camera.render(w, w.start, np.random.default_rng(1), night=True)
    assert float(night.mean()) < float(day.mean()) * 0.9


# ------------------------------------------------------------------ 检测
def test_detect_finds_visible_landmark():
    w = make_scene('corridor', 0)
    rgb, depth = camera.render(w, w.start, np.random.default_rng(0))
    lms, wall = detector.detect(rgb, depth)
    assert len(lms) >= 1
    assert len(wall) > 20, '深度点应覆盖大部分列'
    # 检出距离与真值一致（起点附近那颗绿色柱子）
    lid, rng_m, brg = lms[0]
    lx, ly = w.landmarks[lid][:2]
    truth = math.hypot(lx - w.start[0], ly - w.start[1])
    assert rng_m == pytest.approx(truth, abs=0.35)


def test_detect_no_false_positives_at_night():
    """夜间噪声大，但竖直柱子的"每列像素密度"判据应把散点全滤掉。"""
    w = make_scene('night', 0)
    rgb, depth = camera.render(w, w.start, np.random.default_rng(7), night=True)
    lms, _ = detector.detect(rgb, depth)
    assert len(lms) <= 4, '夜间误检过多：%d' % len(lms)


def test_label_map_ignores_grey_surfaces():
    w = make_scene('corridor', 0)
    rgb, depth = camera.render(w, w.start, np.random.default_rng(0))
    lab = detector.label_map(rgb)
    hits = int((lab >= 0).sum())
    assert 0 < hits < rgb.shape[0] * rgb.shape[1] * 0.2, '彩色像素不该超过全图两成'


# ------------------------------------------------------------------ SLAM
def test_ekf_estimate_is_accurate_with_consistent_prior():
    """给定已知起点与无噪观测，位姿与路标都应几乎无误差。

    注意 EKF-SLAM 只有相对约束：如果初始位姿本身是错的、路标又从这个
    错误位姿初始化，整个地图会自洽地平移过去（规范自由度），不会自己
    收敛回真值。所以这里测的是"在正确先验下的估计精度"。
    """
    truth_pose = np.array([5.0, 5.0, 0.30])
    lms = [(0, (8.0, 5.0)), (1, (6.0, 7.0)), (2, (3.5, 6.5))]
    slam = EKFSLAM(truth_pose)

    for _ in range(25):
        obs = []
        for lid, (mx, my) in lms:
            dx, dy = mx - truth_pose[0], my - truth_pose[1]
            obs.append((lid, math.hypot(dx, dy), wrap(math.atan2(dy, dx) - truth_pose[2])))
        slam.update(obs)

    err = math.hypot(slam.mu[0] - truth_pose[0], slam.mu[1] - truth_pose[1])
    assert err < 0.02, '位姿误差应很小，实际 %.3f m' % err
    assert abs(wrap(slam.mu[2] - truth_pose[2])) < 0.02
    assert len(slam.ids) == 3
    for lid, (mx, my) in lms:
        est = [t for t in slam.landmark_xy() if t[2] == lid][0]
        assert math.hypot(est[0] - mx, est[1] - my) < 0.05


def test_ekf_rejects_outlier_measurement():
    slam = EKFSLAM(np.array([5.0, 5.0, 0.0]))
    slam.update([(0, 3.0, 0.0)])
    before = slam.mu.copy()
    slam.update([(0, 30.0, 1.5)])     # 离谱观测：马氏距离远超门限
    assert np.allclose(slam.mu, before, atol=1e-9)


def test_ekf_landmark_estimate_is_consistent():
    truth_pose = np.array([5.0, 5.0, 0.0])
    mx, my = 8.0, 6.0
    slam = EKFSLAM(truth_pose)
    for _ in range(15):
        dx, dy = mx - truth_pose[0], my - truth_pose[1]
        slam.update([(3, math.hypot(dx, dy), wrap(math.atan2(dy, dx)))])
    est = slam.landmark_xy()[0]
    assert math.hypot(est[0] - mx, est[1] - my) < 0.08


# ------------------------------------------------------------------ 建图
def test_gridmap_updates_and_scores():
    w = make_scene('corridor', 0)
    rgb, depth = camera.render(w, w.start, np.random.default_rng(0))
    _, wall = detector.detect(rgb, depth)
    gm = GridMap()
    for _ in range(3):
        gm.update(w.start, wall)
    occ, explored = gm.counts()
    assert explored > 200, '应该有几百个格子被访问过'
    assert 0.0 < gm.iou(truth_grid(w)) < 1.0
    assert gm.coverage() > 0.0


def test_truth_grid_marks_walls():
    w = make_scene('corridor', 0)
    t = truth_grid(w)
    assert t[0, :].any() and t[-1, :].any(), '上下外墙应该被标出来'
    assert t[:, 0].any() and t[:, -1].any()


# ------------------------------------------------------------------ 规划
def test_astar_returns_collision_free_path():
    w = make_scene('corridor', 0)
    blocked = inflate(truth_grid(w), 5)
    path = astar(blocked, (2.0, 2.0), (17.6, 11.6))
    assert len(path) > 20
    for x, y in path:
        c = int(x / P.GRID_RES)
        r = int(y / P.GRID_RES)
        assert not blocked[r, c], '路径点落在障碍上：(%.2f, %.2f)' % (x, y)


def test_astar_unreachable_returns_empty():
    blocked = np.zeros((P.GH, P.GW), dtype=bool)
    blocked[:, P.GW // 2] = True          # 一堵竖墙把地图劈成两半
    path = astar(blocked, (1.0, 7.0), (P.WORLD_W - 1.0, 7.0))
    assert path == []


def test_proximity_cost_prefers_middle():
    occ = np.zeros((P.GH, P.GW), dtype=bool)
    occ[0, :] = True
    c = proximity_cost(occ)
    assert c[1, 10] > c[5, 10] > c[30, 10], '离障碍越近代价越高'


def test_mark_circle_blocks_cells():
    mask = np.zeros((P.GH, P.GW), dtype=bool)
    mark_circle(mask, 5.0, 5.0, 0.4)
    assert mask[int(5.0 / P.GRID_RES), int(5.0 / P.GRID_RES)]
    assert not mask[0, 0]


# ------------------------------------------------------------------ 控制
def test_pure_pursuit_straight_has_no_steering():
    path = [(1.0, 0.0), (3.0, 0.0), (6.0, 0.0)]
    v, w = pure_pursuit((0.0, 0.0, 0.0), path)
    assert v > 0.2 and abs(w) < 0.05


def test_pure_pursuit_turns_toward_path():
    path = [(1.0, 1.0), (2.0, 2.0), (4.0, 4.0)]
    _v, w = pure_pursuit((0.0, 0.0, 0.0), path)
    assert w > 0.2, '目标在左前方时应左转（w>0）'


def test_pure_pursuit_stops_at_goal():
    path = [(0.20, 0.0)]
    v, _w = pure_pursuit((0.0, 0.0, 0.0), path)
    assert v < 0.12, '距离终点 0.2 m 时应几乎停住，实际 %.3f' % v


def test_goal_tolerance_is_stricter_than_evaluation():
    assert STOP_TOL < P.GOAL_TOL
    assert goal_reached((0.0, 0.0, 0.0), (0.3, 0.0))
    assert not goal_reached((0.0, 0.0, 0.0), (0.3, 0.0), tol=STOP_TOL)


# ------------------------------------------------------------------ 场景
@pytest.mark.parametrize('scene', P.SCENES)
def test_all_scenes_are_solvable(scene):
    from ugv_visual_slam_nav.world import _reachable
    assert _reachable(make_scene(scene, 0)), '%s 场景在真值上不可达' % scene


# ------------------------------------------------------------------ 网络
def test_cnn_conv_gradient_matches_numeric():
    from ugv_visual_slam_nav.cnn import Conv

    conv = Conv(2, 3, 3, 2, 1, np.random.default_rng(1))
    x = np.random.randn(2, 2, 6, 6)
    out = conv.forward(x, train=True)
    dout = np.random.randn(*out.shape)
    conv.backward(dout)

    probe = 1e-5
    n = x.shape[0]
    base = float((conv.forward(x, train=False) * dout).sum()) / n
    conv.W[0, 4] += probe
    after = float((conv.forward(x, train=False) * dout).sum()) / n
    numeric = (after - base) / probe
    analytic = float(conv.gW[0, 4])           # gW 本身已是 batch 平均
    assert numeric == pytest.approx(analytic, rel=0.05), (numeric, analytic)


def test_cnn_output_is_bounded():
    net = CNN(E2EPolicy.IN_H, E2EPolicy.IN_W, np.random.default_rng(0))
    x = np.random.randn(4, IN_CH, E2EPolicy.IN_H, E2EPolicy.IN_W)
    out = net.forward(x, train=False)
    assert out.shape == (4, 2)
    assert np.all(np.abs(out) <= 1.0)


def test_cnn_learns_a_tiny_batch():
    net = CNN(E2EPolicy.IN_H, E2EPolicy.IN_W, np.random.default_rng(0))
    opt = Adam(net.params(), lr=1e-3)
    x = np.random.randn(8, IN_CH, E2EPolicy.IN_H, E2EPolicy.IN_W).astype(np.float32)
    y = np.tanh(np.random.randn(8, 2) * 0.5).astype(np.float32)
    first = None
    for i in range(60):
        loss = net.train_step(x, y, opt)
        if i == 0:
            first = loss
    assert loss < first * 0.6, 'loss 应明显下降：%.4f -> %.4f' % (first, loss)


def test_e2e_policy_action_range():
    net = CNN(E2EPolicy.IN_H, E2EPolicy.IN_W, np.random.default_rng(0))
    pol = E2EPolicy(net)
    rgb = np.random.randint(0, 255, (P.CAM_H, P.CAM_W, 3), dtype=np.uint8)
    obs = {'rgb': rgb, 'true_pose': (0.0, 0.0, 0.0), 'goal': (10.0, 10.0)}
    v, w = pol.act(obs)
    assert 0.0 <= v <= P.MAX_V and abs(w) <= P.MAX_W


# ---------------------------------------------------------------- 目标方位通道
def test_goal_channel_encodes_bearing_side():
    """目标在左/右/正前，亮带应落在对应列上。"""
    h, w = E2EPolicy.IN_H, E2EPolicy.IN_W
    pose = (5.0, 5.0, 0.0)          # 朝 +x
    center = goal_channel(pose, (15.0, 5.0), w, h)
    left = goal_channel(pose, (15.0, 9.0), w, h)      # 目标偏左（y 增大 = 逆时针）
    right = goal_channel(pose, (15.0, 1.0), w, h)

    def argmax_col(img):
        return int(np.argmax(img.sum(axis=0)))

    c0, cl, cr = argmax_col(center), argmax_col(left), argmax_col(right)
    assert abs(c0 - (w - 1) / 2.0) <= 2.0, c0
    assert cl < c0 < cr, (cl, c0, cr)


def test_goal_channel_clips_out_of_view_and_scales_with_distance():
    h, w = E2EPolicy.IN_H, E2EPolicy.IN_W
    pose = (5.0, 5.0, 0.0)
    far_behind = goal_channel(pose, (-30.0, 5.0), w, h)   # 正后方
    near = goal_channel(pose, (7.0, 5.0), w, h)
    far = goal_channel(pose, (16.0, 5.0), w, h)
    assert far_behind[:, :2].sum() == 0.0          # 被压到最左列
    assert far_behind[:, -1].max() > 0.0
    assert near.max() > far.max()                  # 越近越亮


def test_goal_channel_is_zero_without_odom_or_goal():
    h, w = E2EPolicy.IN_H, E2EPolicy.IN_W
    assert goal_channel(None, (3.0, 3.0), w, h).max() == 0.0
    assert goal_channel((1.0, 1.0, 0.0), None, w, h).max() == 0.0


def test_e2e_input_shape_matches_network():
    net = CNN(E2EPolicy.IN_H, E2EPolicy.IN_W, np.random.default_rng(0))
    pol = E2EPolicy(net)
    rgb = np.random.randint(0, 255, (P.CAM_H, P.CAM_W, 3), dtype=np.uint8)
    depth = np.random.rand(P.CAM_H, P.CAM_W).astype(np.float32) * 5.0
    x = pol._input(rgb, depth, (1.0, 1.0, 0.0), (9.0, 3.0))
    assert x.shape == (IN_CH, E2EPolicy.IN_H, E2EPolicy.IN_W)
    assert x.shape[0] == E2EPolicy.IN_CH
    # 消融模式下目标通道整片为零，其余通道不变
    pol_ng = E2EPolicy(net, use_goal=False)
    x_ng = pol_ng._input(rgb, depth, (1.0, 1.0, 0.0), (9.0, 3.0))
    assert np.allclose(x_ng[:-1], x[:-1]) and x_ng[-1].max() == 0.0
    assert x[-1].max() > 0.0


def test_stack_dataset_appends_goal_channel():
    ep = {'frames': np.zeros((4, E2EPolicy.IN_H, E2EPolicy.IN_W), np.float32),
          'depths': np.zeros((4, E2EPolicy.IN_H, E2EPolicy.IN_W), np.float32),
          'odoms': np.tile(np.array([[1.0, 1.0, 0.0]]), (4, 1)),
          'goal': np.array([10.0, 1.0]),
          'actions': np.zeros((4, 2), np.float32)}
    X, Y = stack_dataset([ep])
    assert X.shape == (4, IN_CH, E2EPolicy.IN_H, E2EPolicy.IN_W)
    assert X[:, -1].max() > 0.0 and Y.shape == (4, 2)


# ---------------------------------------------------------------- 局部避障与恢复
def test_avoid_from_depth_fully_stops_at_close_obstacle():
    """贴到墙前必须把线速度压到 0，不能"再近也往前挪一点"。"""
    depth = np.full((P.CAM_H, P.CAM_W), 5.0, dtype=np.float32)
    v, _ = avoid_from_depth(depth, 0.5, 0.0)
    assert v == pytest.approx(0.5)          # 前方通畅时不干预

    depth[:, :] = 0.12                      # 贴脸
    v, w = avoid_from_depth(depth, 0.5, 0.0)
    assert v == 0.0
    assert abs(w) <= P.MAX_W


def test_recovery_prefers_rotation_and_checks_tail_clearance():
    pol = make_policy('oracle', make_scene('corridor', 0))

    pol._recover = 12                        # 恢复前段：只原地转向
    v, om = pol._recovery_cmd((2.0, 2.0, 0.0))
    assert v == 0.0 and om > 0.0

    pol._recover = 3                         # 空旷处允许低速后退
    v, _ = pol._recovery_cmd((2.0, 2.0, 0.0))
    assert v < 0.0

    pol._recover = 3                         # 车尾正对西墙：不许后退
    v, _ = pol._recovery_cmd((0.40, 2.0, 0.0))
    assert v == 0.0
