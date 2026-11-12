# -*- coding: utf-8 -*-
"""离线单元测试：几何、渲染、检测、SLAM、建图、规划、控制、网络梯度。

全部不依赖 ROS，任何装了 numpy 的机器都能跑：
    python -m pytest test/test_offline.py -q
"""

import math

import numpy as np
import pytest

from rov_pipeline_follow import camera, detector, params as P
from rov_pipeline_follow.cnn import Adam, CNN
from rov_pipeline_follow.controllers import (PipeFollower, follow_control,
                                             goal_reached, pure_pursuit,
                                             safety_from_depth)
from rov_pipeline_follow.gridmap import GridMap, pipeline_mask, truth_grid
from rov_pipeline_follow.kinematics import RovDrive, integrate, wrap
from rov_pipeline_follow.planner import astar, inflate, proximity_cost
from rov_pipeline_follow.policies import (E2EPolicy, PERCEP_IN_CH,
                                          goal_channel, make_policy,
                                          mission_errors)
from rov_pipeline_follow.slam import EKFSLAM
from rov_pipeline_follow.sim import rollout, stack_dataset, stack_percep
from rov_pipeline_follow.world import make_scene


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


def test_integrate_adds_current_but_odom_does_not():
    """海流漂移进真值、不进里程计 —— 无 DVL ROV 的核心设定。"""
    rng = np.random.default_rng(0)
    rov = RovDrive([2.0, 2.0, 0.0])
    for _ in range(50):
        rov.step(0.0, 0.0, P.DT, rng, current=(0.15, 0.0))
    assert rov.pose[0] > 2.5, '静止 ROV 应被海流推走'
    assert rov.odom[0] == pytest.approx(2.0, abs=1e-6), '里程计测不到海流'


def test_wrap_range():
    for a in (-7.0, -math.pi, 0.0, 3.0, 9.0):
        assert -math.pi - 1e-9 <= wrap(a) <= math.pi + 1e-9


# ------------------------------------------------------------------ 光线投射
def test_raycast_hits_pipe_ahead():
    w = make_scene('straight', 0)
    # 站在偏置路径上朝 +x，右前方 70° 的射线应打在管道上
    ang = w.start[2] - math.radians(40.0)
    dist, hid = w.raycast(w.start[:2], np.array([ang]))
    n_lm = len(w.landmarks)
    assert hid[0] == n_lm + 1, '应命中管道'
    assert 0.5 < dist[0] < 6.0


def test_raycast_hits_landmark():
    w = make_scene('straight', 0)
    lx, ly = w.landmarks[2][:2]
    origin = (lx - 1.0, ly)
    dist, hid = w.raycast(origin, np.array([0.0]))
    assert hid[0] == 3, '应命中 2 号应答器（hid=索引+1）'
    assert dist[0] == pytest.approx(1.0 - P.LM_RADIUS, abs=0.03)


def test_raycast_returns_max_range_when_clear():
    w = make_scene('straight', 0)
    dist, hid = w.raycast((2.0, 2.0), np.array([math.pi * 0.75]), max_range=3.0)
    assert dist[0] <= 3.0 + 1e-9


def test_point_free_rejects_pipe_and_landmark():
    w = make_scene('straight', 0)
    px, py = w.pipeline[len(w.pipeline) // 2]
    assert not w.point_free(px, py, margin=0.0), '管道内部不是自由空间'
    cx, cy = w.landmarks[0][:2]
    assert not w.point_free(cx, cy, margin=0.0)
    assert w.point_free(cx + 0.5, cy + 0.5, margin=0.0)


# ------------------------------------------------------------------ 相机
def test_render_shapes_and_depth_range():
    w = make_scene('straight', 0)
    rgb, depth = camera.render(w, w.start, np.random.default_rng(0))
    assert rgb.shape == (P.CAM_H, P.CAM_W, 3) and rgb.dtype == np.uint8
    assert depth.shape == (P.CAM_H, P.CAM_W)
    assert depth.min() > 0.0 and depth.max() <= P.CAM_RANGE_MAX + 1e-3


def test_render_turbid_is_dimmer_and_foggier():
    w = make_scene('turbid', 0)
    clear, _ = camera.render(w, w.start, np.random.default_rng(1), turbid=False)
    murky, _ = camera.render(w, w.start, np.random.default_rng(1), turbid=True)
    assert murky.max() <= clear.max() + 1e-6
    assert float(murky.std()) > 0.0


# ------------------------------------------------------------------ 检测
def test_detect_finds_marker_near_start():
    w = make_scene('straight', 0)
    rgb, depth = camera.render(w, w.start, np.random.default_rng(0))
    lms, pipe = detector.detect(rgb, depth)
    assert len(pipe) > 10, '管道点应覆盖右侧视场'
    for lid, rng_m, brg in lms:
        lx, ly = w.landmarks[lid][:2]
        truth = math.hypot(lx - w.start[0], ly - w.start[1])
        assert rng_m <= 8.0, '观测距离不应超过上限'
        assert rng_m == pytest.approx(truth, abs=0.4)


def test_detect_rejects_scattered_noise():
    """强噪声帧上不应产生超远假标记（曾经毒化过 EKF 初始化）。"""
    w = make_scene('straight', 0)
    for seed in range(6):
        rgb, depth = camera.render(w, (8.0, 8.9, 0.0),
                                   np.random.default_rng(seed))
        lms, _ = detector.detect(rgb, depth)
        for lid, rng_m, _b in lms:
            lx, ly = w.landmarks[lid][:2]
            truth = math.hypot(lx - 8.0, ly - 8.9)
            assert rng_m < truth + 1.5, \
                'seed %d: id%d 观测距离 %.1f 偏离真值 %.1f 太远' % (
                    seed, lid, rng_m, truth)


def test_pipe_mask_ignores_red_marker():
    """管道橙掩码不能把红色应答器当成管道（早期版本的真实 bug）。"""
    rgb = np.zeros((8, 8, 3), dtype=np.uint8)
    rgb[:, :] = (int(0.85 * 255), int(0.15 * 255), int(0.10 * 255))  # 红
    assert not detector.pipe_mask(rgb).any()
    rgb[:, :] = (int(0.98 * 255), int(0.48 * 255), int(0.05 * 255))  # 管道橙
    assert detector.pipe_mask(rgb).all()


def test_label_map_ignores_water_background():
    w = make_scene('straight', 0)
    rgb, _ = camera.render(w, (8.0, 8.9, 0.0), np.random.default_rng(0))
    lab = detector.label_map(rgb)
    hits = int((lab >= 0).sum())
    assert 0 < hits < rgb.shape[0] * rgb.shape[1] * 0.25, \
        '标记像素不该超过全图四成（水背景被误分类时会是整幅图）'


def test_pipeline_observation_tracks_offset():
    """沿偏置路径走，几何感知应给出接近零的横向/艏向误差。"""
    w = make_scene('straight', 0)
    from rov_pipeline_follow.world import _left_normal
    for frac in (0.3, 0.5, 0.7):
        i = int(frac * (len(w.pipeline) - 1))
        n = _left_normal(w.pipeline)[i]
        pose = np.array([w.pipeline[i][0] + n[0] * P.FOLLOW_OFFSET,
                         w.pipeline[i][1] + n[1] * P.FOLLOW_OFFSET,
                         math.atan2(w.pipeline[i + 1][1] - w.pipeline[i][1],
                                    w.pipeline[i + 1][0] - w.pipeline[i][0])])
        rgb, depth = camera.render(w, pose, np.random.default_rng(3))
        found, dx, dth = detector.pipeline_observation(rgb, depth)
        assert found, 'frac=%.1f 时应能看到管道' % frac
        assert abs(dx) < 0.35 and abs(dth) < 0.35, (frac, dx, dth)


# ------------------------------------------------------------------ SLAM
def test_ekf_estimate_is_accurate_with_consistent_prior():
    truth_pose = np.array([5.0, 5.0, 0.30])
    lms = [(0, (8.0, 5.0)), (1, (6.0, 7.0)), (2, (3.5, 6.5))]
    slam = EKFSLAM(truth_pose)

    for _ in range(25):
        obs = []
        for lid, (mx, my) in lms:
            dx, dy = mx - truth_pose[0], my - truth_pose[1]
            obs.append((lid, math.hypot(dx, dy),
                        wrap(math.atan2(dy, dx) - truth_pose[2])))
        slam.update(obs)

    err = math.hypot(slam.mu[0] - truth_pose[0], slam.mu[1] - truth_pose[1])
    assert err < 0.02
    assert abs(wrap(slam.mu[2] - truth_pose[2])) < 0.02
    assert len(slam.ids) == 3


def test_ekf_rejects_outlier_measurement():
    slam = EKFSLAM(np.array([5.0, 5.0, 0.0]))
    slam.update([(0, 3.0, 0.0)])
    before = slam.mu.copy()
    slam.update([(0, 30.0, 1.5)])
    assert np.allclose(slam.mu, before, atol=1e-9)


# ------------------------------------------------------------------ 建图
def test_gridmap_updates_and_scores():
    w = make_scene('straight', 0)
    rgb, depth = camera.render(w, w.start, np.random.default_rng(0))
    _, pipe = detector.detect(rgb, depth)
    gm = GridMap()
    for _ in range(3):
        gm.update(w.start, pipe)
    occ, explored = gm.counts()
    assert explored > 200
    assert 0.0 < gm.iou(truth_grid(w)) < 1.0


def test_pipeline_mask_is_capsule_not_centerline():
    """管道真值必须是胶囊：建图打到的是表面点，只比中心线会全算漏。"""
    w = make_scene('straight', 0)
    pm = pipeline_mask(w)
    n = int(pm.sum())
    center = int(len(w.pipeline))
    assert n > center, '胶囊覆盖应明显多于中心线格子数（%d vs %d）' % (n, center)


# ------------------------------------------------------------------ 规划
def test_astar_returns_collision_free_path():
    w = make_scene('straight', 0)
    blocked = inflate(truth_grid(w), 5)
    start = (w.start[0], w.start[1])
    path = astar(blocked, start, (20.0, 10.0))
    assert len(path) > 20
    for x, y in path:
        c = int(x / P.GRID_RES)
        r = int(y / P.GRID_RES)
        assert not blocked[r, c], '路径点落在障碍上：(%.2f, %.2f)' % (x, y)


def test_astar_unreachable_returns_empty():
    blocked = np.zeros((P.GH, P.GW), dtype=bool)
    blocked[:, P.GW // 2] = True
    path = astar(blocked, (1.0, 7.0), (P.WORLD_W - 1.0, 7.0))
    assert path == []


def test_proximity_cost_prefers_middle():
    occ = np.zeros((P.GH, P.GW), dtype=bool)
    occ[0, :] = True
    c = proximity_cost(occ)
    assert c[1, 10] > c[5, 10] > c[30, 10]


# ------------------------------------------------------------------ 控制
def test_follow_control_converges_heading():
    v, w = follow_control(0.0, 0.4)
    assert v > 0.2 and 0.0 < w <= P.MAX_W


def test_follow_control_spins_on_large_error():
    """折返 U 转弯必须原地回转：v=0。"""
    v, w = follow_control(0.0, 2.8)
    assert v == 0.0 and abs(w) > 0.5


def test_follower_integral_cancels_bias():
    """横向积分项应朝误差反方向累积（抗海流稳态偏差）。"""
    f = PipeFollower()
    for _ in range(50):
        f.step(0.4, 0.0)
    assert f.i_lat > 0.05, '持续正的横向误差应积累正的积分项'


def test_follower_turnaround_overrides():
    f = PipeFollower()
    f.begin_turnaround(-1.0)
    v, w = f.step(0.0, 2.5)     # 艏向误差还很大
    assert v == 0.0 and w < 0.0
    v, w = f.step(0.0, 0.1)     # 转到位后交还正常控制
    assert v > 0.0


def test_pure_pursuit_straight_has_no_steering():
    path = [(1.0, 0.0), (3.0, 0.0), (6.0, 0.0)]
    v, w = pure_pursuit((0.0, 0.0, 0.0), path)
    assert v > 0.2 and abs(w) < 0.05


def test_pure_pursuit_turns_toward_path():
    path = [(1.0, 1.0), (2.0, 2.0), (4.0, 4.0)]
    _v, w = pure_pursuit((0.0, 0.0, 0.0), path)
    assert w > 0.2


def test_safety_from_depth_stops_when_close():
    depth = np.full((P.CAM_H, P.CAM_W), 0.12, dtype=np.float32)
    v, w = safety_from_depth(depth, 0.5, 0.0)
    assert v == pytest.approx(0.0, abs=1e-6)
    depth = np.full((P.CAM_H, P.CAM_W), 5.0, dtype=np.float32)
    v, w = safety_from_depth(depth, 0.5, 0.1)
    assert v == pytest.approx(0.5) and w == pytest.approx(0.1)


def test_mission_errors_flip_sign_on_return():
    """返程时横向误差必须换算符号 —— 这是修过的最关键的 bug。"""
    w = make_scene('straight', 0)
    x = w.pipeline[len(w.pipeline) // 2]
    too_close = (x[0], x[1] + P.FOLLOW_OFFSET - 0.4)   # 比目标偏置更靠管
    w.phase = 'go'
    dx_go, _ = mission_errors(w, too_close[0], too_close[1], 0.0)
    w.phase = 'return'
    dx_ret, _ = mission_errors(w, too_close[0], too_close[1], math.pi)
    assert dx_go > 0.0, '去程太近应给正误差（向左挪）'
    assert dx_ret < 0.0, '返程同一个物理位置符号必须翻转'


# ------------------------------------------------------------------ 场景
def test_start_pose_offset_keeps_guide_distance():
    """起始扰动只改横向偏置，仍落在巡检线上（0.9 m ± dlat）。"""
    w = make_scene('straight', 0)
    for dlat in (-0.55, 0.0, 0.55):
        p = w.start_pose(dlat)
        _s, lat, _t = w.progress_of(p[0], p[1])
        assert abs(lat - (P.FOLLOW_OFFSET + dlat)) < 1e-6


def test_jittered_demo_covers_off_nominal_states():
    """加了起始扰动的专家数据必须真的包含"偏了"的样本。

    不加扰动时横向误差几乎恒为 0，感知网络会均值坍缩成恒输出 0；
    部署时偏 1.8 m 也报没偏，ROV 按直行飞然后从管口扫出去。
    """
    r = rollout('straight', make_policy('oracle', make_scene('straight', 0)),
                seed=0, record=True, start_offset=(0.55, 0.0), max_steps=150)
    lab = np.asarray(r['percep_labels'])
    assert len(lab) > 20
    assert np.max(np.abs(lab[:, 0])) > 0.2, '起始扰动没有体现在横向标签里'


@pytest.mark.parametrize('scene', P.SCENES)
def test_all_scenes_are_solvable(scene):
    from rov_pipeline_follow.world import _reachable
    assert _reachable(make_scene(scene, 0)), '%s 场景在真值上不可达' % scene


def test_world_phase_advances_by_arclength():
    w = make_scene('straight', 0)
    assert w.phase == 'go'
    w.advance((w.end_b[0], w.end_b[1]))
    assert w.phase == 'return', '到 B 端应翻转阶段'
    w.advance((w.end_a[0], w.end_a[1]))
    assert w.done, '回到 A 端任务完成'


# ------------------------------------------------------------------ 闭环冒烟
def test_policy_names_are_distinct():
    """五种策略的 name 必须互不相同。

    pipe_slam 与 pipe_cnn 共用同一个实现类，早先实例上没有覆盖类属性
    name，评测 CSV 把两条支路混成了一行（"pipe_slam" 每场景 6 条），
    图上看起来像样本翻倍、实际是少了一条对照曲线。
    """
    net = CNN(E2EPolicy.IN_H, E2EPolicy.IN_W, np.random.default_rng(0))
    percep = CNN(E2EPolicy.IN_H, E2EPolicy.IN_W,
                 np.random.default_rng(1), in_ch=PERCEP_IN_CH)
    names = []
    for kind in ('oracle', 'pipe_slam', 'pipe_cnn', 'e2e', 'e2e_nogoal'):
        pol = make_policy(kind, make_scene('straight', 0), net=net,
                          percep_net=percep)
        names.append(pol.name)
    assert sorted(names) == sorted(['oracle', 'pipe_slam', 'pipe_cnn',
                                    'e2e', 'e2e_nogoal'])
    assert len(set(names)) == 5


def test_deploy_input_matches_training_tensor():
    """部署端的输入张成必须与训练集逐位一致。

    这条契约找不到时踩过两次坑：端到端漏了 net_gray() 归一化（原地打转
    12/12 失败），感知网络多做了同一次归一化（输出饱和、位移为零）。
    修法不是"改一边"，而是让训练与部署共用 frame_gray/net_gray 两个函数。
    """
    from rov_pipeline_follow.policies import (_percep_input, frame_gray,
                                              net_gray)
    w = make_scene('straight', 0)
    rov = RovDrive(w.start)
    rgb, depth = camera.render(w, rov.pose, np.random.default_rng(0))

    pol = E2EPolicy(CNN(E2EPolicy.IN_H, E2EPolicy.IN_W,
                        np.random.default_rng(0)))
    x = pol._input(rgb, depth, rov.odom, w.goal)
    assert np.allclose(x[0], net_gray(frame_gray(rgb))), '端到端灰度通道未归一化'

    xp = _percep_input(rgb, depth)
    assert np.allclose(xp[0], frame_gray(rgb)), '感知网络不该做亮度归一化'


def test_oracle_completes_straight_scene():
    """专家策略必须能跑完一个完整往返 —— 上游一切的地基。"""
    rows = []
    r = rollout('straight', make_policy('oracle', make_scene('straight', 0)),
                seed=0)
    rows.append(r)
    assert r['success'], 'oracle 直管场景应成功'
    assert r['lat_rmse'] < P.LAT_OK


# ------------------------------------------------------------------ 网络
def test_cnn_conv_gradient_matches_numeric():
    from rov_pipeline_follow.cnn import Conv

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
    analytic = float(conv.gW[0, 4])
    assert numeric == pytest.approx(analytic, rel=0.05), (numeric, analytic)


def test_cnn_output_is_bounded_and_supports_2ch():
    net = CNN(E2EPolicy.IN_H, E2EPolicy.IN_W, np.random.default_rng(0),
              in_ch=PERCEP_IN_CH)
    x = np.random.randn(4, PERCEP_IN_CH, E2EPolicy.IN_H, E2EPolicy.IN_W)
    out = net.forward(x, train=False)
    assert out.shape == (4, 2)
    assert np.all(np.abs(out) <= 1.0)


def test_cnn_learns_a_tiny_batch():
    net = CNN(E2EPolicy.IN_H, E2EPolicy.IN_W, np.random.default_rng(0))
    opt = Adam(net.params(), lr=1e-3)
    x = np.random.randn(8, E2EPolicy.IN_CH, E2EPolicy.IN_H,
                        E2EPolicy.IN_W).astype(np.float32)
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
    h, w = E2EPolicy.IN_H, E2EPolicy.IN_W
    pose = (5.0, 5.0, 0.0)
    center = goal_channel(pose, (15.0, 5.0), w, h)
    left = goal_channel(pose, (15.0, 9.0), w, h)
    right = goal_channel(pose, (15.0, 1.0), w, h)

    def argmax_col(img):
        return int(np.argmax(img.sum(axis=0)))

    c0, cl, cr = argmax_col(center), argmax_col(left), argmax_col(right)
    assert abs(c0 - (w - 1) / 2.0) <= 2.0, c0
    assert cl < c0 < cr, (cl, c0, cr)


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
    assert x.shape == (E2EPolicy.IN_CH, E2EPolicy.IN_H, E2EPolicy.IN_W)
    pol_ng = E2EPolicy(net, use_goal=False)
    x_ng = pol_ng._input(rgb, depth, (1.0, 1.0, 0.0), (9.0, 3.0))
    assert np.allclose(x_ng[:-1], x[:-1]) and x_ng[-1].max() == 0.0
    assert x[-1].max() > 0.0


def test_stack_dataset_appends_goal_channel():
    ep = {'frames': np.zeros((4, E2EPolicy.IN_H, E2EPolicy.IN_W), np.float32),
          'depths': np.zeros((4, E2EPolicy.IN_H, E2EPolicy.IN_W), np.float32),
          'odoms': np.tile(np.array([[1.0, 1.0, 0.0]]), (4, 1)),
          'goals': np.tile(np.array([[10.0, 1.0]]), (4, 1)),
          'actions': np.zeros((4, 2), np.float32)}
    X, Y = stack_dataset([ep])
    assert X.shape == (4, E2EPolicy.IN_CH, E2EPolicy.IN_H, E2EPolicy.IN_W)
    assert X[:, -1].max() > 0.0 and Y.shape == (4, 2)


def test_stack_percep_pairs():
    ep = {'frames': np.zeros((4, E2EPolicy.IN_H, E2EPolicy.IN_W), np.float32),
          'depths': np.zeros((4, E2EPolicy.IN_H, E2EPolicy.IN_W), np.float32),
          'percep_labels': np.tile(np.array([[0.1, -0.2]]), (4, 1))}
    X, Y = stack_percep([ep])
    assert X.shape == (4, PERCEP_IN_CH, E2EPolicy.IN_H, E2EPolicy.IN_W)
    assert np.allclose(Y, np.tile([0.1, -0.2], (4, 1)))
