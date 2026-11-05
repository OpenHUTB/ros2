"""mujoco_slam_nav_sync 单元测试（无需 ROS，纯 Python）。"""

from __future__ import annotations

import numpy as np
import pytest

from slam_nav.env import DT, DiffDriveEnv, LIDAR_BEAMS, LIDAR_RANGE, wrap
from slam_nav.nav import (NavConfig, build_truth_grid, govern, inflate,
                          make_expert_dataset, run_episode)
from slam_nav.net import IN_DIM, LocalPlannerNet, features
from slam_nav.planner import astar, lookahead_points, pure_pursuit
from slam_nav.slam import (EKFSLAM, OccupancyGrid, detect_landmarks, GRID_H,
                           GRID_W, GRID_X0, GRID_Y0)


@pytest.fixture(scope="module")
def env():
    return DiffDriveEnv("maze")


# ---------------------------------------------------------------- 资源定位
def test_model_and_weights_are_findable():
    """装成 ROS 包后 models/ 在 share 下，源码运行时在模块目录，两条路都要能找到。"""
    from slam_nav.env import MODEL_PATH, _find_asset

    assert MODEL_PATH.exists() and MODEL_PATH.name == "arena.xml"
    w = _find_asset("planner.npz")
    assert w.exists(), "包内应带训练好的权重，避免节点用随机网络"


def test_env_override_for_assets(tmp_path, monkeypatch):
    from slam_nav.env import _find_asset

    (tmp_path / "x.txt").write_text("ok", encoding="utf-8")
    monkeypatch.setenv("MUJOCO_SLAM_MODELS", str(tmp_path))
    assert _find_asset("x.txt").read_text(encoding="utf-8") == "ok"


# ---------------------------------------------------------------- 环境与传感器
def test_reset_returns_start_pose(env):
    s = env.reset()
    assert np.allclose(s.pos, DiffDriveEnv.START, atol=0.05)
    assert abs(s.yaw) < 1e-6


def test_velocity_servo_tracks_command(env):
    env.reset()
    for _ in range(int(2.0 / DT)):
        env.step(0.5, 0.0)
    s = env.state()
    assert 0.75 < float(np.linalg.norm(s.pos - DiffDriveEnv.START)) < 1.15


def test_yaw_tracks_angular_command(env):
    env.reset()
    for _ in range(int(1.5 / DT)):
        env.step(0.0, 0.6)
    assert 0.5 < env.state().yaw < 1.3


def test_scan_shape_and_range_bounds(env):
    env.reset()
    scan = env.scan()
    assert scan.shape == (LIDAR_BEAMS,)
    assert scan.min() > 0.0 and scan.max() <= LIDAR_RANGE + 1e-6


def test_scan_sees_front_wall(env):
    """起点 (0.8,0.8) 朝 +x，正前方 3.2 m 处是 x=4 的隔断。"""
    env.reset()
    scan = env.scan()
    front = min(float(scan[0]), float(scan[1]), float(scan[-1]))
    assert 0.5 < front < 4.0


def test_crash_detected_when_driving_into_wall(env):
    env.reset(pos=(3.2, 0.0), yaw=0.0)
    for _ in range(int(3.0 / DT)):
        env.step(0.6, 0.0)
        if env.crashed:
            break
    assert env.crashed


def test_odom_applies_slip_and_noise(env):
    env.reset()
    v, w = env.odom(0.5, 0.0, slip=0.10, noise=0.0)
    assert abs(v - 0.55) < 1e-6
    v2, _ = env.odom(0.5, 0.0, slip=0.0, noise=0.0)
    assert abs(v2 - 0.5) < 1e-6


# ---------------------------------------------------------------- 路标检测
def test_landmark_detection_matches_truth(env):
    env.reset()
    dets = detect_landmarks(env.scan())
    assert len(dets) >= 2, "起点应至少看到两个柱子"
    truth = env.landmarks()
    hits = 0
    for r, b in dets:
        px, py = DiffDriveEnv.START[0] + r * np.cos(b), DiffDriveEnv.START[1] + r * np.sin(b)
        if np.min(np.linalg.norm(truth - np.array([px, py]), axis=1)) < 0.45:
            hits += 1
    assert hits >= 2


def test_landmark_detection_angle_convention(env):
    """给一条只打到正前方柱子的假扫描，方位应接近 0。"""
    fake = np.full(LIDAR_BEAMS, LIDAR_RANGE, dtype=np.float64)
    k0 = 2
    for k in (k0 - 1, k0, k0 + 1):
        fake[k] = 2.0
    dets = detect_landmarks(fake)
    assert dets and abs(dets[0][1]) < 0.2 and abs(dets[0][0] - 2.0) < 0.3


# ---------------------------------------------------------------- EKF-SLAM
def test_ekf_predict_moves_pose_forward():
    ekf = EKFSLAM(init_pose=(1.0, 1.0, 0.0))
    for _ in range(20):
        ekf.predict(0.5, 0.0, DT)
    x, y, _ = ekf.pose
    assert x > 1.4 and abs(y - 1.0) < 0.05


def test_ekf_landmark_init_and_update_reduces_error():
    ekf = EKFSLAM(init_pose=(0.0, 0.0, 0.0))
    for _ in range(5):                      # 先观测到 (2,0) 附近的柱子
        ekf.observe([(2.0, 0.0)])
    idx = int(np.argmax(ekf.seen))
    est = ekf.landmarks_estimate()[idx]
    assert np.linalg.norm(est - np.array([2.0, 0.0])) < 0.4
    before = float(np.trace(ekf.Sigma[3 + 2 * idx:5 + 2 * idx, 3 + 2 * idx:5 + 2 * idx]))
    ekf.observe([(2.05, 0.02)])
    after = float(np.trace(ekf.Sigma[3 + 2 * idx:5 + 2 * idx, 3 + 2 * idx:5 + 2 * idx]))
    assert after <= before + 1e-9


def test_ekf_treats_far_observation_as_new_landmark():
    """远处的观测不该把已有路标/位姿拉走，而应初始化为新路标。"""
    ekf = EKFSLAM(init_pose=(0.0, 0.0, 0.0))
    for _ in range(3):
        ekf.observe([(2.0, 0.0)])
    pose_before = ekf.pose.copy()
    seen_before = int(ekf.seen[:].sum())
    ekf.observe([(9.0, 0.5)])               # 远处另一个物体
    assert int(ekf.seen[:].sum()) == seen_before + 1
    assert np.linalg.norm(ekf.pose[:2] - pose_before[:2]) < 0.05


def test_ekf_covariance_stays_symmetric():
    ekf = EKFSLAM(init_pose=(0.5, 0.5, 0.0))
    for _ in range(10):
        ekf.predict(0.4, 0.1, DT)
        ekf.observe([(3.0, 0.3)])
    assert np.allclose(ekf.Sigma, ekf.Sigma.T, atol=1e-9)


# ---------------------------------------------------------------- 占据栅格
def test_grid_update_marks_hit_cells(env):
    env.reset()
    grid = OccupancyGrid()
    scan = env.scan()
    grid.update((0.8, 0.8, 0.0), scan)
    assert (grid.logodds > 0.5).sum() > 10
    assert grid.known_ratio() > 0.05


def test_grid_known_ratio_grows(env):
    env.reset()
    grid = OccupancyGrid()
    r1 = grid.known_ratio()
    for k in range(20):
        grid.update((0.8 + 0.1 * k, 0.8, 0.0), env.scan())
    assert grid.known_ratio() > r1


def test_grid_dimensions_cover_arena():
    grid = OccupancyGrid()
    assert (grid.w, grid.h) == (GRID_W, GRID_H)
    cx, cy = grid.world_to_cell(11.0, -4.8)
    assert 0 <= cx < grid.w and 0 <= cy < grid.h
    assert abs(grid.cell_to_world(cx, cy)[0] - 11.0) < 0.1


# ---------------------------------------------------------------- 规划
def test_astar_straight_line_on_empty_grid():
    occ = np.zeros((GRID_H, GRID_W), dtype=bool)
    path = astar(occ, (1.0, 1.0), (5.0, 1.0))
    assert path is not None
    assert abs(sum(np.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:])) - 4.0) < 0.25


def test_astar_detours_around_wall():
    occ = np.zeros((GRID_H, GRID_W), dtype=bool)
    cx = int((4.0 - GRID_X0) / 0.1)
    cy0 = int((-3.0 - GRID_Y0) / 0.1)
    occ[cy0:cy0 + 40, cx] = True            # 一堵高 4 m 的墙
    path = astar(occ, (1.0, 0.0), (7.0, 0.0))
    assert path is not None
    cxw = int((4.0 - GRID_X0) / 0.1)
    for p in path:                          # 路径不得落在墙格上
        assert not occ[int((p[1] - GRID_Y0) / 0.1), int((p[0] - GRID_X0) / 0.1)]
    assert max(abs(p[1]) for p in path) > 1.0, "应当绕过墙端"


def test_astar_returns_none_when_sealed():
    occ = np.zeros((GRID_H, GRID_W), dtype=bool)
    cx = int((4.0 - GRID_X0) / 0.1)
    occ[:, cx] = True                       # 整幅图竖墙
    assert astar(occ, (1.0, 0.0), (7.0, 0.0)) is None


def test_pure_pursuit_turns_toward_target():
    v, w = pure_pursuit((0.0, 0.0, 0.0), (1.0, 1.0), v_max=0.7, w_max=1.5)
    assert w > 0 and 0 < v <= 0.7
    v2, w2 = pure_pursuit((0.0, 0.0, 0.0), (1.0, -1.0), v_max=0.7, w_max=1.5)
    assert w2 < 0


def test_lookahead_points_are_ordered():
    path = [(1.0 * i, 0.0) for i in range(1, 12)]
    pts = lookahead_points(path, (0.0, 0.0, 0.0), ld=0.9, n=3)
    d = [np.hypot(p[0], p[1]) for p in pts]
    assert len(pts) == 3 and d[0] <= d[1] <= d[2]


def test_governor_stops_toward_wall():
    scan = np.full(LIDAR_BEAMS, LIDAR_RANGE)
    k = 3
    scan[:k] = 0.3                          # 正前方很近
    v, w = govern(scan, 0.6, 0.0, NavConfig())
    assert v == 0.0 and abs(w) > 0.3


# ---------------------------------------------------------------- 神经网络
def test_features_dimension_and_scale(env):
    env.reset()
    f = features(env.scan(), (0.8, 0.8, 0.0), [(2.0, 0.8), (3.0, 0.8)], (11.0, -4.8))
    assert f.shape == (IN_DIM,)
    assert np.all(np.isfinite(f))


def test_net_forward_shapes():
    net = LocalPlannerNet()
    x = np.zeros((4, IN_DIM), dtype=np.float32)
    out, _ = net.forward(x)
    assert out.shape == (4, 2)
    assert np.all((out[:, 0] >= 0) & (out[:, 0] <= 1))
    assert np.all(np.abs(out[:, 1]) <= 1)


def test_net_gradient_matches_numeric():
    net = LocalPlannerNet(seed=1)
    rng = np.random.default_rng(0)
    x = rng.normal(0, 0.5, (6, IN_DIM)).astype(np.float32)
    y = rng.uniform(0, 1, (6, 2)).astype(np.float32)
    _, cache = net.forward(x)
    grads = net.backward(cache, y)
    key, idx = "w2", (3, 5)
    eps = 1e-4
    old = float(net.p[key][idx])
    net.p[key][idx] = old + eps
    lp = float(np.mean((net.forward(x)[0] - y) ** 2))
    net.p[key][idx] = old - eps
    lm = float(np.mean((net.forward(x)[0] - y) ** 2))
    net.p[key][idx] = old
    numeric = (lp - lm) / (2 * eps)
    assert abs(numeric - float(grads[key][idx])) < 2e-2 * max(1.0, abs(numeric))


def test_net_save_load_roundtrip(tmp_path):
    net = LocalPlannerNet(seed=2)
    x = np.random.default_rng(3).normal(0, 0.4, (3, IN_DIM)).astype(np.float32)
    before, _ = net.forward(x)
    p = net.save(tmp_path / "w.npz")
    other = LocalPlannerNet(seed=9).load(p)
    after, _ = other.forward(x)
    assert np.allclose(before, after, atol=1e-6)


# ---------------------------------------------------------------- 闭环与流程
def test_expert_dataset_restores_env(env):
    env.reset("maze", 0)
    pose0 = env.state().pos.copy()
    make_expert_dataset(env, n=20, seed=0, log=lambda *_: None)
    assert np.allclose(env.state().pos, pose0, atol=1e-9)
    assert env.scene == "maze"


def test_run_episode_is_deterministic(env):
    cfg = NavConfig(steps=120)
    a = run_episode(env, use_slam=True, slip=0.05, seed=3, cfg=cfg)
    b = run_episode(env, use_slam=True, slip=0.05, seed=3, cfg=cfg)
    assert a.steps == b.steps and abs(a.ate_rmse - b.ate_rmse) < 1e-9


def test_slam_beats_dead_reckoning_under_drift(env):
    cfg = NavConfig(steps=900)
    slam = run_episode(env, use_slam=True, slip=0.20, seed=0, cfg=cfg)
    dead = run_episode(env, use_slam=False, slip=0.20, seed=0, cfg=cfg)
    # 死推算可能很早就撞墙，ATE 反而"看起来小"；比终点误差才有意义
    assert slam.final_dist < dead.final_dist
    assert not slam.crashed
