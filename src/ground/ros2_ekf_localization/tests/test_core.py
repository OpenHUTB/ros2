"""核心算法单元测试。

运行：
    python3 -m pytest tests -v
或：
    python3 tests/test_core.py
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from ros2_ekf_localization.ekf import EKFConfig, EKFLocalizer, normalize_angle  # noqa: E402
from ros2_ekf_localization.safety import (  # noqa: E402
    FreshnessTracker,
    repair_covariance,
    sanitize_sigma,
)
from ros2_ekf_localization.sensors import (  # noqa: E402
    ImuNoise,
    OdometryNoise,
    PositionSensor,
    integrate_odometry,
)
from ros2_ekf_localization.trajectories import TRAJECTORY_BUILDERS  # noqa: E402


# --------------------------------------------------------------------- 基础

def test_normalize_angle_wraps_to_pm_pi():
    """角度必须折算到 (-pi, pi]，跨 ±pi 时残差不能"绕远路"。"""
    # 归一化到 (-pi, pi]，±pi 端点两种符号都是合法表示，故按绝对值判定
    assert abs(abs(normalize_angle(3.0 * math.pi)) - math.pi) < 1e-9
    assert abs(abs(normalize_angle(-3.0 * math.pi)) - math.pi) < 1e-9
    assert abs(normalize_angle(0.5) - 0.5) < 1e-12
    assert abs(normalize_angle(2.0 * math.pi - 0.1) + 0.1) < 1e-9


def test_predict_matches_differential_drive_kinematics():
    """一步预测应与差速运动学解析解一致。"""
    ekf = EKFLocalizer(EKFConfig(sigma_v=0.0, sigma_w=0.0), x0=np.array([1.0, 2.0, 0.5]))
    ekf.predict(0.8, 0.3, 0.1)
    th = 0.5
    assert abs(ekf.x[0] - (1.0 + 0.8 * math.cos(th) * 0.1)) < 1e-12
    assert abs(ekf.x[1] - (2.0 + 0.8 * math.sin(th) * 0.1)) < 1e-12
    assert abs(normalize_angle(ekf.x[2] - (th + 0.3 * 0.1))) < 1e-12


def test_process_jacobian_matches_numeric():
    """回归：P 的传播必须使用与运动学一致的解析雅可比 F。

    用差商从 predict 的输出构造数值雅可比，再与 P_{k+1} = F P F^T（Q=0）比对。
    """
    v, w, dt = 0.8, 0.3, 0.1
    x0 = np.array([1.0, 2.0, 0.5])

    def step_once(x):
        e = EKFLocalizer(EKFConfig(sigma_v=0.0, sigma_w=0.0), x0=x.copy(), P0=np.eye(3))
        e.predict(v, w, dt)
        return e.x.copy()

    eps = 1e-6
    F_num = np.zeros((3, 3))
    for j in range(3):
        dx = np.zeros(3)
        dx[j] = eps
        F_num[:, j] = (step_once(x0 + dx) - step_once(x0)) / eps

    ekf = EKFLocalizer(EKFConfig(sigma_v=0.0, sigma_w=0.0), x0=x0.copy(), P0=np.eye(3))
    ekf.predict(v, w, dt)
    expected = F_num @ np.eye(3) @ F_num.T
    assert np.allclose(ekf.P, expected, atol=1e-5)


def test_trajectory_is_constant_speed_and_closed_form_v():
    """等弧长重采样后，反推出的线速度应恒等于设定速度。"""
    traj = TRAJECTORY_BUILDERS["circle"](radius=8.0, speed=0.8, dt=0.05)
    assert np.allclose(traj.v[:-1], 0.8, atol=1e-3)
    assert len(traj) > 100


def test_noiseless_odometry_reproduces_ground_truth():
    """无噪声时，航位推算积分应严格复现真值轨迹（验证实验设计本身没问题）。"""
    traj = TRAJECTORY_BUILDERS["eight"](speed=0.8, dt=0.05)
    poses = integrate_odometry(traj.v, traj.w, traj.dt, traj.state(0))
    err = math.hypot(poses[-1, 0] - traj.x[-1], poses[-1, 1] - traj.y[-1])
    assert err < 1e-2


# --------------------------------------------------------------------- 滤波行为

def test_predict_increases_covariance():
    """预测步引入过程噪声，协方差迹必须增大。"""
    ekf = EKFLocalizer(EKFConfig(sigma_v=0.15, sigma_w=0.15), P0=np.eye(3) * 1e-3)
    before = ekf.trace_cov()
    ekf.predict(0.8, 0.1, 0.05)
    assert ekf.trace_cov() > before


def test_update_decreases_covariance():
    """有效观测必须降低不确定度。"""
    ekf = EKFLocalizer(EKFConfig(), P0=np.eye(3) * 0.5)
    ekf.predict(0.8, 0.1, 0.05)
    after_predict = ekf.trace_cov()
    assert ekf.update_yaw(0.0)
    assert ekf.trace_cov() < after_predict


def test_covariance_stays_symmetric_after_many_updates():
    """连续上百次预测/更新后，协方差仍应保持对称。"""
    rng = np.random.default_rng(0)
    ekf = EKFLocalizer(EKFConfig())
    for k in range(300):
        ekf.predict(0.8, 0.2 * math.sin(k * 0.05), 0.05)
        ekf.update_yaw(0.1 + rng.normal(0.0, 0.03))
        if k % 20 == 0:
            ekf.update_position(rng.normal(0.0, 0.1), rng.normal(0.0, 0.1))
    assert np.allclose(ekf.P, ekf.P.T, atol=1e-12)
    assert np.all(np.diag(ekf.P) > 0.0)


def test_yaw_residual_is_normalized_across_pm_pi():
    """回归：航向残差若不归一化，±pi 边界处滤波器会被巨大残差带偏。

    估计 -3.10 rad、观测 +3.10 rad，真实残差只有约 0.08 rad；
    若直接相减会得到 6.2 rad 的错误残差。
    """
    ekf = EKFLocalizer(EKFConfig(), x0=np.array([0.0, 0.0, -3.10]), P0=np.eye(3) * 0.2)
    ekf.update_yaw(3.10)
    # 更新后航向应留在估计值附近（靠近 ±pi），而不是被拉向 0
    assert abs(ekf.x[2]) > 3.0


def test_nan_observation_is_rejected():
    """NaN 观测必须被拒绝，且不能污染状态。"""
    ekf = EKFLocalizer(EKFConfig(), x0=np.array([0.1, 0.2, 0.3]))
    before = ekf.x.copy()
    assert ekf.update_yaw(float("nan")) is False
    assert ekf.update_position(float("nan"), 0.0) is False
    assert np.allclose(ekf.x, before)
    assert ekf.rejected_updates == 2


def test_invalid_predict_input_is_rejected():
    """非有限控制量或非正 dt 必须被拒绝。"""
    ekf = EKFLocalizer(EKFConfig())
    before = ekf.x.copy()
    assert ekf.predict(float("nan"), 0.1, 0.05) is False
    assert ekf.predict(0.8, 0.1, 0.0) is False
    assert ekf.predict(0.8, 0.1, -0.05) is False
    assert np.allclose(ekf.x, before)


def test_position_update_pulls_state_toward_measurement():
    """位置观测应把估计拉向观测值，但不能一步跳到观测值（取决于置信度）。"""
    ekf = EKFLocalizer(EKFConfig(sigma_pos=0.15), x0=np.array([0.0, 0.0, 0.0]),
                       P0=np.eye(3) * 0.3)
    ekf.update_position(1.0, 0.0)
    assert 0.0 < ekf.x[0] < 1.0
    ekf.update_position(1.0, 0.0)
    assert ekf.x[0] > 0.5


def test_nees_matches_closed_form():
    """NEES = d^T P^-1 d，用对角协方差手算校验。"""
    ekf = EKFLocalizer(EKFConfig(), x0=np.zeros(3), P0=np.eye(3))
    assert abs(ekf.nees(np.array([1.0, 0.0, 0.0])) - 1.0) < 1e-9
    assert abs(ekf.nees(np.array([2.0, 0.0, 0.0])) - 4.0) < 1e-9


def test_dead_reckoning_drifts_with_scale_error():
    """航位推算在比例误差下必须随时间累积漂移。"""
    traj = TRAJECTORY_BUILDERS["circle"](radius=8.0, speed=0.8, dt=0.05)
    rng = np.random.default_rng(1)
    odom = OdometryNoise()
    v_meas = np.empty(len(traj))
    w_meas = np.empty(len(traj))
    for k in range(len(traj)):
        v_meas[k], w_meas[k] = odom.corrupt(traj.v[k], traj.w[k], rng)
    poses = integrate_odometry(v_meas, w_meas, traj.dt, traj.state(0))
    err = math.hypot(poses[-1, 0] - traj.x[-1], poses[-1, 1] - traj.y[-1])
    assert err > 0.5


def test_ekf_fusion_beats_dead_reckoning():
    """融合定位的 RMS 误差应显著低于纯航位推算（同一噪声序列）。"""
    traj = TRAJECTORY_BUILDERS["eight"](speed=0.8, dt=0.05)
    rng = np.random.default_rng(3)
    odom, imu = OdometryNoise(), ImuNoise()
    pos = PositionSensor(sigma=0.15, rate_hz=1.0)

    ekf = EKFLocalizer(EKFConfig(), x0=traj.state(0), P0=np.eye(3) * 1e-3)
    v_list, w_list, ekf_err = [], [], []
    for k in range(len(traj)):
        ekf_err.append(ekf.position_error(traj.state(k)))
        v_m, w_m = odom.corrupt(traj.v[k], traj.w[k], rng)
        v_list.append(v_m)
        w_list.append(w_m)
        ekf.predict(v_m, w_m, traj.dt)
        ekf.update_yaw(imu.observe(traj.theta[k], rng))
        if pos.available(traj.t[k]):
            px, py = pos.observe(traj.x[k], traj.y[k], rng)
            ekf.update_position(px, py)

    odom_poses = integrate_odometry(np.array(v_list), np.array(w_list), traj.dt, traj.state(0))
    odom_err = np.hypot(odom_poses[:, 0] - traj.x, odom_poses[:, 1] - traj.y)
    assert float(np.sqrt(np.mean(np.array(ekf_err) ** 2))) < 0.2
    assert float(np.sqrt(np.mean(odom_err**2))) > 5.0 * float(np.sqrt(np.mean(np.array(ekf_err) ** 2)))


def test_fused_heading_error_is_small():
    """融合后航向误差应远小于航位推算。"""
    traj = TRAJECTORY_BUILDERS["circle"](radius=8.0, speed=0.8, dt=0.05)
    rng = np.random.default_rng(5)
    odom, imu = OdometryNoise(), ImuNoise()
    ekf = EKFLocalizer(EKFConfig(), x0=traj.state(0), P0=np.eye(3) * 1e-3)
    for k in range(len(traj)):
        v_m, w_m = odom.corrupt(traj.v[k], traj.w[k], rng)
        ekf.predict(v_m, w_m, traj.dt)
        ekf.update_yaw(imu.observe(traj.theta[k], rng))
    assert ekf.heading_error(traj.state(len(traj) - 1)) < 0.1


def test_nees_of_fused_filter_is_consistent():
    """一致性检查：调好参数后 NEES 均值应接近自由度 3（宽松区间 1~9）。"""
    traj = TRAJECTORY_BUILDERS["eight"](speed=0.8, dt=0.05)
    rng = np.random.default_rng(3)
    odom, imu = OdometryNoise(), ImuNoise()
    pos = PositionSensor(sigma=0.15, rate_hz=1.0)
    ekf = EKFLocalizer(EKFConfig(), x0=traj.state(0), P0=np.eye(3) * 1e-3)
    nees = []
    for k in range(len(traj)):
        nees.append(ekf.nees(traj.state(k)))
        v_m, w_m = odom.corrupt(traj.v[k], traj.w[k], rng)
        ekf.predict(v_m, w_m, traj.dt)
        ekf.update_yaw(imu.observe(traj.theta[k], rng))
        if pos.available(traj.t[k]):
            px, py = pos.observe(traj.x[k], traj.y[k], rng)
            ekf.update_position(px, py)
    mean_nees = float(np.mean(nees))
    assert 1.0 < mean_nees < 9.0


def test_long_run_does_not_diverge():
    """长时间运行（2000 步）不得出现 NaN 或协方差爆炸。"""
    rng = np.random.default_rng(7)
    ekf = EKFLocalizer(EKFConfig())
    for k in range(2000):
        ekf.predict(0.8 + rng.normal(0.0, 0.05), 0.3 * math.sin(k * 0.01) + rng.normal(0.0, 0.02), 0.05)
        ekf.update_yaw(rng.normal(0.0, 0.05))
        if k % 50 == 0:
            ekf.update_position(rng.normal(0.0, 0.2), rng.normal(0.0, 0.2))
    assert np.all(np.isfinite(ekf.x))
    assert np.all(np.isfinite(ekf.P))
    assert ekf.trace_cov() < 1e3
    assert ekf.cov_repairs == 0


# --------------------------------------------------------------------- 数值安全

def test_repair_covariance_symmetrizes():
    """非对称协方差应被对称化。"""
    P = np.array([[1.0, 0.5, 0.0], [0.1, 1.0, 0.0], [0.0, 0.0, 1.0]])
    fixed, repaired = repair_covariance(P)
    assert repaired is True
    assert np.allclose(fixed, fixed.T)


def test_repair_covariance_handles_nan():
    """含 NaN 的协方差必须被重置。"""
    P = np.array([[np.nan, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    fixed, repaired = repair_covariance(P)
    assert repaired is True
    assert np.all(np.isfinite(fixed))


def test_repair_covariance_clamps_diagonal():
    """过小的对角元应被抬到下限，防止滤波器过度自信。"""
    P = np.diag([0.0, 1e-12, 1.0])
    fixed, repaired = repair_covariance(P, min_diag=1e-9)
    assert repaired is True
    assert np.all(np.diag(fixed) >= 1e-9)


def test_repair_covariance_keeps_healthy_matrix():
    """健康矩阵不应被改动。"""
    P = np.diag([0.1, 0.2, 0.3])
    fixed, repaired = repair_covariance(P)
    assert repaired is False
    assert np.allclose(fixed, P)


def test_sanitize_sigma_rejects_non_positive():
    """sigma 为 0 会让滤波器把预测当真理，必须回退。"""
    v, warn = sanitize_sigma(0.0, default=0.1)
    assert v == 0.1 and warn is not None
    v, warn = sanitize_sigma(-1.0, default=0.1)
    assert v == 0.1 and warn is not None
    v, warn = sanitize_sigma(float("nan"), default=0.1)
    assert v == 0.1 and warn is not None
    v, warn = sanitize_sigma(0.05, default=0.1)
    assert v == 0.05 and warn is None


def test_freshness_tracker_detects_stale_sources():
    """超时判定应对多路数据源独立生效。"""
    ft = FreshnessTracker()
    ft.register("odom", 0.5)
    ft.register("imu", 0.5)
    assert ft.is_stale("odom", 0.0) is True        # 从未收到
    ft.mark("odom", 10.0)
    assert ft.is_fresh("odom", 10.3) is True
    assert ft.is_stale("odom", 10.6) is True
    assert ft.is_stale("imu", 10.3) is True


# --------------------------------------------------------------------- 传感器

def test_odometry_noise_scale_error_is_systematic():
    """比例误差方向确定：v>0 时读数恒偏大。"""
    odom = OdometryNoise(scale_v=0.02, scale_w=0.05, sigma_v=0.0, sigma_w=0.0)
    rng = np.random.default_rng(0)
    v_m, w_m = odom.corrupt(1.0, 0.5, rng)
    assert abs(v_m - 1.02) < 1e-12
    assert abs(w_m - 0.525) < 1e-12


def test_noise_free_models_return_input():
    """噪声全为 0 时传感器模型应直通真值。"""
    odom = OdometryNoise(scale_v=0.0, scale_w=0.0, sigma_v=0.0, sigma_w=0.0)
    imu = ImuNoise(sigma_yaw=0.0, bias_yaw=0.0)
    rng = np.random.default_rng(0)
    assert odom.corrupt(0.8, 0.2, rng) == (0.8, 0.2)
    assert abs(imu.observe(0.4, rng) - 0.4) < 1e-12


def test_position_sensor_sampling_rate():
    """1 Hz 抽样在 dt=0.05 下大约每 20 步给一次观测。"""
    sensor = PositionSensor(sigma=0.1, rate_hz=1.0)
    dt = 0.05
    count = sum(1 for k in range(200) if sensor.available(k * dt))
    assert 9 <= count <= 11


def test_position_sensor_disabled_when_rate_zero():
    """rate<=0 表示不提供位置观测。"""
    sensor = PositionSensor(rate_hz=0.0)
    assert all(not sensor.available(k * 0.05) for k in range(100))


if __name__ == "__main__":
    import traceback

    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except Exception:
            failed += 1
            print(f"FAIL  {fn.__name__}")
            traceback.print_exc()
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    raise SystemExit(1 if failed else 0)
