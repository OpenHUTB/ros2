#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""已知答案对拍（known-answer tests）。

为什么需要这个文件
------------------
`test_traj_eval.py` 里的检查是**自洽**的：它验证"实现与自己一致"
（比如 RPE 随 delta 线性增长、ATE 随噪声单调增长）。
但自洽测试查不出"公式整体写错但内部一致"的错误——而这恰好是 ATE 最典型的坑：
对齐方式选错、旋转与平移的求解顺序颠倒、尺度估计用错变量，
这些错误都能通过自洽检查。

本文件用两种**独立于被测实现**的答案源来对拍：

1. **解析解**：构造误差已知的轨迹，直接算出 ATE / RPE / 漂移率应当等于多少；
2. **暴力搜索**：不用 SVD，直接对旋转角做网格搜索 + 局部细化求最优刚体对齐，
   与 Umeyama 闭式解的结果比对。

运行::

    pytest test/ -q          # 或 python3 test/test_known_answer.py

所有断言用 ``assert``，失败时 pytest 会明确指出是哪一条。
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from traj_eval import (  # noqa: E402
    OdomErrorModel, Pose, align_umeyama, ate, drift_percent, make_ground_truth,
    positions, rpe, simulate_odometry,
)

TOL = 1e-9

# Windows 默认控制台为 GBK，无法编码部分数学符号；保留原编码把不可编码字符
# 替换掉，避免打印结果时抛 UnicodeEncodeError 而中断断言（评审 #464 反馈）。
_SYMBOL_FALLBACK = {
    "\u2218": "o",          # ∘  复合
    "\u207b\u00b9": "^-1",  # ⁻¹ 逆
    "\u207b": "^-",
    "\u00b9": "^1",
    "\u00b2": "^2",
    "\u00b3": "^3",
}


def _harden_console() -> None:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except (AttributeError, ValueError, OSError):
            pass


def _safe(text: str) -> str:
    enc = getattr(sys.stdout, "encoding", None) or "utf-8"
    # 先按多字符组合替换（如 ⁻¹ → ^-1），再逐字符兜底
    for src, dst in _SYMBOL_FALLBACK.items():
        try:
            src.encode(enc)
        except (UnicodeEncodeError, LookupError):
            text = text.replace(src, dst)
    out = []
    for ch in text:
        try:
            ch.encode(enc)
            out.append(ch)
        except (UnicodeEncodeError, LookupError):
            out.append("?")
    return "".join(out)


_harden_console()


# ------------------------------------------------------------------ 辅助构造
def circle_traj(radius: float, n: int = 360, turns: float = 1.0) -> list:
    """匀速圆周运动：半径固定、角速度恒定（严格的解析轨迹）。"""
    return [Pose(radius * math.cos(2 * math.pi * turns * i / n),
                 radius * math.sin(2 * math.pi * turns * i / n),
                 (2 * math.pi * turns * i / n) % (2 * math.pi))
            for i in range(n)]


def line_traj(speed: float = 1.5, dt: float = 0.05, n: int = 400) -> list:
    """匀速直线运动（航向恒为 0）。"""
    return [Pose(speed * dt * i, 0.0, 0.0) for i in range(n)]


def rotate_translate(poses, angle: float, tx: float, ty: float) -> list:
    c, s = math.cos(angle), math.sin(angle)
    return [Pose(c * p.x - s * p.y + tx, s * p.x + c * p.y + ty,
                 p.yaw + angle) for p in poses]


def bruteforce_ate(src: np.ndarray, dst: np.ndarray,
                   coarse_deg: float = 0.05, refine_deg: float = 1e-6) -> tuple:
    """完全不用 SVD 求最优刚体对齐后的 ATE。

    对固定旋转角 θ，最优平移的闭式解是 ``t = mean(dst) - R(θ) mean(src)``，
    因此只需对 θ 做一维搜索。返回值 ``(ate_rmse, best_angle_rad)``。
    """
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    src_c = src - mu_s

    def sse_at(theta: float) -> float:
        c, s = math.cos(theta), math.sin(theta)
        rot = np.array([[c, -s], [s, c]])
        proj = src_c @ rot.T
        resid = proj - (dst - mu_d)
        return float((resid ** 2).sum())

    # 粗搜索
    grid = np.radians(np.arange(-180.0, 180.0, coarse_deg))
    errs = [sse_at(t) for t in grid]
    best = float(grid[int(np.argmin(errs))])

    # 局部细化（黄金分割，区间取相邻网格点的跨度）
    step = math.radians(coarse_deg)
    lo, hi = best - step, best + step
    phi = (math.sqrt(5.0) - 1.0) / 2.0
    for _ in range(200):
        if hi - lo < math.radians(refine_deg):
            break
        m1 = hi - phi * (hi - lo)
        m2 = lo + phi * (hi - lo)
        if sse_at(m1) < sse_at(m2):
            hi = m2
        else:
            lo = m1
    theta = 0.5 * (lo + hi)
    rmse = math.sqrt(sse_at(theta) / src.shape[0])
    return rmse, theta


# ------------------------------------------------------- 1. 相同轨迹 ATE=0
def test_identical_trajectory_gives_zero_ate() -> None:
    gt = circle_traj(radius=8.0, n=200)
    res = ate(gt, gt)
    # 用容差而不是 == 0：对齐解来自 SVD，相同点集时旋转会带 1e-16 量级的数值残差
    assert res.rmse < 1e-12, f"相同轨迹 ATE 应接近 0，实际 {res.rmse}"
    assert res.max < 1e-12
    assert res.rmse_rot_deg < 1e-12


# --------------------------------------- 2. 不加对齐时 ATE 等于已知平移量
def test_ate_equals_known_offset_without_alignment() -> None:
    gt = line_traj(n=100)
    d = 3.7
    est = [Pose(p.x, p.y + d, p.yaw) for p in gt]      # 整体横向平移 d
    res = ate(est, gt, align=False)
    assert abs(res.rmse - d) < 1e-12, f"未对齐 ATE 应等于平移量 {d}，实际 {res.rmse}"

    # 对齐后整体平移应被完全吸收
    aligned = ate(est, gt, align=True)
    assert aligned.rmse < 1e-9, f"对齐后 ATE 应接近 0，实际 {aligned.rmse}"


# ------------------------------------- 3. 刚体变换不变性（旋转 + 平移组合）
def test_ate_invariant_under_rigid_transform() -> None:
    gt = make_ground_truth("figure8", speed=1.5, duration=10.0, dt=0.05)
    moved = rotate_translate(gt, angle=math.radians(37.0), tx=-12.0, ty=4.5)
    res = ate(moved, gt, align=True)
    assert res.rmse < 1e-9, f"刚体变换后对齐 ATE 应为 0，实际 {res.rmse}"
    # 注意方向：ate(est, gt) 把 est 对齐到 gt；est 相对 gt 转了 +37°，
    # 因此求解出的对齐旋转是 -37°。
    assert abs(res.yaw_offset_deg + 37.0) < 1e-6, (
        f"应恢复出 -37° 的对齐旋转，实际 {res.yaw_offset_deg:.6f}")


# --------------------------- 4. 同心圆半径误差：ATE 严格等于半径差（解析解）
def test_concentric_circle_ate_equals_radius_error() -> None:
    """两个同心圆，半径分别为 R 与 R+ΔR。

    对固定旋转角，最优平移为 0（质心相同）；由于圆对称，任何旋转都不改变残差，
    因此每个点的残差恒为 ΔR，ATE 必须**严格等于** ΔR。这是最干净的解析答案。
    """
    R, dR = 10.0, 0.35
    gt = circle_traj(radius=R, n=240)
    est = circle_traj(radius=R + dR, n=240)
    res = ate(est, gt)
    assert abs(res.rmse - dR) < 1e-6, f"ATE 应等于半径差 {dR}，实际 {res.rmse}"
    assert abs(res.max - dR) < 1e-6, f"最大误差应等于 {dR}，实际 {res.max}"


# ------------------------- 5. Umeyama 闭式解 vs 暴力搜索（独立算法交叉验证）
def test_umeyama_matches_bruteforce_search() -> None:
    rng = np.random.default_rng(7)
    src = rng.normal(0.0, 5.0, (120, 2))
    theta_true = math.radians(-63.0)
    t_true = np.array([7.5, -2.5])
    c, s = math.cos(theta_true), math.sin(theta_true)
    rot = np.array([[c, -s], [s, c]])
    dst = src @ rot.T + t_true + rng.normal(0.0, 0.3, src.shape)   # 加噪声

    al = align_umeyama(src, dst, with_scale=False)
    ours = al.apply_points(src)
    ours_rmse = float(np.sqrt(((ours - dst) ** 2).sum(axis=1).mean()))
    bf_rmse, bf_theta = bruteforce_ate(src, dst)

    assert abs(ours_rmse - bf_rmse) < 1e-6, (
        f"Umeyama 与暴力搜索的最小 RMSE 应一致：{ours_rmse:.9f} vs {bf_rmse:.9f}")
    # 两种方法求出的最优旋转角应相同（圆周等价，取到 (-π, π]）
    da = (al.yaw_offset - bf_theta + math.pi) % (2 * math.pi) - math.pi
    assert abs(da) < 1e-4, (
        f"最优旋转角应一致：Umeyama {math.degrees(al.yaw_offset):.4f}° "
        f"vs 暴力搜索 {math.degrees(bf_theta):.4f}°")


# ------------------- 6. 匀速直线 + 尺度误差：RPE 与漂移率都有解析答案
def test_rpe_and_drift_closed_form_on_straight_line() -> None:
    dt, speed, k = 0.05, 1.5, 1.02
    gt = line_traj(speed=speed, dt=dt, n=600)
    est = simulate_odometry(gt, OdomErrorModel(wheel_scale=k, yaw_bias_deg_per_m=0.0))

    ds = speed * dt
    for delta in (1, 10, 100):
        r = rpe(est, gt, delta=delta)
        expected = (k - 1.0) * ds * delta
        assert abs(r.trans_rmse - expected) < expected * 1e-6 + 1e-12, (
            f"RPE(delta={delta}) 应等于 {expected:.9f}，实际 {r.trans_rmse:.9f}")
        assert r.rot_rmse_deg < 1e-9, f"直行时旋转误差应为 0，实际 {r.rot_rmse_deg}"

    # 漂移率：直线上尺度误差直接体现为 (k-1) 的比例
    drift = drift_percent(est, gt, align=False)
    assert abs(drift - (k - 1.0) * 100.0) < 0.05, (
        f"漂移率应约等于 {(k - 1.0) * 100:.3f}%，实际 {drift:.4f}%")


# ------------------------- 7. 随机噪声下 ATE 的期望值可预测（统计性答案）
def test_ate_expected_value_under_known_noise() -> None:
    """给真值加各向同性高斯噪声 σ，对齐后的 ATE RMSE 应接近 σ·√2。

    二维各向同性噪声的模长服从 Rayleigh 分布，其均方根为 σ·√2；
    对齐只去掉整体平移（对大量点影响很小），因此 ATE ≈ σ√2。
    """
    rng = np.random.default_rng(123)
    gt = circle_traj(radius=30.0, n=2000)          # 大半径 + 多点，减小对齐的影响
    sigma = 0.25
    pts = positions(gt) + rng.normal(0.0, sigma, (len(gt), 2))
    est = [Pose(float(x), float(y), p.yaw) for (x, y), p in zip(pts, gt)]
    observed = ate(est, gt, align=True).rmse
    expected = sigma * math.sqrt(2.0)
    assert abs(observed - expected) < 0.03, (
        f"σ={sigma} 的噪声下 ATE 应接近 {expected:.4f}，实际 {observed:.4f}")


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(_safe(f"[PASS] {fn.__name__}"))
        except AssertionError as exc:
            failed += 1
            print(_safe(f"[FAIL] {fn.__name__}: {exc}"))
    print(f"\n{len(fns) - failed}/{len(fns)} 通过")
    sys.exit(0 if failed == 0 else 1)
