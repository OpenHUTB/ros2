#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线单元测试：不需要 ROS，只需 numpy。

运行::

    python3 test/test_traj_eval.py

覆盖内容：
1. SE2 位姿运算（复合/求逆/角度归一化）
2. Umeyama 对齐（纯旋转、带尺度、镜像数据不产生反射）
3. ATE（完全相同为 0、随噪声单调增大、对齐能消除整体刚体偏移）
4. RPE（纯直行 + 尺度误差时与解析值一致）
5. 漂移率（解析值对照）
6. 误差模型 + 评估的端到端一致性（无误差→近零；2% 尺度→RPE≈2%）
7. CSV 读写往返
8. 异常输入
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
    OdomErrorModel, Pose, align_umeyama, ate, compose, drift_percent, evaluate_all,
    inverse, load_trajectory, make_ground_truth, path_length, positions, rpe,
    save_trajectory, simulate_odometry, wrap_angle, yaws,
)

PASS, FAIL = 0, 0

# Windows 默认控制台是 GBK，无法编码 ∘ / ⁻ / ¹ 等符号，
# 会在打印第一条结果时就抛 UnicodeEncodeError，导致断言没跑完就中断。
# 这里做两层保护：保留原编码但把无法编码的字符替换掉，并把少量符号换成 ASCII 等价物。
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
    """把当前输出编码无法表示的符号换成 ASCII 等价物。

    先按**多字符组合**替换（如 ``⁻¹`` → ``^-1``），否则逐字符替换会得到 ``^-^1``。
    """
    enc = getattr(sys.stdout, "encoding", None) or "utf-8"
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


def check(cond: bool, msg: str) -> None:
    global PASS, FAIL
    if cond:
        PASS += 1
        print(_safe(f"  [PASS] {msg}"))
    else:
        FAIL += 1
        print(_safe(f"  [FAIL] {msg}"))


def close(actual: float, expected: float, tol: float, msg: str) -> None:
    check(abs(actual - expected) <= tol,
          f"{msg}（实际 {actual:.6f}，期望 {expected:.6f} ± {tol:g}）")


# ------------------------------------------------------------------ 1 SE2
def test_se2() -> None:
    print("\n[1] SE2 位姿运算")
    close(wrap_angle(3 * math.pi), -math.pi, 1e-9, "wrap_angle(3π) 落在 (-π, π]")
    close(wrap_angle(0.0), 0.0, 1e-12, "wrap_angle(0)=0")

    a = Pose(1.0, 2.0, 0.3)
    b = Pose(0.5, -0.2, -0.1)
    # compose(a, inverse(a)) 应为单位位姿
    i = compose(a, inverse(a))
    close(math.hypot(i.x, i.y), 0.0, 1e-12, "a ∘ a⁻¹ 的平移为 0")
    close(i.yaw, 0.0, 1e-12, "a ∘ a⁻¹ 的角度为 0")

    # 位移复合：沿 x 前进 1 m 再左转 90° 再前进 1 m
    p = compose(compose(Pose(0, 0, 0), Pose(1, 0, 0)), Pose(0, 0, 0))
    close(p.x, 1.0, 1e-12, "沿 x 前进 1 m")
    q = compose(Pose(1, 0, math.pi / 2), Pose(1, 0, 0))
    close(q.y, 1.0, 1e-12, "左转 90° 后前进 1 m 落在 y=1")

    # path_length
    pts = [Pose(0, 0, 0), Pose(3, 0, 0), Pose(3, 4, 0)]
    close(path_length(pts), 7.0, 1e-12, "折线长度 = 7 m")


# --------------------------------------------------------------- 2 对齐
def test_alignment() -> None:
    print("\n[2] Umeyama 对齐")
    rng = np.random.default_rng(0)
    src = rng.normal(0, 3.0, (30, 2))

    # 纯刚体变换：旋转 30° + 平移
    th = math.radians(30.0)
    rot = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    t = np.array([5.0, -2.0])
    dst = src @ rot.T + t

    al = align_umeyama(src, dst, with_scale=False)
    close(al.scale, 1.0, 1e-12, "只做刚体对齐时 s=1")
    close(math.degrees(al.yaw_offset), 30.0, 1e-8, "恢复出 30° 旋转")
    resid = np.abs(al.apply_points(src) - dst).max()
    check(resid < 1e-10, f"对齐残差接近 0（实际 {resid:.2e}）")

    # 带尺度
    al2 = align_umeyama(src, dst * 2.0, with_scale=True)
    close(al2.scale, 2.0, 1e-10, "带尺度对齐恢复出 s=2")

    # 镜像数据（det<0 的伪变换）不应产生反射矩阵
    mirrored = src * np.array([1.0, -1.0])
    al3 = align_umeyama(src, mirrored, with_scale=False)
    check(np.linalg.det(al3.rotation) > 0, "对齐矩阵保持 det(R)=+1（无反射）")

    # 点数不足应报错
    try:
        align_umeyama(np.zeros((1, 2)), np.zeros((1, 2)))
        check(False, "单点对齐应报错")
    except ValueError:
        check(True, "单点对齐抛 ValueError")


# ---------------------------------------------------------------- 3 ATE
def test_ate() -> None:
    print("\n[3] 绝对轨迹误差 ATE")
    gt = make_ground_truth("figure8", speed=1.5, duration=8.0, dt=0.05)
    close(ate(gt, gt).rmse, 0.0, 1e-12, "完全相同轨迹 ATE=0")

    # 整体平移不应影响对齐后的 ATE
    shifted = [Pose(p.x + 10.0, p.y - 5.0, p.yaw) for p in gt]
    close(ate(shifted, gt, align=True).rmse, 0.0, 1e-9, "整体平移后对齐 ATE≈0")
    check(ate(shifted, gt, align=False).rmse > 5.0, "不做对齐时整体平移会体现为误差")

    # 噪声越大 ATE 越大
    rng = np.random.default_rng(1)
    errs = []
    for sigma in (0.01, 0.05, 0.2):
        noisy = [Pose(p.x + rng.normal(0, sigma), p.y + rng.normal(0, sigma), p.yaw) for p in gt]
        errs.append(ate(noisy, gt, align=True).rmse)
    check(errs[0] < errs[1] < errs[2], f"ATE 随噪声单调增大 {['%.4f' % e for e in errs]}")


# ---------------------------------------------------------------- 4 RPE
def test_rpe() -> None:
    print("\n[4] 相对位姿误差 RPE（与解析值对照）")
    dt, speed, scale = 0.05, 1.5, 1.02
    gt = make_ground_truth("straight", speed=speed, duration=10.0, dt=dt)
    est = simulate_odometry(gt, OdomErrorModel(wheel_scale=scale, yaw_bias_deg_per_m=0.0))

    # 每步前进 ds = speed*dt，尺度误差 scale 使估计多走 (scale-1)*ds
    ds = speed * dt
    for delta in (1, 5, 20):
        r = rpe(est, gt, delta=delta)
        expected = (scale - 1.0) * ds * delta
        close(r.trans_rmse, expected, expected * 0.02 + 1e-6,
              f"RPE(delta={delta}) 平移 ≈ 解析值 {expected:.5f} m")
        close(r.rot_rmse_deg, 0.0, 1e-9, f"RPE(delta={delta}) 旋转误差为 0（直行不转）")


# ----------------------------------------------------------- 5 漂移率
def test_drift() -> None:
    print("\n[5] 终点漂移率")
    gt = make_ground_truth("straight", speed=1.5, duration=20.0, dt=0.05)
    est = simulate_odometry(gt, OdomErrorModel(wheel_scale=1.02))
    d_raw = drift_percent(est, gt, align=False)
    # 直行时尺度误差直接体现为终点漂移 ≈ 2%
    close(d_raw, 2.0, 0.2, "直行 + 2% 尺度误差 → 原始漂移率 ≈ 2%")

    perfect = simulate_odometry(gt, OdomErrorModel())
    check(drift_percent(perfect, gt, align=False) < 0.5, "无误差模型漂移率接近 0")


# ------------------------------------------------- 6 端到端一致性
def test_end_to_end() -> None:
    print("\n[6] 误差模型 + 评估 端到端")
    gt = make_ground_truth("figure8", speed=1.5, duration=30.0, dt=0.05)
    print(f"  真值长度 {path_length(gt):.2f} m，{len(gt)} 个采样点")

    perfect = simulate_odometry(gt, OdomErrorModel())
    m0 = evaluate_all(perfect, gt)
    check(m0["ate_rmse_m"] < 0.05, f"无误差时 ATE 很小（{m0['ate_rmse_m']:.4f} m）")
    check(m0["drift_percent"] < 0.5, f"无误差时漂移率很小（{m0['drift_percent']:.3f}%）")

    biased = simulate_odometry(gt, OdomErrorModel(wheel_scale=1.02))
    m1 = evaluate_all(biased, gt)
    check(m1["ate_rmse_m"] > m0["ate_rmse_m"], "2% 尺度误差使 ATE 明显增大")
    check(m1["drift_percent"] > 1.5, f"2% 尺度误差使漂移率 > 1.5%（实际 {m1['drift_percent']:.3f}%）")
    for key in ("rpe_1_trans_rmse_m", "rpe_10_trans_rmse_m", "rpe_50_trans_rmse_m"):
        check(key in m1, f"指标包含 {key}")

    # yaw 偏置应主要体现为角度误差
    yaw_bias = simulate_odometry(gt, OdomErrorModel(yaw_bias_deg_per_m=0.1))
    m2 = evaluate_all(yaw_bias, gt)
    check(m2["ate_rmse_rot_deg"] > 0.5,
          f"航向偏置使朝向误差增大（{m2['ate_rmse_rot_deg']:.3f}°）")


# ---------------------------------------------------------------- 7 IO
def test_io() -> None:
    print("\n[7] CSV 读写往返")
    gt = make_ground_truth("circle", speed=1.0, duration=5.0, dt=0.1)
    # 刻意不用 tempfile：受限环境下系统临时目录可能不可写。
    # 直接用测试文件旁边的目录，跑完删除，保证任何机器上都能运行。
    out_dir = os.path.join(_HERE, "_test_output")
    os.makedirs(out_dir, exist_ok=True)
    try:
        path = os.path.join(out_dir, "t.csv")
        save_trajectory(path, gt, dt=0.1)
        check(os.path.getsize(path) > 0, "CSV 已写出且非空")
        back = load_trajectory(path)
        check(len(back) == len(gt), "读回的位姿数量一致")
        close(back[5].x, gt[5].x, 1e-5, "读回的 x 一致")
        close(back[5].yaw, gt[5].yaw, 1e-5, "读回的 yaw 一致")
    finally:
        for name in ("t.csv",):
            p = os.path.join(out_dir, name)
            if os.path.isfile(p):
                os.remove(p)

    try:
        load_trajectory(os.path.join(out_dir, "__not_exist__.csv"))
        check(False, "读取不存在的文件应报错")
    except FileNotFoundError:
        check(True, "读取不存在的文件抛 FileNotFoundError")


# ------------------------------------------------------------ 8 异常输入
def test_errors() -> None:
    print("\n[8] 异常输入")
    gt = make_ground_truth("straight", duration=2.0, dt=0.1)
    try:
        ate(gt[:-1], gt)
        check(False, "长度不一致应报错")
    except ValueError:
        check(True, "长度不一致抛 ValueError")

    try:
        rpe(gt, gt, delta=len(gt))
        check(False, "delta 过大应报错")
    except ValueError:
        check(True, "delta 超过轨迹长度抛 ValueError")

    try:
        make_ground_truth("not_exist")
        check(False, "未知轨迹类型应报错")
    except KeyError:
        check(True, "未知轨迹类型抛 KeyError")


def main() -> int:
    print("=" * 72)
    print("轨迹/里程计精度评估 —— 离线单元测试（不需要 ROS）")
    print("=" * 72)
    test_se2()
    test_alignment()
    test_ate()
    test_rpe()
    test_drift()
    test_end_to_end()
    test_io()
    test_errors()
    print("\n" + "=" * 72)
    print(f"通过 {PASS} 项，失败 {FAIL} 项")
    print("=" * 72)
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
