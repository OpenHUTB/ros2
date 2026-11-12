#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""相机-IMU 时间同步标定的共享核心。

包含四部分：

1. ``omega``          解析角速度轨迹（真值，可任意时刻求值）；
2. ``simulate``       相机 / IMU 传感器模型：帧率、帧积分、噪声、零偏、时间偏移；
3. ``estimate_offset`` 归一化互相关粗搜 + 细化，估计相机相对 IMU 的常量时间偏移；
4. ``attitude_error`` 把角速度积分成姿态，量化"偏移导致的姿态误差"。

只依赖 NumPy，不依赖 ROS 与 SciPy，可在 Windows / Ubuntu 上离线运行。

约定
----
时间偏移 ``offset``（记作 d）定义为：**相机时间戳滞后真值 d 秒**。
也就是第 k 帧在时间戳 ``t_k`` 上记录的，实际是 ``t_k - d`` 时刻的姿态变化。
因此估计量 ``d_hat`` 的物理含义是"要把相机时间戳往前推多少才对齐 IMU"。
"""

from __future__ import annotations

import numpy as np

# 默认随机种子：所有文档中的数值都来自这个种子（或文中显式给出的种子）。
DEFAULT_SEED = 0

# ---------------------------------------------------------------------------
# 1. 真值轨迹
# ---------------------------------------------------------------------------
# 三个轴用不同频率的正弦叠加，避免三轴同相位导致互相关峰值很"钝"。
# 最高频率约 1.45 Hz，远低于相机 10 Hz 的奈奎斯特频率（5 Hz），
# 这样 10 Hz 相机仍能重建角速度，误差只来自离散化和噪声。
_OMEGA_TERMS = (
    # (轴, 幅值 rad/s, 频率 Hz, 相位 rad)
    ((0.55, 0.35, 0.4), (0.25, 1.10, 1.0)),   # x
    ((0.40, 0.52, 2.1), (0.20, 1.45, 0.0)),   # y
    ((0.70, 0.21, 0.9), (0.30, 0.83, 1.7)),   # z
)


def omega(t) -> np.ndarray:
    """解析角速度真值，单位 rad/s。

    参数
    ----
    t : 标量或数组，单位秒。

    返回
    ----
    (N, 3) 数组；``t`` 为标量时返回形状 ``(3,)``。顺序为 (x, y, z)。
    """
    t = np.asarray(t, dtype=float)
    axes = []
    for terms in _OMEGA_TERMS:
        acc = np.zeros_like(t)
        for amp, freq, phase in terms:
            acc = acc + amp * np.sin(2.0 * np.pi * freq * t + phase)
        axes.append(acc)
    return np.stack(axes, axis=-1)


def omega_peak() -> float:
    """真值角速度的峰值 |ω|（rad/s），用于解析估计姿态误差的量级。"""
    t = np.linspace(0.0, 20.0, 200001)
    return float(np.linalg.norm(omega(t), axis=1).max())


# ---------------------------------------------------------------------------
# 2. 传感器模型
# ---------------------------------------------------------------------------
def simulate(
    seed: int = DEFAULT_SEED,
    duration: float = 20.0,
    imu_rate: float = 200.0,
    cam_rate: float = 30.0,
    offset: float = 0.050,
    cam_noise: float = 0.010,
    imu_noise: float = 0.003,
    imu_bias: float = 0.005,
    cam_gain: float = 1.0,
    excitation: float = 1.0,
    timestamp_at: str = "mid",
    frame_average: bool = True,
    exposure: float = None,
) -> dict:
    """合成一段相机 + IMU 角速度观测。

    参数
    ----
    seed : 随机种子，控制噪声。
    duration : 时长 (s)。
    imu_rate : IMU 采样率 (Hz)。
    cam_rate : 相机帧率 (Hz)，相机侧角速度一般由视觉里程计/VIO 前端给出。
    offset : 相机时间戳滞后真值的秒数 d（真值）。
    cam_noise : 相机角速度噪声标准差 (rad/s)。
    imu_noise : IMU 陀螺噪声标准差 (rad/s)。
    imu_bias : IMU 陀螺常值零偏 (rad/s)，三轴同值。
    cam_gain : 相机侧角速度的尺度误差（1.0 表示无误差）。
    excitation : 运动激励缩放系数。0.05 表示车辆几乎不动（角速度只有真值的 5%），
        用于说明"激励不足时估计会失效"。IMU 零偏与噪声不随此系数缩放。
    timestamp_at :
        ``'mid'``   时间戳打在曝光中点（相机常规约定），积分窗口以中点对称；
        ``'start'`` 时间戳打在帧（曝光）开始，积分窗口从时间戳向后 ``exposure``。
    frame_average : 是否模拟一帧内的角速度积分（曝光/前端滑窗平均）。
    exposure : 单帧**曝光（积分）时长** (s)。``None`` 表示等于帧周期
        ``1/cam_rate``，也就是"整帧窗口平均"这个理想化情形（旧行为）。
        必须满足 ``0 < exposure <= 1/cam_rate``。

        **曝光时长与帧周期是两个独立的物理量**。真实相机的曝光通常短于帧周期
        （30 Hz 的帧周期 33.3 ms，曝光可能只有 5~10 ms）。二者的关系决定
        ``timestamp_at='start'`` 时的固定偏差：等效测量时刻是

            t_eff = t_stamp + exposure / 2

        所以偏差是 ``-exposure/2``，**不是固定的半帧**；只有 ``exposure`` 恰好
        等于帧周期时，才退化为教科书写法里的 ``-T/2``。``timestamp_at='mid'``
        时窗口关于时间戳对称，偏差与曝光时长无关。

    返回
    ----
    dict，含 ``t_imu`` / ``w_imu`` / ``t_cam`` / ``w_cam`` / ``offset`` /
    ``exposure`` / ``frame_period`` 等。
    """
    if timestamp_at not in ("mid", "start"):
        raise ValueError("timestamp_at 只能是 'mid' 或 'start'")
    frame_period = 1.0 / cam_rate
    if exposure is None:
        exposure = frame_period
    exposure = float(exposure)
    # 允许浮点误差：exposure == frame_period 是合法边界。
    if not (0.0 < exposure <= frame_period * (1.0 + 1e-9)):
        raise ValueError(
            f"exposure 必须满足 0 < exposure <= 帧周期 {frame_period * 1000:.3f} ms，"
            f"当前为 {exposure * 1000:.3f} ms")
    rng = np.random.default_rng(seed)

    # --- 真值网格 ---
    dt = 1.0 / imu_rate
    n_imu = int(round(duration * imu_rate)) + 1
    t_true = np.arange(n_imu) * dt
    w_true = omega(t_true) * excitation

    # --- IMU：真值 + 常值零偏 + 白噪声 ---
    w_imu = w_true + imu_bias + rng.normal(0.0, imu_noise, size=w_true.shape)

    # --- 相机：低帧率 + 帧内平均 + 尺度误差 + 噪声 + 时间偏移 ---
    cam_dt = frame_period
    n_cam = int(np.floor(duration * cam_rate))
    t_cam = np.arange(n_cam) * cam_dt  # 时间戳（未含偏移，偏移体现在采样内容上）

    if timestamp_at == "mid":
        # 时间戳 = 曝光中点：采样窗口 [中心-T_exp/2, 中心+T_exp/2]，关于时间戳对称
        centers = t_cam - offset
        half = 0.5 * exposure if frame_average else 0.0
        win_lo, win_hi = centers - half, centers + half
    else:
        # 时间戳 = 曝光开始：采样窗口 [起点, 起点+T_exp]，
        # 等效测量时刻比时间戳晚 T_exp/2（帧周期 T 不等于曝光时长 T_exp）
        starts = t_cam - offset
        win_lo = starts
        win_hi = starts + (exposure if frame_average else 0.0)

    w_cam = np.empty((n_cam, 3))
    if frame_average:
        # 帧内平均一次性向量化：构造 (n_cam, n_sub) 的子样本时间矩阵，
        # 只调用一次 omega，避免 600 次 Python 级循环。
        n_sub = 17
        s = np.linspace(0.0, 1.0, n_sub)[None, :]
        tk = win_lo[:, None] + s * (win_hi - win_lo)[:, None]     # (n_cam, n_sub)
        w_cam = omega(tk).mean(axis=1) * excitation
    else:
        w_cam = omega(win_lo) * excitation

    w_cam = cam_gain * w_cam + rng.normal(0.0, cam_noise, size=w_cam.shape)

    return {
        "seed": seed,
        "duration": duration,
        "imu_rate": imu_rate,
        "cam_rate": cam_rate,
        "frame_period": frame_period,
        "exposure": exposure,
        "offset": offset,
        "t_imu": t_true,
        "w_imu": w_imu,
        "t_cam": t_cam,
        "w_cam": w_cam,
        "t_true": t_true,
        "w_true": w_true,
        "timestamp_at": timestamp_at,
    }


# ---------------------------------------------------------------------------
# 3. 时间偏移估计
# ---------------------------------------------------------------------------
def _rho_many(t_cam, w_cam, t_imu, w_imu, taus) -> np.ndarray:
    """一次算多个候选偏移的归一化互相关（向量化，比逐个调用快约两个数量级）。

    把 ``t_cam - tau`` 铺成 (M, N) 查询矩阵，每个轴只调用一次 ``np.interp``；
    落在 IMU 时间范围外的点用掩码剔除，再做逐行去均值与归一化。
    """
    taus = np.atleast_1d(np.asarray(taus, dtype=float))
    m, n = len(taus), len(t_cam)
    T = t_cam[None, :] - taus[:, None]                  # (M, N)
    valid = (T >= t_imu[0]) & (T <= t_imu[-1])
    cnt = valid.sum(axis=1)
    W = valid.astype(float)[:, :, None]                 # (M, N, 1)

    rho = np.full(m, -np.inf)
    if not np.any(cnt >= 10):
        return rho

    Tc = np.clip(T, t_imu[0], t_imu[-1])                # 端点外推值随后被掩码剔除
    flat = Tc.ravel()
    B = np.empty((m * n, 3))
    for axis in range(3):
        B[:, axis] = np.interp(flat, t_imu, w_imu[:, axis])
    B = B.reshape(m, n, 3)
    A = np.broadcast_to(w_cam[None, :, :], B.shape)

    safe = np.maximum(cnt, 1)[:, None, None]
    a_mean = (A * W).sum(axis=1, keepdims=True) / safe
    b_mean = (B * W).sum(axis=1, keepdims=True) / safe
    A0 = (A - a_mean) * W
    B0 = (B - b_mean) * W
    num = (A0 * B0).sum(axis=(1, 2))
    den = np.sqrt((A0 * A0).sum(axis=(1, 2)) * (B0 * B0).sum(axis=(1, 2)))

    good = (cnt >= 10) & (den > 0.0)
    rho[good] = num[good] / den[good]
    return rho


def normalized_correlation(t_cam, w_cam, t_imu, w_imu, tau: float) -> float:
    """在单个候选偏移 ``tau`` 下的归一化互相关。

    把 IMU 重采样到 ``t_cam - tau`` 上，与相机角速度做去均值后的归一化内积。
    去均值使常值零偏不影响结果（零偏在相关性里是常数项，被减掉了）。

    返回 [-1, 1] 内的相关系数；有效重叠点不足时返回 -inf。
    """
    return float(_rho_many(t_cam, np.asarray(w_cam, dtype=float),
                           np.asarray(t_imu, dtype=float),
                           np.asarray(w_imu, dtype=float),
                           np.atleast_1d(tau))[0])


def _scan(t_cam, w_cam, t_imu, w_imu, lo: float, hi: float, step: float):
    grid = np.arange(lo, hi + 0.5 * step, step)
    rho = _rho_many(t_cam, w_cam, t_imu, w_imu, grid)
    return grid, rho


def trim_buffers(buffers, window: float) -> None:
    """按**消息时间戳**裁掉过旧样本（就地修改 deque）。

    缓冲区以 ``header.stamp`` 为键，而时钟基准未必等于节点时钟（设备时钟、
    独立的 ``use_sim_time`` 来源、rosbag 回放的时钟）。若拿节点时钟去比，
    窗口要么永不裁（消息时间落后，缓冲无限增长），要么一裁到底（消息时间
    超前）。这里以「所有缓冲区里最新的消息时间戳」为基准，既与任何外部
    时钟无关，也让两路窗口对齐到同一时间跨度。
    """

    newest = max((buf[-1][0] for buf in buffers if buf), default=None)
    if newest is None:
        return
    horizon = newest - window
    for buf in buffers:
        while buf and buf[0][0] < horizon:
            buf.popleft()


def estimate_offset(
    t_cam,
    w_cam,
    t_imu,
    w_imu,
    search_range: float = 0.25,
    coarse_step: float = 0.005,
    fine_step: float = 0.0001,
    fine_span: float = 0.006,
) -> dict:
    """估计相机相对 IMU 的常量时间偏移。

    两阶段：先在 ``±search_range`` 上以 ``coarse_step`` 粗搜互相关峰，
    再在峰值 ±``fine_span`` 内以 ``fine_step`` 细搜。互相关峰很宽（半高宽约
    100 ms），所以粗搜 5 ms 已足够，精度由细搜的 0.1 ms 决定。

    返回
    ----
    dict：``offset``（秒）、``correlation``（峰值）、``grid``/``rho``（粗搜曲线，
    便于画图）、``scale``（相机/IMU 角速度尺度比，最小二乘估计）。
    """
    t_cam = np.asarray(t_cam, dtype=float)
    w_cam = np.asarray(w_cam, dtype=float)
    t_imu = np.asarray(t_imu, dtype=float)
    w_imu = np.asarray(w_imu, dtype=float)

    # 粗搜范围不能超出数据端点，否则重采样会全被裁掉。
    span = min(search_range, 0.5 * (t_cam[-1] - t_cam[0]))
    grid, rho = _scan(t_cam, w_cam, t_imu, w_imu, -span, span, coarse_step)
    best = int(np.argmax(rho))
    tau0 = float(grid[best])

    # 细搜
    lo = tau0 - fine_span
    hi = tau0 + fine_span
    fine_grid, fine_rho = _scan(t_cam, w_cam, t_imu, w_imu, lo, hi, fine_step)
    tau_hat = float(fine_grid[int(np.argmax(fine_rho))])

    # 在估计偏移处用最小二乘求尺度比（相机 = scale * IMU）。
    # 必须逐轴去均值后再回归：IMU 侧存在常值零偏，带截距不为 0，用
    # 「截距为 0」的最小二乘会让零偏漏进 scale（零偏 0.05 rad/s 时偏
    # 到 0.984）。互相关那边本来就是去均值的，这里同口径才自洽。
    tq = t_cam - tau_hat
    ok = (tq >= t_imu[0]) & (tq <= t_imu[-1])
    scale = float("nan")
    if int(ok.sum()) >= 10:
        b = np.empty((int(ok.sum()), 3))
        for axis in range(3):
            b[:, axis] = np.interp(tq[ok], t_imu, w_imu[:, axis])
        a = w_cam[ok]
        a0 = a - a.mean(axis=0)
        b0 = b - b.mean(axis=0)
        den = float((b0 * b0).sum())
        if den > 0.0:
            scale = float((a0 * b0).sum() / den)

    return {
        "offset": tau_hat,
        "correlation": float(np.max(fine_rho)),
        "grid": grid,
        "rho": rho,
        "scale": scale,
    }


def trial(seed: int = DEFAULT_SEED, **sim_kwargs) -> tuple:
    """跑一次"仿真 + 估计"，返回 ``(sim, result)``。扫描脚本统一走这里。"""
    sim = simulate(seed=seed, **sim_kwargs)
    res = estimate_offset(sim["t_cam"], sim["w_cam"], sim["t_imu"], sim["w_imu"])
    return sim, res


# ---------------------------------------------------------------------------
# 4. 姿态积分与误差
# ---------------------------------------------------------------------------


def _quat_mul(q1, q2) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def _quat_from_rotvec(v) -> np.ndarray:
    theta = float(np.linalg.norm(v))
    if theta < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = np.asarray(v, dtype=float) / theta
    half = 0.5 * theta
    return np.concatenate([[np.cos(half)], np.sin(half) * axis])


def integrate_attitude(t, w) -> np.ndarray:
    """零阶保持积分角速度，返回与 ``t`` 同长的时间序列四元数 (w, x, y, z)。"""
    t = np.asarray(t, dtype=float)
    w = np.asarray(w, dtype=float)
    out = np.empty((len(t), 4))
    q = np.array([1.0, 0.0, 0.0, 0.0])
    for i in range(len(t)):
        out[i] = q
        if i + 1 < len(t):
            q = _quat_mul(q, _quat_from_rotvec(w[i] * (t[i + 1] - t[i])))
            q = q / np.linalg.norm(q)
    return out


def _quat_to_grid(t_src, q_src, t_dst) -> np.ndarray:
    """把四元数序列线性插值（归一化）到目标时间网格。"""
    out = np.empty((len(t_dst), 4))
    for k in range(4):
        out[:, k] = np.interp(t_dst, t_src, q_src[:, k])
    norms = np.linalg.norm(out, axis=1, keepdims=True)
    return out / norms


def attitude_error(offset_used: float, sim: dict) -> dict:
    """把相机角速度按 ``offset_used`` 对齐后积分，与真值姿态比较。

    流程：相机角速度重采样到统一网格 → 零阶保持积分 → 与真值姿态求夹角。

    参数
    ----
    offset_used : 补偿时使用的偏移（秒）。0 表示完全不补偿。
    sim : :func:`simulate` 的返回值。

    返回
    ----
    dict：``t``、``err_deg``（逐时刻夹角，度）、``max_deg``、``rms_deg``。
    统计只在**相机与真值网格的重叠区间**内进行：偏移越大，相机样本整体前移，
    区间两端会缺数据，若用插值端点的钳位值会让误差虚高（这不是估计器的锅）。
    """
    t_grid = sim["t_true"]
    w_cam = sim["w_cam"]

    # .copy() 防止对只读视图做插值时出现越界告警
    t_src = (sim["t_cam"] - offset_used).astype(float)
    w_on_grid = np.empty((len(t_grid), 3))
    for axis in range(3):
        w_on_grid[:, axis] = np.interp(t_grid, t_src, w_cam[:, axis])

    # 真值必须取仿真实际使用的轨迹：simulate() 会按 excitation 缩放，
    # 这里若重新调用 omega(t_grid)，excitation != 1 时真值幅值会偏大。
    q_true = integrate_attitude(t_grid, sim["w_true"])
    q_est = integrate_attitude(t_grid, w_on_grid)

    dot = np.abs(np.sum(q_true * q_est, axis=1))
    dot = np.clip(dot, -1.0, 1.0)
    err = np.degrees(2.0 * np.arccos(dot))

    # 只保留相机样本覆盖到的区间，避免端点钳位污染统计
    valid = (t_grid >= t_src[0]) & (t_grid <= t_src[-1])
    err = err[valid]
    return {
        "t": t_grid[valid],
        "err_deg": err,
        "max_deg": float(err.max()),
        "rms_deg": float(np.sqrt(np.mean(err ** 2))),
    }


# ---------------------------------------------------------------------------
# 5. 路径与绘图辅助
# ---------------------------------------------------------------------------
def repo_root():
    """从本文件位置回溯到仓库根目录（src/ground/<模块>/<包>/core.py）。"""
    from pathlib import Path
    return Path(__file__).resolve().parents[4]


def default_out_dir():
    """图片默认输出到 docs/img/ground/camera_imu_time_calibration。"""
    from pathlib import Path
    d = repo_root() / "docs" / "img" / "ground" / "camera_imu_time_calibration"
    return d if (repo_root() / "docs").is_dir() else Path.cwd() / "out"


def style_axes(ax, xlabel: str, ylabel: str, title: str = None):
    """统一坐标轴样式：浅色网格 + 无上/右边框，便于放到浅色文档里。"""
    ax.grid(True, alpha=0.3, linewidth=0.6)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.set_xlabel(xlabel, fontsize=9)
    ax.set_ylabel(ylabel, fontsize=9)
    if title:
        ax.set_title(title, fontsize=10)
    ax.tick_params(labelsize=8)


def savefig(fig, path, dpi: int = 150):
    """保存并关闭图像，同时打印实际文件大小（提交前需保证 PNG < 3 MB）。"""
    from pathlib import Path
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=dpi, bbox_inches="tight", facecolor="white")
    import matplotlib.pyplot as plt
    plt.close(fig)
    size_kb = path.stat().st_size / 1024.0
    print(f"  [图] {path.name}  {size_kb:.0f} KB")
    return path
