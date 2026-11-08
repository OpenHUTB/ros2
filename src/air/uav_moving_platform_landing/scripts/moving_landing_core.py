#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""移动平台（车 / 船 / 移动甲板）精准降落：纯算法核心，不依赖 ROS，可离线回归。

本文件只做三件事，全部是纯函数 / 纯类，导入时不会碰 rospy：

1. ``DeckTracker`` —— 只用「平台中心位置」这一路**带延迟、带噪声**的观测，
   在线估出平台速度，并能回答「再过 T 秒平台在哪里」；
2. ``predict_touchdown`` —— 由当前相对高度与下降剖面反解「还要多久接地」（T），
   于是候选落点是 ``deck(t + T)``，而不是平台「现在」的位置；
3. ``MovingLandingController`` —— 状态机 + 制导：对准 → 下降 → 接地，
   全程用平台速度做前馈匹配，预计落偏超限时主动拉起复飞（go-around）。

为什么不能「直接追平台当前位置」
--------------------------------
设平台速度 ``v``、从感知到执行的总延迟 ``d``、下降段剩余时间 ``T``。
只追「最近一条实测位置」的朴素方案，接地时横向必然残留 ``v*(d+T)`` 量级的偏差：
``v = 1.0 m/s``、``d = 0.25 s``、``T = 3 s`` 时就是 3.25 m，远大于一块车顶/甲板。

本模块给了三种制导口径（``guide/lead_mode``），差别只在「朝哪一刻的平台位置飞」：

* ``measured``  —— 最近一条实测样本（朴素基线，不补偿任何延迟）；
* ``now``       —— 估计器给出的**当前**平滑位置 + 平台速度前馈（贴着平台飞）；
* ``touchdown`` —— ``deck(t+T)`` 相遇点（默认）。由于 ``dT/dt ≈ -1``，
  相遇点在世界上几乎静止，无人机是「飞到相遇点上方等平台滑过来」，
  因此**平台比无人机飞得还快时也能落**（跟着飞是不可能的，等它过来是可以的）。

坐标约定
--------
全部世界系 ENU（x 东、y 北、z 上为正），单位 m / s / s。
若上游高度是「下为正」，由调用方取反，本模块内部只认 z 向上为正。

数值口径与踩过的坑都写在文档 ``docs/air/uav_moving_platform_landing.md`` 里，
所有断言由 ``tests/test_uav_moving_platform_landing_local.py``（离线闭环）与
``tests/test_uav_moving_platform_landing_ros_sim.py``（ROS1 假桥接层闭环）实跑。
"""
from __future__ import annotations

import math

import numpy as np

# ---------------------------------------------------------------- 默认参数
#: 控制器 / 估计器的默认参数。ROS 侧 ``config/moving_platform_landing.yaml``
#: 与之同名同义，节点里用 ``rospy.get_param`` 逐个覆盖。
DEFAULT_PARAMS = {
    # ---- 平台状态估计 ----
    "tracker/window_s": 2.5,          # 滑窗长度（s）：太长跟不上加减速，太短噪声大
    "tracker/min_samples": 4,         # 少于这么多样本不做估计（此时原地悬停等）
    "tracker/half_life_s": 1.0,       # 指数衰减权重半衰期：越新的样本权重越大
    "tracker/robust_sigma": 3.0,      # MAD 粗筛倍数，去掉偶发跳点
    "tracker/quad_min_span_s": 1.0,   # 期望跨度不足这么久就只拟合一次项
    "tracker/max_accel": 2.5,         # 平台加速度上限（m/s^2），防止噪声放大二次项
    "tracker/quad_signif": 3.0,       # 二次项显著性阈值（几倍标准误）才算"在转弯"
    "tracker/curv_window_s": 4.0,     # 判曲率专用的长窗（4 m 半径 / 0.6 m/s 的圆周在
                                      # 2.5 s 窗内拱高只有 7.0 cm，和 4 cm 噪声同量级，
                                      # 判不出来；拉到 4 s 拱高 17.9 cm，检测率 113/115）
    "tracker/curv_half_life_s": 60.0,  # 长窗里用近似均匀权重（不做新样本加权）
    "tracker/gate_m": 1.0,            # 单样本跳点闸门（m）：先挡在拟合矩阵之外
    # ---- 下降剖面 ----
    "descend/speed": 0.4,             # 下降段恒定垂直速度（m/s）
    "descend/accel": 1.0,             # 垂直速度切换的加速度约束（m/s^2）
    "descend/touchdown_rel_height": 0.12,  # 相对高度低于此值即判接地（m）
    # ---- 制导 ----
    "guide/kp_xy": 1.2,               # 横向位置误差 → 速度指令的增益（1/s）
    "guide/kp_z": 1.5,                # 对准段高度保持增益（1/s）
    "guide/max_xy_speed": 1.6,        # 横向速度指令上限（m/s）
    "guide/max_z_speed": 1.0,         # 垂直速度指令上限（m/s）
    "guide/vff_gain": 1.0,            # 落点速度前馈系数（1.0 = 完全匹配）
    "guide/track_altitude": 2.5,      # 对准段保持在甲板上方多高（m）
    "guide/align_tol": 0.35,          # 与落点的横向偏差小于此值视为对准（m）
    "guide/align_settle_s": 0.5,      # 连续满足对准条件这么久才进入下降（s）
    "guide/abort_xy_tol": 0.9,        # 预计落偏超过此值就复飞（m）
    "guide/abort_hold_s": 0.4,        # 且持续这么久才复飞（s），避免单帧噪声触发
    "guide/lead_mode": "auto",        # auto / measured / now / touchdown
    "guide/vff": True,                # 是否叠加落点速度前馈
    "guide/max_lead_s": 30.0,         # 相遇点的最大预测时长（s），太长外推不可信
    "guide/deck_vz_smooth": 0.3,      # 平台垂向速度的一阶平滑系数（10 Hz 下约 0.3 s）
    "guide/deck_vz_deadband": 0.08,   # 平滑后仍小于此值就当平地（m/s），防止噪声放大
    "guide/lead_dev_tol": 0.25,       # 匀速外推允许的横向偏差（m），超过就收缩预测时长
    "guide/curv_lam_smooth": 0.2,     # 预测时长收缩系数的一阶平滑（10 Hz 下约 0.5 s）
    "guide/lead_smooth": 0.35,        # 相遇点低通（只对 touchdown 模式生效）
    "guide/curv_fallback_accel": 0.05,  # 横向加速度超过此值判定平台在转弯（m/s^2）
    "guide/curv_fallback_hold_s": 1.0,  # 持续这么久就把相遇点模式降级为速度匹配
    "guide/keep_up_ratio": 0.8,       # 平台速度超过 0.8*max_xy_speed 就认为"跟不住"
    "guide/heave_fallback_vz": 0.10,  # 平台垂向速度超过此值也降级（升沉平台，m/s）
    # ---- 保护与超时 ----
    "mission/align_timeout_s": 45.0,
    "mission/descend_timeout_s": 40.0,
    "mission/max_go_around": 3,
    "mission/min_below_deck_m": -0.35,  # 相对高度低于此值（穿到甲板下方）→ ABORT
}

#: 状态常量
WAIT_DECK = "WAIT_DECK"
ALIGN = "ALIGN"
DESCEND = "DESCEND"
TOUCHDOWN = "TOUCHDOWN"
LANDED = "LANDED"
ABORT = "ABORT"

#: 已终结的状态（再做 step 不会改变结果）
TERMINAL_STATES = (LANDED, ABORT)


class DeckMeasurement(object):
    """一次平台中心位置观测。

    Parameters
    ----------
    t : float
        观测时刻（ROS 里用 ``header.stamp``，**不是**收到消息的时刻）。
    pos : array-like (3,)
        平台中心位置（m，ENU）。
    """

    __slots__ = ("t", "pos")

    def __init__(self, t, pos):
        self.t = float(t)
        self.pos = np.asarray(pos, dtype=float).reshape(3)

    def __repr__(self):
        return "DeckMeasurement(t=%.3f, pos=[%.3f, %.3f, %.3f])" % (
            self.t, self.pos[0], self.pos[1], self.pos[2])


class DeckTracker(object):
    """只用位置观测，在线估计平台速度并做时间外推。

    实现上的四个取舍（都是实测逼出来的）：

    * **不用「最近两点差分」**：10 Hz 量测、5 cm 噪声时两点差分把噪声放大 1/dt 倍，
      速度误差能到 0.5~0.7 m/s，下降末段指令会来回抖。这里对滑窗内所有样本做
      **加权最小二乘**，噪声按样本数的平方根衰减。
    * **时间自变量取「相对查询时刻的偏移」**：拟合的常数项直接就是**查询时刻的平滑
      位置**，既不需要额外低通滤波，也天然把「观测比 now 旧 0.15 s」这件事吸收掉。
    * **先粗筛再拟合**：视觉 / 仿真接口偶尔会吐跳点（RPC 抖动几十米），用 MAD 的
      3σ 规则先剔掉再拟合，否则一个跳点能把速度估计带偏半秒。
    * **位置外推只用一次项**：二次项在窗口外不可信（窗口 2 s、加速度 2.5 m/s² 时，
      往外推 8 s 的误差能有几十米）。加速度只用于「判断平台在不在转弯」。
    """

    def __init__(self, window_s=2.5, min_samples=4, half_life_s=1.0,
                 robust_sigma=3.0, quad_min_span_s=1.0, max_accel=2.5,
                 quad_signif=3.0, gate_m=1.0):
        self.window_s = float(window_s)
        self.min_samples = int(min_samples)
        self.half_life_s = float(half_life_s)
        self.robust_sigma = float(robust_sigma)
        self.quad_min_span_s = float(quad_min_span_s)
        self.max_accel = float(max_accel)
        self.quad_signif = float(quad_signif)
        self.gate_m = float(gate_m)
        self.t = []
        self.p = []
        self._coef = None       # (degree+1, 3)，自变量是 (t - t_anchor)
        self._anchor = 0.0
        self._degree = 0

    # ------------------------------------------------------------ 样本管理
    def reset(self):
        self.t = []
        self.p = []
        self._coef = None
        self._anchor = 0.0
        self._degree = 0

    def __len__(self):
        return len(self.t)

    def push(self, t, pos):
        """塞一个观测。乱序 / 重复时间戳 / 不符合物理的跳点都会被挡掉。

        跳点必须在**进拟合矩阵之前**挡掉：一个 40 m 的跳点会把整条直线拉歪，
        之后所有样本的残差都跟着变，MAD 粗筛都救不回来（实测速度估计会从
        0.50 m/s 直接跳到 15.6 m/s）。闸门用「上一次拟合外推到本样本时刻的距离」，
        平台 0.1 s 内不可能移动 1 m 以上，所以取 1 m 是宽松且安全的。
        """
        t = float(t)
        pos = np.asarray(pos, dtype=float).reshape(3)
        if not np.all(np.isfinite(pos)):
            return False
        if self.t and t <= self.t[-1] + 1e-6:
            return False
        if (self.gate_m > 0 and len(self.t) >= max(2, self.min_samples)
                and (t - self.t[-1]) < 1.0):
            self._ensure(t)
            if self._coef is not None:
                pred = self._eval_pos(np.array([t - self._anchor]), self._coef)[0]
                if float(np.linalg.norm(pred - pos)) > self.gate_m:
                    return False
        self.t.append(t)
        self.p.append(pos)
        self._trim(t)
        self._coef = None
        return True

    def _trim(self, now):
        keep_from = now - self.window_s
        n = 0
        while n < len(self.t) - 2 and self.t[n] < keep_from:
            n += 1
        if n:
            self.t = self.t[n:]
            self.p = [np.asarray(x, dtype=float) for x in self.p[n:]]

    # ------------------------------------------------------------ 拟合
    def _fit(self, now):
        """在 ``now`` 处重新拟合。返回次数，0 表示样本不足。"""
        if len(self.t) < max(2, self.min_samples):
            self._coef = None
            self._degree = 0
            return 0
        ts = np.asarray(self.t, dtype=float)
        ps = np.asarray(self.p, dtype=float)
        mask = ts <= now + 1e-6          # 时间戳来自别的时钟时可能比 now 还"新"
        if mask.sum() < max(2, self.min_samples):
            self._coef = None
            self._degree = 0
            return 0
        ts = ts[mask]
        ps = ps[mask]
        tau = ts - now
        w = np.exp(-np.abs(tau) / max(1e-3, self.half_life_s))
        span = float(ts[-1] - ts[0])

        # 一次项打底；二次项要**统计显著**才采用
        deg = 1
        coef = self._wls(tau, ps, w, deg)
        if len(ts) >= 8 and span >= self.quad_min_span_s:
            coef2, se2 = self._wls_quad_stats(tau, ps, w)
            if coef2 is not None and np.any(
                    np.abs(coef2[2]) > self.quad_signif * se2[2] + 1e-9):
                deg = 2
                coef = coef2
                acc = np.clip(2.0 * coef[2], -self.max_accel, self.max_accel)
                coef = coef.copy()
                coef[2] = acc / 2.0

        # 粗筛：MAD 3σ 剔掉跳点后重拟合一次
        if self.robust_sigma > 0 and len(ts) >= 6:
            res = ps - self._eval(tau, coef)
            mad = np.median(np.abs(res - np.median(res, axis=0)), axis=0)
            sigma = np.maximum(1.4826 * mad, 1e-3)
            keep = np.all(np.abs(res) < self.robust_sigma * sigma, axis=1)
            if self.min_samples <= keep.sum() < len(ts):
                coef = self._wls(tau[keep], ps[keep], w[keep], deg)

        self._coef = coef
        self._anchor = now
        self._degree = deg
        return deg

    @staticmethod
    def _wls(tau, ps, w, deg):
        """加权最小二乘，返回 (deg+1, 3)；矩阵退化时自动降阶。"""
        for d in range(deg, -1, -1):
            A = np.vstack([tau ** k for k in range(d + 1)]).T
            try:
                coef, *_ = np.linalg.lstsq(A * w[:, None], ps * w[:, None], rcond=None)
            except np.linalg.LinAlgError:       # pragma: no cover - 极端退化
                continue
            if np.all(np.isfinite(coef)):
                return coef
        return np.zeros((deg + 1, 3))           # pragma: no cover

    @staticmethod
    def _wls_quad_stats(tau, ps, w):
        """二次拟合，并返回各轴系数 2 的标准误（用于显著性判定）。

        为什么用「显著性」而不是「残差下降了多少」：噪声为主时二次拟合根本降不了
        多少残差（4 m 半径、0.6 m/s 的圆周，2.5 s 窗口的拱高约 7.0 cm、2 s 窗口约
        4.5 cm，和 4 cm 的量测噪声同量级），靠残差比例判会一直判不出"在转弯"；反过来
        纯噪声又偶尔能凑出明显的残差下降，判进来就是伪曲率。用最小二乘的协方差算
        ``c2`` 的标准误再比阈值，两种情况都能分辨。
        """
        A = np.vstack([tau ** k for k in range(3)]).T
        Aw = A * w[:, None]
        try:
            coef, *_ = np.linalg.lstsq(Aw, ps * w[:, None], rcond=None)
            ata = Aw.T @ Aw
            cov_diag = np.diag(np.linalg.inv(ata))
        except np.linalg.LinAlgError:          # pragma: no cover - 极端退化
            return None, None
        res = ps * w[:, None] - Aw @ coef
        dof = max(1, int(len(tau)) - 3)
        sigma2 = np.sum(res ** 2, axis=0) / dof
        se = np.sqrt(np.maximum(cov_diag[2] * sigma2, 1e-12))
        if not (np.all(np.isfinite(coef)) and np.all(np.isfinite(se))):
            return None, None
        return coef, np.vstack([np.zeros(3), np.zeros(3), se])

    @staticmethod
    def _eval(tau, coef):
        out = np.zeros((len(tau), 3))
        for k in range(coef.shape[0]):
            out += np.outer(tau ** k, coef[k])
        return out

    @staticmethod
    def _eval_pos(tau, coef):
        """位置外推**只用一次项**（常数 + 速度）。

        二次项只在窗口内可信：窗口 2 s、加速度 2.5 m/s² 时往外推 8 s 的误差能有
        几十米。实测把二次项带进落点预测后，相遇点会随机跳动、状态机一直对不准。
        因此加速度只用于 ``curvature_accel`` 的模式判定，绝不参与外推 —— 这也是
        绝大多数船舶 / 车辆甲板着陆算法的做法：预测窗口内按匀速假设，加速度交给
        控制器的反馈去吸收。
        """
        out = np.empty((len(tau), 3))
        out[:] = coef[0]
        if coef.shape[0] > 1:
            out += np.outer(tau, coef[1])
        return out

    # ------------------------------------------------------------ 查询
    def _ensure(self, t_ref):
        """有新样本进来后第一次查询时重拟合（锚点取本次查询时刻）。"""
        if self._coef is None:
            self._fit(float(t_ref))

    def ready(self, now):
        """样本够不够做估计。"""
        self._ensure(now)
        return self._coef is not None

    def age(self, now):
        """最新样本离 now 有多旧（s）。没样本时返回 inf。"""
        if not self.t:
            return float("inf")
        return float(now - self.t[-1])

    def is_stale(self, now, max_age_s=0.6):
        return self.age(now) > max_age_s

    def position_at(self, t_query):
        """预测 ``t_query`` 时刻的平台中心位置。"""
        if not self.p:
            raise RuntimeError("DeckTracker 还没有任何观测")
        self._ensure(t_query)
        return self._eval_pos(np.array([float(t_query) - self._anchor]), self._coef)[0]

    def positions_at(self, t_queries):
        """向量化版本：一次算一排时刻的预测位置，返回 (N, 3)。"""
        tq = np.asarray(t_queries, dtype=float).reshape(-1)
        if not self.p:
            raise RuntimeError("DeckTracker 还没有任何观测")
        self._ensure(tq[0] if len(tq) else 0.0)
        return self._eval_pos(tq - self._anchor, self._coef)

    def velocity_at(self, t_query=None):
        """``t_query``（默认最新样本时刻）时刻的平台速度估计。"""
        if not self.p:
            return np.zeros(3)
        tau_ref = self.t[-1] if t_query is None else float(t_query)
        self._ensure(tau_ref)
        tau = tau_ref - self._anchor
        v = np.zeros(3)
        for k in range(1, self._coef.shape[0]):
            v += self._coef[k] * (k * tau ** (k - 1))
        return v

    def acceleration_at(self, t_query=None):
        if not self.p:
            return np.zeros(3)
        tau_ref = self.t[-1] if t_query is None else float(t_query)
        self._ensure(tau_ref)
        if self._coef is None or self._coef.shape[0] < 3:
            return np.zeros(3)
        return 2.0 * self._coef[2]

    def curvature_accel(self, t_query=None, axis="xy"):
        """横向加速度幅值（m/s^2），由二次项拟合给出并限幅。

        只用来判断「平台在不在转弯」，不参与位置外推 —— 即使噪声凑出伪加速度，
        后果也只是把制导降级成「贴着平台飞」，不会把落点甩飞。
        """
        if not self.p:
            return 0.0
        a = self.acceleration_at(t_query)
        n = math.hypot(a[0], a[1]) if axis == "xy" else abs(a[2])
        return float(min(self.max_accel, n))

    def degree(self):
        return self._degree


# ---------------------------------------------------------------- 下降剖面
def descend_profile(t, vz0, descend_speed, accel):
    """从当前垂直速度 ``vz0`` 按加速度约束过渡到 ``-descend_speed`` 的位移 / 速度。

    Parameters
    ----------
    t : float or array
        从剖面起点开始经过的时间（s）。
    vz0 : float
        起点垂直速度（m/s，向上为正）。
    descend_speed : float
        目标下降速度（m/s，正数，内部取负）。
    accel : float
        速度切换的加速度约束（m/s^2，正数）。

    Returns
    -------
    (dz, vz) : 垂直位移（m，向上为正）与垂直速度（m/s）
    """
    t = np.asarray(t, dtype=float)
    target = -abs(float(descend_speed))
    accel = max(1e-6, abs(float(accel)))
    if vz0 > target:
        ramp = (vz0 - target) / accel
        dz = np.where(t < ramp,
                      vz0 * t - 0.5 * accel * t ** 2,
                      vz0 * ramp - 0.5 * accel * ramp ** 2 + target * (t - ramp))
        vz = np.where(t < ramp, vz0 - accel * t, target)
    else:                                       # 本来就在以更快的速度下降
        dz = vz0 * t
        vz = np.full_like(t, vz0)
    return dz, vz


def predict_touchdown(z_rel, tracker, now, vz0, descend_speed, accel,
                      horizon_s=None, step_s=0.05, max_lead_s=12.0, deck_vz=None):
    """反解「还要多久接地」，返回 ``(T, 落点预测)``；解不出返回 ``(None, None)``。

    ``z_rel`` 是**当前**无人机相对平台的垂直高度（m）。判据：

        z_rel + dz(T) == deck_z(now + T) - deck_z(now)

    平台有升沉（船、起伏地面）时 ``deck_z`` 本身随时间变，用细扫描 + 线性插值解，
    比假设 ``T = z_rel / v`` 更准（后者在 0.15 m 升沉、4 s 周期时误差约 0.6 s，
    对应 0.3 m 的落点误差）。

    ``deck_vz`` 给定时用「平台垂向速度恒为该值」的线性模型，而不是估计器的外推。
    **这条很重要**：``T = z_rel / (descend_speed + v_deck_z)`` 对 ``v_deck_z`` 的
    放大倍数是 ``T/descend_speed ≈ 15``，平台是平地（真值 0）时，估计器零点几
    m/s 的噪声就能让 T 在 5 s 和 8 s 之间来回跳，落点跟着横向抖 1 m 以上。
    所以调用方先对垂向速度做平滑 + 死区，再喂进来。
    """
    if z_rel is None or not np.isfinite(z_rel):
        return None, None
    if z_rel <= 0.0:
        return 0.0, tracker.position_at(now)
    if horizon_s is None:
        horizon_s = min(60.0, max(2.0, 3.0 * z_rel / max(1e-3, abs(descend_speed)) + 2.0))
    if max_lead_s is not None:
        horizon_s = min(float(horizon_s), float(max_lead_s))
    ts = np.arange(0.0, float(horizon_s) + 1e-9, float(step_s))
    dz, _ = descend_profile(ts, vz0, descend_speed, accel)
    deck_now = tracker.position_at(now)
    if deck_vz is None:
        deck_z = tracker.positions_at(now + ts)[:, 2] - deck_now[2]
    else:
        deck_z = float(deck_vz) * ts
    f = z_rel + dz - deck_z
    if f[0] <= 0.0:
        return 0.0, tracker.position_at(now)
    neg = np.where(f <= 0.0)[0]
    if len(neg) == 0:
        return None, None
    i = int(neg[0])
    f0, f1 = float(f[i - 1]), float(f[i])
    frac = f0 / (f0 - f1) if abs(f0 - f1) > 1e-12 else 0.0
    T = float(ts[i - 1] + frac * (ts[i] - ts[i - 1]))
    return T, tracker.position_at(now + T)


# ---------------------------------------------------------------- 控制器
class MovingLandingController(object):
    """移动平台降落状态机 + 制导。

    用法（10 Hz 调用）::

        ctl = MovingLandingController()
        out = ctl.step(now, drone_pos, drone_vel, DeckMeasurement(t, deck_pos))
        # out["cmd_vel"] -> (vx, vy, vz) 世界系 ENU
        # out["state"]   -> WAIT_DECK / ALIGN / DESCEND / TOUCHDOWN / LANDED / ABORT
        # out["events"]  -> [{"name": "TOUCHDOWN", "error_xy": 0.06, ...}, ...]

    状态流转::

        WAIT_DECK ──样本足够──▶ ALIGN ──连续 align_settle_s 满足对准──▶ DESCEND
        DESCEND ──预计落偏超限持续 abort_hold_s──▶ ALIGN（复飞，计数 +1）
        DESCEND ──相对高度 ≤ touchdown_rel_height──▶ TOUCHDOWN ──▶ LANDED
        任意状态 ──超时 / 复飞次数用完 / 穿到甲板下面──▶ ABORT（原地悬停）
    """

    def __init__(self, params=None):
        p = dict(DEFAULT_PARAMS)
        if params:
            p.update(params)
        self.p = p
        self.tracker = DeckTracker(
            window_s=p["tracker/window_s"],
            min_samples=p["tracker/min_samples"],
            half_life_s=p["tracker/half_life_s"],
            robust_sigma=p["tracker/robust_sigma"],
            quad_min_span_s=p["tracker/quad_min_span_s"],
            max_accel=p["tracker/max_accel"],
            quad_signif=p["tracker/quad_signif"],
            gate_m=p["tracker/gate_m"],
        )
        # 判「平台在转弯吗」用一条独立的长窗估计：窗口越长拱高越大、越容易和噪声
        # 分开，但长窗的速度估计跟不上加减速 —— 所以只拿它做模式判定，不拿它算落点。
        self.curv_tracker = DeckTracker(
            window_s=p["tracker/curv_window_s"],
            min_samples=max(8, p["tracker/min_samples"]),
            half_life_s=p["tracker/curv_half_life_s"],
            robust_sigma=p["tracker/robust_sigma"],
            quad_min_span_s=p["tracker/quad_min_span_s"],
            max_accel=p["tracker/max_accel"],
            quad_signif=p["tracker/quad_signif"],
            gate_m=p["tracker/gate_m"],
        )
        self.reset()

    # ------------------------------------------------------------ 生命周期
    def reset(self, autostart=True):
        self.tracker.reset()
        self.curv_tracker.reset()
        self.state = WAIT_DECK if autostart else ABORT
        self.state_since = None
        self.align_since = None
        self.bad_since = None
        self.curv_since = None
        self.fast_since = None
        self.lead_latch = None
        self.lead_fallback = False
        self.lead_mode_eff = self.p["guide/lead_mode"]
        self.vz_deck_smooth = None
        self.lam_smooth = 1.0
        self.lead_smooth = None
        self.go_around = 0
        self.events = []
        self.landed_error = None
        self.last_T = None
        self.last_lead = None
        self.last_deck = None
        self.last_vdeck = None
        self.last_z_rel = None
        self.start_time = None
        self.last_step_time = None
        self.dt = 0.1
        self.n_events_before = 0
        self.state_history = []

    def _set_state(self, state, now, reason=None):
        if state == self.state:
            return
        self.state_history.append((now, state, reason))
        self.state = state
        self.state_since = now
        self.events.append({"name": "STATE", "state": state,
                            "t": now, "reason": reason})

    # ------------------------------------------------------------ 主步骤
    def step(self, now, drone_pos, drone_vel, measurement=None):
        """推进一帧。

        Parameters
        ----------
        now : float
            当前时刻（s）。ROS 里用 ``rospy.Time`` 换算的秒数。
        drone_pos, drone_vel : array-like (3,)
            无人机位置 / 速度（m, m/s，ENU）。
        measurement : DeckMeasurement or None
            本帧新到的平台观测（可能是 None）。

        Returns
        -------
        dict
            见类文档。
        """
        now = float(now)
        pos = np.asarray(drone_pos, dtype=float).reshape(3)
        vel = np.asarray(drone_vel, dtype=float).reshape(3)
        if self.start_time is None:
            self.start_time = now
            self.state_since = now
            self.state_history.append((now, self.state, "启动"))
        if self.last_step_time is not None:
            self.dt = max(1e-3, min(0.5, now - self.last_step_time))
        self.last_step_time = now
        self.n_events_before = len(self.events)

        if self.state in TERMINAL_STATES:
            return self._result(now, np.zeros(3), pos, vel)

        if measurement is not None:
            self.tracker.push(measurement.t, measurement.pos)
            self.curv_tracker.push(measurement.t, measurement.pos)

        # 平台在转弯时「匀速外推 6 s 的相遇点」本身就不可信（4 m 半径、1 m/s 的
        # 圆周在 6 s 内转过 90°，外推点偏离圆 1.4 m），此时一次性降级成速度匹配式
        # 跟踪；只降级一次，不来回抖。
        self._update_lead_mode(now)

        if self.state == WAIT_DECK:
            if self.tracker.ready(now) and not self.tracker.is_stale(now):
                self._set_state(ALIGN, now, "平台观测就绪")
            else:
                return self._result(now, np.zeros(3), pos, vel)   # 原地悬停等

        deck_now = self.tracker.position_at(now)
        vdeck = self.tracker.velocity_at(now)
        z_rel = pos[2] - deck_now[2]
        self.last_deck = deck_now
        self.last_vdeck = vdeck
        self.last_z_rel = z_rel

        if self.state == ALIGN:
            return self._step_align(now, pos, vel, deck_now, vdeck, z_rel)
        if self.state == DESCEND:
            return self._step_descend(now, pos, vel, deck_now, vdeck, z_rel)
        return self._result(now, np.zeros(3), pos, vel)           # TOUCHDOWN / LANDED

    # ------------------------------------------------------------ 模式降级
    def _update_lead_mode(self, now):
        """决定本帧用哪种口径：``measured`` / ``now`` / ``touchdown``。

        ``auto``（默认）的行为：
        * 平台速度在无人机横向速度上限之内 → ``now``（贴着平台飞，精度最好，
          实测落点误差 0.03~0.09 m）；
        * 平台比无人机还快（跟不住）→ 锁存成 ``touchdown``（飞到相遇点等它滑过来）；
        * 平台在转弯或有升沉 → 相遇点本身不可信，退回 ``now``。
        """
        p = self.p
        req = p["guide/lead_mode"]
        if req in ("measured", "now"):
            self.lead_mode_eff = req
            return
        ready = self.tracker.ready(now) and not self.tracker.is_stale(now)
        v = self.tracker.velocity_at(now) if ready else np.zeros(3)
        speed = math.hypot(v[0], v[1])

        if req == "auto":
            if speed > p["guide/keep_up_ratio"] * p["guide/max_xy_speed"]:
                if self.fast_since is None:
                    self.fast_since = now
                elif now - self.fast_since >= p["guide/curv_fallback_hold_s"]:
                    if self.lead_latch != "touchdown":
                        self.events.append({
                            "name": "LEAD_INTERCEPT", "t": now, "speed": speed,
                            "reason": "平台速度 %.2f m/s 超过无人机横向上限的 %.0f%%，"
                                      "改为相遇点等待式降落"
                                      % (speed, 100 * p["guide/keep_up_ratio"])})
                    self.lead_latch = "touchdown"
            else:
                self.fast_since = None

        mode = "touchdown" if (req == "touchdown" or self.lead_latch == "touchdown") else "now"
        if self.lead_fallback:
            mode = "now"
        self.lead_mode_eff = mode
        if mode != "touchdown" or not ready:
            return

        # 升沉平台（船、起伏地面）同样不适合"等相遇点"：T 随升沉相位在 3~7 s 之间
        # 变，相遇点横着来回跑一两米，根本上不了对准容差。
        heave = abs(self.vz_deck_smooth or 0.0)
        acc = self.curv_tracker.curvature_accel(now)
        hard = (acc > p["guide/curv_fallback_accel"]
                or heave > p["guide/heave_fallback_vz"])
        if hard:
            if self.curv_since is None:
                self.curv_since = now
            elif now - self.curv_since >= p["guide/curv_fallback_hold_s"]:
                self.lead_fallback = True
                self.lead_mode_eff = "now"
                self.curv_since = None
                self.events.append({
                    "name": "LEAD_FALLBACK", "t": now, "accel": acc, "heave_vz": heave,
                    "reason": "平台横向加速度 %.2f m/s^2 / 垂向速度 %.2f m/s，"
                              "相遇点外推不可信，改为速度匹配式跟踪" % (acc, heave)})
        else:
            self.curv_since = None

    # ------------------------------------------------------------ 落点与引导
    def _lead_point(self, now, pos, vel, deck_now, vdeck, z_rel, mode, z_for_T=None):
        """算「该往哪飞」：返回 ``(T, lead, v_lead)``。

        ``v_lead`` 是落点自身的速度，用作速度前馈。它不等于平台速度：由于
        ``dT/dt ≈ -1``（高度以 ``descend_speed`` 减小、剩余时间同步减少），

            d/dt deck(t + T(t)) = v_deck * (1 + dT/dt) ≈ 0

        也就是**下降段的目标点在世界上几乎是静止的**：无人机飞到相遇点上方悬停，
        平台自己滑到它下面完成降落。这一点很关键 —— 当平台速度超过无人机横向
        速度上限时，「跟着平台飞」不可能，但「在相遇点等它」仍然可行。
        """
        if mode == "measured":
            last = self.tracker.p[-1] if len(self.tracker) else deck_now
            return 0.0, np.asarray(last, dtype=float), np.zeros(3)
        if mode == "now":
            return 0.0, deck_now, vdeck

        # T 用**标称下降剖面**算，而不是当前瞬时垂直速度：对准段机体可能在用
        # 1 m/s 快速下降接近甲板高度，把那个瞬时速度代进去 T 会从 6 s 掉到 2.5 s，
        # 相遇点跟着横向跳 1.8 m，目标点抖得没法对准（实测确实一直对不准）。
        # 对按时的高度也要用**目标巡航高度**而不是当前高度：在 8 m 高空按"现在就
        # 开始下降"算 T 会得到 20 s（被 max_lead_s 截到 12 s），相遇点直接甩到
        # 12 m 开外，等它降到 2.5 m 时 T 又掉到 6 s，目标点横跳 6 m。
        if z_for_T is None:
            z_for_T = z_rel
        # 平台垂向速度：平滑 + 死区后再进 T 的求解（平地时估计器那点噪声会被
        # T = z/(v_desc + vz) 放大 15 倍，落点跟着抖 1 m 以上）。
        vz_deck = vdeck[2]
        alpha = self.p["guide/deck_vz_smooth"]
        if self.vz_deck_smooth is None:
            self.vz_deck_smooth = vz_deck
        else:
            self.vz_deck_smooth = (1.0 - alpha) * self.vz_deck_smooth + alpha * vz_deck
        vz_used = self.vz_deck_smooth
        if abs(vz_used) < self.p["guide/deck_vz_deadband"]:
            vz_used = 0.0
        T, lead = predict_touchdown(
            z_for_T, self.tracker, now,
            -abs(self.p["descend/speed"]),
            self.p["descend/speed"], self.p["descend/accel"],
            max_lead_s=self.p["guide/max_lead_s"], deck_vz=vz_used)
        if T is None:                       # 解不出（例如平台一直在往上升）
            return 0.0, deck_now, vdeck

        # 曲率收缩：匀速外推在转弯平台上的横向偏差是 0.5*|a|*T^2（4 m 半径、0.6 m/s
        # 的圆周、T=6.25 s 时 1.75 m，比整块甲板还大，一直对不准）。按"偏差不超过
        # guide/lead_dev_tol"把预测时长压到 T_need；直线时 |a|≈0，不生效。
        # 收缩系数再三做一阶平滑：圆周转弯时显著性判据会在"曲率显著/不显著"之间
        # 抖（拱高和噪声同量级），不滤波的话 T 会在 6.25 s 和 1.9 s 之间来回跳，
        # 落点跟着横跳好几米，状态机永远对不准（实测确实卡在这里）。
        acc = self.curv_tracker.curvature_accel(now)
        tol = self.p["guide/lead_dev_tol"]
        lam_raw = 1.0
        if acc > 1e-3 and tol > 0:
            lam_raw = min(1.0, math.sqrt(2.0 * tol / acc) / T)
        alpha = self.p["guide/curv_lam_smooth"]
        self.lam_smooth = (1.0 - alpha) * self.lam_smooth + alpha * lam_raw
        lam = self.lam_smooth
        if lam < 0.999:
            T = T * lam
            lead = self.tracker.position_at(now + T)

        # 落点平滑：相遇点在理想情况下是**静止**的，所以对它有低通不会带来稳态滞后，
        # 却能把速度估计噪声（0.6 m/s 平台上约 0.03 m/s，乘 T=6 s 就是 0.2 m）滤掉。
        # 平滑强度按 lam 缩放：lam 小（转弯、相遇点本身在快速移动）就不平滑，
        # 否则低通滞后会变成 0.3 m 的稳态偏差，正好把对准卡在容差边上。
        beta = self.p["guide/lead_smooth"] * lam
        if beta > 0:
            if self.lead_smooth is None:
                self.lead_smooth = np.asarray(lead, dtype=float)
            else:
                self.lead_smooth = (1.0 - beta) * self.lead_smooth + beta * np.asarray(lead, float)
            lead = self.lead_smooth
        else:
            self.lead_smooth = np.asarray(lead, dtype=float)

        denom = self.p["descend/speed"] + vz_used
        if abs(denom) > 0.05:
            tdot = (vel[2] - vz_used) / denom
        else:                               # pragma: no cover - 极端情形
            tdot = -1.0
        tdot = max(-1.5, min(0.5, tdot))
        return float(T), np.asarray(lead, dtype=float), vdeck * (1.0 + lam * tdot)

    # ------------------------------------------------------------ 对准
    def _step_align(self, now, pos, vel, deck_now, vdeck, z_rel):
        p = self.p
        dt = self.dt
        mode = self.lead_mode_eff
        # 平台比无人机横向速度上限还快时，"保持巡航高度跟着飞"根本做不到（越追越远）。
        # 这种情况下直接按"从现在开始下降"算相遇点，边下降边等平台滑到下面来。
        cannot_keep_up = (math.hypot(vdeck[0], vdeck[1])
                          > p["guide/keep_up_ratio"] * p["guide/max_xy_speed"])
        T, lead, v_lead = self._lead_point(
            now, pos, vel, deck_now, vdeck, z_rel, mode,
            z_for_T=None if cannot_keep_up else p["guide/track_altitude"])
        self.last_T = T
        self.last_lead = lead

        vff = p["guide/vff_gain"] if p["guide/vff"] else 0.0
        vx = vff * v_lead[0] + p["guide/kp_xy"] * (lead[0] - pos[0])
        vy = vff * v_lead[1] + p["guide/kp_xy"] * (lead[1] - pos[1])
        if cannot_keep_up:
            vz = max(vel[2] - p["descend/accel"] * dt, -abs(p["descend/speed"]))
        else:
            vz = (vff * vdeck[2]
                  + p["guide/kp_z"] * (deck_now[2] + p["guide/track_altitude"] - pos[2]))
        cmd = self._clip((vx, vy, vz))

        err_xy = float(math.hypot(lead[0] - pos[0], lead[1] - pos[1]))
        height_ok = cannot_keep_up or abs(z_rel - p["guide/track_altitude"]) <= 0.6
        if err_xy <= p["guide/align_tol"] and height_ok and not self.tracker.is_stale(now):
            if self.align_since is None:
                self.align_since = now
            elif now - self.align_since >= p["guide/align_settle_s"]:
                self._set_state(DESCEND, now, "对准完成（与落点差 %.2f m）" % err_xy)
                self.bad_since = None
                return self._step_descend(now, pos, vel, deck_now, vdeck, z_rel)
        else:
            self.align_since = None
            if self.tracker.is_stale(now):
                self._set_state(WAIT_DECK, now, "平台观测断流")

        if z_rel < p["mission/min_below_deck_m"]:
            return self._abort(now, pos, vel, "相对高度 %.2f m 低于甲板" % z_rel)

        if cannot_keep_up and z_rel <= p["descend/touchdown_rel_height"]:
            # 边对准边下降时可能先接地（平台从下面滑过来了）
            self.landed_error = err_xy
            self._set_state(TOUCHDOWN, now, "接地，横向误差 %.2f m" % err_xy)
            self.events.append({"name": "TOUCHDOWN", "t": now, "error_xy": err_xy,
                                "error_lead": err_xy})
            self._set_state(LANDED, now, "落稳")
            return self._result(now, np.zeros(3), pos, vel)

        if now - (self.state_since or now) > p["mission/align_timeout_s"]:
            return self._abort(now, pos, vel,
                               "对准超时 %.0f s（离落点还差 %.2f m）"
                               % (p["mission/align_timeout_s"], err_xy))
        return self._result(now, np.asarray(cmd, dtype=float), pos, vel)

    # ------------------------------------------------------------ 下降
    def _step_descend(self, now, pos, vel, deck_now, vdeck, z_rel):
        p = self.p
        dt = self.dt
        mode = self.lead_mode_eff
        err_now_xy = float(math.hypot(pos[0] - deck_now[0], pos[1] - deck_now[1]))

        # 1) 落点
        T, lead, v_lead = self._lead_point(now, pos, vel, deck_now, vdeck, z_rel, mode)
        self.last_T = T
        self.last_lead = lead
        err_lead_xy = float(math.hypot(lead[0] - pos[0], lead[1] - pos[1]))

        # 2) 横向：朝落点 + 落点速度前馈
        vff = p["guide/vff_gain"] if p["guide/vff"] else 0.0
        vx = vff * v_lead[0] + p["guide/kp_xy"] * (lead[0] - pos[0])
        vy = vff * v_lead[1] + p["guide/kp_xy"] * (lead[1] - pos[1])
        # 3) 垂直：按加速度约束一步步过渡到恒定下降速度。必须按控制周期推进剖面 ——
        #    用 dt=0 算的话指令永远是"当前的垂直速度"，无人机压根不会往下走
        #    （实测就卡在 2.3 m 高度一直到下降超时）。
        vz_cmd = max(vel[2] - p["descend/accel"] * dt, -abs(p["descend/speed"]))
        cmd = self._clip((vx, vy, float(vz_cmd)))

        # 4) 安全：会不会钻到甲板下面
        if z_rel < p["mission/min_below_deck_m"]:
            return self._abort(now, pos, vel, "相对高度 %.2f m 低于甲板" % z_rel)

        # 5) 接地判定（只看相对高度，仿真 / 真机口径一致）
        if z_rel <= p["descend/touchdown_rel_height"]:
            self.landed_error = err_now_xy
            self._set_state(TOUCHDOWN, now, "接地，横向误差 %.2f m" % err_now_xy)
            self.events.append({"name": "TOUCHDOWN", "t": now,
                                "error_xy": err_now_xy,
                                "error_lead": err_lead_xy})
            self._set_state(LANDED, now, "落稳")
            return self._result(now, np.zeros(3), pos, vel)

        # 6) 预计会落偏 → 复飞。判据看「离落点多远」而不是「离平台现在多远」：
        #    相遇点在前方时后者本来就很大，用它当判据会一直误触发。
        if err_lead_xy > p["guide/abort_xy_tol"]:
            if self.bad_since is None:
                self.bad_since = now
            elif now - self.bad_since > p["guide/abort_hold_s"]:
                self.go_around += 1
                self.bad_since = None
                self.align_since = None
                self.events.append({"name": "GO_AROUND", "t": now,
                                    "error_xy": err_lead_xy,
                                    "count": self.go_around})
                if self.go_around > p["mission/max_go_around"]:
                    return self._abort(now, pos, vel,
                                       "复飞 %d 次仍对不准（预计落偏 %.2f m）"
                                       % (self.go_around, err_lead_xy))
                self._set_state(ALIGN, now, "预计落偏 %.2f m，复飞" % err_lead_xy)
                return self._result(now, np.asarray(cmd, dtype=float), pos, vel)
        else:
            self.bad_since = None

        if now - (self.state_since or now) > p["mission/descend_timeout_s"]:
            return self._abort(now, pos, vel,
                               "下降超时 %.0f s（剩余 %.2f m）"
                               % (p["mission/descend_timeout_s"], z_rel))
        return self._result(now, np.asarray(cmd, dtype=float), pos, vel)

    # ------------------------------------------------------------ 收尾
    def _abort(self, now, pos, vel, reason):
        self._set_state(ABORT, now, reason)
        self.events.append({"name": "ABORT", "t": now, "reason": reason})
        return self._result(now, np.zeros(3), pos, vel)

    def _clip(self, v):
        p = self.p
        v = np.asarray(v, dtype=float).reshape(3)
        vmax = p["guide/max_xy_speed"]
        n = math.hypot(v[0], v[1])
        if n > vmax and n > 1e-9:
            v[0] *= vmax / n
            v[1] *= vmax / n
        vzmax = p["guide/max_z_speed"]
        v[2] = max(-vzmax, min(vzmax, v[2]))
        return v

    def _result(self, now, cmd, pos, vel):
        return {
            "state": self.state,
            "cmd_vel": (float(cmd[0]), float(cmd[1]), float(cmd[2])),
            "t": now,
            "time_to_touchdown": self.last_T,
            "lead": None if self.last_lead is None else np.array(self.last_lead),
            "deck": None if self.last_deck is None else np.array(self.last_deck),
            "v_deck": None if self.last_vdeck is None else np.array(self.last_vdeck),
            "z_rel": self.last_z_rel,
            "go_around": self.go_around,
            "landed_error": self.landed_error,
            "lead_mode": self.lead_mode_eff,
            "events": list(self.events),
            "new_events": list(self.events[self.n_events_before:]),
            "drone_pos": np.array(pos),
            "drone_vel": np.array(vel),
        }

    # ------------------------------------------------------------ 报告
    def report(self):
        return {
            "state": self.state,
            "landed_error": self.landed_error,
            "go_around": self.go_around,
            "lead_mode": self.lead_mode_eff,
            "states": [(float(t), s, r) for (t, s, r) in self.state_history],
        }


def summarize_trial(ctl, truth_error_xy=None):
    """把一次降落的控制器状态压成一行可比较的指标。"""
    return {
        "state": ctl.state,
        "landed": ctl.state == LANDED,
        "aborted": ctl.state == ABORT,
        "go_around": ctl.go_around,
        "error_xy": (ctl.landed_error if ctl.landed_error is not None
                     else (truth_error_xy if truth_error_xy is not None
                           else float("nan"))),
    }
