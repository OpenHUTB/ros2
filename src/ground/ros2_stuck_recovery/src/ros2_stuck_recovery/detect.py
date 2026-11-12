"""卡死检测：用"指令 / 轮式里程计 / IMU（+前向测距）"三源一致性判定故障类型。

关键点：轮式里程计来自编码器，**打滑时它也在"正常"往前走**，只看
``指令 vs 里程计`` 只能抓到"顶住不动"这一类。因此检测器并联三路证据：

* 通道 A（无需外部传感器）：指令要求前进，里程计几乎不前进 -> 判「被顶住 / 原地打转」；
* 通道 B（有 IMU 时）：里程计角速度与陀螺仪角速度对不上 -> 判「单轮悬空 / 编码器失效」；
* 通道 C（有前向测距时）：里程计说走了 0.3 m，前向距离却几乎没变 -> 判「轮胎空转打滑」。

三路各自累计"持续时间"，超过确认时长才报警，避免把起步/加减速的暂态误判成卡死。
通道 C 额外要求"最近确实接近过某个面"：贴墙平行行驶时前向距离恒定、里程计正常，
与打滑在单线测距下完全同构，只能靠这条记忆把它们分开（见文档"已知局限"）。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

CAUSE_NONE = "none"
CAUSE_BLOCKED = "blocked"
CAUSE_SPIN = "spin"
CAUSE_SLIP = "slip"
CAUSE_ONE_WHEEL = "one_wheel"

CAUSE_ZH = {
    CAUSE_NONE: "正常",
    CAUSE_BLOCKED: "被顶住",
    CAUSE_SPIN: "原地打转",
    CAUSE_SLIP: "轮胎打滑",
    CAUSE_ONE_WHEEL: "单轮失效/悬空",
}


@dataclass(frozen=True)
class Sample:
    """一个控制周期内的观测。角度单位 rad，长度单位 m，时间单位 s。"""

    t: float
    v_cmd: float
    w_cmd: float
    v_odom: float
    w_odom: float
    w_imu: Optional[float] = None
    range_front: Optional[float] = None


@dataclass(frozen=True)
class DetectorConfig:
    window: float = 1.40            # 滑窗长度 [s]
    v_min: float = 0.05             # 认为"指令要求移动"的速度下限 [m/s]
    ratio_thr: float = 0.30         # 实测/指令位移比低于此值算"几乎没动"
    confirm: float = 0.70           # 证据需持续的时间 [s]
    min_cmd_dist: float = 0.02      # 窗口内累计指令位移下限 [m]，滤掉抖动
    spin_w: float = 0.30            # 判定"原地打转"的里程计角速度下限 [rad/s]
    imu_gap: float = 0.20           # 里程计与 IMU 角速度差阈值 [rad/s]
    imu_w_min: float = 0.15         # 两侧角速度至少要有一侧超过该值 [rad/s]
    range_near: float = 1.00        # 只在前向距离小于该值时才相信测距证据 [m]
    range_slip_ratio: float = 0.35  # 前向距离减少量 / 里程计位移，低于此值算空转
    min_odom_dist_slip: float = 0.15  # 打滑判定所需的最小里程计位移 [m]
    range_ema: float = 0.30         # 测距指数平滑系数（抑制单帧噪声）
    approach_memory: float = 4.00   # "刚接近过某个面"的记忆时长 [s]
    approach_eps: float = 0.005     # 刷新"最近最低测距"所需的最小下降量 [m]


@dataclass(frozen=True)
class Verdict:
    """检测结论。``span`` 记录三路证据各自累计了多少秒。"""

    stuck: bool = False
    cause: str = CAUSE_NONE
    slip_ratio: float = 1.0
    confidence: float = 0.0
    cmd_dist: float = 0.0
    odom_dist: float = 0.0
    span: dict = field(default_factory=dict)

    @property
    def cause_zh(self) -> str:
        return CAUSE_ZH.get(self.cause, self.cause)


class StuckDetector:
    """滑窗三路一致性检测器。有状态的流式接口：每个控制周期调用一次 ``update``。"""

    def __init__(self, config: Optional[DetectorConfig] = None) -> None:
        self.cfg = config or DetectorConfig()
        self._buf: list[Sample] = []
        self._range_ema: Optional[float] = None
        self._ema_buf: list[tuple[float, float]] = []
        self._range_min = float("inf")
        self._approach_t = -1e9
        self._slip_hold = 0.0

    def reset(self) -> None:
        self._buf.clear()
        self._ema_buf.clear()
        self._range_ema = None
        self._range_min = float("inf")
        self._approach_t = -1e9
        self._slip_hold = 0.0

    # ------------------------------------------------------------------ 内部
    def _update_range(self, sample: Sample) -> None:
        """测距做指数平滑，并用"是否刷新近期最低值"来积累接近证据。

        直接对相邻两帧做差会被 1 cm 级噪声淹没（噪声的整流效应会凭空累加出
        几十厘米的"接近量"），因此这里平滑后再比较。
        """
        if sample.range_front is None:
            return
        cfg = self.cfg
        r = sample.range_front
        if self._range_ema is None:
            self._range_ema = r
            self._range_min = r
        else:
            self._range_ema = (1.0 - cfg.range_ema) * self._range_ema + cfg.range_ema * r
            if self._range_ema < self._range_min - cfg.approach_eps:
                self._range_min = self._range_ema
                self._approach_t = sample.t
        self._ema_buf.append((sample.t, self._range_ema))
        cutoff = sample.t - cfg.window
        while len(self._ema_buf) > 2 and self._ema_buf[0][0] < cutoff:
            self._ema_buf.pop(0)

    # ------------------------------------------------------------------ 对外
    def update(self, sample: Sample) -> Verdict:
        cfg = self.cfg
        self._buf.append(sample)
        cutoff = sample.t - cfg.window
        while len(self._buf) > 2 and self._buf[0].t < cutoff:
            self._buf.pop(0)
        self._update_range(sample)

        if len(self._buf) < 3:
            return Verdict(span={"starved": 0.0, "imu_gap": 0.0, "range_stall": 0.0})

        cmd_dist = 0.0
        odom_dist = 0.0
        starved = 0.0
        imu_gap_span = 0.0
        spin_time = 0.0
        moving_time = 0.0

        for prev, cur in zip(self._buf, self._buf[1:]):
            dt = cur.t - prev.t
            if dt <= 0.0:
                continue
            v_cmd = 0.5 * (prev.v_cmd + cur.v_cmd)
            v_odom = 0.5 * (prev.v_odom + cur.v_odom)
            w_odom = 0.5 * (prev.w_odom + cur.w_odom)
            cmd_dist += abs(v_cmd) * dt
            odom_dist += abs(v_odom) * dt
            if abs(v_odom) >= cfg.v_min:
                moving_time += dt

            # 通道 A：要求走却几乎没走
            if abs(v_cmd) >= cfg.v_min and abs(v_odom) < cfg.ratio_thr * abs(v_cmd):
                starved += dt
                if abs(w_odom) >= cfg.spin_w:
                    spin_time += dt

            # 通道 B：里程计角速度与陀螺仪角速度对不上（单轮失抓地两侧都会差）
            if prev.w_imu is not None and cur.w_imu is not None:
                w_imu = 0.5 * (prev.w_imu + cur.w_imu)
                if max(abs(w_odom), abs(w_imu)) >= cfg.imu_w_min and abs(w_odom - w_imu) >= cfg.imu_gap:
                    imu_gap_span += dt

        # 通道 C：窗口级判断，避免逐帧差分被噪声主导
        covered = len(self._ema_buf) / max(len(self._buf), 1)
        if self._ema_buf:
            range_drop = max(self._ema_buf[0][1] - self._ema_buf[-1][1], 0.0)
            range_mean = sum(v for _, v in self._ema_buf) / len(self._ema_buf)
        else:
            range_drop = 0.0
            range_mean = float("inf")

        recent_approach = (sample.t - self._approach_t) <= cfg.approach_memory
        slip_now = (
            covered >= 0.8
            and range_mean <= cfg.range_near
            and odom_dist >= cfg.min_odom_dist_slip
            and moving_time >= 0.5 * cfg.window
            and range_drop < cfg.range_slip_ratio * odom_dist
            and recent_approach
        )
        dt_last = self._buf[-1].t - self._buf[-2].t
        if slip_now:
            self._slip_hold += dt_last
        else:
            self._slip_hold = max(0.0, self._slip_hold - 2.0 * dt_last)

        ratio = odom_dist / max(cmd_dist, 1e-6)

        cause = CAUSE_NONE
        confidence = 0.0
        # 优先级：里程计"完全不动"的判据最硬，优先于"编码器说走了但车没动"的打滑
        if imu_gap_span >= cfg.confirm:
            cause = CAUSE_ONE_WHEEL
            confidence = min(1.0, imu_gap_span / cfg.confirm)
        elif starved >= cfg.confirm and cmd_dist >= cfg.min_cmd_dist:
            cause = CAUSE_SPIN if spin_time >= 0.5 * cfg.confirm else CAUSE_BLOCKED
            confidence = min(1.0, starved / cfg.confirm)
        elif self._slip_hold >= cfg.confirm:
            cause = CAUSE_SLIP
            confidence = min(1.0, self._slip_hold / cfg.confirm)

        return Verdict(
            stuck=cause != CAUSE_NONE,
            cause=cause,
            slip_ratio=ratio,
            confidence=confidence,
            cmd_dist=cmd_dist,
            odom_dist=odom_dist,
            span={
                "starved": starved,
                "imu_gap": imu_gap_span,
                "range_stall": self._slip_hold,
                "range_drop": range_drop,
            },
        )
