"""磁力计干扰判据与航向一致性状态机。

本文件只依赖 Python 标准库，不导入 rclpy 或任何 ROS 消息，
因此可以在没有 ROS 2 / 仿真器的机器上直接运行单元测试与离线回放。
ROS 2 节点只负责把这一层的结论转发成话题，判据本身集中在这里。

两部分能力：
1) 起飞前电机干扰自检：电机怠速与上油门各采一组整圈旋转样本，
   对水平磁场分量做圆拟合估计硬铁偏差，比较两次拟合的偏差偏移量，
   偏移过大说明动力系统磁场干扰超标，应在起飞前处理。
2) 飞行中航向一致性门控：用 GNSS 航迹向（地速足够大时才可信）
   与磁航向互检，配合水平磁场模值相对标定半径的偏离度，判断磁航向
   是否可用；不可用时建议改用 GNSS 航迹向作为航向来源。

关于拟合维度的说明：整机绕竖轴旋转一周时，机体系下的磁场样本只落在
一个水平圆上，z 分量没有展布，三维球面拟合在这个条件下是退化的
（bz 与半径不可分辨）。因此这里只对水平分量做圆拟合，垂直方向单独用
均值偏移衡量。这一取舍在 README 的「适用范围与局限」里写明了。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Sequence, Tuple

__all__ = [
    "PreflightState",
    "HeadingState",
    "MagSample",
    "FlightSample",
    "Config",
    "CircleFit",
    "fit_circle",
    "PreflightResult",
    "run_preflight",
    "HeadingOutput",
    "HeadingMonitor",
    "wrap_angle",
]


class PreflightState(str, Enum):
    """起飞前电机干扰自检的结论。"""

    PASS = "PASS"        # 电机干扰在允许范围内
    WARN = "WARN"        # 轻微超标，可飞行但建议复查
    FAIL = "FAIL"        # 干扰超标，应先处理再起飞


class HeadingState(str, Enum):
    """飞行中磁航向可信度状态。"""

    INIT = "INIT"              # 尚未建立可信航向
    TRUSTED = "TRUSTED"        # 磁航向可信
    DEGRADED = "DEGRADED"      # 出现异常，仍输出磁航向但标记降级
    UNRELIABLE = "UNRELIABLE"  # 磁航向不可信，改用 GNSS 航迹向


@dataclass(frozen=True)
class MagSample:
    """一个磁力计样本，单位 µT，时间为秒。"""

    t: float
    mx: float
    my: float
    mz: float

    def is_finite(self) -> bool:
        return all(math.isfinite(v) for v in (self.t, self.mx, self.my, self.mz))


@dataclass(frozen=True)
class FlightSample:
    """飞行中的一拍观测：磁力计原始值 + 可选的 GNSS 地速矢量。

    speed: 水平地速 (m/s)；course: GNSS 航迹向 (rad，北为 0 顺时针)，
    speed 低于门限时 course 不可信，应传 None。
    """

    t: float
    mx: float
    my: float
    mz: float
    speed: float = 0.0
    course: Optional[float] = None

    def is_finite(self) -> bool:
        vals = [self.t, self.mx, self.my, self.mz, self.speed]
        if self.course is not None:
            vals.append(self.course)
        return all(math.isfinite(v) for v in vals)


@dataclass(frozen=True)
class Config:
    """判据阈值。默认值对应小型多旋翼，可按机型与当地地磁环境调整。"""

    # --- 起飞前自检 ---
    min_preflight_samples: int = 48     # 每组自检最少样本数（约一圈 @ 20 Hz）
    max_bias_shift_pct: float = 5.0     # 偏差偏移占地磁水平分量的百分比上限
    warn_bias_shift_pct: float = 3.0    # 超过该值但未达上限时给出 WARN
    max_fit_residual_ut: float = 4.0    # 圆拟合残差 RMS 上限 (µT)

    # --- 飞行中门控 ---
    min_ground_speed: float = 2.0       # GNSS 航迹向可信所需的最小地速 (m/s)
    max_heading_error_deg: float = 15.0       # 磁航向与航迹向允许的偏差
    critical_heading_error_deg: float = 30.0  # 超过该偏差视为严重不一致
    norm_tolerance_pct: float = 15.0    # 水平磁场模值相对标定半径允许的偏离
    degraded_limit: int = 3             # 连续异常达到该值进入 DEGRADED
    unreliable_limit: int = 8           # 连续异常达到该值进入 UNRELIABLE
    recovery_streak: int = 5            # 退出 UNRELIABLE 所需的连续正常帧数
    stale_timeout: float = 1.0          # 两帧之间允许的最大间隔 (s)

    def __post_init__(self) -> None:
        positive = (
            self.min_ground_speed,
            self.max_heading_error_deg,
            self.critical_heading_error_deg,
            self.norm_tolerance_pct,
            self.stale_timeout,
        )
        if any(v <= 0 for v in positive):
            raise ValueError("速度、角度、模值与超时门限必须为正数")
        if not (0 < self.warn_bias_shift_pct < self.max_bias_shift_pct):
            raise ValueError("warn_bias_shift_pct 必须在 (0, max_bias_shift_pct) 内")
        if self.min_preflight_samples < 8:
            raise ValueError("min_preflight_samples 至少为 8")
        if self.max_fit_residual_ut <= 0:
            raise ValueError("max_fit_residual_ut 必须为正数")
        if self.degraded_limit < 1 or self.unreliable_limit < self.degraded_limit:
            raise ValueError("需要 1 <= degraded_limit <= unreliable_limit")
        if self.recovery_streak < 1:
            raise ValueError("recovery_streak 必须 >= 1")


def _solve_linear(a: Sequence[Sequence[float]], b: Sequence[float]) -> List[float]:
    """高斯消元解小型线性方程组（带部分主元），纯标准库。"""

    n = len(b)
    m = [list(row) + [rhs] for row, rhs in zip(a, b)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-9:
            raise ValueError("病态方程组：样本可能退化为直线或不足")
        m[col], m[pivot] = m[pivot], m[col]
        inv = 1.0 / m[col][col]
        for r in range(col + 1, n):
            factor = m[r][col] * inv
            if factor == 0.0:
                continue
            for c in range(col, n + 1):
                m[r][c] -= factor * m[col][c]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        acc = m[r][n] - sum(m[r][c] * x[c] for c in range(r + 1, n))
        x[r] = acc / m[r][r]
    return x


@dataclass(frozen=True)
class CircleFit:
    """水平磁场样本的圆拟合结果：硬铁偏差 (bx, by)、半径与残差。"""

    bias: Tuple[float, float]
    radius: float
    residual_rms: float
    z_mean: float
    n_samples: int


def fit_circle(samples: Sequence[MagSample]) -> CircleFit:
    """对整圈旋转采到的水平磁场分量做最小二乘圆拟合。

    圆方程 (mx-bx)² + (my-by)² = r² 展开后关于 (bx, by, r²-bx²-by²) 线性，
    用正规方程 + 高斯消元求解，不需要第三方数值库。
    """

    pts = [s for s in samples if s.is_finite()]
    if len(pts) < 8:
        raise ValueError("圆拟合至少需要 8 个有效样本")

    # 正规方程 (A^T A) u = A^T y，u = [2bx, 2by, c]，c = r² - bx² - by²
    ata = [[0.0] * 3 for _ in range(3)]
    aty = [0.0] * 3
    for s in pts:
        row = (s.mx, s.my, 1.0)
        y = s.mx * s.mx + s.my * s.my
        for i in range(3):
            for j in range(3):
                ata[i][j] += row[i] * row[j]
            aty[i] += row[i] * y
    u = _solve_linear(ata, aty)
    bx, by = u[0] / 2.0, u[1] / 2.0
    r2 = u[2] + bx * bx + by * by
    radius = math.sqrt(r2) if r2 > 0 else 0.0

    residuals = [
        math.hypot(s.mx - bx, s.my - by) - radius for s in pts
    ]
    rms = math.sqrt(sum(e * e for e in residuals) / len(residuals))
    z_mean = sum(s.mz for s in pts) / len(pts)
    return CircleFit(
        bias=(bx, by),
        radius=radius,
        residual_rms=rms,
        z_mean=z_mean,
        n_samples=len(pts),
    )


@dataclass(frozen=True)
class PreflightResult:
    """起飞前自检结论。"""

    state: PreflightState
    bias_shift_ut: float          # 两组拟合偏差的偏移量 (µT，含垂直均值)
    bias_shift_pct: float         # 偏移量占地磁水平分量的百分比
    idle: CircleFit
    spin: CircleFit
    reason: str


def run_preflight(
    idle_samples: Sequence[MagSample],
    spin_samples: Sequence[MagSample],
    cfg: Config = Config(),
) -> PreflightResult:
    """比较怠速与上油门两组整圈样本，评估动力系统的磁场干扰。

    典型用法：起飞前先怠速（或断电）旋转一周采样，再上油门旋转一周采样，
    电机电流产生的磁场会把拟合出的硬铁偏差"推"离怠速位置，偏移量与
    干扰强度成正比。垂直方向用两组样本的均值差单独计入偏移量。
    """

    if len(idle_samples) < cfg.min_preflight_samples:
        raise ValueError(f"怠速样本不足：{len(idle_samples)} < {cfg.min_preflight_samples}")
    if len(spin_samples) < cfg.min_preflight_samples:
        raise ValueError(f"上油门样本不足：{len(spin_samples)} < {cfg.min_preflight_samples}")

    idle = fit_circle(idle_samples)
    spin = fit_circle(spin_samples)
    shift_xy = math.dist(idle.bias, spin.bias)
    shift_z = spin.z_mean - idle.z_mean
    shift = math.hypot(shift_xy, shift_z)
    base = idle.radius if idle.radius > 1e-6 else max(spin.radius, 1e-6)
    shift_pct = shift / base * 100.0

    if shift_pct > cfg.max_bias_shift_pct:
        state, reason = PreflightState.FAIL, "bias_shift_exceeds_limit"
    elif spin.residual_rms > cfg.max_fit_residual_ut:
        state, reason = PreflightState.FAIL, "fit_residual_exceeds_limit"
    elif shift_pct > cfg.warn_bias_shift_pct or spin.residual_rms > idle.residual_rms * 3.0:
        state, reason = PreflightState.WARN, "mild_interference"
    else:
        state, reason = PreflightState.PASS, "ok"
    return PreflightResult(
        state=state,
        bias_shift_ut=shift,
        bias_shift_pct=shift_pct,
        idle=idle,
        spin=spin,
        reason=reason,
    )


@dataclass(frozen=True)
class HeadingOutput:
    """飞行中每帧输出。heading_source 告诉下游该用哪个航向来源。"""

    state: HeadingState
    heading: float            # 推荐航向 (rad，磁北基准)
    heading_source: str       # "mag" / "gnss_course" / "last_trusted"
    valid: bool               # 输出航向是否可直接用于控制
    reason: str
    heading_error_deg: float  # 磁航向与 GNSS 航迹向的偏差；不可比时为 nan
    field_norm_pct: float     # 当前水平磁场模值相对标定半径的百分比


def wrap_angle(a: float) -> float:
    """把角度归一化到 (-pi, pi]。"""

    return (a + math.pi) % (2.0 * math.pi) - math.pi


class HeadingMonitor:
    """飞行中航向一致性门控。

    逐帧调用 update()。需要先用起飞前自检得到的硬铁偏差构造本对象
    （传两维水平偏差即可，传三维时第三维被忽略，因为航向只由水平分量决定），
    并建议同时传入自检得到的水平半径 expected_radius 作为模值基准；
    判据包含三路信号：水平磁场模值偏离、磁航向与 GNSS 航迹向的偏差、
    数据流新鲜度，任意一路持续异常都会驱动状态机降级。
    """

    def __init__(self, bias: Sequence[float], cfg: Config = Config(),
                 expected_radius: Optional[float] = None) -> None:
        if len(bias) < 2:
            raise ValueError("至少需要 (bx, by) 两个水平偏差分量")
        self.cfg = cfg
        self.bias = (float(bias[0]), float(bias[1]))
        # 标定半径基准：优先用起飞前自检得到的水平半径。若调用方不给，
        # 则退化为「首帧模值」，此时必须保证首帧是在无干扰的地面采集的，
        # 否则正常数据会被反向判为异常（这一点在 README 里明确写出）。
        self.cal_radius: Optional[float] = (
            float(expected_radius)
            if expected_radius is not None and expected_radius > 1e-6
            else None
        )
        self.reset()

    def reset(self) -> None:
        self.state = HeadingState.INIT
        self._streak = 0
        self._good_streak = 0
        self.last_seen_t: Optional[float] = None
        self.last_trusted_heading: Optional[float] = None

    # --- 内部工具 -------------------------------------------------------

    def _horizontal(self, s: FlightSample) -> Tuple[float, float]:
        """去掉硬铁偏差后的水平磁场分量。"""

        return (s.mx - self.bias[0], s.my - self.bias[1])

    def _mag_heading(self, s: FlightSample) -> Optional[float]:
        """由校准后的水平磁场分量求磁航向。

        约定：机体 x 轴指向机头、y 轴向右（NED/FRD 体轴系，与 PX4、ArduPilot
        的磁力计发布约定一致），传感器水平放置（忽略倾斜补偿，见 README 的
        适用范围）。
        """

        cx, cy = self._horizontal(s)
        if math.hypot(cx, cy) < 1e-6:
            return None
        return math.atan2(cy, cx)

    def _emit(
        self,
        state: HeadingState,
        heading: Optional[float],
        source: str,
        valid: bool,
        reason: str,
        heading_error_deg: float,
        field_norm_pct: float,
    ) -> HeadingOutput:
        h = heading if heading is not None else (
            self.last_trusted_heading if self.last_trusted_heading is not None else 0.0
        )
        return HeadingOutput(
            state=state,
            heading=h,
            heading_source=source,
            valid=valid,
            reason=reason,
            heading_error_deg=heading_error_deg,
            field_norm_pct=field_norm_pct,
        )

    def _downgrade(self) -> HeadingState:
        """按连续异常计数推进状态机（INIT 阶段不会直接跳到 UNRELIABLE）。"""

        if self._streak >= self.cfg.unreliable_limit:
            self.state = HeadingState.UNRELIABLE
        elif self._streak >= self.cfg.degraded_limit:
            self.state = HeadingState.DEGRADED
        return self.state

    def _fallback(self, s: FlightSample) -> Tuple[Optional[float], str, bool]:
        """降级时选择兜底航向：GNSS 航迹向优先，其次冻结最后可信磁航向。"""

        if s.course is not None and s.speed >= self.cfg.min_ground_speed:
            return s.course, "gnss_course", True
        if self.last_trusted_heading is not None:
            return self.last_trusted_heading, "last_trusted", False
        return None, "last_trusted", False

    # --- 对外接口 -------------------------------------------------------

    def note_timeout(self, t: float,
                     course: Optional[float] = None) -> Optional[HeadingOutput]:
        """看门狗调用：磁场话题长时间没有新数据时主动降级。

        每调用一次视作一次"缺失观测"，连续调用会让状态机按 degraded_limit /
        unreliable_limit 推进，与逐帧异常的处理保持一致。
        course 只在调用方确认其可信（地速足够）时传入，此时直接用航迹向兜底。
        """

        if self.last_seen_t is None:
            return None
        self._streak += 1
        self._good_streak = 0
        if course is not None:
            heading, source, valid = course, "gnss_course", True
        elif self.last_trusted_heading is not None:
            heading, source, valid = self.last_trusted_heading, "last_trusted", False
        else:
            heading, source, valid = None, "last_trusted", False
        return self._emit(
            self._downgrade(), heading, source, valid, "stream_timeout",
            float("nan"), float("nan"),
        )

    def update(self, s: FlightSample) -> HeadingOutput:
        """处理一帧观测，输出推荐航向与状态。"""

        if not s.is_finite():
            self._streak += 1
            self._good_streak = 0
            heading, source, valid = self._fallback(s)
            return self._emit(
                self._downgrade(), heading, source, valid, "non_finite",
                float("nan"), float("nan"),
            )

        # 数据流新鲜度
        gap = (
            s.t - self.last_seen_t
            if self.last_seen_t is not None and s.t >= self.last_seen_t
            else 0.0
        )
        stale = gap > self.cfg.stale_timeout
        if self.last_seen_t is None or s.t > self.last_seen_t:
            self.last_seen_t = s.t

        cx, cy = self._horizontal(s)
        norm = math.hypot(cx, cy)
        if self.cal_radius is None and norm > 1e-6:
            # 第一帧建立标定基准，避免依赖具体的当地地磁参数
            self.cal_radius = norm
        field_norm_pct = (
            norm / self.cal_radius * 100.0 if self.cal_radius else float("nan")
        )

        mag_h = self._mag_heading(s)

        # 磁航向 vs GNSS 航迹向（地速足够大才可比）
        heading_error_deg = float("nan")
        if s.course is not None and mag_h is not None:
            heading_error_deg = math.degrees(abs(wrap_angle(mag_h - s.course)))

        reasons: List[str] = []
        if stale:
            reasons.append("stale")
        if math.isfinite(field_norm_pct) and abs(
            field_norm_pct - 100.0
        ) > self.cfg.norm_tolerance_pct:
            reasons.append("field_norm_out_of_range")
        if math.isfinite(heading_error_deg):
            if heading_error_deg > self.cfg.critical_heading_error_deg:
                reasons.append("critical_heading_error")
            elif heading_error_deg > self.cfg.max_heading_error_deg:
                reasons.append("heading_error")

        if reasons:
            self._streak += 1
            self._good_streak = 0
            heading, source, valid = self._fallback(s)
            return self._emit(
                self._downgrade(), heading, source, valid, ";".join(reasons),
                heading_error_deg, field_norm_pct,
            )

        # 一切正常
        self._good_streak += 1
        if mag_h is not None:
            self.last_trusted_heading = mag_h
        if self.state == HeadingState.UNRELIABLE:
            if self._good_streak >= self.cfg.recovery_streak:
                self.state = HeadingState.TRUSTED
                self._streak = 0
                reason = "recovered"
            else:
                reason = "awaiting_recovery"
        else:
            self.state = HeadingState.TRUSTED
            self._streak = 0
            reason = "ok"
        # 仅在可信且模值本身正常时缓慢跟随环境变化（温漂、载荷变化），
        # 避免把真实干扰"学"进基准里
        if (
            self.state == HeadingState.TRUSTED
            and self.cal_radius
            and math.isfinite(field_norm_pct)
            and abs(field_norm_pct - 100.0) <= self.cfg.norm_tolerance_pct * 0.5
        ):
            self.cal_radius = 0.99 * self.cal_radius + 0.01 * norm
        return self._emit(
            self.state, mag_h, "mag", True, reason,
            heading_error_deg, field_norm_pct,
        )
