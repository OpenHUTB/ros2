"""电机推力不对称检测的核心判据（纯标准库，不依赖 ROS）。

面向多旋翼「执行输出侧」的失效保护：飞控 mixer 输出各电机指令、ESC 遥测
回报各电机实际转速，两者结合可以区分两类典型动力故障：

  1) 电机/电调性能衰减（进沙、轴承磨损、电调过流降额）
     —— 同样指令下达不到应有转速，表现为 **RPM 跟踪误差**；
  2) 螺旋桨损伤/桨叶异物（转速正常但拉力下降）
     —— 转速跟踪正常，但姿态控制器必须持续抬高该电机的指令才能维持
        姿态，表现为 **指令相对健康基线的持续偏移**。

两类信号任意一路持续异常都会推动状态机降级：

    INIT ──基线就绪──▶ MONITORING ──warn 级连帧──▶ IMBALANCE
                          ▲                        │
                          └──连续正常帧── IMBALANCE  │ emg 级连帧
                                                   ▼
                                               EMERGENCY

  * IMBALANCE：推力不对称已确认但仍在可控范围。输出逐电机补偿指令
    （抬升衰减电机的指令、封顶防饱和），建议尽快降落；
  * EMERGENCY：不对称超出补偿能力，`emergency_land=True`，下游应
    触发紧急降落（补偿已无意义，置零）。

适用范围与简化假设（README 有完整讨论）：
  * 转速-指令映射取线性 rpm = rpm_max * cmd（静态动压曲线的保守简化，
    判据用相对误差，对映射斜率不敏感）；
  * 命令偏移判据只在悬停/小机动窗口内生效（远离悬停点的机动本身
    就需要不对称指令），需要先在健康段学习每电机的指令基线；
  * 不估计总拉力，不做故障电机隔离（reallocation），只做检测、
    补偿建议与紧急降落触发。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Sequence, Tuple


class MotorState(str, Enum):
    INIT = "INIT"
    MONITORING = "MONITORING"
    IMBALANCE = "IMBALANCE"
    EMERGENCY = "EMERGENCY"


@dataclass
class Config:
    """判据参数（默认值针对 4 电机、10 Hz 遥测的小型多旋翼）。"""

    n_motors: int = 4
    rpm_max: float = 8000.0        # 指令 1.0 对应的空载转速（线性映射）
    hover_cmd: float = 0.5         # 名义悬停指令（基线学习与近悬停判定用）
    track_err_warn_pct: float = 10.0   # RPM 跟踪误差告警门限（%）
    track_err_emg_pct: float = 25.0    # RPM 跟踪误差紧急门限（%）
    cmd_dev_warn_pct: float = 15.0     # 指令相对基线偏移告警门限（% 悬停指令）
    cmd_dev_emg_pct: float = 40.0      # 指令偏移紧急门限（% 悬停指令）
    near_hover_band: float = 0.15      # 平均指令在 hover_cmd±band 内才算近悬停
    imbalance_limit: int = 3           # warn 级连续帧数 → IMBALANCE
    emergency_limit: int = 5           # emg 级连续帧数 → EMERGENCY
    recovery_streak: int = 5           # 连续正常帧数 → 恢复 MONITORING
    baseline_frames: int = 25          # 健康帧数足够后冻结指令基线
    ema_alpha: float = 0.3             # 判据信号的平滑系数
    stale_timeout: float = 1.0         # 遥测超时（s），看门狗用
    comp_gain: float = 0.5             # 补偿系数：指令增量 = 增益×跟踪误差
    comp_limit: float = 0.15           # 单电机补偿上限（绝对指令量）
    cmd_max: float = 1.0               # 指令饱和上限（补偿不得越过）


@dataclass
class MotorSample:
    """一帧动力遥测：mixer 指令 + ESC 实测转速（转速缺失可传 None）。"""

    t: float
    commands: Sequence[float]
    rpm: Optional[Sequence[float]] = None


@dataclass
class MotorOutput:
    """一帧判据输出。"""

    state: MotorState
    reason: str
    healthy: bool                       # MONITORING/INIT 视为健康
    emergency_land: bool                # True → 建议立即紧急降落
    track_err_pct: List[float]          # 各电机 RPM 跟踪误差（无转速时为 NaN）
    cmd_dev_pct: List[float]            # 各电机指令相对基线偏移（基线未就绪为 NaN）
    worst_motor: Optional[int]          # 当前最差电机下标
    compensation: List[float]           # 建议叠加到各电机指令上的补偿量
    rpm_available: bool


def expected_rpm(cmd: float, cfg: Config) -> float:
    """由指令反解应有转速（线性映射）。"""

    return max(0.0, float(cmd)) * cfg.rpm_max


class MotorImbalanceMonitor:
    """逐帧喂 MotorSample，输出 MotorOutput。"""

    def __init__(self, cfg: Config = Config()) -> None:
        self.cfg = cfg
        n = cfg.n_motors
        self._ema_track: List[float] = [float("nan")] * n
        self._ema_dev: List[float] = [float("nan")] * n
        self._baseline: Optional[List[float]] = None
        self._baseline_acc: List[float] = [0.0] * n
        self._baseline_n: int = 0
        self.state = MotorState.INIT
        self.last_seen_t: Optional[float] = None
        self._bad_streak = 0
        self._emg_streak = 0
        self._good_streak = 0

    # --- 内部工具 -------------------------------------------------------

    def _isfinite_all(self, xs: Sequence[float]) -> bool:
        return all(math.isfinite(float(x)) for x in xs)

    def _downgrade(self, emg: bool) -> MotorState:
        self._good_streak = 0
        if emg:
            self._emg_streak += 1
        self._bad_streak += 1
        if self._emg_streak >= self.cfg.emergency_limit or (
            emg and self._bad_streak >= self.cfg.emergency_limit
        ):
            return MotorState.EMERGENCY
        if self._bad_streak >= self.cfg.imbalance_limit:
            return MotorState.IMBALANCE
        return self.state

    def _note_good(self) -> MotorState:
        self._bad_streak = 0
        self._emg_streak = 0
        self._good_streak += 1
        if self.state in (MotorState.IMBALANCE, MotorState.EMERGENCY):
            if self._good_streak >= self.cfg.recovery_streak:
                return MotorState.MONITORING
            return self.state
        if self.state == MotorState.INIT and self._baseline_n >= (
            self.cfg.baseline_frames
        ):
            return MotorState.MONITORING
        return self.state

    def _compensation(self, reason_worst: Optional[int]) -> List[float]:
        n = self.cfg.n_motors
        if self.state == MotorState.EMERGENCY or reason_worst is None:
            return [0.0] * n
        comp = [0.0] * n
        for i in range(n):
            err = self._ema_track[i]
            if math.isfinite(err) and err > 0.0:
                comp[i] = min(
                    self.cfg.comp_limit,
                    self.cfg.comp_gain * err / 100.0,
                )
        return comp

    # --- 对外接口 -------------------------------------------------------

    def update(self, s: MotorSample) -> MotorOutput:
        n = self.cfg.n_motors
        if (
            len(s.commands) != n
            or (s.rpm is not None and len(s.rpm) != n)
            or not self._isfinite_all(s.commands)
            or not math.isfinite(s.t)
            or (s.rpm is not None and not self._isfinite_all(s.rpm))
        ):
            state = self._downgrade(emg=False)
            return MotorOutput(
                state=state, reason="non_finite", healthy=False,
                emergency_land=state == MotorState.EMERGENCY,
                track_err_pct=[float("nan")] * n,
                cmd_dev_pct=[float("nan")] * n,
                worst_motor=None, compensation=[0.0] * n,
                rpm_available=False,
            )

        if self.last_seen_t is not None and s.t < self.last_seen_t:
            return MotorOutput(
                state=self.state, reason="out_of_order", healthy=False,
                emergency_land=self.state == MotorState.EMERGENCY,
                track_err_pct=[float("nan")] * n,
                cmd_dev_pct=[float("nan")] * n,
                worst_motor=None, compensation=[0.0] * n,
                rpm_available=s.rpm is not None,
            )
        # 长时间断流后的第一帧：视作一次缺失观测，连帧计数与看门狗一致
        gap = (
            self.last_seen_t is not None
            and s.t - self.last_seen_t > self.cfg.stale_timeout
        )
        self.last_seen_t = s.t
        if gap:
            state = self._downgrade(emg=False)
            return MotorOutput(
                state=state, reason="stream_gap", healthy=False,
                emergency_land=state == MotorState.EMERGENCY,
                track_err_pct=[float("nan")] * n,
                cmd_dev_pct=[float("nan")] * n,
                worst_motor=None, compensation=[0.0] * n,
                rpm_available=s.rpm is not None,
            )

        # --- 判据 1：RPM 跟踪误差（电机/电调衰减的直判据） -------------
        track = [float("nan")] * n
        if s.rpm is not None:
            for i in range(n):
                exp = expected_rpm(s.commands[i], self.cfg)
                if exp > 1e-3:
                    track[i] = (exp - float(s.rpm[i])) / exp * 100.0
                else:
                    track[i] = 0.0 if float(s.rpm[i]) < 1.0 else 100.0
            for i in range(n):
                self._ema_track[i] = (
                    track[i]
                    if not math.isfinite(self._ema_track[i])
                    else (1.0 - self.cfg.ema_alpha) * self._ema_track[i]
                    + self.cfg.ema_alpha * track[i]
                )

        # --- 判据 2：指令相对健康基线的偏移（桨损的间接判据） -----------
        mean_cmd = sum(s.commands) / n
        near_hover = abs(mean_cmd - self.cfg.hover_cmd) <= self.cfg.near_hover_band
        dev = [float("nan")] * n
        if self._baseline is not None and near_hover:
            for i in range(n):
                dev[i] = (
                    (float(s.commands[i]) - self._baseline[i])
                    / max(self.cfg.hover_cmd, 1e-6)
                    * 100.0
                )
                self._ema_dev[i] = (
                    dev[i]
                    if not math.isfinite(self._ema_dev[i])
                    else (1.0 - self.cfg.ema_alpha) * self._ema_dev[i]
                    + self.cfg.ema_alpha * dev[i]
                )
        elif self._baseline is None and self.state == MotorState.INIT:
            # 基线学习：只在健康初始化窗口内累积
            for i in range(n):
                self._baseline_acc[i] += float(s.commands[i])
            self._baseline_n += 1
            if self._baseline_n >= self.cfg.baseline_frames:
                self._baseline = [
                    a / self._baseline_n for a in self._baseline_acc
                ]

        # --- 汇总：单帧最差证据 ----------------------------------------
        worst_i: Optional[int] = None
        worst_level = 0            # 0 正常 / 1 warn / 2 emg
        reason = "ok"
        for i in range(n):
            level, why = 0, "ok"
            te = self._ema_track[i]
            if math.isfinite(te):
                if te >= self.cfg.track_err_emg_pct:
                    level, why = 2, "rpm_tracking_emergency"
                elif te >= self.cfg.track_err_warn_pct:
                    level, why = 1, "rpm_tracking_warn"
            de = self._ema_dev[i]
            if math.isfinite(de):
                if de >= self.cfg.cmd_dev_emg_pct:
                    if level < 2:
                        level, why = 2, "cmd_deviation_emergency"
                elif de >= self.cfg.cmd_dev_warn_pct and level < 1:
                    level, why = 1, "cmd_deviation_warn"
            if level > worst_level:
                worst_level, worst_i, reason = level, i, why

        if worst_level >= 2:
            self.state = self._downgrade(emg=True)
        elif worst_level == 1:
            self.state = self._downgrade(emg=False)
        else:
            prev = self.state
            self.state = self._note_good()
            if prev in (MotorState.IMBALANCE, MotorState.EMERGENCY) and (
                self.state == MotorState.MONITORING
            ):
                reason = "recovered"

        emg = self.state == MotorState.EMERGENCY
        return MotorOutput(
            state=self.state,
            reason=reason if worst_level > 0 or reason == "recovered" else "ok",
            healthy=self.state in (MotorState.INIT, MotorState.MONITORING),
            emergency_land=emg,
            track_err_pct=list(self._ema_track),
            cmd_dev_pct=list(self._ema_dev),
            worst_motor=worst_i,
            compensation=self._compensation(worst_i),
            rpm_available=s.rpm is not None,
        )

    def note_timeout(self, t: float) -> Optional[MotorOutput]:
        """看门狗：遥测超时逐拍降级（与逐帧异常同一套连帧计数）。

        注意：纯数据流超时**封顶在 IMBALANCE**，不会升级 EMERGENCY——
        遥测丢失无法区分「动力失效」和「链路/监视器自身掉线」，此时
        触发紧急降落比保持降级监视更危险。EMERGENCY 只由实测判据
        （跟踪误差/指令偏移达到紧急门限）驱动。
        """

        if self.last_seen_t is None:
            return None
        if t - self.last_seen_t <= self.cfg.stale_timeout:
            return None
        n = self.cfg.n_motors
        self.state = self._downgrade(emg=False)
        return MotorOutput(
            state=self.state, reason="stream_timeout", healthy=False,
            emergency_land=self.state == MotorState.EMERGENCY,
            track_err_pct=[float("nan")] * n,
            cmd_dev_pct=[float("nan")] * n,
            worst_motor=None, compensation=[0.0] * n,
            rpm_available=False,
        )


def clamp_commands(
    commands: Sequence[float], comp: Sequence[float], cfg: Config
) -> List[float]:
    """把补偿量叠加到指令上并封顶（供节点与回放复用）。"""

    out = []
    for c, k in zip(commands, comp):
        out.append(min(cfg.cmd_max, max(0.0, float(c) + float(k))))
    return out
