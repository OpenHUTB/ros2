# -*- coding: utf-8 -*-
"""运行指标记录与导出（纯逻辑，仅绘图时用到可选的 matplotlib）。

指标定义（文档"性能评价"一节的表格与此一致）：
    d_触发减速   进入 SLOW_DOWN 时的射线距离 (m)
    d_触发刹停   进入 FULL_BRAKE 时的射线距离 (m)
    d_最小       全程射线距离最小值 (m)
    c_停车       停车时刻车头到障碍物前表面的真实间距 (m)
    c_最小       全程真实间距最小值 (m)，用于判断是否发生碰撞
    v_最大/平均  实际车速的峰值与均值 (m/s)
    a_最大制动   全程减速度峰值 (m/s^2)
    t_停车       从起步到停车的时间 (s)
    实时率      仿真推进时长 / 实际墙钟耗时（越大越实时）
"""

import csv
from dataclasses import dataclass, field
from typing import List, Optional

EPS = 1e-6

#: 判定"已真正起步"的速度阈值 (m/s)。低于它不算起步 ——
#: 否则起步静止段会被误判成"停稳"（实测 t=0.62 s 就把初始间距 7.48 m 记成了停车间距）。
START_SPEED_THRESHOLD_MPS = 0.3
#: 判定"已停稳"的速度阈值 (m/s)
STOP_SPEED_THRESHOLD_MPS = 0.02
#: 连续低速多久才算真正停稳 (s)。只判单帧会被接近段的接触抖动误触发 ——
#: 实测低速接近时 v 会瞬间掉到 0.01，把停车间距记成 2.86 m，而车实际停在 2.05 m。
STOP_CONFIRM_SECONDS = 0.3
#: 只在处于这些状态时统计制动减速度，过滤巡航阶段接触抖动造成的瞬时跳变
BRAKING_STATES = ("SLOW_DOWN", "FULL_BRAKE", "STALE")
#: 减速度估计的滑动窗口长度 (s)。
#: 用单帧差分会被接触抖动放大 —— 实测相邻两帧速度跳 0.3 m/s，
#: 差分得到 0.3/0.02 = 15 m/s²，而指令限幅只有 3.5 m/s²。
BRAKE_SMOOTH_SECONDS = 0.1


@dataclass
class Sample:
    """单帧采样。"""

    t: float
    state: str
    d_sensor: Optional[float]
    v_cmd: float
    v_actual: float
    x: float
    clearance: float


@dataclass
class MetricsRecorder:
    """逐帧累积指标，结束后汇总导出。"""

    samples: List[Sample] = field(default_factory=list)

    d_slowdown: Optional[float] = None
    d_fullbrake: Optional[float] = None
    d_min: Optional[float] = None
    clearance_min: Optional[float] = None
    clearance_stop: Optional[float] = None
    v_max: float = 0.0
    a_brake_max: float = 0.0
    t_stop: Optional[float] = None
    collision: bool = False

    _recent: List[tuple] = field(default_factory=list)   #: 最近 BRAKE_SMOOTH_SECONDS 内的 (t, v)
    _stopped: bool = False
    _started: bool = False       #: 是否已经真正起步（速度超过阈值）
    _low_speed_since: Optional[float] = None      #: 连续低速窗口的起始时刻
    _low_speed_clearance: Optional[float] = None  #: 该窗口起始时刻的间距

    # ------------------------------------------------------------------ 记录
    def record(self, t, state, d_sensor, v_cmd, v_actual, x, clearance) -> None:
        """写入一帧；同时更新所有极值。"""
        self.samples.append(
            Sample(float(t), str(state), d_sensor, float(v_cmd), float(v_actual),
                   float(x), float(clearance))
        )

        if d_sensor is not None:
            self.d_min = d_sensor if self.d_min is None else min(self.d_min, d_sensor)
            if self.d_slowdown is None and str(state) == "SLOW_DOWN":
                self.d_slowdown = float(d_sensor)
            if self.d_fullbrake is None and str(state) in ("FULL_BRAKE", "STALE"):
                self.d_fullbrake = float(d_sensor)

        self.clearance_min = (clearance if self.clearance_min is None
                              else min(self.clearance_min, clearance))
        if clearance <= 0.0:
            self.collision = True

        self.v_max = max(self.v_max, abs(v_actual))
        if abs(v_actual) > START_SPEED_THRESHOLD_MPS:
            self._started = True

        # 减速度用 BRAKE_SMOOTH_SECONDS 的滑动窗口估计。
        # 单帧差分会被接触抖动放大：实测相邻两帧速度跳 0.3 m/s → 差分得到 15 m/s²，
        # 而本模块的减速度指令限幅只有 3.5 m/s²，那个 15 是假值。
        self._recent.append((float(t), float(v_actual)))
        while len(self._recent) > 1 and (t - self._recent[0][0]) > BRAKE_SMOOTH_SECONDS:
            self._recent.pop(0)
        if str(state) in BRAKING_STATES and len(self._recent) >= 2:
            t_first, v_first = self._recent[0]
            span = t - t_first
            if span > EPS:
                accel = (v_actual - v_first) / span
                if accel < 0.0:
                    self.a_brake_max = max(self.a_brake_max, -accel)

        # 停车判定：必须"先真正起步过"，并且**连续低速 ≥ STOP_CONFIRM_SECONDS** 才算停稳。
        #   - 只判 |v|<0.02：起步瞬间（t=0.62 s、v=0.005 m/s）会被误判成已停车；
        #   - 只判单帧：低速接近段偶发的接触抖动（v 瞬间掉到 0.01）也会误判，
        #     实测把停车间距记成 2.86 m，而车实际停在 2.05 m。
        low_speed = self._started and abs(v_actual) < STOP_SPEED_THRESHOLD_MPS
        if low_speed:
            if self._low_speed_since is None:
                self._low_speed_since = t
                self._low_speed_clearance = clearance
        else:
            self._low_speed_since = None
            self._low_speed_clearance = None

        if (not self._stopped) and self._low_speed_since is not None:
            if (t - self._low_speed_since) >= STOP_CONFIRM_SECONDS:
                self._stopped = True
                self.t_stop = self._low_speed_since
                self.clearance_stop = self._low_speed_clearance

    # ------------------------------------------------------------------ 汇总
    def summary(self) -> dict:
        """返回可直接打印/写文档的指标字典。"""
        speeds = [abs(s.v_actual) for s in self.samples]
        return {
            "采样帧数": len(self.samples),
            "d_触发减速(m)": self.d_slowdown,
            "d_触发刹停(m)": self.d_fullbrake,
            "d_最小(m)": self.d_min,
            "c_停车(m)": self.clearance_stop,
            "c_最小(m)": self.clearance_min,
            "v_最大(m/s)": self.v_max,
            "v_平均(m/s)": (sum(speeds) / len(speeds)) if speeds else 0.0,
            "a_最大制动(m/s^2)": self.a_brake_max,
            "t_停车(s)": self.t_stop,
            "是否碰撞": self.collision,
        }

    def format_summary(self) -> str:
        """格式化为对齐的多行文本，供终端输出与文档引用。"""
        lines = []
        for key, value in self.summary().items():
            if isinstance(value, float):
                lines.append("  {:<22}{:.3f}".format(key, value))
            else:
                lines.append("  {:<22}{}".format(key, value))
        return "\n".join(lines)

    # ------------------------------------------------------------------ 导出
    def write_csv(self, path) -> None:
        """逐帧数据导出为 CSV（便于在文档中给出可复现的原始数据）。"""
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["t", "state", "d_sensor", "v_cmd", "v_actual", "x", "clearance"])
            for s in self.samples:
                writer.writerow([
                    "{:.4f}".format(s.t), s.state,
                    "" if s.d_sensor is None else "{:.4f}".format(s.d_sensor),
                    "{:.4f}".format(s.v_cmd), "{:.4f}".format(s.v_actual),
                    "{:.4f}".format(s.x), "{:.4f}".format(s.clearance),
                ])

    def plot(self, path) -> bool:
        """绘制速度-距离曲线；matplotlib 缺失时返回 False（不算失败）。"""
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception:
            return False

        times = [s.t for s in self.samples]
        dists = [s.d_sensor if s.d_sensor is not None else float("nan") for s in self.samples]
        speeds = [s.v_actual for s in self.samples]
        clears = [s.clearance for s in self.samples]

        fig, axes = plt.subplots(3, 1, figsize=(8, 9), sharex=True)
        axes[0].plot(times, dists, color="tab:red")
        axes[0].set_ylabel("d_sensor (m)")
        axes[0].set_title("ray distance / speed / true clearance")
        axes[0].grid(alpha=0.3)

        axes[1].plot(times, speeds, color="tab:blue")
        axes[1].set_ylabel("v (m/s)")
        axes[1].grid(alpha=0.3)

        axes[2].plot(times, clears, color="tab:green")
        axes[2].set_ylabel("clearance (m)")
        axes[2].set_xlabel("t (s)")
        axes[2].grid(alpha=0.3)

        path.parent.mkdir(parents=True, exist_ok=True)
        fig.tight_layout()
        fig.savefig(path, dpi=120)
        plt.close(fig)
        return True
