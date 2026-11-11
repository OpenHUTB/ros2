"""参考轨迹：按弧长参数化的二维路径。

所有路径都提供 x(s)、y(s)、航向 psi(s)、曲率 kappa(s)，以及最近点投影。
投影采用“先找折线最近段，再用切向做 Gauss-Newton 修正”的方式，精度远高于
直接用采样点，因此 Stanley 后轴稳态误差可以稳定落在毫米量级。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


def wrap_angle(angle):
    """把角度归一化到 [-pi, pi)。标量或数组均可。"""
    return (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi


@dataclass(frozen=True)
class PathPose:
    """路径在某个弧长处的几何信息。"""

    s: float
    x: float
    y: float
    psi: float
    kappa: float

    @property
    def tangent(self) -> np.ndarray:
        return np.array([np.cos(self.psi), np.sin(self.psi)], dtype=float)

    @property
    def normal_left(self) -> np.ndarray:
        """左法向。车辆位于路径左侧时横向偏差 e > 0。"""
        return np.array([-np.sin(self.psi), np.cos(self.psi)], dtype=float)


@dataclass(frozen=True)
class Projection:
    """车辆参考点到路径的投影结果。"""

    s: float
    x: float
    y: float
    psi: float
    kappa: float
    e: float
    distance: float


class Path:
    """离散弧长参数化路径。

    Parameters
    ----------
    s, x, y, psi, kappa:
        等弧长间隔的路径采样，长度一致。
    name:
        用于绘图和报告的路径名。
    """

    def __init__(self, s, x, y, psi, kappa, name: str = "path"):
        self.s = np.asarray(s, dtype=float)
        self.x = np.asarray(x, dtype=float)
        self.y = np.asarray(y, dtype=float)
        self.psi = np.unwrap(np.asarray(psi, dtype=float))
        self.kappa = np.asarray(kappa, dtype=float)
        self.name = name

        if not (self.s.ndim == self.x.ndim == self.y.ndim == self.psi.ndim == self.kappa.ndim == 1):
            raise ValueError("path arrays must be one-dimensional")
        if len(self.s) < 3:
            raise ValueError("path needs at least three samples")
        if not np.all(np.isfinite(np.column_stack([self.s, self.x, self.y, self.psi, self.kappa]))):
            raise ValueError("path contains non-finite values")
        if np.any(np.diff(self.s) <= 0.0):
            raise ValueError("s must be strictly increasing")

        self.ds = float(np.mean(np.diff(self.s)))
        self.length = float(self.s[-1])

    # ------------------------------------------------------------------
    # 构造
    # ------------------------------------------------------------------
    @classmethod
    def from_xy(cls, x, y, ds: float = 0.02, name: str = "path", kappa=None):
        """把任意光滑折线重采样成等弧长路径。

        曲率既可以由重采样后的航向差分得到，也可以传入原始采样点上的解析曲率。
        """
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        if x.shape != y.shape or x.ndim != 1 or len(x) < 3:
            raise ValueError("x and y must be equal-length 1D arrays")

        chord = np.hypot(np.diff(x), np.diff(y))
        keep = np.concatenate(([True], chord > 1e-12))
        x, y = x[keep], y[keep]
        chord = np.hypot(np.diff(x), np.diff(y))
        cumulative = np.concatenate(([0.0], np.cumsum(chord)))
        total = float(cumulative[-1])
        if total < 2.0:
            raise ValueError("path is too short")

        s = np.arange(0.0, total, ds)
        if s[-1] < total:
            s = np.append(s, total)
        xr = np.interp(s, cumulative, x)
        yr = np.interp(s, cumulative, y)

        dx = np.gradient(xr, s, edge_order=2)
        dy = np.gradient(yr, s, edge_order=2)
        psi = np.unwrap(np.arctan2(dy, dx))

        if kappa is None:
            kappa_r = np.gradient(psi, s, edge_order=2)
        else:
            kappa = np.asarray(kappa, dtype=float)
            if kappa.shape != x.shape:
                raise ValueError("kappa must have the same shape as x and y")
            kappa_r = np.interp(s, cumulative, kappa[keep])
        return cls(s, xr, yr, psi, kappa_r, name=name)

    @classmethod
    def straight(cls, length: float = 130.0, ds: float = 0.02):
        s = np.arange(0.0, length + 0.5 * ds, ds)
        s[-1] = length
        return cls(s, s, np.zeros_like(s), np.zeros_like(s), np.zeros_like(s), name="straight")

    @classmethod
    def arc(cls, radius: float = 20.0, length: float = 130.0, ds: float = 0.02):
        if radius <= 0.0:
            raise ValueError("radius must be positive")
        s = np.arange(0.0, length + 0.5 * ds, ds)
        s[-1] = length
        theta = s / radius
        return cls(
            s,
            radius * np.sin(theta),
            radius * (1.0 - np.cos(theta)),
            theta,
            np.full_like(s, 1.0 / radius),
            name="arc",
        )

    @classmethod
    def sine(cls, amplitude: float = 3.0, wavelength: float = 40.0,
             x_end: float = 130.0, ds: float = 0.02):
        """正弦中心线。先按 x 采样，再重采样成真实弧长。"""
        k = 2.0 * np.pi / wavelength
        x_dense = np.linspace(0.0, x_end, int(x_end / 0.01) + 1)
        y_dense = amplitude * np.sin(k * x_dense)
        d1 = amplitude * k * np.cos(k * x_dense)
        d2 = -amplitude * k * k * np.sin(k * x_dense)
        kappa_dense = d2 / np.power(1.0 + d1 * d1, 1.5)
        return cls.from_xy(x_dense, y_dense, ds=ds, name="sine", kappa=kappa_dense)

    @classmethod
    def double_lane(cls, offset: float = 3.5, x_end: float = 130.0,
                    ds: float = 0.02, name: str = "double_lane"):
        """ISO 3888 风格的双移线：两个 C2 光滑的横向平移段。

        横向位移使用五次 smootherstep，起止点的横向位移、速度和加速度均为零，
        因此参考曲率连续且不会出现脉冲。
        """
        x_dense = np.linspace(0.0, x_end, int(x_end / 0.01) + 1)

        def smoothstep(t):
            t = np.clip(t, 0.0, 1.0)
            return 10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5

        y_dense = np.zeros_like(x_dense)
        first = (x_dense >= 25.0) & (x_dense < 55.0)
        hold = (x_dense >= 55.0) & (x_dense < 75.0)
        second = (x_dense >= 75.0) & (x_dense < 105.0)
        y_dense[first] = offset * smoothstep((x_dense[first] - 25.0) / 30.0)
        y_dense[hold] = offset
        y_dense[second] = offset * (1.0 - smoothstep((x_dense[second] - 75.0) / 30.0))

        # 解析一阶、二阶导数（smootherstep 的导数在区间外为 0）
        def smoothstep_d1(t):
            t = np.clip(t, 0.0, 1.0)
            return (30.0 * t**2 - 60.0 * t**3 + 30.0 * t**4) / 30.0

        def smoothstep_d2(t):
            t = np.clip(t, 0.0, 1.0)
            return (60.0 * t - 180.0 * t**2 + 120.0 * t**3) / (30.0 * 30.0)

        d1 = np.zeros_like(x_dense)
        d2 = np.zeros_like(x_dense)
        d1[first] = offset * smoothstep_d1((x_dense[first] - 25.0) / 30.0)
        d2[first] = offset * smoothstep_d2((x_dense[first] - 25.0) / 30.0)
        d1[second] = -offset * smoothstep_d1((x_dense[second] - 75.0) / 30.0)
        d2[second] = -offset * smoothstep_d2((x_dense[second] - 75.0) / 30.0)
        kappa_dense = d2 / np.power(1.0 + d1 * d1, 1.5)
        return cls.from_xy(x_dense, y_dense, ds=ds, name=name, kappa=kappa_dense)

    # ------------------------------------------------------------------
    # 查询
    # ------------------------------------------------------------------
    def pose_at(self, s: float) -> PathPose:
        """线性插值路径位姿；s 自动截断到 [0, length]。"""
        s = float(np.clip(s, 0.0, self.length))
        idx = int(np.clip(np.searchsorted(self.s, s, side="right") - 1, 0, len(self.s) - 2))
        s0 = self.s[idx]
        frac = (s - s0) / (self.s[idx + 1] - s0)
        frac = float(np.clip(frac, 0.0, 1.0))
        x = (1.0 - frac) * self.x[idx] + frac * self.x[idx + 1]
        y = (1.0 - frac) * self.y[idx] + frac * self.y[idx + 1]
        psi0 = self.psi[idx]
        psi = psi0 + frac * wrap_angle(self.psi[idx + 1] - psi0)
        kappa = (1.0 - frac) * self.kappa[idx] + frac * self.kappa[idx + 1]
        return PathPose(s=s, x=float(x), y=float(y), psi=float(psi), kappa=float(kappa))

    def sample_at_s(self, s_values):
        """批量查询，返回 (x, y, psi, kappa)。"""
        s_values = np.asarray(s_values, dtype=float)
        x = np.interp(s_values, self.s, self.x)
        y = np.interp(s_values, self.s, self.y)
        kappa = np.interp(s_values, self.s, self.kappa)
        idx = np.clip(np.searchsorted(self.s, s_values, side="right") - 1, 0, len(self.s) - 2)
        frac = (s_values - self.s[idx]) / (self.s[idx + 1] - self.s[idx])
        psi = self.psi[idx] + frac * wrap_angle(self.psi[idx + 1] - self.psi[idx])
        return x, y, psi, kappa

    def project(self, x: float, y: float, s_hint: float | None = None) -> Projection:
        """投影点到路径，返回弧长、参考位姿和带符号横向偏差。

        先在所有折线段（或 s_hint 附近的局部窗口）上找最近点，然后沿路径切向
        做 4 次 Gauss-Newton 修正。左偏 e > 0。
        """
        if s_hint is None:
            i0, i1 = 0, len(self.s) - 1
        else:
            center = int(np.clip(round(s_hint / self.ds), 0, len(self.s) - 1))
            half = max(50, int(round(6.0 / self.ds)))
            i0 = max(0, center - half)
            i1 = min(len(self.s) - 1, center + half)

        if i1 <= i0:
            i0, i1 = 0, len(self.s) - 1

        p0x = self.x[i0:i1]
        p0y = self.y[i0:i1]
        dx = self.x[i0 + 1:i1 + 1] - p0x
        dy = self.y[i0 + 1:i1 + 1] - p0y
        seg2 = dx * dx + dy * dy
        if np.any(seg2 <= 1e-16):
            seg2 = np.maximum(seg2, 1e-16)
        t = ((x - p0x) * dx + (y - p0y) * dy) / seg2
        t = np.clip(t, 0.0, 1.0)
        qx = p0x + t * dx
        qy = p0y + t * dy
        d2 = (x - qx) ** 2 + (y - qy) ** 2
        j = int(np.argmin(d2))
        idx = i0 + j
        s = self.s[idx] + float(t[j]) * self.ds

        # 若局部窗口的最优段贴边，可能投影落在窗口外，回退到全局搜索。
        if s_hint is not None and (idx == i0 or idx == i1 - 1):
            if float(d2[j]) > 1.0:
                return self.project(x, y, s_hint=None)

        s = float(np.clip(s, 0.0, self.length))
        for _ in range(4):
            pose = self.pose_at(s)
            tangent = pose.tangent
            delta = (x - pose.x) * tangent[0] + (y - pose.y) * tangent[1]
            s = float(np.clip(s + delta, 0.0, self.length))
            if abs(delta) < 1e-10:
                break

        pose = self.pose_at(s)
        normal = pose.normal_left
        e = (x - pose.x) * normal[0] + (y - pose.y) * normal[1]
        distance = float(np.hypot(x - pose.x, y - pose.y))
        return Projection(s=s, x=pose.x, y=pose.y, psi=pose.psi,
                          kappa=pose.kappa, e=float(e), distance=distance)


@dataclass
class Scenario:
    """场景：路径 + 初始条件。"""

    key: str
    title: str
    path: Path
    s0: float = 2.0
    e0: float = 1.2
    psi0_error: float = 0.10
    v0: float = 3.0
    duration: float = 10.0
    offroad_threshold: float = 0.8
    metadata: dict = field(default_factory=dict)

    def initial_state(self):
        pose = self.path.pose_at(self.s0)
        normal = pose.normal_left
        x = pose.x + self.e0 * normal[0]
        y = pose.y + self.e0 * normal[1]
        psi = pose.psi + self.psi0_error
        return np.array([x, y, psi, self.v0], dtype=float)


def make_scenarios() -> dict[str, Scenario]:
    """构建四个基准场景。路径长度均覆盖 10 s、8 m/s 的行程。"""
    return {
        "straight": Scenario(
            key="straight", title="Straight", path=Path.straight(130.0),
            duration=10.0, metadata={"curvature": "0"},
        ),
        "arc": Scenario(
            key="arc", title="Arc R=20 m", path=Path.arc(20.0, 130.0),
            duration=10.0, metadata={"radius_m": 20.0},
        ),
        "sine": Scenario(
            key="sine", title="Sine A=3 m", path=Path.sine(3.0, 35.0, 130.0),
            duration=10.0, metadata={"amplitude_m": 3.0, "wavelength_m": 35.0},
        ),
        "double_lane": Scenario(
            key="double_lane", title="ISO 3888 Double Lane", path=Path.double_lane(3.5, 130.0),
            duration=10.0, metadata={"offset_m": 3.5},
        ),
    }
