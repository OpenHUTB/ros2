"""把实测制动数据整理成数据集，并做饱和曲线拟合与瓶颈判定。"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .model import (
    GRAVITY,
    BrakingSpec,
    braking_distance_m,
    bottleneck,
    friction_bound_decel_mps2,
    saturated_decel_mps2,
)


@dataclass(frozen=True)
class BrakingPoint:
    """一次制动试验的汇总。"""

    tire_friction: float
    entry_speed_mps: float
    distance_m: float
    duration_s: float

    @property
    def decel_mps2(self) -> float:
        """按 ``a = v²/(2s)`` 反算的平均减速度。"""
        if self.distance_m <= 0.0:
            return float("nan")
        return float(self.entry_speed_mps**2 / (2.0 * self.distance_m))

    @property
    def ideal_distance_m(self) -> float:
        """纯附着限制下的理论最短距离 ``v²/(2μg)``。"""
        return float(braking_distance_m(self.entry_speed_mps, self.tire_friction * GRAVITY))

    @property
    def distance_ratio(self) -> float:
        """实测距离与纯附着理论上限之比（≥1 表示达不到附着极限）。"""
        return float(self.distance_m / self.ideal_distance_m)


@dataclass(frozen=True)
class SaturationFit:
    """``a = a_max μ / (μ + μ_half)`` 的拟合结果。"""

    a_max_mps2: float
    mu_half: float
    r2: float
    rmse_mps2: float

    def predict(self, tire_friction) -> np.ndarray:
        return saturated_decel_mps2(tire_friction, self.a_max_mps2, self.mu_half)

    def as_dict(self) -> dict:
        return {
            "a_max_mps2": round(self.a_max_mps2, 3),
            "mu_half": round(self.mu_half, 4),
            "r2": round(self.r2, 5),
            "rmse_mps2": round(self.rmse_mps2, 4),
        }


@dataclass
class SweepDataset:
    """一次扫掠的全部数据点，以及可选的模拟器运行信息。"""

    spec: BrakingSpec
    points: list[BrakingPoint] = field(default_factory=list)
    source: str = "offline"
    environment: dict = field(default_factory=dict)

    def sorted_points(self) -> list[BrakingPoint]:
        return sorted(self.points, key=lambda point: point.tire_friction)

    def frictions(self) -> np.ndarray:
        return np.array([point.tire_friction for point in self.sorted_points()])

    def distances(self) -> np.ndarray:
        return np.array([point.distance_m for point in self.sorted_points()])

    def decelerations(self) -> np.ndarray:
        return np.array([point.decel_mps2 for point in self.sorted_points()])

    def as_dict(self) -> dict:
        return {
            "source": self.source,
            "environment": self.environment,
            "spec": {
                "mass_kg": self.spec.mass_kg,
                "reference_speed_kmh": self.spec.reference_speed_kmh,
                "brake_ratio": self.spec.brake_ratio,
            },
            "points": [
                {
                    "tire_friction": round(point.tire_friction, 3),
                    "entry_speed_kmh": round(point.entry_speed_mps * 3.6, 2),
                    "distance_m": round(point.distance_m, 3),
                    "duration_s": round(point.duration_s, 3),
                    "decel_mps2": round(point.decel_mps2, 3),
                    "ideal_distance_m": round(point.ideal_distance_m, 3),
                    "distance_ratio": round(point.distance_ratio, 3),
                }
                for point in self.sorted_points()
            ],
        }


def fit_saturation(points: list[BrakingPoint]) -> SaturationFit:
    """用 ``1/a`` 对 ``1/μ`` 的线性回归拟合饱和曲线。

    ``a = a_max μ/(μ+μ_half)`` 取倒数得 ``1/a = 1/a_max + (μ_half/a_max)(1/μ)``，
    因此对 ``(1, 1/μ)`` 做最小二乘即可。
    """
    if len(points) < 3:
        raise ValueError("need at least 3 points to fit")
    mu = np.array([point.tire_friction for point in points], dtype=float)
    decel = np.array([point.decel_mps2 for point in points], dtype=float)
    mask = (mu > 0.0) & (decel > 0.0) & np.isfinite(decel)
    if int(mask.sum()) < 3:
        raise ValueError("not enough valid points to fit")
    design = np.column_stack([np.ones(int(mask.sum())), 1.0 / mu[mask]])
    solution, *_ = np.linalg.lstsq(design, 1.0 / decel[mask], rcond=None)
    intercept, slope = float(solution[0]), float(solution[1])
    if intercept <= 0.0:
        raise ValueError("fit failed: non-positive intercept")
    a_max = 1.0 / intercept
    mu_half = slope / intercept
    predicted = saturated_decel_mps2(mu[mask], a_max, mu_half)
    residual = decel[mask] - predicted
    ss_res = float(np.sum(residual**2))
    ss_tot = float(np.sum((decel[mask] - decel[mask].mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0.0 else 1.0
    return SaturationFit(
        a_max_mps2=float(a_max),
        mu_half=float(mu_half),
        r2=float(r2),
        rmse_mps2=float(np.sqrt(np.mean(residual**2))),
    )


def elasticity(points: list[BrakingPoint]) -> list[float]:
    """相邻两点的制动距离弹性：`(Δd/d) / (Δμ/μ)`。

    绝对值越小，说明继续提高附着系数对缩短距离越没用——即瓶颈已转移到制动器。
    第一个点没有前一点，弹性记为 `nan`。
    """
    ordered = sorted(points, key=lambda point: point.tire_friction)
    values: list[float] = [float("nan")]
    for previous, current in zip(ordered, ordered[1:]):
        dm = current.tire_friction - previous.tire_friction
        dd = current.distance_m - previous.distance_m
        if dm == 0.0 or previous.distance_m == 0.0:
            values.append(float("nan"))
            continue
        values.append((dd / previous.distance_m) / (dm / previous.tire_friction))
    return values


def classify_points(
    points: list[BrakingPoint],
    fit: SaturationFit | None = None,
    threshold: float = 0.20,
) -> list[str]:
    """判定每个点属于哪种限制。

    用弹性判据：与更低附着系数的那一点相比，距离弹性绝对值小于 `threshold`
    时判为“制动器受限”，否则“附着受限”。第一个点沿用第二个点的判定。
    """
    ordered = sorted(points, key=lambda point: point.tire_friction)
    slopes = elasticity(ordered)
    labels: list[str] = []
    for index in range(len(ordered)):
        slope = slopes[index] if index > 0 else slopes[1] if len(slopes) > 1 else float("nan")
        if slope != slope:  # nan
            labels.append("附着受限")
        elif abs(slope) < threshold:
            labels.append("制动器受限")
        else:
            labels.append("附着受限")
    return labels


def summary(dataset: SweepDataset) -> dict:
    """把数据集、拟合与环境信息整理成可直接写 JSON 的摘要。"""
    points = dataset.sorted_points()
    if len(points) < 3:
        raise ValueError("need at least 3 points")
    fit = fit_saturation(points)
    labels = classify_points(points, fit)
    elastic = elasticity(points)
    rows = []
    for index, (point, label) in enumerate(zip(points, labels)):
        slope = elastic[index]
        rows.append(
            {
                "elasticity": None if slope != slope else round(float(slope), 4),
                "tire_friction": round(point.tire_friction, 3),
                "entry_speed_kmh": round(point.entry_speed_mps * 3.6, 2),
                "distance_m": round(point.distance_m, 3),
                "decel_mps2": round(point.decel_mps2, 3),
                "friction_bound_mps2": round(float(friction_bound_decel_mps2(point.tire_friction)), 3),
                "ideal_distance_m": round(point.ideal_distance_m, 3),
                "distance_ratio": round(point.distance_ratio, 3),
                "bottleneck": label,
            }
        )
    low = points[0]
    high = points[-1]
    return {
        "source": dataset.source,
        "environment": dataset.environment,
        "spec": dataset.as_dict()["spec"],
        "fit": fit.as_dict(),
        "points": rows,
        "range": {
            "mu_min": round(low.tire_friction, 3),
            "mu_max": round(high.tire_friction, 3),
            "distance_at_mu_min_m": round(low.distance_m, 3),
            "distance_at_mu_max_m": round(high.distance_m, 3),
            "distance_reduction_percent": round(
                (low.distance_m - high.distance_m) / low.distance_m * 100.0, 2
            ),
        },
    }
