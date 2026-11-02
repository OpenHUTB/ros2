"""任务参数及显式坐标变换。"""

import math
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class CruiseConfig:
    backend: str = "mujoco"
    style: str = "brick"
    output: Path = Path("output/town10_cruise")
    duration: float = 100.0
    distance: float = 4.0
    speed: float = 0.8
    rate: float = 20.0
    capture_rate: float = 2.0
    host: str = "127.0.0.1"
    port: int = 41451
    vehicle: str = ""
    camera: str = "0"
    route: str = "relative"
    takeoff: bool = False
    gif: bool = False
    model: Path | None = None
    waypoint_timeout: float = 35.0
    route_offset: tuple = (0.0, 0.0, 0.0)

    def validate(self):
        for name in (
            "duration",
            "distance",
            "speed",
            "rate",
            "capture_rate",
            "waypoint_timeout",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} 必须为正的有限数")
        if self.backend not in {"mujoco", "airsim"} or self.route not in {
            "relative",
            "town10",
        }:
            raise ValueError("不支持的 backend 或 route")
        if (
            not 1 <= self.rate <= 100
            or self.capture_rate > self.rate
            or not 0.05 <= self.speed <= 2
        ):
            raise ValueError(
                "rate须在1..100Hz，capture_rate不得超过rate，speed须在0.05..2m/s"
            )
        if self.style not in {"brick", "gray", "glass"} or not 1 <= self.port <= 65535:
            raise ValueError("style或port无效")
        if len(self.route_offset) != 3 or not np.isfinite(self.route_offset).all():
            raise ValueError("route_offset 必须为三个有限NED偏移")
        if self.backend == "mujoco" and self.route != "relative":
            raise ValueError("Town10绝对路线仅用于AirSim后端")
        if self.backend == "mujoco" and not math.isclose(
            100 / self.rate, round(100 / self.rate), abs_tol=1e-8
        ):
            raise ValueError("MuJoCo控制周期必须为0.01秒积分周期的整数倍")


def ned_to_enu(vector):
    x, y, z = np.asarray(vector, dtype=float)
    return np.array([y, x, -z])


def town10_route(offset=(0.0, 0.0, 0.0)):
    path = Path(__file__).resolve().parent / "assets/route_town10.txt"
    points = []
    for line in path.read_text(encoding="utf-8").splitlines():
        match = re.match(r"\s*-\s*\[([^]]+)\]", line)
        if match:
            point = [float(value) for value in match.group(1).split(",")]
            points.append([*ned_to_enu(np.array(point[:3]) + offset), point[3]])
    values = np.asarray(points)
    if values.shape != (36, 4) or not np.isfinite(values).all():
        raise ValueError("Town10路线需包含36个有限四维航点")
    return values
