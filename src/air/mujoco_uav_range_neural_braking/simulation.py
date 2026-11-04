"""定高飞行与连续制动。 Altitude hold and continuous braking."""
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np


@dataclass(frozen=True)
class Scenario:
    """可复现场景参数。 Reproducible scenario parameters."""
    name: str = "corridor"
    start_x: float = -7.0
    speed: float = 1.5
    height: float = 1.2
    wall_x: float = 0.0
    duration: float = 10.0

    def __post_init__(self):
        values = (self.start_x, self.speed, self.height, self.wall_x, self.duration)
        if not np.isfinite(values).all():
            raise ValueError("参数须为有限值 / Parameters must be finite")
        if self.wall_x - self.start_x < 2 or not 0 <= self.speed <= 3:
            raise ValueError("初始间距至少2米，速度0至3米每秒 / Clearance >=2m, speed 0..3m/s")
        if not 0.5 <= self.height <= 2 or not 0 < self.duration <= 60:
            raise ValueError("高度0.5至2米，时长0至60秒 / Height 0.5..2m, duration 0..60s")


class Simulation:
    """简化纵向动力学；保留重力和质量。 Simplified longitudinal dynamics with gravity and mass."""
    def __init__(self, scenario=Scenario()):
        self.scenario = scenario
        self.model = mujoco.MjModel.from_xml_path(str(Path(__file__).with_name("scene.xml")))
        self.model.body_pos[self.model.body("drone").id] = [scenario.start_x, 0, scenario.height]
        self.model.geom_pos[self.model.geom("obstacle").id, 0] = scenario.wall_x
        self.data = mujoco.MjData(self.model)
        self.data.qvel[0] = scenario.speed
        self.drone_id = self.model.body("drone").id
        self.mass = float(self.model.body_mass[self.drone_id])
        mujoco.mj_forward(self.model, self.data)

    def observe(self):
        """几何基线观测，仅用于参照控制。 Geometric observation for reference control."""
        x, _, z = self.data.body("drone").xpos
        clearance = self.scenario.wall_x - 0.25 - (float(x) + 0.25)
        return float(clearance), float(self.data.qvel[0]), float(z)

    @staticmethod
    def control(clearance, speed):
        """按剩余制动距离给定目标速度。 Set target speed from remaining braking distance."""
        target_speed = min(1.5, np.sqrt(2 * 0.8 * max(clearance - 0.8, 0)))
        return float(np.clip(3 * (target_speed - speed), -2.5, 1.2))

    def step(self):
        clearance, speed, height = self.observe()
        acceleration = self.control(clearance, speed)
        # 升力抵消重力，PD反馈保持高度；不直接修改速度。
        # Lift balances gravity; PD feedback holds altitude without resetting velocity.
        vertical_acceleration = 16 * (self.scenario.height - height) - 8 * self.data.qvel[1]
        self.data.xfrc_applied[self.drone_id] = 0
        self.data.xfrc_applied[self.drone_id, :3] = [self.mass * acceleration, 0, self.mass * (9.81 + vertical_acceleration)]
        row = {"time_s": float(self.data.time), "range_m": clearance, "speed_m_s": speed,
               "height_m": height, "accel_m_s2": acceleration, "collision": int(self.data.ncon > 0)}
        mujoco.mj_step(self.model, self.data)
        return row

    def run(self, callback=None):
        """执行固定步长并返回指标。 Run fixed steps and return metrics."""
        rows = []
        for _ in range(round(self.scenario.duration / self.model.opt.timestep)):
            rows.append(self.step())
            if callback:
                callback(self)
        summary = {"scenario": self.scenario.name, "steps": len(rows),
                   "collision_steps": sum(r["collision"] for r in rows),
                   "min_clearance_m": min(r["range_m"] for r in rows),
                   "final_speed_m_s": rows[-1]["speed_m_s"],
                   "max_height_error_m": max(abs(r["height_m"] - self.scenario.height) for r in rows)}
        return rows, summary
