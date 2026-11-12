#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""闭环飞行：策略出高度/爬升率设定，底层几何控制器出旋翼推力。

两种被控对象可选：

* ``mujoco``：仓库 `uav_mujoco_sim` 的四旋翼模型 + 几何控制器，真推力、真姿态，
  高度通道的响应由物理决定（默认路径）；
* ``kinematic``：一阶滞后（τ = 0.15 s）的纯运动学替代，不装 mujoco 也能跑，
  用来做离线复算与 CI 上的快速回归。

地形不进入 MuJoCo 场景，只以解析高程场进入「参考指令」与「间隙评估」——
底座模型（``models/quadrotor.xml``）里机身 geom 的 ``contype/conaffinity`` 都是
0，本来就不参与碰撞，本模块不改共享模型，因此间隙为负的事件按「侵入地形」记录
并终止架次，而不是去等一个不会发生的碰撞。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from .altimeter import RadarAltimeter
from .cases import Case
from .control import TerrainFollowStrategy, make_strategy
from .dem import DigitalElevationModel
from .evaluate import summary

#: 控制周期（与底座 100 Hz 一致）
DT = 0.01
#: 每个控制周期内的物理子步数
SUBSTEPS = 4


@dataclass
class Sample:
    """一帧真值 + 当帧指令，事后算指标与画图都用它。"""

    t: float
    x: float
    z: float
    vx: float
    vz: float
    terrain_z: float
    agl_true: float
    agl_meas: Optional[float]
    clearance: float
    z_cmd: float
    vz_cmd: float


@dataclass
class Flight:
    """一条架次的结果。"""

    case: Case
    strategy: str
    samples: List[Sample] = field(default_factory=list)
    crashed: bool = False
    plant: str = "mujoco"

    def metrics(self) -> dict:
        return summary(self.samples, self.case.agl_set_m, self.case.safety_m,
                       self.case.warmup_s)

    @property
    def duration_s(self) -> float:
        return float(self.samples[-1].t) if self.samples else 0.0


def speed_profile(t: float, speed_mps: float, ramp_s: float):
    """梯形速度参考的解析积分：返回 (x, vx, ax)，起步匀加速后匀速。"""
    t = float(t)
    if t <= 0.0:
        return 0.0, 0.0, 0.0
    a = speed_mps / ramp_s
    if t < ramp_s:
        return 0.5 * a * t * t, a * t, a
    return 0.5 * speed_mps * ramp_s + speed_mps * (t - ramp_s), speed_mps, 0.0


class KinematicPlant:
    """一阶滞后的垂向运动学：vz ← vz + (vz_cmd − vz)·dt/τ，水平位置跟随参考。"""

    tau_s = 0.15

    def __init__(self, dt: float = DT):
        self.dt = float(dt)
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)

    def reset(self, pos) -> None:
        self.pos = np.asarray(pos, dtype=float).copy()
        self.vel = np.zeros(3)

    def step(self, p_d, v_d, a_d):
        alpha = min(1.0, self.dt / self.tau_s)
        self.vel[2] += (float(v_d[2]) - self.vel[2]) * alpha
        self.pos[2] += self.vel[2] * self.dt
        self.pos[0] = float(p_d[0])
        self.vel[0] = float(v_d[0])
        return self.pos.copy(), self.vel.copy()


class MujocoPlant:
    """仓库底座的四旋翼：几何控制器 + 真物理。"""

    def __init__(self, dt: float = DT, substeps: int = SUBSTEPS):
        try:
            from uav_mujoco_sim.core.quadrotor import GeometricController, Quadrotor
        except ImportError as exc:
            raise RuntimeError(
                "MuJoCo 后端需要同仓库的 src/air/uav_mujoco_sim；"
                "请在仓库源码目录运行，或改用 --plant kinematic（等价 --no-sim）。"
            ) from exc

        self.quad = Quadrotor(dt=dt, substeps=substeps)
        self.ctrl = GeometricController(self.quad.params)

    def reset(self, pos) -> None:
        self.quad.reset(tuple(float(v) for v in pos))

    def step(self, p_d, v_d, a_d):
        thrusts = self.ctrl.act(self.quad, np.asarray(p_d, float),
                                np.asarray(v_d, float), np.asarray(a_d, float), 0.0)
        self.quad.step_with_control(thrusts, 1)
        st = self.quad.state()
        return st.pos, st.vel


def fly(case: Case, strategy="dem_preview", plant: str = "mujoco",
        record: bool = True, stop_on_intrusion: bool = False) -> Flight:
    """跑一条架次：沿 +x 直线飞 ``case.length_m`` 米。

    ``strategy`` 既可以是名字，也可以直接传一个策略对象（做参数消融时用）。

    ``stop_on_intrusion`` 为真时，一出现负间隙就终止（那种情况下最小间隙只会
    报成 −0.0x m）；默认跑完整条航迹，好把「会深到多少」量出来。
    """
    terrain = case.terrain
    dem = DigitalElevationModel(terrain, case.dem)
    z0 = float(terrain.elevation(0.0, 0.0)) + float(case.agl_set_m)

    strat = strategy if isinstance(strategy, TerrainFollowStrategy) \
        else make_strategy(strategy, case.limits)
    strat.reset(terrain, dem, 0.0, z0, case.agl_set_m)
    alt = RadarAltimeter(case.altimeter, seed=case.seed)

    if plant == "mujoco":
        sim = MujocoPlant()
    elif plant == "kinematic":
        sim = KinematicPlant()
    else:
        raise ValueError(f"未知被控对象 {plant!r}，可选 mujoco / kinematic")
    sim.reset((0.0, 0.0, z0))

    flight = Flight(case=case, strategy=getattr(strat, "name", str(strategy)),
                    plant=plant)
    t_max = case.duration_s + 10.0
    n_max = int(math.ceil(t_max / DT))
    x_true, z_true, vx_true = 0.0, z0, 0.0

    for i in range(n_max):
        t = i * DT
        agl_true = z_true - float(terrain.elevation(x_true, 0.0))
        alt.update(t, agl_true)
        z_cmd, vz_cmd = strat.step(t, DT, x_true, z_true, vx_true, alt.reading)

        x_ref, vx_ref, ax_ref = speed_profile(t, case.speed_mps, case.ramp_s)
        pos, vel = sim.step((x_ref, 0.0, z_cmd), (vx_ref, 0.0, vz_cmd),
                            (ax_ref, 0.0, 0.0))
        x_true, z_true = float(pos[0]), float(pos[2])
        vx_true, vz_true = float(vel[0]), float(vel[2])

        terrain_z = float(terrain.elevation(x_true, 0.0))
        clearance = z_true - terrain_z
        if record:
            flight.samples.append(Sample(
                t=t, x=x_true, z=z_true, vx=vx_true, vz=vz_true,
                terrain_z=terrain_z, agl_true=z_true - terrain_z,
                agl_meas=alt.reading, clearance=clearance,
                z_cmd=float(z_cmd), vz_cmd=float(vz_cmd)))
        if clearance < 0.0:
            flight.crashed = True
            if stop_on_intrusion:
                break
        if x_true >= case.length_m:
            break

    return flight


def fly_all(case: Case, strategies=("constant_msl", "reactive_agl", "dem_preview"),
            plant: str = "mujoco") -> List[Flight]:
    """同一算例上依次跑三种策略（各自独立重置，互不影响）。"""
    return [fly(case, name, plant=plant) for name in strategies]
