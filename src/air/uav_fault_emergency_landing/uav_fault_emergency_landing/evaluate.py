"""闭环迫降评测：三种模式 × 若干故障工况，比较安全迫降率与触地质量。

一次仿真分两段：

1. **任务段**（``0 ≤ t < onset``）：原地悬停，但激光一直在扫、地图一直在建；
2. **迫降段**（``t ≥ onset``）：注入故障，按所选模式决定去哪落、怎么降。

三种模式共用同一套底层（几何控制器 + 加权分配），差别只在"选不选落点、用什么给速度指令"。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np

from . import allocation as A
from . import controller as C
from . import landing as L
from . import mapping as M

#: 默认评测的故障工况：健康基线 + 两组不同严重度的单桨衰减。
#:
#: 悬停时单桨要出 2.33 N，桨的指令上限是 6.0 N，所以坏桨增益低于
#: ``2.33 / 6.0 ≈ 0.39``（即 severity > 0.61）时，分配层再怎么加大坏桨指令
#: 也补不回来——这时才会出现倾斜与饱和。
#:
#: * ``healthy``：没有故障，用来证明"选点"本身带来的差别，而不是故障带来的；
#: * ``degradation@2`` severity 0.5（剩余 50%）：仍可被分配层完全补偿；
#: * ``degradation@0`` severity 0.78（剩余 22%）：越过补偿边界，指令饱和。
FLIGHT_PLANS = (
    ("healthy", dict(kind="healthy", onset=2.0)),
    ("degradation@2", dict(kind="degradation", rotor=2, severity=0.50)),
    ("degradation@0", dict(kind="degradation", rotor=0, severity=0.78)),
)

#: 姿态超过这个角度就算失控
HARD_TILT_DEG = 75.0
#: 低于这个高度当作已经落地（地面 z=0，机体中心 0.16 m 时螺旋桨基本贴地）
LAND_Z = 0.16


def quat_to_rot(quat) -> np.ndarray:
    """四元数 (w, x, y, z) → 旋转矩阵。"""
    w, x, y, z = np.asarray(quat, dtype=float).reshape(4)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def tilt_deg(quat) -> float:
    """机体 z 轴与竖直方向的夹角（度）。"""
    rot = quat_to_rot(quat)
    cos_tilt = float(np.clip(rot[2, 2], -1.0, 1.0))
    return float(np.degrees(np.arccos(cos_tilt)))


@dataclass
class LandingResult:
    mode: str
    plan: str
    scene: str
    seed: int
    landed: bool = False
    crashed: bool = False
    landed_on_obstacle: bool = False
    touchdown_time: Optional[float] = None
    touchdown_vz: float = 0.0
    touchdown_vxy: float = 0.0
    touchdown_tilt: float = 0.0
    max_tilt: float = 0.0
    min_altitude: float = 0.0
    target_xy: Optional[np.ndarray] = None
    final_xy: Optional[np.ndarray] = None
    target_error: float = float("nan")
    steps: int = 0
    min_clearance: float = float("inf")
    #: 全程最大的单桨指令占满量程的比例。> 1.0 说明分配层已经在硬顶，
    #: 这一列是用来解释"为什么有的工况轨迹一样、有的不一样"的。
    max_command_ratio: float = 0.0
    times: List[float] = field(default_factory=list)
    altitudes: List[float] = field(default_factory=list)
    targets: List[Optional[np.ndarray]] = field(default_factory=list)

    @property
    def safe(self) -> bool:
        return bool(self.landed and not self.crashed and not self.landed_on_obstacle
                    and self.touchdown_vz <= 1.2 and self.touchdown_vxy <= 1.0
                    and self.touchdown_tilt <= 35.0)

    @property
    def status(self) -> str:
        if self.crashed:
            return "失控"
        if not self.landed:
            return "未落地"
        if self.landed_on_obstacle:
            return "撞到障碍物"
        return "安全迫降" if self.safe else "硬着陆"

    def summary(self) -> Dict[str, Any]:
        return {
            "模式": self.mode,
            "工况": self.plan,
            "结果": self.status,
            "触地时刻s": None if self.touchdown_time is None else round(self.touchdown_time, 2),
            "触地垂直速度m/s": round(self.touchdown_vz, 2),
            "触地水平速度m/s": round(self.touchdown_vxy, 2),
            "触地倾角deg": round(self.touchdown_tilt, 1),
            "落点误差m": None if np.isnan(self.target_error) else round(self.target_error, 2),
            "最大倾角deg": round(self.max_tilt, 1),
            "最大指令比例": round(self.max_command_ratio, 2),
        }


def build_context(api):
    """返回 (params, effectiveness)，多个用例复用同一份。"""
    quad = api.Quadrotor(scene=api.make_scene("open_ground", seed=0), dt=0.01, substeps=4)
    params = quad.params
    return params, A.build_effectiveness(api.mixer, params)


def _drone_body_id(quad) -> int:
    """四旋翼自身机体在模型里的 body id；拿不到时返回 0（= 不排除任何 body）。"""
    import mujoco

    try:
        bid = mujoco.mj_name2id(quad.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    except Exception:
        return 0
    return int(bid) if bid >= 0 else 0


def surface_height(api, quad, xy, start_z: float = 16.0) -> float:
    """从高处垂直向下打一条射线，返回该 (x, y) 正下方表面的高度。

    评测时用它判断"最后停在障碍物上还是平地上"——地图是估计值，判断结果要用真值，
    否则会出现"迫降落在楼顶、却报告成安全迫降"这种自欺欺人的结果。

    注意必须用 ``bodyexclude`` 把飞行器自己的机体排除掉：迫降结束时它就贴在地面上，
    不排除的话射线先打到机身（约 0.3~0.4 m），会把平地误判成障碍物。
    """
    import mujoco

    mujoco.mj_forward(quad.model, quad.data)
    pnt = np.array([float(xy[0]), float(xy[1]), float(start_z)])
    geomid = np.zeros(1, dtype=np.int32)
    dist = mujoco.mj_ray(quad.model, quad.data, pnt, np.array([0.0, 0.0, -1.0]),
                         None, True, _drone_body_id(quad), geomid)
    return float("nan") if dist < 0 else float(start_z - dist)


def _plan_of(spec: Dict[str, Any]):
    from . import faults as F

    return F.FaultPlan(**spec)


def pick_obstacle_spot(api, scene: str = "urban_block", seed: int = 0,
                       scan_altitude: float = 15.0, span: float = 12.0,
                       step: float = 6.0) -> tuple:
    """找一个"正下方就是障碍物"的悬停点，返回 ``(xy, hover_z)``。

    这个模块要回答的问题是"故障后往哪落"，所以起飞点故意选在楼顶上方：
    垂直迫降会直接砸在楼顶上，而选点迫降应该飞到旁边的空地。
    先在几个位置各扫一遍摸清地形，再挑最高的那一格，并悬停在其上方 2 m。
    """
    import mujoco

    quad = api.Quadrotor(scene=api.make_scene(scene, seed=seed), dt=0.01, substeps=4)
    mapper = M.DepthMapper(api).bind(quad)
    world = M.HeightMap(M.GridConfig())
    n = int(round(span / step))
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            quad.reset(pos=(i * step, j * step, scan_altitude))
            mujoco.mj_forward(quad.model, quad.data)
            world.update(mapper.scan(quad.data))

    if not np.any(world.observed):
        return np.array([0.0, 0.0]), 3.0
    obstacles = np.where(world.obstacle_mask(), world.height, 0.0)
    row, col = np.unravel_index(int(np.argmax(obstacles)), obstacles.shape)
    height = float(obstacles[row, col])
    if height < 0.4:
        return np.array([0.0, 0.0]), 3.0
    center = world.cell_center(row, col)
    return np.asarray(center, dtype=float), float(height + 3.0)


def run_landing(api, mode: str = "expert", plan_spec: Optional[Dict[str, Any]] = None,
                scene: str = "urban_block", seed: int = 0, seconds: float = 40.0,
                dt: float = 0.01, substeps: int = 4, start_xy: Optional[np.ndarray] = None,
                hover_z: Optional[float] = None,
                score_net=None, guidance_net=None, cfg: Optional[L.LandingConfig] = None,
                record: bool = False, params=None, effectiveness=None) -> LandingResult:
    """跑一次"悬停 → 注入故障 → 迫降"的完整过程。"""
    cfg = cfg or L.LandingConfig()
    plan = _plan_of(plan_spec or {"kind": "degradation", "rotor": 2, "severity": 0.5})
    if params is None or effectiveness is None:
        params, effectiveness = build_context(api)

    quad = api.Quadrotor(scene=api.make_scene(scene, seed=seed), dt=dt, substeps=substeps)
    if start_xy is None:
        hover_xy, auto_z = pick_obstacle_spot(api, scene, seed)
        hover_z = auto_z if hover_z is None else float(hover_z)
    else:
        hover_xy = np.asarray(start_xy, dtype=float).reshape(2)
        hover_z = 3.0 if hover_z is None else float(hover_z)
    quad.reset(pos=(float(hover_xy[0]), float(hover_xy[1]), float(hover_z)))

    base = api.GeometricController(quad.params)
    mapper = M.DepthMapper(api).bind(quad)
    ctrl = C.ForcedLandingController(quad.params, api.mixer, base, effectiveness, cfg=cfg,
                                     mode=mode, score_net=score_net, guidance_net=guidance_net)

    result = LandingResult(mode=mode,
                           plan=plan.label() if hasattr(plan, "label") else "healthy",
                           scene=scene, seed=seed)
    n_steps = int(round(seconds / dt))
    t_fault = float(plan.onset_time()) if hasattr(plan, "onset_time") else 0.0
    rest_count = 0
    import mujoco

    for k in range(n_steps):
        t = k * dt
        gains = plan.gains(t) if hasattr(plan, "gains") else np.ones(4)
        bias = plan.bias(t, params.max_thrust) if hasattr(plan, "bias") else None
        ctrl.set_health(gains, bias)
        state = quad.state()

        points = None
        if k % C.MAP_EVERY == 0:
            mujoco.mj_forward(quad.model, quad.data)
            points = mapper.scan(quad.data)

        if t < t_fault:
            # 任务段：原地悬停，地图照建
            if points is not None:
                ctrl.update_map(points)
            thrust, tau = base.compute(state, np.array([hover_xy[0], hover_xy[1], hover_z]),
                                       np.zeros(3), np.zeros(3), 0.0)
            commands = api.mixer(thrust, tau, params)
        else:
            commands = ctrl.act(state, points)

        # 指令 → 真实推力（故障后的桨只能给出 g·u + b），再交给 MuJoCo
        actual = np.clip(commands * gains + (0.0 if bias is None else bias),
                         0.0, params.max_thrust)
        if float(np.max(commands)) > 0.0:
            result.max_command_ratio = max(
                result.max_command_ratio, float(np.max(commands)) / float(params.max_thrust))
        quad.apply_rotor_thrust(actual)
        quad.step(1)
        state = quad.state()

        altitude = float(state.pos[2])
        tilt = tilt_deg(state.quat)
        result.steps = k + 1
        # 与地面/障碍物的最小几何间隙：本底座的四旋翼没有碰撞体，光看高度看不出来
        # "是不是从楼里穿过去了"，所以每 10 步沿当前 (x,y) 打一条竖直射线量一次。
        # 只看离地 0.6 m 以上的航段，否则正常触地那一刻也会被算成"间隙为负"。
        if k % 10 == 0 and altitude > 0.6:
            surf = surface_height(api, quad, state.pos[:2])
            if np.isfinite(surf):
                result.min_clearance = min(result.min_clearance, altitude - surf)
        if k == 0:
            result.min_altitude = altitude
        else:
            result.min_altitude = min(result.min_altitude, altitude)
        result.max_tilt = max(result.max_tilt, tilt)
        if record and t >= t_fault:
            result.times.append(t)
            result.altitudes.append(altitude)
            result.targets.append(None if ctrl.plan.target_xy is None
                                  else np.array(ctrl.plan.target_xy, copy=True))

        if tilt > HARD_TILT_DEG:
            result.crashed = True
            break
        # 停在障碍物顶上（机体被挡住、速度归零）也算一次"落地"，否则会一直判"未落地"。
        # 只在故障之后、且确实已经下降过一段距离时才认，否则正常悬停会被当成落地。
        if (t >= t_fault and altitude < float(hover_z) - 0.5
                and abs(state.vel[2]) < 0.15
                and np.hypot(state.vel[0], state.vel[1]) < 0.15):
            rest_count += 1
        else:
            rest_count = 0
        if altitude <= LAND_Z or rest_count >= int(round(0.5 / dt)):
            result.landed = True
            result.touchdown_time = t
            result.touchdown_vz = float(-state.vel[2])
            result.touchdown_vxy = float(np.hypot(state.vel[0], state.vel[1]))
            result.touchdown_tilt = tilt
            result.final_xy = np.asarray(state.pos[:2], dtype=float)
            break

    final_state = quad.state()
    if result.final_xy is None:
        result.final_xy = np.asarray(final_state.pos[:2], dtype=float)
    if not result.landed:
        result.touchdown_vz = float(-final_state.vel[2])
        result.touchdown_vxy = float(np.hypot(final_state.vel[0], final_state.vel[1]))
        result.touchdown_tilt = tilt_deg(final_state.quat)

    result.target_xy = None if ctrl.plan.target_xy is None else np.asarray(ctrl.plan.target_xy)
    if result.final_xy is not None:
        # 用真值判断这次迫降有没有撞到障碍物：地图是估计值，不能拿它给自己打分
        surface = surface_height(api, quad, result.final_xy)
        result.landed_on_obstacle = bool(result.min_clearance < -0.15
                                         or (np.isfinite(surface) and surface > M.OBSTACLE_HEIGHT))
    if result.target_xy is not None and result.final_xy is not None:
        result.target_error = float(np.hypot(result.final_xy[0] - result.target_xy[0],
                                             result.final_xy[1] - result.target_xy[1]))
    return result


def flight_table(api, modes=C.MODES, plans=FLIGHT_PLANS, scene: str = "obstacle_field",
                 seeds=(0,), seconds: float = 40.0,
                 score_net=None, guidance_net=None, params=None,
                 effectiveness=None) -> List[LandingResult]:
    """跑完整的模式 × 工况 × seed 组合。"""
    if params is None or effectiveness is None:
        params, effectiveness = build_context(api)
    rows: List[LandingResult] = []
    for mode in modes:
        for _label, spec in plans:
            for seed in seeds:
                rows.append(run_landing(api, mode=mode, plan_spec=spec, scene=scene,
                                        seed=int(seed), seconds=seconds,
                                        score_net=score_net, guidance_net=guidance_net,
                                        params=params, effectiveness=effectiveness))
    return rows


def aggregate(results: List[LandingResult]) -> List[Dict[str, Any]]:
    """按模式汇总。"""
    out = []
    for mode in dict.fromkeys(r.mode for r in results):
        group = [r for r in results if r.mode == mode]
        safe = [r for r in group if r.safe]
        errors = [r.target_error for r in group if not np.isnan(r.target_error)]
        out.append({
            "模式": mode,
            "安全迫降率": round(len(safe) / len(group), 3),
            "平均触地垂直速度m/s": round(float(np.mean([r.touchdown_vz for r in group])), 2),
            "平均触地水平速度m/s": round(float(np.mean([r.touchdown_vxy for r in group])), 2),
            "平均落点误差m": round(float(np.mean(errors)), 2) if errors else None,
            "平均迫降耗时s": round(float(np.mean([r.touchdown_time or 0.0 for r in group])), 1),
            "最大倾角deg": round(float(np.max([r.max_tilt for r in group])), 1),
            "平均最大指令比例": round(float(np.mean([r.max_command_ratio for r in group])), 2),
        })
    return out


def format_dicts(rows: List[Dict[str, Any]], columns: Optional[List[str]] = None) -> str:
    """中文按 2 字宽对齐的文本表。"""
    if not rows:
        return "（无数据）"
    columns = columns or list(rows[0].keys())
    widths = {c: max(_width(c), max(_width(r.get(c)) for r in rows)) for c in columns}
    head = "  ".join(c + " " * (widths[c] - _width(c)) for c in columns)
    lines = [head, "-" * _width(head)]
    for r in rows:
        lines.append("  ".join(_cell(r.get(c)) + " " * (widths[c] - _width(r.get(c)))
                               for c in columns))
    return "\n".join(lines)


def _cell(value) -> str:
    return "-" if value is None else str(value)


def _width(text) -> int:
    text = _cell(text)
    return sum(2 if ord(ch) > 127 else 1 for ch in text)


def save_json(path, payload) -> None:
    import json
    import dataclasses
    from pathlib import Path

    def default(value):
        if dataclasses.is_dataclass(value):
            return dataclasses.asdict(value)
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, (np.integer, np.floating)):
            return value.item()
        raise TypeError(str(type(value)))

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, default=default),
                    encoding="utf-8")
