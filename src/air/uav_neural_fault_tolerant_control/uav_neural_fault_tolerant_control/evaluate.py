"""评测：容错分配到底有没有用，以及神经网络离精确解有多远、快多少。

两类实验：

``flight_table``
    同一组故障工况下，跑 ``geometric`` / ``naive`` / ``ls`` / ``neural`` 四种模式，
    比生存率、最低高度、位置跟踪误差、控制能量。
``allocator_table``
    离线比对三种分配方式：``ls``（近似）、``qp``（精确枚举）、``neural``，
    比指令误差与单步耗时。
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass

import numpy as np

from . import allocation as A
from . import data as D
from . import faults as FL
from . import model as M


@dataclass
class FlightRow:
    """一次飞行的汇总指标。"""

    mode: str
    plan: str
    trajectory: str
    crashed: bool
    crash_time: float | None
    min_altitude: float
    final_altitude: float
    pos_rmse: float
    mean_error: float
    effort: float
    steps: int

    def as_dict(self) -> dict:
        return {
            "模式": self.mode,
            "工况": self.plan,
            "轨迹": self.trajectory,
            "坠机": "是" if self.crashed else "否",
            "坠机时刻s": "" if self.crash_time is None else round(self.crash_time, 2),
            "最低高度m": round(self.min_altitude, 3),
            "末端高度m": round(self.final_altitude, 3),
            "位置RMSEm": round(self.pos_rmse, 3),
            "平均误差m": round(self.mean_error, 3),
            "控制能量": round(self.effort, 1),
        }


def run_mode(api, mode: str, plan: FL.FaultPlan, trajectory: str = "hover",
             seconds: float = 8.0, seed: int = 0, net=None,
             params=None, effectiveness=None,
             ) -> FlightRow:
    """用指定模式飞一次，返回汇总指标。"""
    from . import controller as C
    from . import trajectory as TJ

    if params is None or effectiveness is None:
        params, effectiveness = D.build_context(api)

    quad = api.Quadrotor(scene=api.make_scene("open_ground", seed=int(seed)),
                         dt=0.01, substeps=4)
    quad.reset(np.array([0.0, 0.0, 1.5]), 0.0)
    base = api.GeometricController(params)
    ctrl = C.FaultTolerantController(params, api.mixer, base, effectiveness,
                                     mode=mode, net=net)

    alts, errs, efforts = [], [], []
    crashed, crash_time = False, None
    n_steps = int(round(seconds / 0.01))

    for k in range(n_steps):
        t = k * 0.01
        gains = plan.gains(t)
        bias = plan.bias(t, params.hover_thrust)
        ctrl.set_health(gains, bias)
        p_d, v_d, a_d, yaw_d = TJ.reference(trajectory, t)
        u = ctrl.act(quad, p_d, v_d, a_d, yaw_d)

        u_act = np.clip(gains * u + bias, 0.0, float(params.max_thrust))
        quad.apply_rotor_thrust(u_act)
        quad.step(1)
        state = quad.state()

        alts.append(float(state.pos[2]))
        errs.append(float(np.linalg.norm(state.pos - p_d)))
        efforts.append(float(np.sum(u_act ** 2)))
        if not crashed and state.pos[2] < 0.15:
            crashed, crash_time = True, t
            break

    alts_arr = np.asarray(alts)
    errs_arr = np.asarray(errs)
    return FlightRow(
        mode=mode, plan=plan.label(), trajectory=trajectory,
        crashed=crashed, crash_time=crash_time,
        min_altitude=float(alts_arr.min()), final_altitude=float(alts_arr[-1]),
        pos_rmse=float(np.sqrt(np.mean(errs_arr ** 2))),
        mean_error=float(errs_arr.mean()),
        effort=float(np.mean(efforts)), steps=int(alts_arr.size),
    )


#: 飞行对比用的工况矩阵
FLIGHT_PLANS = (
    FL.FaultPlan(kind="healthy"),
    FL.FaultPlan(kind="degradation", rotor=1, onset=2.0, severity=0.5),
    FL.FaultPlan(kind="degradation", rotor=2, onset=2.0, severity=0.3),
    FL.FaultPlan(kind="stuck", rotor=1, onset=2.0),
    FL.FaultPlan(kind="loss", rotor=1, onset=2.0),
    FL.FaultPlan(kind="dual_loss", rotor=0, rotor2=2, onset=2.0),
)
#: 对比用的控制模式
FLIGHT_MODES = ("geometric", "naive", "ls", "neural")


def flight_table(api, modes=FLIGHT_MODES, plans=FLIGHT_PLANS,
                 trajectories=("hover", "zigzag"), seconds: float = 8.0,
                 seed: int = 0, net=None, progress=None) -> list[FlightRow]:
    params, effectiveness = D.build_context(api)
    rows: list[FlightRow] = []
    total = len(modes) * len(plans) * len(trajectories)
    done = 0
    for trajectory in trajectories:
        for plan in plans:
            for mode in modes:
                if mode == "neural" and net is None:
                    continue
                rows.append(run_mode(api, mode, plan, trajectory, seconds, seed,
                                     net=net, params=params,
                                     effectiveness=effectiveness))
                done += 1
                if progress is not None:
                    progress(done, total, mode, plan.label(), trajectory)
    return rows


def aggregate_rows(rows: list[FlightRow], key: str = "模式") -> list[dict]:
    """按模式/工况聚合：生存率、平均最低高度、平均 RMSE。"""
    groups: dict[str, list[FlightRow]] = {}
    for row in rows:
        name = getattr(row, {"模式": "mode", "工况": "plan", "轨迹": "trajectory"}[key])
        groups.setdefault(str(name), []).append(row)

    out = []
    for name, items in groups.items():
        n = len(items)
        out.append({
            key: name,
            "飞行次数": n,
            "生存率": round(sum(0 if it.crashed else 1 for it in items) / n, 3),
            "平均最低高度m": round(float(np.mean([it.min_altitude for it in items])), 3),
            "平均末端高度m": round(float(np.mean([it.final_altitude for it in items])), 3),
            "平均位置RMSEm": round(float(np.mean([it.pos_rmse for it in items])), 3),
            "平均控制能量": round(float(np.mean([it.effort for it in items])), 1),
        })
    return out


# --------------------------------------------------------------------------- #
# 分配器精度与耗时
# --------------------------------------------------------------------------- #
def allocator_table(dataset: dict, net=None, n_samples: int = 600,
                    seed: int = 0) -> dict:
    """在数据集样本上比 ls / neural 的指令误差（相对精确解 u_star）。

    数据集里已经存了 ``u_ls`` 与 ``F.build_target(u_star, u_ls)``（残差 / 单桨最大推力），
    所以误差可以精确还原，不需要重跑仿真。
    """
    x = dataset["X"]
    y = dataset["Y"]          # (u_star − u_ls) / T_max
    scale = float(dataset["max_thrust"])

    rng = np.random.default_rng(int(seed))
    n = int(min(n_samples, x.shape[0]))
    idx = rng.choice(x.shape[0], size=n, replace=False)
    x_sel = x[idx]
    residual_true = y[idx]

    err_ls = 0.0  # ls 基线在残差空间里恒为 0，直接用这个基准

    start = time.perf_counter()
    if net is not None:
        pred = M.predict_numpy(net, x_sel)
    else:
        pred = np.zeros_like(residual_true)
    neural_ms = (time.perf_counter() - start) / n * 1e3

    diff = np.abs(pred - residual_true) * scale
    return {
        "n_samples": n,
        "command_mae_n": float(np.mean(diff)),          # 单桨指令误差（牛顿）
        "command_max_n": float(np.max(diff)),
        "per_rotor_mae_n": [float(v) for v in np.mean(diff, axis=0)],
        "residual_mae": float(np.mean(np.abs(pred - residual_true))),
        "neural_ms": neural_ms,
    }


def trace_mode(api, mode: str, plan: FL.FaultPlan, trajectory: str = "hover",
               seconds: float = 8.0, seed: int = 0, net=None,
               params=None, effectiveness=None) -> dict:
    """飞一次并返回逐步轨迹，供画图用。"""
    from . import controller as C
    from . import trajectory as TJ

    if params is None or effectiveness is None:
        params, effectiveness = D.build_context(api)
    quad = api.Quadrotor(scene=api.make_scene("open_ground", seed=int(seed)),
                         dt=0.01, substeps=4)
    quad.reset(np.array([0.0, 0.0, 1.5]), 0.0)
    base = api.GeometricController(params)
    ctrl = C.FaultTolerantController(params, api.mixer, base, effectiveness,
                                     mode=mode, net=net)

    out = {"t": [], "alt": [], "alt_d": [], "err": [], "u": [], "gains": [],
           "crashed": False, "crash_time": None}
    for k in range(int(round(seconds / 0.01))):
        t = k * 0.01
        gains = plan.gains(t)
        ctrl.set_health(gains, plan.bias(t, params.hover_thrust))
        p_d, v_d, a_d, yaw_d = TJ.reference(trajectory, t)
        u = ctrl.act(quad, p_d, v_d, a_d, yaw_d)
        quad.apply_rotor_thrust(np.clip(gains * u + plan.bias(t, params.hover_thrust),
                                        0.0, float(params.max_thrust)))
        quad.step(1)
        st = quad.state()
        out["t"].append(t)
        out["alt"].append(float(st.pos[2]))
        out["alt_d"].append(float(p_d[2]))
        out["err"].append(float(np.linalg.norm(st.pos - p_d)))
        out["u"].append(np.array(u))
        out["gains"].append(np.array(gains))
        if st.pos[2] < 0.15:
            out["crashed"], out["crash_time"] = True, t
            break
    for key in ("t", "alt", "alt_d", "err"):
        out[key] = np.asarray(out[key], dtype=float)
    out["u"] = np.asarray(out["u"], dtype=float)
    out["gains"] = np.asarray(out["gains"], dtype=float)
    return out



def time_allocators(params, effectiveness, n: int = 200, seed: int = 0) -> dict:
    """纯计算耗时：ls（一次 4×4 解） vs qp（最多 81 种枚举）。"""
    rng = np.random.default_rng(int(seed))
    desired = np.array([4.0 * params.hover_thrust, 0.05, -0.03, 0.01])
    gains = np.array([1.0, 0.0, 1.0, 1.0])
    out = {}
    for name, fn in (("ls", A.ls_allocation), ("qp", A.solve_box_qp)):
        start = time.perf_counter()
        for _ in range(int(n)):
            fn(effectiveness, desired, params, gains)
        out[name] = (time.perf_counter() - start) / int(n) * 1e3
    return out


def format_table(rows: list[FlightRow]) -> str:
    """把飞行结果排成等宽中文表。"""
    headers = ["模式", "工况", "轨迹", "坠机", "坠机时刻s", "最低高度m",
               "位置RMSEm", "控制能量"]
    data = []
    for row in rows:
        d = row.as_dict()
        data.append([str(d[h]) for h in headers])

    def width(text: str) -> int:
        return sum(2 if ord(ch) > 127 else 1 for ch in text)

    widths = [max(width(h), *(width(r[i]) for r in data)) for i, h in enumerate(headers)]

    def pad(text: str, size: int) -> str:
        return text + " " * (size - width(text))

    lines = ["  ".join(pad(h, w) for h, w in zip(headers, widths)),
             "-" * (sum(widths) + 2 * (len(widths) - 1))]
    for row in data:
        lines.append("  ".join(pad(c, w) for c, w in zip(row, widths)))
    return "\n".join(lines)


def format_dicts(rows: list[dict]) -> str:
    if not rows:
        return "(空)"
    headers = list(rows[0].keys())
    data = [[str(row.get(h, "")) for h in headers] for row in rows]

    def width(text: str) -> int:
        return sum(2 if ord(ch) > 127 else 1 for ch in text)

    widths = [max(width(h), *(width(r[i]) for r in data)) for i, h in enumerate(headers)]

    def pad(text: str, size: int) -> str:
        return text + " " * (size - width(text))

    lines = ["  ".join(pad(h, w) for h, w in zip(headers, widths)),
             "-" * (sum(widths) + 2 * (len(widths) - 1))]
    for row in data:
        lines.append("  ".join(pad(c, w) for c, w in zip(row, widths)))
    return "\n".join(lines)


def save_json(path, payload) -> None:
    from pathlib import Path
    Path(path).write_text(json.dumps(payload, ensure_ascii=False, indent=2),
                          encoding="utf-8")
