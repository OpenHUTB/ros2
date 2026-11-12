"""闭环评估：让策略自己飞，统计成功率、末端进度、姿态与指令偏离。

评估和训练用的是同一套环境（``env.ThrustEnv``），区别只有一个：
训练时标签来自专家，评估时策略输出什么就执行什么。

对比的基线有两条：

* ``expert``：非神经网络的完整手工链路（几何控制器 + 加权分配），故障下能飞，
  但没有"感知"——它知道目标点在哪儿；
* ``neural`` / 各结构：只有深度图与本体量，不知道任何位置，端到端输出桨推力。

所以这张表本身就在回答"端到端模型能不能替代手工链路"。
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field

import numpy as np

from . import policy as P
from .data import EVAL_CASES, make_plan
from .env import ThrustEnv
from .expert import TaskConfig


@dataclass
class FlightRow:
    mode: str
    case: str
    seed: int
    success: bool
    crashed: bool
    t: float
    final_x: float
    final_z: float
    min_altitude: float
    max_tilt_deg: float
    cmd_dev: float
    steps: int

    def as_dict(self) -> dict:
        return asdict(self)


def _tilt_deg(state) -> float:
    """倾角（deg）：机体 z 轴与世界 z 轴的夹角。"""
    roll = float(getattr(state, "roll", 0.0))
    pitch = float(getattr(state, "pitch", 0.0))
    cos_tilt = np.cos(roll) * np.cos(pitch)
    return float(np.degrees(np.arccos(np.clip(cos_tilt, -1.0, 1.0))))


def run_episode(api, mode: str = "expert", policy_module=None, spec: dict | None = None,
                label: str = "", seed: int = 0, task: TaskConfig | None = None,
                stack: int = P.DEPTH_STACK) -> FlightRow:
    task = task or TaskConfig()
    plan = make_plan(spec or {"kind": "healthy"})
    env = ThrustEnv(api, task=task, plan=plan, seed=seed)
    try:
        ob = env.reset(seed=seed, plan=plan)
        history = [ob["depth"][-1].copy()] * stack
        min_alt, max_tilt, devs, n = 1e9, 0.0, [], 0
        info = None
        done = False
        while not done:
            expert_cmd = env.expert_command()
            if mode == "expert" or policy_module is None:
                act = expert_cmd
            else:
                act = P.predict(policy_module, {
                    "depth": P.stack_depth(history, policy_module),
                    "proprio": ob["proprio"][None]})[0]
                act = np.clip(act, 0.0, 1.0)
                devs.append(float(np.mean(np.abs(act - expert_cmd))))
            history.append(ob["depth"][-1].copy())
            ob, info = env.step(act)
            min_alt = min(min_alt, float(info.position[2]))
            max_tilt = max(max_tilt, _tilt_deg(env.quad.state()))
            n += 1
            done = info.done
        return FlightRow(
            mode=mode, case=label or plan.label(), seed=seed,
            success=bool(info.success), crashed=bool(info.crashed),
            t=float(info.t), final_x=float(info.position[0]),
            final_z=float(info.position[2]), min_altitude=float(min_alt),
            max_tilt_deg=float(max_tilt),
            cmd_dev=float(np.mean(devs)) if devs else 0.0, steps=n,
        )
    finally:
        env.close()


def flight_table(api, policies: dict[str, object] | None = None,
                 cases=EVAL_CASES, seeds=(0,), task: TaskConfig | None = None,
                 verbose: bool = True) -> list[FlightRow]:
    """``policies`` 里 key 是显示名（如 ``cnn``），value 是已加载的模块。"""
    rows: list[FlightRow] = []
    for label, spec in cases:
        for seed in seeds:
            rows.append(run_episode(api, mode="expert", spec=spec, label=label,
                                    seed=seed, task=task))
            for name, module in (policies or {}).items():
                rows.append(run_episode(api, mode=name, policy_module=module,
                                        spec=spec, label=label, seed=seed,
                                        task=task))
        if verbose:
            done = [r for r in rows if r.case == label]
            print(f"  [{label}] " + " ".join(
                f"{r.mode}={'OK' if r.success else ('X' if r.crashed else '-')}"
                for r in done))
    return rows


def aggregate(rows: list[FlightRow]) -> dict:
    out = {}
    for mode in sorted({r.mode for r in rows}):
        sub = [r for r in rows if r.mode == mode]
        out[mode] = {
            "n": len(sub),
            "success_rate": float(np.mean([r.success for r in sub])),
            "crash_rate": float(np.mean([r.crashed for r in sub])),
            "mean_final_x": float(np.mean([r.final_x for r in sub])),
            "mean_min_altitude": float(np.mean([r.min_altitude for r in sub])),
            "mean_max_tilt_deg": float(np.mean([r.max_tilt_deg for r in sub])),
            "mean_cmd_dev": float(np.mean([r.cmd_dev for r in sub])),
            "mean_t": float(np.mean([r.t for r in sub])),
        }
    return out


def format_summary(summary: dict) -> str:
    header = (f"{'模式':<12}{'样本':>6}{'成功率':>8}{'坠机率':>8}"
              f"{'末端进度m':>11}{'最低高度m':>11}{'最大倾角deg':>13}{'指令偏离':>10}")
    lines = [header, "-" * _width(header)]
    for mode, row in summary.items():
        lines.append(
            f"{mode:<12}{row['n']:>6}{row['success_rate']:>8.2f}{row['crash_rate']:>8.2f}"
            f"{row['mean_final_x']:>11.2f}{row['mean_min_altitude']:>11.2f}"
            f"{row['mean_max_tilt_deg']:>13.1f}{row['mean_cmd_dev']:>10.3f}")
    return "\n".join(lines)


def _width(text: str) -> int:
    return sum(2 if ord(c) > 127 else 1 for c in text)


def format_table(rows: list[FlightRow]) -> str:
    header = (f"{'模式':<12}{'工况':<20}{'结果':<10}{'t/s':>7}{'末端x':>8}"
              f"{'最低z':>8}{'倾角':>7}")
    lines = [header, "-" * _width(header)]
    for r in rows:
        status = "到达" if r.success else ("坠机" if r.crashed else "超时")
        lines.append(
            f"{r.mode:<12}{r.case:<20}{status:<10}{r.t:>7.2f}{r.final_x:>8.2f}"
            f"{r.min_altitude:>8.2f}{r.max_tilt_deg:>7.1f}")
    return "\n".join(lines)


def save_json(path, payload) -> None:
    import json
    from pathlib import Path

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
