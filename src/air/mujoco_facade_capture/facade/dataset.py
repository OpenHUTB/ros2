"""按完整仿真场景采集监督数据，避免相邻时间步跨集合泄漏。"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable

import numpy as np

from .learning import control_features
from .route import scan_route
from .simulation import FacadeSimulation, expert_velocity
from .scene import ray_features


TRAIN_SCENARIOS = (
    {"style": "grid", "distance": 3.5, "offset": -.4},
    {"style": "grid", "distance": 4.5, "offset": .4},
    {"style": "brick", "distance": 4.0, "offset": -.45},
    {"style": "brick", "distance": 4.0, "offset": .45},
)
TEST_SCENARIOS = ({"style": "plain", "distance": 4.2, "offset": .5},)


def collect_scenarios(output: Path, scenarios: Iterable[dict],
                      sample_every: int = 5, duration: float = 70.) -> dict:
    """运行指定场景并保存输入、专家速度标签与场景级划分记录。"""
    if sample_every < 1 or duration <= 0:
        raise ValueError("采样间隔与运行时间必须为正数")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    all_features, all_targets = [], []
    manifest = []
    for episode, scenario in enumerate(scenarios):
        sim = FacadeSimulation(**scenario)
        route = scan_route(distance=float(scenario["distance"]))
        waypoint = 0
        samples = 0
        try:
            steps = int(duration / sim.model.opt.timestep)
            for step in range(steps):
                target = route[waypoint]
                error = target - sim.position
                ranges = sim.ranges()
                projected = ray_features(ranges)
                valid = projected[np.isfinite(projected)]
                if valid.size:
                    error[1] = float(np.median(valid)) - float(scenario["distance"])
                    velocity = sim.velocity
                    command = expert_velocity(error, sim.velocity)
                    if step % sample_every == 0:
                        all_features.append(control_features(error, ranges, velocity,
                                                             float(scenario["distance"])))
                        all_targets.append(command)
                        samples += 1
                else:
                    command = np.zeros(3)
                sim.step(command)
                if np.linalg.norm(target - sim.position) < .12 and np.linalg.norm(sim.velocity) < .2:
                    waypoint += 1
                    if waypoint == len(route):
                        break
        finally:
            sim.close()
        manifest.append({**scenario, "episode": episode, "samples": samples,
                         "waypoints_completed": waypoint, "waypoints_total": len(route)})
    if not all_features:
        raise RuntimeError("仿真没有生成有效测距训练样本")
    np.savez_compressed(output / "samples.npz", features=np.asarray(all_features),
                        targets=np.asarray(all_targets))
    report = {"samples": len(all_features), "sample_interval_steps": sample_every,
              "episodes": manifest}
    (output / "manifest.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return report
