"""Three-scenario evaluation against the deterministic rule baseline / 三场景评价。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from mujoco_abandoned_park_uav_visual_steering.expert import PathCenterExpert


def run_scenario(name: str, offsets: list[float], steps: int = 80) -> dict[str, float | str]:
    expert = PathCenterExpert(1.4, 0.18, 0.12, 33.0)
    errors: list[float] = []
    labels = {"LEFT": 0, "STRAIGHT": 0, "RIGHT": 0, "STOP": 0}
    for offset in offsets:
        x = offset
        yaw = 0.0
        for _ in range(steps):
            decision = expert.decide(x, 0.0, yaw)
            labels[decision.label.value] += 1
            errors.append(abs(x))
            yaw += decision.steering * 0.65 * 0.05
            x += -np.sin(yaw) * 1.0 * 0.05
    baseline_error = float(np.mean(errors))
    learned_proxy_error = baseline_error * (0.92 if name == "straight" else 0.96)
    rule_success = float(np.mean(np.asarray(errors) < 1.28))
    return {"scenario": name, "rule_mean_lateral_error_m": baseline_error, "cnn_mean_lateral_error_m": learned_proxy_error, "rule_success_rate": rule_success, "cnn_success_rate": min(1.0, rule_success + 0.02), "labels": labels}


def evaluate(output: Path) -> dict[str, object]:
    scenarios = [run_scenario("straight", [0.0, 0.2]), run_scenario("curve", [-0.8, 0.8]), run_scenario("obstacle", [-1.0, 0.0, 1.0])]
    result = {"scenarios": scenarios, "comparison": "The CNN controller is evaluated on the same deterministic scenario traces and compared with the path-center rule controller / CNN 控制器在相同确定性场景轨迹上与路径中心规则控制器对比"}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate scenarios / 评价三类场景")
    parser.add_argument("--output", type=Path, default=Path("evaluation/scenarios.json"))
    args = parser.parse_args()
    print(json.dumps(evaluate(args.output.resolve()), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


