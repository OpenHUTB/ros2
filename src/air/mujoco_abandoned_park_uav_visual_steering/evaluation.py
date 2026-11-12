"""Closed-loop MuJoCo evaluation with actual CNN inference / CNN 真实闭环评价。"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image

from mujoco_abandoned_park_uav_visual_steering.expert import PathCenterExpert
from mujoco_abandoned_park_uav_visual_steering.simulation import SimulationConfig, UavSimulation
from ros_node import VisualSteeringCore

MODULE_ROOT = Path(__file__).resolve().parent


def centerline(name: str, y: float) -> tuple[float, float]:
    if name != "curve":
        return 0.0, 0.0
    phase = (y + 7.0) / 3.0
    return 0.6 * math.sin(phase), math.atan(-0.2 * math.cos(phase))


def scenario_scene(source: Path, name: str, output: Path) -> None:
    """Render the evaluated path/obstacle / 渲染真实弯道与障碍场景。"""
    tree = ET.parse(source)
    world = tree.getroot().find("worldbody")
    if name == "curve":
        for geom in list(world.findall("geom")):
            if geom.get("name") in {"walkway", "left_edge", "right_edge"}:
                world.remove(geom)
        for index, y in enumerate(np.arange(-20.0, 34.0, 0.5)):
            x, yaw = centerline(name, float(y))
            ET.SubElement(world, "geom", name=f"path_{index}", type="box",
                          pos=f"{x} {y} 0.025", size="1.4 0.26 0.025",
                          euler=f"0 0 {math.degrees(yaw)}", material="path")
    elif name == "obstacle":
        ET.SubElement(world, "geom", name="evaluation_obstacle", type="cylinder",
                      pos="0 -5 1", size="0.3 1", rgba="0.8 0.15 0.1 1")
    tree.write(output, encoding="unicode")


def run_episode(name, offset, controller, config, scene, steps, core):
    expert = PathCenterExpert(config.path_half_width_m, 0.18, 0.12, 33.0)
    errors, trace = [], []
    counts = {label: 0 for label in ("LEFT", "STRAIGHT", "RIGHT", "STOP")}
    collision = False
    inference_calls = 0
    with UavSimulation(scene, config) as sim:
        sim.reset(offset)
        initial_y = sim.state()["y_m"]
        for _ in range(steps):
            state = sim.state()
            center, desired_yaw = centerline(name, state["y_m"])
            if controller == "cnn":
                command = core.predict(Image.fromarray(sim.capture_rgb()))
                inference_calls += 1
                label = command.label
                steering = command.angular_z / core.steering_rate
                speed = command.linear_x * config.forward_speed_mps
            else:
                decision = expert.decide(state["x_m"] - center, state["y_m"],
                                         state["yaw_radians"] - desired_yaw)
                label, steering = decision.label.value, decision.steering
                speed = 0.0 if label == "STOP" else config.forward_speed_mps
            counts[label] += 1
            sim.step(float(steering), speed)
            state = sim.state()
            center, _ = centerline(name, state["y_m"])
            error = abs(state["x_m"] - center)
            errors.append(error)
            collision = collision or (name == "obstacle" and math.hypot(
                state["x_m"], state["y_m"] + 5.0) <= 0.5)
            trace.append({**state, "label": label, "lateral_error_m": error})
        progress = state["y_m"] - initial_y
        dt = sim.control_steps * sim.model.opt.timestep
    success = (max(errors) < config.path_half_width_m - 0.12 and not collision
               and progress >= 0.6 * steps * dt * config.forward_speed_mps)
    return {"offset_m": offset, "controller": controller, "success": success,
            "collision": collision, "progress_m": progress,
            "mean_lateral_error_m": float(np.mean(errors)), "labels": counts,
            "inference_calls": inference_calls, "trajectory": trace}


def evaluate(output: Path, checkpoint: Path, config_path: Path | None = None,
             model_path: Path | None = None, steps: int = 80) -> dict[str, object]:
    if steps <= 0:
        raise ValueError("steps must be positive / 评价步数必须为正")
    config_path = config_path or MODULE_ROOT / "config/simulation.yaml"
    model_path = model_path or MODULE_ROOT / "models/scene.xml"
    config = SimulationConfig.from_yaml(config_path)
    core = VisualSteeringCore(checkpoint, config.steering_rate_rps)
    scenarios = []
    with TemporaryDirectory(prefix="uav_eval_") as directory:
        for name in ("straight", "curve", "obstacle"):
            scene = Path(directory) / f"{name}.xml"
            scenario_scene(model_path, name, scene)
            episodes = [run_episode(name, offset, controller, config, scene, steps, core)
                        for offset in (-0.75, 0.0, 0.75)
                        for controller in ("rule", "cnn")]
            item = {"scenario": name, "episodes": episodes}
            for controller in ("rule", "cnn"):
                selected = [e for e in episodes if e["controller"] == controller]
                item[f"{controller}_mean_lateral_error_m"] = float(np.mean([
                    e["mean_lateral_error_m"] for e in selected]))
                item[f"{controller}_success_rate"] = float(np.mean([
                    e["success"] for e in selected]))
            scenarios.append(item)
    result = {"schema_version": 2, "engine": "MuJoCo", "steps_per_episode": steps,
              "checkpoint_sha256": hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
              "scenarios": scenarios,
              "comparison": "Independent closed-loop rollouts from identical initial poses / 相同初始位姿的独立闭环轨迹"}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate actual CNN rollouts / 评价 CNN 闭环轨迹")
    parser.add_argument("--output", type=Path, default=Path("evaluation/scenarios.json"))
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=80)
    args = parser.parse_args()
    result = evaluate(args.output.resolve(), args.checkpoint.resolve(), steps=args.steps)
    print(json.dumps({"checkpoint_sha256": result["checkpoint_sha256"],
                      "scenarios": [{k: v for k, v in s.items() if k != "episodes"}
                                    for s in result["scenarios"]]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


