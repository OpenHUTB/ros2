#!/usr/bin/env python3
"""Record an animated MuJoCo camera view of neural pallet insertion."""

from __future__ import annotations

import argparse
import json
import platform
import shlex
import sys
from pathlib import Path

import numpy as np
import mujoco

# Allow `python3 src/.../tools/record_demo.py ...` from the repository root.
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from PIL import Image

from mujoco_forklift_pallet_docking.policy import ImagePolicy
from mujoco_forklift_pallet_docking.simulation import ForkliftDockingSimulation, train_policy


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, help="GIF 输出路径")
    parser.add_argument("--json-output", required=True, help="与本次 GIF 同回合的 JSON 输出路径")
    parser.add_argument("--start-x", type=float, default=-4.0)
    parser.add_argument("--speed", type=float, default=2.0)
    parser.add_argument("--brake-scale", type=float, default=1.0, help="显式制动力校准系数")
    parser.add_argument("--seed", type=int, default=17, help="无权重文件时训练策略所用的种子")
    args = parser.parse_args()
    checkpoint = Path(__file__).parents[1] / "mujoco_forklift_pallet_docking" / "models" / "pallet_docking_policy.npz"
    policy_source = "checkpoint" if checkpoint.is_file() else "trained_fallback"
    policy = ImagePolicy.load(str(checkpoint)) if checkpoint.is_file() else train_policy(seed=args.seed)[0]
    sim = ForkliftDockingSimulation(start_x=args.start_x, speed=args.speed, brake_scale=args.brake_scale)
    frames = []
    final = None
    try:
        for step in range(451):
            camera = np.clip((sim.image().astype(np.float32) - 4.0) * 4.0, 0.0, 255.0).astype(np.uint8)
            # Keep the recording independent of host fonts. The camera image
            # is the evidence; numeric state is written to the companion JSON.
            image = Image.fromarray(camera, mode="L").resize((640, 640), Image.Resampling.NEAREST).convert("RGB")
            frames.append(image)
            if sim.distance < -0.5 or (step > 0 and sim.speed <= 0.025) or step == 450:
                break
            action = policy.predict(sim.image())[0].copy()
            action[1] *= args.brake_scale
            sim.step(action)
        final = {
            "steps": sim.steps,
            "start_x_m": args.start_x,
            "initial_speed_mps": args.speed,
            "insertion_depth_m": round(max(0.0, -sim.distance), 4),
            "final_speed_mps": round(sim.speed, 4),
            "over_inserted": bool(sim.distance < -0.5),
        }
    finally:
        sim.close()
    path = Path(args.output).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(path, save_all=True, append_images=frames[1:], duration=80, loop=0, optimize=True)
    with Image.open(path) as saved_gif:
        encoded_frames = saved_gif.n_frames
    json_path = Path(args.json_output).resolve()
    json_path.parent.mkdir(parents=True, exist_ok=True)
    # The multi-start records are produced by the same process and policy as the
    # GIF episode, so their metadata can be compared without mixing runs.
    scenarios = []
    for start_x in (-5.0, -4.0, -3.0):
        evaluation = ForkliftDockingSimulation(start_x=start_x, speed=args.speed, brake_scale=args.brake_scale)
        try:
            result = evaluation.run(policy)
            scenarios.append({
                "start_x_m": start_x,
                "insertion_depth_m": result["insertion_depth_m"],
                "final_speed_mps": result["final_speed_mps"],
                "over_inserted": result["over_inserted"],
                "steps": result["steps"],
            })
        finally:
            evaluation.close()
    metadata = {
        "schema_version": "1.0",
        "mujoco_version": mujoco.__version__,
        "brake_scale": args.brake_scale,
        "seed": args.seed,
        "seed_usage": "fallback training only; checkpoint evaluation is deterministic",
        "command": "python " + " ".join(shlex.quote(value) for value in sys.argv),
        "platform": platform.platform(),
        "policy_source": policy_source,
        "gif": {"path": args.output, "captured_frames": len(frames),
                "encoded_frames": encoded_frames, "capture_interval_ms": 80},
        "recorded_episode": final,
        "multi_start_evaluation": scenarios,
        "font": "none",
        "image_processing": "clip((gray - 4) * 4, 0, 255), nearest-neighbor resize to 640x640",
    }
    json_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"frames={len(frames)} output={path} json={json_path}")


if __name__ == "__main__":
    main()
