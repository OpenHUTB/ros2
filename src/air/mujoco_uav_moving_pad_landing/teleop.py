"""Keyboard teleoperation for the MuJoCo landing simulation."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Callable

import cv2
import numpy as np

from .landing import DT, LandingSimulation


def command_for_key(key: str, speed: float = 0.35) -> np.ndarray | None:
    """Map one keyboard command to bounded x/y/z velocity or stop on quit."""
    normalized = key.strip().lower()
    commands = {
        "w": (speed, 0.0, 0.0),
        "s": (-speed, 0.0, 0.0),
        "a": (0.0, speed, 0.0),
        "d": (0.0, -speed, 0.0),
        "r": (0.0, 0.0, speed * 0.6),
        "f": (0.0, 0.0, -speed * 0.6),
    }
    if normalized == "q":
        return None
    if normalized in ("", "space"):
        return np.zeros(3, dtype=np.float32)
    if normalized not in commands:
        raise ValueError("use W/A/S/D to move, R/F for altitude, Space to hold, or Q to quit")
    return np.asarray(commands[normalized], dtype=np.float32)


def run_keyboard_teleop(
    output_dir: Path,
    input_fn: Callable[[str], str] = input,
    steps_per_command: int = 10,
) -> dict[str, int | float]:
    """Apply line-oriented keyboard commands and save the final rendered state."""
    if steps_per_command < 1:
        raise ValueError("steps_per_command must be positive")
    output_dir.mkdir(parents=True, exist_ok=True)
    sim = LandingSimulation()
    trace: list[dict[str, float | str]] = []
    time_s = 0.0
    try:
        while True:
            key = input_fn("W/A/S/D 移动，R/F 升降，空格悬停，Q 结束：").strip().lower()
            command = command_for_key(key)
            if command is None:
                break
            for _ in range(steps_per_command):
                sim.step(command, time_s)
                time_s += DT
            position = sim.position
            trace.append({"key": key, "x_m": float(position[0]), "y_m": float(position[1]),
                          "z_m": float(position[2])})
            print(f"位置：x={position[0]:.3f} m, y={position[1]:.3f} m, z={position[2]:.3f} m")
        image, _ = sim.render(time_s)
        cv2.imwrite(str(output_dir / "teleop_scene.png"), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        (output_dir / "teleop_trace.json").write_text(
            json.dumps(trace, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        return {"commands": len(trace), "simulation_steps": len(trace) * steps_per_command,
                "final_x_m": float(sim.position[0]), "final_y_m": float(sim.position[1]),
                "final_z_m": float(sim.position[2])}
    finally:
        sim.close()
