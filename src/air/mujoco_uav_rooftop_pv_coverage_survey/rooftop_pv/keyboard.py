"""Deterministic keyboard teleoperation and MuJoCo replay / 键盘遥操作与 MuJoCo 回放。"""

from __future__ import annotations

from pathlib import Path
from typing import Any
import json

from PIL import Image

from .rendering import render_rgb
from .scene import load_model, set_drone_pose


KEYMAP = {
    "w": (1.0, 0.0), "s": (-1.0, 0.0),
    "a": (0.0, 1.0), "d": (0.0, -1.0),
    "q": (0.0, 0.5), "e": (0.0, -0.5),
    " ": (0.0, 0.0),
}


def command_for_key(key: str, speed: float = 2.0) -> dict[str, float]:
    """Map W/S/A/D/Q/E/space to safe velocity commands / 将键盘映射为安全速度指令。"""
    forward, yaw = KEYMAP.get(key.lower(), (0.0, 0.0))
    return {"forward_speed": forward * speed, "yaw_rate": yaw}


def run_keyboard_demo(output: Path, scenario: str = "sparse") -> dict[str, Any]:
    """Replay a documented key sequence in MuJoCo and save an animated sensor trace."""
    output.mkdir(parents=True, exist_ok=True)
    model, data = load_model(scenario)
    keys = ["w", "w", "a", "w", "d", "w", " "]
    x, y, yaw = -8.0, -5.0, 0.0
    frames: list[Image.Image] = []
    commands: list[dict[str, float]] = []
    for key in keys:
        command = command_for_key(key)
        commands.append({"key": key, **command})
        x += command["forward_speed"] * 0.25
        yaw += command["yaw_rate"] * 0.25
        set_drone_pose(model, data, x, y, 8.0, yaw)
        frames.append(render_rgb(model, data, width=320, height=240))
    gif_path = output / "keyboard_demo.gif"
    frames[0].save(gif_path, save_all=True, append_images=frames[1:], duration=180, loop=0, optimize=True)
    (output / "keyboard_commands.json").write_text(json.dumps(commands, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return {"scenario": scenario, "frames": len(frames), "gif": gif_path.name, "keys": keys, "mode": "keyboard_teleop_replay"}
