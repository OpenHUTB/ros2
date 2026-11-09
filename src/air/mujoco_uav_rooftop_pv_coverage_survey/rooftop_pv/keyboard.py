"""Deterministic keyboard teleoperation and MuJoCo replay / 键盘遥操作与 MuJoCo 回放。"""

from __future__ import annotations

from pathlib import Path
from typing import Any
import json
import math
import time

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


class KeyboardState:
    """Viewer key events with a dead-man timeout / 带超时停车的查看器键盘事件。"""

    def __init__(self, timeout: float = 0.4) -> None:
        self.timeout = timeout
        self.key = " "
        self.last_event = -float("inf")

    def on_key(self, keycode: int) -> None:
        key = chr(keycode).lower() if 0 <= keycode < 128 else " "
        self.key = key if key in KEYMAP else " "
        self.last_event = time.monotonic()

    def command(self, now: float | None = None) -> dict[str, float]:
        current = time.monotonic() if now is None else now
        return command_for_key(self.key if current - self.last_event <= self.timeout else " ")


def run_interactive_keyboard(
    output: Path, scenario: str = "sparse", duration: float = 60.0
) -> dict[str, Any]:
    """Control a visible fixed-altitude MuJoCo vehicle with real key events.

    W/S move along the current heading, A/D and Q/E turn, space stops.
    Movement uses velocity integration at fixed height; it is not rotor dynamics.
    W/S沿当前航向运动，A/D与Q/E转向，空格停车；采用固定高度速度积分。
    """
    import mujoco
    import mujoco.viewer

    if duration <= 0:
        raise ValueError("duration must be positive / 时长必须为正数")
    output.mkdir(parents=True, exist_ok=True)
    model, data = load_model(scenario)
    keyboard = KeyboardState()
    x, y, yaw = -5.0, -3.0, 0.0
    set_drone_pose(model, data, x, y, 8.0, yaw)
    renderer = mujoco.Renderer(model, height=240, width=320)
    frames: list[Image.Image] = []
    trace: list[dict[str, Any]] = []
    started = previous = last_frame = time.monotonic()
    try:
        with mujoco.viewer.launch_passive(model, data, key_callback=keyboard.on_key) as viewer:
            viewer.cam.lookat[:] = (0.0, 0.0, 2.0)
            viewer.cam.distance = 22.0
            viewer.cam.elevation = -65.0
            while viewer.is_running() and time.monotonic() - started < duration:
                now = time.monotonic()
                dt = min(now - previous, 0.05)
                previous = now
                command = keyboard.command(now)
                yaw += command["yaw_rate"] * dt
                x = max(-9.0, min(9.0, x + math.cos(yaw) * command["forward_speed"] * dt))
                y = max(-6.0, min(6.0, y + math.sin(yaw) * command["forward_speed"] * dt))
                with viewer.lock():
                    set_drone_pose(model, data, x, y, 8.0, yaw)
                viewer.sync()
                if now - last_frame >= 0.1:
                    renderer.update_scene(data, camera="downward_rgb")
                    frames.append(Image.fromarray(renderer.render().copy()))
                    trace.append({"time_s": now - started, "key": keyboard.key, "x": x, "y": y, "yaw": yaw, **command})
                    last_frame = now
                time.sleep(0.01)
    finally:
        renderer.close()
    if frames:
        frames[0].save(output / "keyboard_interactive.gif", save_all=True, append_images=frames[1:], duration=100, loop=0)
    (output / "keyboard_interactive_trace.json").write_text(json.dumps(trace, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    result = {"mode": "interactive_keyboard", "motion_model": "fixed_altitude_kinematic", "frames": len(frames), "scenario": scenario}
    (output / "keyboard_interactive_summary.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="MuJoCo keyboard control / MuJoCo 键盘控制")
    parser.add_argument("--output", type=Path, default=Path("artifacts/keyboard"))
    parser.add_argument("--scenario", choices=("sparse", "dense", "occluded"), default="sparse")
    parser.add_argument("--interactive", action="store_true", help="open viewer and use keyboard events / 打开查看器并接受键盘事件")
    parser.add_argument("--duration", type=float, default=60.0)
    args = parser.parse_args()
    result = run_interactive_keyboard(args.output, args.scenario, args.duration) if args.interactive else run_keyboard_demo(args.output, args.scenario)
    print(json.dumps(result, ensure_ascii=False))
