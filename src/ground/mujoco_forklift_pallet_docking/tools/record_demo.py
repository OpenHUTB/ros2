#!/usr/bin/env python3
"""Record an animated MuJoCo camera view of neural pallet insertion."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from mujoco_forklift_pallet_docking.policy import ImagePolicy
from mujoco_forklift_pallet_docking.simulation import ForkliftDockingSimulation, train_policy


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    checkpoint = Path(__file__).parents[1] / "mujoco_forklift_pallet_docking" / "models" / "pallet_docking_policy.npz"
    policy = ImagePolicy.load(str(checkpoint)) if checkpoint.is_file() else train_policy()[0]
    font = None
    for font_path in ("C:/Windows/Fonts/msyh.ttc", "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"):
        if Path(font_path).is_file():
            font = ImageFont.truetype(font_path, 20)
            break
    sim = ForkliftDockingSimulation(start_x=-4.0, speed=2.0)
    frames = []
    try:
        for step in range(360):
            camera = np.clip((sim.image().astype(np.float32) - 4.0) * 4.0, 0.0, 255.0).astype(np.uint8)
            image = Image.fromarray(camera, mode="L").resize((640, 640), Image.Resampling.NEAREST).convert("RGB")
            draw = ImageDraw.Draw(image)
            if font:
                draw.rectangle((0, 0, 639, 52), fill=(18, 30, 38))
                draw.text((16, 14), f"叉齿距离：{sim.distance:.2f} 米  车速：{sim.speed:.2f} 米/秒",
                          fill=(242, 245, 247), font=font)
            frames.append(image)
            if sim.distance < -0.5 or sim.speed < 0.03:
                break
            sim.step(policy.predict(sim.image())[0])
    finally:
        sim.close()
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(path, save_all=True, append_images=frames[1:], duration=80, loop=0, optimize=True)
    print(f"frames={len(frames)} output={path}")


if __name__ == "__main__":
    main()
