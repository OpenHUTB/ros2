"""Render a deterministic replay from the roundabout kinematics model."""

from __future__ import annotations

import math
import shutil
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from carla_nn_roundabout_entry.roundabout_sim import (
    GapAcceptanceController,
    RoundaboutWorld,
)


def render(output: Path) -> None:
    world = RoundaboutWorld(seed=7)
    controller = GapAcceptanceController()
    frames = []
    scale, center_x, center_y = 13.0, 400, 300
    font_path = Path("C:/Windows/Fonts/msyh.ttc")
    font = ImageFont.truetype(str(font_path), 18) if font_path.exists() else ImageFont.load_default()

    for step in range(300):
        throttle, brake, steer = controller.command(world, step)
        world.step(throttle, brake, steer)
        if step % 5:
            continue

        image = Image.new("RGB", (800, 600), "#f4f5f7")
        draw = ImageDraw.Draw(image)
        draw.ellipse((center_x - 10 * scale, center_y - 10 * scale,
                      center_x + 10 * scale, center_y + 10 * scale),
                     fill="#d8dce2", outline="#4b5563", width=3)
        draw.ellipse((center_x - 6 * scale, center_y - 6 * scale,
                      center_x + 6 * scale, center_y + 6 * scale),
                     fill="#f4f5f7", outline="#4b5563", width=3)
        draw.rectangle((center_x - 24 * scale, center_y - 3 * scale,
                        center_x - 6 * scale, center_y + 3 * scale),
                       fill="#d8dce2", outline="#4b5563", width=3)

        for car in world.traffic:
            x, y = car.position(*world.center)
            px, py = center_x + x * scale, center_y - y * scale
            draw.rounded_rectangle((px - 20, py - 11, px + 20, py + 11),
                                   radius=5, fill="#d97706")

        ego = world.ego
        ego_x, ego_y = center_x + ego.x * scale, center_y - ego.y * scale
        draw.rounded_rectangle((ego_x - 23, ego_y - 13, ego_x + 23, ego_y + 13),
                               radius=5, fill="#047857")
        metrics = world.metrics()
        draw.text((18, 16),
                  f"种子 7  |  时间 {metrics['elapsed_s']:.1f} 秒  |  速度 {ego.speed:.1f} 米/秒",
                  fill="#111827", font=font)
        draw.text((18, 44),
                  f"间隙策略  |  碰撞帧 {metrics['collision_frames']}  |  "
                  f"最小净间距 {metrics['minimum_clearance_m']:.1f} 米",
                  fill="#111827", font=font)
        frames.append(image)

    output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(output, save_all=True, append_images=frames[1:],
                    duration=100, loop=0, optimize=True)


def main() -> None:
    package_root = Path(__file__).resolve().parents[1]
    repository_root = package_root.parents[2]
    source = package_root / "assets" / "roundabout_demo.gif"
    still = package_root / "assets" / "roundabout_demo.png"
    documentation = repository_root / "docs" / "img" / "roundabout_demo.gif"
    documentation_still = repository_root / "docs" / "img" / "roundabout_demo.png"
    render(source)
    with Image.open(source) as animation:
        animation.seek(0)
        animation.convert("RGB").save(still, optimize=True)
    documentation.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, documentation)
    shutil.copyfile(still, documentation_still)
    print(f"动画已生成：{source}")
    print(f"文档动画：{documentation}")


if __name__ == "__main__":
    main()
