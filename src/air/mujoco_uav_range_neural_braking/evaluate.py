"""输出多场景指标与物理场景图。 Export scenario metrics and physics views."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
from PIL import Image, ImageDraw

from simulation import Scenario, Simulation


def evaluate(output):
    """覆盖静止、巡航与高速接近。 Cover rest, cruise and fast approach."""
    output.mkdir(parents=True, exist_ok=True)
    reports = []
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.5))
    for speed in (0.0, 1.5, 3.0):
        sim = Simulation(Scenario(name=f"speed_{speed}", speed=speed))
        rows, summary = sim.run()
        reports.append(summary)
        axes[0].plot([r["time_s"] for r in rows], [r["range_m"] for r in rows], label=f"v0={speed} m/s")
        axes[1].plot([r["time_s"] for r in rows], [r["speed_m_s"] for r in rows])
    axes[0].set(xlabel="Time (s)", ylabel="Clearance (m)")
    axes[1].set(xlabel="Time (s)", ylabel="Speed (m/s)")
    axes[0].legend()
    for axis in axes:
        axis.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(output / "braking_curves.png", dpi=140)
    plt.close(fig)
    (output / "baseline_metrics.json").write_text(json.dumps(reports, indent=2), encoding="utf-8")
    return reports


def render(output):
    """记录同一物理过程，抽帧不改变步长。 Sample the same fixed-step physics process."""
    output.mkdir(parents=True, exist_ok=True)
    sim = Simulation()
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [-3.4, 0, 0.8]
    camera.distance, camera.azimuth, camera.elevation = 10, 100, -22
    frames = []
    with mujoco.Renderer(sim.model, height=400, width=720) as renderer:
        for index in range(1000):
            row = sim.step()
            if index % 15:
                continue
            renderer.update_scene(sim.data, camera=camera)
            frame = Image.fromarray(renderer.render())
            ImageDraw.Draw(frame).text((16, 16), f"MuJoCo | t={row['time_s']:.2f}s | gap={row['range_m']:.2f}m | v={row['speed_m_s']:.2f}m/s", fill="white", stroke_width=1, stroke_fill="black")
            frames.append(frame)
    frames[0].save(output / "scene.png")
    frames[0].save(output / "braking.gif", save_all=True, append_images=frames[1:], duration=150, loop=0, optimize=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("results"))
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()
    print(json.dumps(evaluate(args.output), indent=2))
    if args.render:
        render(args.output)
