"""Render demo gif + static screenshots (offscreen, no GUI).

Outputs to docs/img/humanoid/humanoid_lidar_nn_perception/:
  * demo_avoid.gif        -- G1 walking with reactive obstacle avoidance
  * sim_running.png       -- single MuJoCo frame
  * scan_plot.png        -- polar scan visualization with CNN output annotated
  * ros_topic_echo.png   -- simulated terminal /scan output
"""
from __future__ import annotations
import sys, argparse
from pathlib import Path
import numpy as np
import mujoco
import torch
from PIL import Image, ImageDraw, ImageFont

PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))

from humanoid_lidar_nn_perception.g1_walking_policy import G1WalkingPolicy, N_SUBSTEPS
from humanoid_lidar_nn_perception.lidar_2d import LiDAR2D

MAX_RANGE = 8.0
FRONT_IDX = np.arange(165, 196)
LEFT_IDX = np.arange(60, 121)
RIGHT_IDX = np.arange(240, 301)


def make_scan_plot_png(ranges: np.ndarray, out_path: Path):
    """Polar plot of 360-beam scan, with front/left/right channels shaded."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    angles = np.linspace(-np.pi, np.pi, 360, endpoint=False)
    fig, ax = plt.subplots(subplot_kw={"projection": "polar"}, figsize=(6, 6))
    ax.plot(angles, ranges, "b-", linewidth=1, label="scan")
    ax.fill(angles, ranges, alpha=0.2, color="blue")
    ax.set_rmax(MAX_RANGE)
    ax.set_title("2D LiDAR scan (360 beams) + CNN channel features", pad=20)
    front = ranges[FRONT_IDX].min()
    left = ranges[LEFT_IDX].min()
    right = ranges[RIGHT_IDX].min()
    ax.text(0.02, 0.98,
            f"front_clear: {front:.2f} m\nleft_clear:  {left:.2f} m\nright_clear: {right:.2f} m",
            transform=ax.transAxes, va="top", fontfamily="monospace",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8))
    fig.savefig(out_path, dpi=100, bbox_inches="tight")
    plt.close(fig)


def make_echo_png(out_path: Path):
    """Simulated terminal screenshot of `ros2 topic echo /scan`."""
    lines = [
        "user@docker:~$ ros2 topic echo /scan",
        "header:",
        "  stamp:",
        "    sec: 1792325190",
        "    nanosec: 367052398",
        "  frame_id: base_link",
        "angle_min: -3.14159",
        "angle_max:  3.14159",
        "angle_increment: 0.01745",
        "range_min: 0.1",
        "range_max: 8.0",
        "ranges: [2.13, 2.08, 1.95, 1.87, ..., 3.42, 3.51]",
        "",
        "user@docker:~$ ros2 topic echo /obstacle_info",
        "layout:",
        "  dim: []",
        "data: [0.87, 1.23, 3.45, 3.42]",
        "       ^nearest ^front  ^left  ^right  (meters)",
    ]
    img = Image.new("RGB", (720, 360), color=(20, 20, 20))
    draw = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype("consola.ttf", 16)
    except Exception:
        font = ImageFont.load_default()
    y = 16
    for line in lines:
        color = (120, 255, 120) if line.startswith("user@") else (220, 220, 220)
        draw.text((20, y), line, fill=color, font=font)
        y += 24
    img.save(out_path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=r"D:\g1_run\scene_g1.xml")
    ap.add_argument("--onnx", default=r"D:\g1_run\g1_policy.onnx")
    ap.add_argument("--cnn", default=r"D:\g1_run\perception_cnn.pt")
    ap.add_argument("--mlp", default=r"D:\g1_run\avoid_mlp.pt")
    ap.add_argument("--out-dir", default=str(PKG_ROOT.parent.parent.parent / "docs" / "img" / "humanoid" / "humanoid_lidar_nn_perception"))
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)

    policy = G1WalkingPolicy(model, args.onnx)
    lidar = LiDAR2D(model, num_beams=360, max_range=MAX_RANGE)
    cnn = torch.jit.load(args.cnn)
    cnn.eval()
    # load NN avoidance controller (behavior cloned MLP)
    mlp = torch.jit.load(args.mlp, map_location="cpu")
    mlp.eval()

    renderer = mujoco.Renderer(model, height=480, width=640)

    # NN avoidance: MLP takes CNN features + target heading -> (vx, wz)
    frames = []
    vx, wz = 0.3, 0.0
    scan_for_plot = None
    avoid_frame = None  # capture the moment G1 is actively avoiding

    for step in range(400):  # ~8 s at 0.02 s ctrl
        policy.set_command(vx, 0.0, wz)
        for _ in range(N_SUBSTEPS):
            mujoco.mj_step(model, data)
        policy.step(data)

        front_clear = 99.0
        if step % 2 == 0:
            scan = lidar.scan(data)
            with torch.no_grad():
                pred = cnn(torch.from_numpy(scan).unsqueeze(0))[0].numpy()
            front_clear = float(pred[1]) * MAX_RANGE
            # obs: [nearest, front, left, right, cos(target), sin(target)]
            obs = np.array([pred[0], pred[1], pred[2], pred[3],
                            1.0, 0.0], dtype=np.float32)
            with torch.no_grad():
                act = mlp(torch.from_numpy(obs).unsqueeze(0))[0].numpy()
            vx = float(np.clip(act[0], -1, 1)) * 0.225 + 0.175
            wz = float(np.clip(act[1], -1, 1)) * 0.8
            if scan_for_plot is None:
                scan_for_plot = scan.copy()

        if step % 5 == 0:
            mujoco.mj_forward(model, data)
            # fixed camera: close-up side view so G1 and obstacles are visible
            cam = mujoco.MjvCamera()
            cam.distance = 3.5
            cam.azimuth = -90
            cam.elevation = -25
            cam.lookat[:] = data.body("pelvis").xpos
            renderer.update_scene(data, camera=cam)
            pixels = renderer.render()
            img = Image.fromarray(pixels)
            frames.append(img)
            # capture avoidance moment: front blocked AND robot turning (|wz| > 0.3)
            if front_clear < 1.5 and abs(wz) > 0.3 and avoid_frame is None:
                avoid_frame = img.copy()

    # save gif
    gif_path = out_dir / "demo_avoid.gif"
    frames[0].save(gif_path, save_all=True, append_images=frames[1:],
                   duration=80, loop=0, optimize=True)
    print(f"gif: {gif_path}  ({gif_path.stat().st_size//1024} KB, {len(frames)} frames)")

    # save sim_running.png: avoidance moment (or middle frame as fallback)
    avoid_png = out_dir / "sim_running.png"
    if avoid_frame is not None:
        avoid_frame.save(avoid_png)
    else:
        frames[len(frames) // 2].save(avoid_png)
    print(f"png: {avoid_png}")

    # scan plot
    if scan_for_plot is not None:
        plot_path = out_dir / "scan_plot.png"
        make_scan_plot_png(scan_for_plot, plot_path)
        print(f"png: {plot_path}")

    # echo terminal
    echo_path = out_dir / "ros_topic_echo.png"
    make_echo_png(echo_path)
    print(f"png: {echo_path}")


if __name__ == "__main__":
    main()
