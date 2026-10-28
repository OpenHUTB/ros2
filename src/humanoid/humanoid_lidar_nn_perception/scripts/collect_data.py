"""Collect scan + ground-truth obstacle labels for training the perception CNN.

Runs the G1 ONNX walking policy with random velocity commands in the bundled
obstacle scene. At 10 Hz we record:
  * scan[360] (clean, ground truth)
  * nearest distance / bearing
  * front / left / right channel clear distance

Output: data/scan_dataset.npz  (NOT committed to git)
"""
from __future__ import annotations
import sys, time, argparse
from pathlib import Path

import numpy as np
import mujoco

# make the package importable when running from scripts/
PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))

from humanoid_lidar_nn_perception.g1_walking_policy import G1WalkingPolicy, SIM_DT, N_SUBSTEPS
from humanoid_lidar_nn_perception.lidar_2d import LiDAR2D
from humanoid_lidar_nn_perception.scene_builder import load_model, reset_episode


# channel windows (indices in 360-beam array, angle = linspace(-pi,pi,360))
# index 180 == forward (+x), 90 == left (+y), 270 == right (-y)
FRONT_IDX = np.arange(165, 196)     # +/- 15 deg around forward
LEFT_IDX = np.arange(60, 121)       # 60..120 deg
RIGHT_IDX = np.arange(240, 301)     # -120..-60 deg


def labels_from_scan(ranges: np.ndarray) -> np.ndarray:
    """Return [nearest_dist, nearest_bearing, front_clear, left_clear, right_clear]."""
    nearest_i = int(np.argmin(ranges))
    nearest_dist = float(ranges[nearest_i])
    # angle for beam i
    angles = np.linspace(-np.pi, np.pi, 360, endpoint=False)
    nearest_bearing = float(angles[nearest_i])
    front = float(ranges[FRONT_IDX].min())
    left = float(ranges[LEFT_IDX].min())
    right = float(ranges[RIGHT_IDX].min())
    return np.array([nearest_dist, nearest_bearing, front, left, right], dtype=np.float32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=r"D:\g1_run\scene_g1.xml")
    ap.add_argument("--onnx", default=r"D:\g1_run\g1_policy.onnx")
    ap.add_argument("--out", default=str(PKG_ROOT / "data" / "scan_dataset.npz"))
    ap.add_argument("--num-frames", type=int, default=20000)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    model = load_model(args.model)
    policy = G1WalkingPolicy(model, args.onnx)
    lidar = LiDAR2D(model, num_beams=360, max_range=8.0)

    scans, labels = [], []
    frames_collected = 0
    episodes = 0
    t0 = time.time()

    while frames_collected < args.num_frames:
        episodes += 1
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, 0)
        # randomize start
        x, y = rng.uniform(-6, 6), rng.uniform(-6, 6)
        yaw = rng.uniform(-np.pi, np.pi)
        data.qpos[0], data.qpos[1] = x, y
        data.qpos[3:7] = [np.cos(yaw/2), 0, 0, np.sin(yaw/2)]
        mujoco.mj_forward(model, data)

        # random command held for 2~6 seconds
        vx = rng.uniform(0.1, 0.4)
        wz = rng.uniform(-0.5, 0.5)
        policy.set_command(vx, 0.0, wz)

        episode_steps = 0
        max_steps = int(rng.uniform(2.0, 6.0) / SIM_DT)
        scan_period = 0.1  # 10 Hz
        next_scan_t = 0.0

        while episode_steps < max_steps and frames_collected < args.num_frames:
            for _ in range(N_SUBSTEPS):
                mujoco.mj_step(model, data)
            policy.step(data)
            episode_steps += N_SUBSTEPS

            if data.time >= next_scan_t:
                next_scan_t += scan_period
                scan = lidar.scan(data)
                lbl = labels_from_scan(scan)
                scans.append(scan.copy())
                labels.append(lbl)
                frames_collected += 1

            if policy.fell(data):
                break

        if episodes % 50 == 0:
            rate = frames_collected / max(time.time() - t0, 1e-6)
            print(f"ep={episodes} frames={frames_collected}/{args.num_frames} "
                  f"rate={rate:.0f} f/s  fell={policy.fell(data)}")

    scans = np.stack(scans)
    labels = np.stack(labels)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, scans=scans, labels=labels)
    print(f"\nSaved {scans.shape} scans + {labels.shape} labels -> {out}")
    print(f"wall time: {time.time()-t0:.1f}s, episodes: {episodes}")


if __name__ == "__main__":
    main()
