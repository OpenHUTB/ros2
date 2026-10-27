"""Live 3D interactive viewer for Unitree G1 LiDAR avoidance.

Launches the MuJoCo native 3D GUI window via WSLg / local display.
You can rotate, pan, and zoom using the mouse while watching G1 walk and avoid obstacles.
"""
from __future__ import annotations
import sys, time, argparse
from pathlib import Path
import numpy as np
import mujoco
import mujoco.viewer
import torch

PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))

from humanoid_lidar_nn_perception.g1_walking_policy import G1WalkingPolicy, N_SUBSTEPS, CTRL_DT
from humanoid_lidar_nn_perception.lidar_2d import LiDAR2D

MAX_RANGE = 8.0


def main():
    ap = argparse.ArgumentParser(description="Live 3D GUI Viewer for G1 LiDAR Obstacle Avoidance")
    ap.add_argument("--model", default=str(PKG_ROOT / "models" / "g1" / "scene_g1.xml"))
    ap.add_argument("--onnx", default=str(PKG_ROOT / "weights" / "g1_policy.onnx"))
    ap.add_argument("--cnn", default=str(PKG_ROOT / "weights" / "perception_cnn.pt"))
    ap.add_argument("--mlp", default=str(PKG_ROOT / "weights" / "avoid_mlp.pt"))
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)

    policy = G1WalkingPolicy(model, args.onnx)
    lidar = LiDAR2D(model, num_beams=360, max_range=MAX_RANGE)
    cnn = torch.jit.load(args.cnn)
    cnn.eval()
    mlp = torch.jit.load(args.mlp, map_location="cpu")
    mlp.eval()

    print("\n[INFO] Launching MuJoCo 3D Interactive Viewer...")
    print("[INFO] Controls:")
    print("  - Left click + drag: Rotate view")
    print("  - Right click + drag: Zoom in/out")
    print("  - Middle click + drag: Pan view")
    print("  - Space: Pause/Resume")
    print("  - Esc: Exit\n")

    vx, wz = 0.3, 0.0
    step = 0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 4.0
        viewer.cam.elevation = -20
        viewer.cam.azimuth = 120
        viewer.cam.lookat[:] = data.body("pelvis").xpos

        while viewer.is_running():
            step_start = time.time()

            policy.set_command(vx, 0.0, wz)
            for _ in range(N_SUBSTEPS):
                mujoco.mj_step(model, data)
            policy.step(data)

            # Every 2 control steps (~25 Hz), run perception + avoidance
            if step % 2 == 0:
                scan = lidar.scan(data)
                with torch.no_grad():
                    pred = cnn(torch.from_numpy(scan).unsqueeze(0))[0].numpy()
                obs = np.array([pred[0], pred[1], pred[2], pred[3], 1.0, 0.0], dtype=np.float32)
                with torch.no_grad():
                    act = mlp(torch.from_numpy(obs).unsqueeze(0))[0].numpy()
                vx = float(np.clip(act[0], -1, 1)) * 0.225 + 0.175
                wz = float(np.clip(act[1], -1, 1)) * 0.8

            step += 1
            viewer.cam.lookat[:] = data.body("pelvis").xpos
            viewer.sync()

            # Maintain real-time simulation speed (CTRL_DT = 0.02s = 50Hz)
            elapsed = time.time() - step_start
            if CTRL_DT > elapsed:
                time.sleep(CTRL_DT - elapsed)


if __name__ == "__main__":
    main()
