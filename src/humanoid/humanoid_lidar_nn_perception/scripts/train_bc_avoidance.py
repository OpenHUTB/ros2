"""Behavior cloning for avoidance controller.

Uses the reactive rule (slow/turn when front blocked) as expert, collects
(obs -> vx,wz) pairs, trains a tiny MLP. This guarantees convergence and
gives a real NN controller (not if-else) for /cmd_vel_safe.
"""
from __future__ import annotations
import sys, time, argparse
from pathlib import Path
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))

from humanoid_lidar_nn_perception.g1_walking_policy import G1WalkingPolicy, N_SUBSTEPS
from humanoid_lidar_nn_perception.lidar_2d import LiDAR2D
import mujoco

MAX_RANGE = 8.0


def rule_expert(front: float, left: float, right: float, target_wz: float):
    """Reactive policy: if front blocked, steer toward clearer side; else go straight."""
    if front < 1.5:
        vx = 0.0
        wz = 0.8 if left > right else -0.8
    else:
        vx = 0.3
        wz = target_wz * 0.5  # gently follow target heading
    return vx, wz


class AvoidMLP(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(6, 32), nn.ReLU(),
            nn.Linear(32, 32), nn.ReLU(),
            nn.Linear(32, 2), nn.Tanh(),
        )
    def forward(self, x):
        return self.net(x)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=r"D:\g1_run\scene_g1.xml")
    ap.add_argument("--onnx", default=r"D:\g1_run\g1_policy.onnx")
    ap.add_argument("--cnn", default=r"D:\g1_run\perception_cnn.pt")
    ap.add_argument("--out", default=r"D:\g1_run\avoid_mlp.pt")
    ap.add_argument("--samples", type=int, default=10000)
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(args.model)
    policy = G1WalkingPolicy(model, args.onnx)
    lidar = LiDAR2D(model, num_beams=360, max_range=MAX_RANGE)
    cnn = torch.jit.load(args.cnn); cnn.eval()
    rng = np.random.default_rng(0)

    # collect (obs, expert_action) pairs
    obs_list, act_list = [], []
    frames = 0
    while frames < args.samples:
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, 0)
        x, y = rng.uniform(-4, 4), rng.uniform(-4, 4)
        yaw = rng.uniform(-np.pi, np.pi)
        data.qpos[0], data.qpos[1] = x, y
        data.qpos[3:7] = [np.cos(yaw/2), 0, 0, np.sin(yaw/2)]
        target_wz = rng.uniform(-0.5, 0.5)
        mujoco.mj_forward(model, data)

        for step in range(500):
            scan = lidar.scan(data)
            with torch.no_grad():
                pred = cnn(torch.from_numpy(scan).unsqueeze(0))[0].numpy()
            front = float(pred[1]) * MAX_RANGE
            left = float(pred[2]) * MAX_RANGE
            right = float(pred[3]) * MAX_RANGE
            obs = np.array([pred[0], pred[1], pred[2], pred[3],
                            np.cos(target_wz), np.sin(target_wz)], dtype=np.float32)
            vx, wz = rule_expert(front, left, right, target_wz)
            # normalize action to [-1,1]
            act = np.array([(vx - 0.175) / 0.225, wz / 0.8], dtype=np.float32)
            obs_list.append(obs); act_list.append(act)
            frames += 1

            policy.set_command(vx, 0, wz)
            for _ in range(N_SUBSTEPS):
                mujoco.mj_step(model, data)
            policy.step(data)
            if policy.fell(data):
                break

    obs = torch.from_numpy(np.stack(obs_list))
    acts = torch.from_numpy(np.stack(act_list))
    print(f"collected {obs.shape} obs, {acts.shape} acts")

    mlp = AvoidMLP()
    opt = torch.optim.Adam(mlp.parameters(), lr=1e-3)
    loss_fn = nn.MSELoss()
    dl = DataLoader(TensorDataset(obs, acts), batch_size=256, shuffle=True)
    for ep in range(30):
        tot = 0
        for xb, yb in dl:
            pred = mlp(xb)
            loss = loss_fn(pred, yb)
            opt.zero_grad(); loss.backward(); opt.step()
            tot += loss.item() * xb.shape[0]
        if (ep+1) % 10 == 0:
            print(f"ep {ep+1} mse={tot/len(dl.dataset):.5f}")

    ts = torch.jit.trace(mlp, torch.randn(1, 6))
    ts.save(args.out)
    print(f"saved -> {args.out}")


if __name__ == "__main__":
    main()
