"""Train the 1D-CNN perception network.

Input:  scan[360] (with Gaussian noise augmentation at train time)
Output: 4-dim channel clear distances normalized by MAX_RANGE:
  [nearest_dist, front_clear, left_clear, right_clear] / MAX_RANGE
The network acts as a denoiser / feature extractor. Bearing is computed
downstream directly via argmin(scan) (no point learning a noisy regression).
Export:  weights/perception_cnn.pt  (TorchScript)
"""
from __future__ import annotations
import sys, time, argparse
from pathlib import Path
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

PKG_ROOT = Path(__file__).resolve().parents[1]
MAX_RANGE = 8.0


class PerceptionCNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv1d(1, 16, kernel_size=5, padding=2), nn.ReLU(),
            nn.Conv1d(16, 32, kernel_size=5, padding=2), nn.ReLU(),
            nn.Conv1d(32, 64, kernel_size=5, padding=2), nn.ReLU(),
        )
        self.pool = nn.AdaptiveAvgPool1d(1)
        self.head = nn.Sequential(
            nn.Linear(64, 64), nn.ReLU(),
            nn.Linear(64, 4),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = x.unsqueeze(1)             # (B, 1, 360)
        f = self.features(x)           # (B, 64, 360)
        f = self.pool(f).squeeze(-1)   # (B, 64)
        return self.head(f)            # (B, 4) in [0,1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(PKG_ROOT / "data" / "scan_dataset.npz"))
    ap.add_argument("--out", default=r"D:\g1_run\perception_cnn.pt")
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--batch", type=int, default=256)
    ap.add_argument("--noise", type=float, default=0.10)  # 10cm noise augmentation
    args = ap.parse_args()

    d = np.load(args.data)
    scans = d["scans"].astype(np.float32)
    labels = d["labels"].astype(np.float32)
    # use cols 0,2,3,4 = nearest_dist, front, left, right (drop bearing col 1)
    y = np.stack([labels[:, 0], labels[:, 2], labels[:, 3], labels[:, 4]], axis=1) / MAX_RANGE

    n = scans.shape[0]
    n_train = int(n * 0.8)
    perm = np.random.default_rng(0).permutation(n)
    train_idx, val_idx = perm[:n_train], perm[n_train:]

    train_x = torch.from_numpy(scans[train_idx])
    train_y = torch.from_numpy(y[train_idx])
    val_x = torch.from_numpy(scans[val_idx])
    val_y = torch.from_numpy(y[val_idx])

    dl = DataLoader(TensorDataset(train_x, train_y), batch_size=args.batch, shuffle=True)

    model = PerceptionCNN()
    opt = torch.optim.Adam(model.parameters(), lr=1e-3)
    loss_fn = nn.MSELoss()

    t0 = time.time()
    for ep in range(args.epochs):
        model.train()
        tot = 0.0
        for xb, yb in dl:
            xb_noisy = (xb + torch.randn_like(xb) * args.noise).clamp(0.0, MAX_RANGE)
            pred = model(xb_noisy)
            loss = loss_fn(pred, yb)
            opt.zero_grad(); loss.backward(); opt.step()
            tot += loss.item() * xb.shape[0]
        train_loss = tot / len(dl.dataset)

        model.eval()
        with torch.no_grad():
            vpred = model(val_x)
            val_loss = loss_fn(vpred, val_y).item()
            mae_m = (vpred - val_y).abs().mean(dim=0).numpy() * MAX_RANGE
        if (ep + 1) % 5 == 0 or ep == 0:
            print(f"ep {ep+1}/{args.epochs}  train_mse={train_loss:.5f}  val_mse={val_loss:.5f}  "
                  f"MAE_m[nearest={mae_m[0]:.3f} front={mae_m[1]:.3f} left={mae_m[2]:.3f} right={mae_m[3]:.3f}]")

    model.eval()
    example = torch.randn(1, 360)
    ts = torch.jit.trace(model, example)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    ts.save(str(out))
    print(f"\nExported TorchScript -> {out}  ({out.stat().st_size//1024} KB)")
    print(f"wall time: {time.time()-t0:.1f}s")


if __name__ == "__main__":
    main()
