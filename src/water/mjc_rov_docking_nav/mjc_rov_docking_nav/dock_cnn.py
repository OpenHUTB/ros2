"""
dock_cnn.py — CNN 检测声呐剖面中是否有对接标记。
输入：1×32 的距离剖面，输出：二分类（有标记/无标记）。
"""
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset
from . import common


class DockCNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv1d(1, 16, 3, padding=1), nn.ReLU(), nn.MaxPool1d(2),
            nn.Conv1d(16, 32, 3, padding=1), nn.ReLU(), nn.MaxPool1d(2),
            nn.Flatten(),
            nn.Linear(32*8, 64), nn.ReLU(),
            nn.Linear(64, 2),
        )

    def forward(self, x):
        return self.net(x)


def _make_dataset(n=400):
    rng = np.random.RandomState(42)
    X, y = [], []
    for _ in range(n):
        # 随机 ROV 位置
        rov = np.array([rng.uniform(-2, 2), rng.uniform(-1, 1)])
        dock = np.array([3.0, 0.0])
        obs = [[1.5, 1.5], [1.5, -1.5]]
        has_dock = rng.rand() > 0.5
        if not has_dock:
            dock = np.array([99.0, 99.0])  # 假装对接站在远处
        scan = common.sonar_scan(rov, dock, obs)
        X.append(scan)
        y.append(1 if has_dock else 0)
    X = torch.tensor(np.array(X), dtype=torch.float32).unsqueeze(1)
    y = torch.tensor(np.array(y), dtype=torch.long)
    return TensorDataset(X, y)


def train_dock_cnn(epochs=20):
    ds = _make_dataset(400)
    loader = DataLoader(ds, batch_size=32, shuffle=True)
    net = DockCNN()
    opt = torch.optim.Adam(net.parameters(), lr=1e-3)
    lossfn = nn.CrossEntropyLoss()
    for ep in range(epochs):
        total = 0
        for xb, yb in loader:
            opt.zero_grad()
            out = net(xb)
            loss = lossfn(out, yb)
            loss.backward(); opt.step()
            total += loss.item()
        if ep % 5 == 0 or ep == epochs-1:
            print(f"[train] ep {ep:2d}  loss={total/len(loader):.4f}")
    wdir = common.get_package_root() / "weights"
    wdir.mkdir(exist_ok=True)
    torch.save(net.state_dict(), common.find_weights_path())
    print(f"[train] 权重保存到 {common.find_weights_path()}")


def test_dock_cnn():
    wpath = common.find_weights_path()
    if not wpath.exists():
        raise FileNotFoundError(f"找不到权重 {wpath}，请先运行: python3 main.py train")
    net = DockCNN()
    net.load_state_dict(torch.load(wpath, map_location="cpu"))
    net.eval()
    ds = _make_dataset(200)
    correct = 0
    with torch.no_grad():
        for xb, yb in DataLoader(ds, batch_size=64):
            pred = net(xb).argmax(dim=1)
            correct += (pred == yb).sum().item()
    acc = correct / len(ds)
    print(f"[test] 对接标记检测准确率: {acc:.3f} ({correct}/{len(ds)})")
    return acc
