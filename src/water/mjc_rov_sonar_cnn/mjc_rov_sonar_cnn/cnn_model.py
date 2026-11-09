"""
cnn_model.py — 三类 CNN 分类器：礁石 / 管道 / 残骸。
"""
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader

from . import common


class SonarCNN(nn.Module):
    """
    3 层卷积 + 2 层全连接，输入 1x64x64，输出 3 类 logits。
    为什么这么小：CPU 虚拟机训练，参数量约 548k（主要来自 Linear(4096→128)），几秒钟收敛。
    """
    def __init__(self, num_classes=3):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(1, 16, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
            nn.Conv2d(16, 32, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
            nn.Conv2d(32, 64, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
        )
        self.classifier = nn.Sequential(
            nn.Linear(64 * 8 * 8, 128), nn.ReLU(),
            nn.Linear(128, num_classes),
        )

    def forward(self, x):
        x = self.features(x)
        x = x.view(x.size(0), -1)
        return self.classifier(x)


class SyntheticSonarDataset(Dataset):
    """
    合成数据集：随机生成 ROV 位置，对应一张声呐图 + 随机目标物标签。
    标签：0=礁石为主，1=管道为主，2=残骸为主。
    为什么用合成数据：真实声呐数据需要海试采集，课程环境不可能有；
    用 MuJoCo 场景几何生成合成图，CNN 学会"从声呐纹理分类目标物"这个任务。
    """
    def __init__(self, n_samples=600):
        self.samples = []
        rng = np.random.RandomState(42)
        for _ in range(n_samples):
            # 随机 ROV 位置
            rov_xy = np.array([rng.uniform(-4, 4), rng.uniform(-4, 4)])
            # 随机选一类作为主目标（该类亮斑最明显）
            label = rng.randint(0, 3)
            # 三类目标位置随机扰动
            rock = [[1.5+rng.uniform(-0.5,0.5), 1.5+rng.uniform(-0.5,0.5)],
                    [-2.0+rng.uniform(-0.5,0.5), 1.0+rng.uniform(-0.5,0.5)]]
            pipe = [[2.5+rng.uniform(-0.5,0.5), -1.5+rng.uniform(-0.5,0.5)]]
            wreck = [[0.5+rng.uniform(-0.5,0.5), -2.5+rng.uniform(-0.5,0.5)]]
            # 关键：按 label 调整各类亮斑强度，让 CNN 有东西可学
            # label=0 礁石主，label=1 管道主，label=2 残骸主
            rock_int = 0.8 if label == 0 else 0.15
            pipe_int = 0.9 if label == 1 else 0.15
            wreck_int = 0.6 if label == 2 else 0.12
            img = common.generate_sonar_image(
                rov_xy, rock, pipe, wreck, size=64,
                rock_intensity=rock_int, pipe_intensity=pipe_int, wreck_intensity=wreck_int,
            )
            self.samples.append((img, label))

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        img, label = self.samples[idx]
        img_t = torch.tensor(img, dtype=torch.float32).unsqueeze(0) / 255.0
        return img_t, label


def train_cnn(epochs: int = 30, lr: float = 1e-3):
    """训练 SonarCNN，返回 train loss 历史。"""
    ds = SyntheticSonarDataset(n_samples=600)
    loader = DataLoader(ds, batch_size=32, shuffle=True)
    net = SonarCNN()
    opt = torch.optim.Adam(net.parameters(), lr=lr)
    lossfn = nn.CrossEntropyLoss()
    history = []
    for ep in range(epochs):
        total = 0.0
        for imgs, labels in loader:
            opt.zero_grad()
            out = net(imgs)
            loss = lossfn(out, labels)
            loss.backward()
            opt.step()
            total += loss.item()
        history.append(total / len(loader))
        if ep % 10 == 0 or ep == epochs-1:
            print(f"[train] epoch {ep:3d}/{epochs}  loss={history[-1]:.4f}")
    # 保存权重
    wdir = common.get_package_root() / "weights"
    wdir.mkdir(exist_ok=True)
    wpath = common.find_weights_path()
    torch.save(net.state_dict(), wpath)
    print(f"[train] 权重保存到 {wpath}")
    return history


def test_cnn():
    """在合成测试集上打印准确率。"""
    wpath = common.find_weights_path()
    if not wpath.exists():
        raise FileNotFoundError(f"找不到权重 {wpath}，请先运行: python3 main.py train")
    net = SonarCNN()
    net.load_state_dict(torch.load(wpath, map_location="cpu"))
    net.eval()
    ds = SyntheticSonarDataset(n_samples=200)
    correct = 0
    with torch.no_grad():
        for img, label in ds:
            out = net(img.unsqueeze(0))
            pred = out.argmax(dim=1).item()
            if pred == label:
                correct += 1
    acc = correct / len(ds)
    print(f"[test] 合成测试集准确率: {acc:.3f} ({correct}/{len(ds)})")
    return acc
