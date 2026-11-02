"""神经网络模型库。

为任务2（深度避障）、任务4（端到端图像→控制）与对比实验（同 RL 算法不同
模型结构）提供统一的策略/网络定义。``torch`` 惰性导入，开发机可语法校验。

模型结构（输出为 3 维连续速度指令 vx, vy, vz ∈ [-1, 1]）：
- ``MLPPolicy``        ：状态（深度向量/位姿误差）→ 控制，纯 MLP；
- ``CNNPolicy``        ：单帧深度/RGB 图像 → 控制，CNN；
- ``CNNLSTMPolicy``    ：多帧图像序列 → 控制，CNN + 单层 LSTM，适合时序；
- ``DepthAvoidNet``    ：任务2 专用，ROI 深度块 → 避障加速度修正。
"""

try:  # pragma: no cover
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
    _HAS_TORCH = True
except ImportError:  # pragma: no cover
    torch = None
    nn = object  # type: ignore
    _HAS_TORCH = False


if _HAS_TORCH:

    class MLPPolicy(nn.Module):
        """状态向量（位姿/速度误差）→ 3 维速度指令。"""

        def __init__(self, state_dim=6, hidden=(128, 64), act_dim=3):
            super().__init__()
            layers, last = [], state_dim
            for h in hidden:
                layers += [nn.Linear(last, h), nn.ReLU(inplace=True)]
                last = h
            layers += [nn.Linear(last, act_dim), nn.Tanh()]
            self.net = nn.Sequential(*layers)

        def forward(self, state):
            return self.net(state)

    class _Backbone(nn.Module):
        """共享 CNN 主干：1 或 3 通道图像 → 128 维特征。"""

        def __init__(self, in_ch=1):
            super().__init__()
            self.conv = nn.Sequential(
                nn.Conv2d(in_ch, 32, 5, stride=2, padding=2), nn.ReLU(inplace=True),
                nn.Conv2d(32, 64, 3, stride=2, padding=1), nn.ReLU(inplace=True),
                nn.Conv2d(64, 128, 3, stride=2, padding=1), nn.ReLU(inplace=True),
                nn.AdaptiveAvgPool2d((4, 4)),
            )
            self.fc = nn.Linear(128 * 4 * 4, 128)

        def forward(self, x):
            if x.dim() == 3:
                x = x.unsqueeze(1)  # B,H,W -> B,1,H,W
            h = self.conv(x)
            return F.relu(self.fc(h.flatten(1)))

    class CNNPolicy(nn.Module):
        """单帧图像 → 3 维速度指令。"""

        def __init__(self, in_ch=1, act_dim=3):
            super().__init__()
            self.backbone = _Backbone(in_ch)
            self.head = nn.Sequential(nn.Linear(128, 64), nn.ReLU(),
                                      nn.Linear(64, act_dim), nn.Tanh())

        def forward(self, x):
            return self.head(self.backbone(x))

    class CNNLSTMPolicy(nn.Module):
        """(B, T, H, W) 图像序列 → 3 维速度指令。"""

        def __init__(self, in_ch=1, hidden=128, act_dim=3):
            super().__init__()
            self.backbone = _Backbone(in_ch)
            self.lstm = nn.LSTM(128, hidden, batch_first=True)
            self.head = nn.Sequential(nn.Linear(hidden, 64), nn.ReLU(),
                                      nn.Linear(64, act_dim), nn.Tanh())

        def forward(self, x):
            # x: (B,T,C,H,W) 或 (B,T,H,W)
            if x.dim() == 5:
                b, t, c, h, w = x.shape
                x = x.view(b * t, c, h, w)
            elif x.dim() == 4:
                b, t, h, w = x.shape
                x = x.view(b * t, 1, h, w)
            feat = self.backbone(x).view(-1, t, 128)
            out, _ = self.lstm(feat)
            return self.head(out[:, -1, :])

    class DepthAvoidNet(nn.Module):
        """任务2 避障网络：前向深度向量（近距占空比）→ 横向/纵向加速度修正。"""

        def __init__(self, bins=16):
            super().__init__()
            self.net = nn.Sequential(
                nn.Linear(bins, 64), nn.ReLU(),
                nn.Linear(64, 32), nn.ReLU(),
                nn.Linear(32, 2), nn.Tanh(),   # (横向, 纵向) ∈ [-1,1]
            )

        def forward(self, depth_bins):
            return self.net(depth_bins)

else:  # 无 torch 时的占位，保证 import 不报错

    class MLPPolicy:  # type: ignore
        def __init__(self, *a, **k):
            raise RuntimeError("未安装 PyTorch，请在运行机执行 pip install torch")

    CNNPolicy = MLPPolicy            # type: ignore
    CNNLSTMPolicy = MLPPolicy         # type: ignore
    DepthAvoidNet = MLPPolicy        # type: ignore
