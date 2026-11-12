"""GRU imputer: masked telemetry window -> full state window.

PyTorch 是可选依赖：没有 torch 时本模块仍可被导入（便于在无 torch 的机器上
跑 NumPy 后端与测试发现），只是无法构造 ``Imputer``。
"""
try:
    import torch
    from torch import nn
    TORCH_AVAILABLE = True
except ImportError:      # 无 torch：保留 numpy 后端与测试发现能力
    torch = None
    nn = None
    TORCH_AVAILABLE = False

IN_DIM = 9     # 6 state channels (masked to 0) + thrust + mask + dt
OUT_DIM = 6


if TORCH_AVAILABLE:
    class Imputer(nn.Module):
        def __init__(self, hidden=48):
            super().__init__()
            self.gru = nn.GRU(IN_DIM, hidden, batch_first=True)
            self.head = nn.Linear(hidden, OUT_DIM)

        def forward(self, x):
            h, _ = self.gru(x)
            return self.head(h)
else:
    class Imputer:  # pragma: no cover - 仅无 torch 时生效
        def __init__(self, *args, **kwargs):
            raise RuntimeError('PyTorch is required to build the GRU imputer')
