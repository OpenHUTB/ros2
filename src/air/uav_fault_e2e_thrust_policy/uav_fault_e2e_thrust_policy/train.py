"""监督训练：把 DAgger 数据集里的"专家指令"当回归目标。

损失就是四个桨推力指令的均方误差——端到端策略输出的是"执行器层"的量，
不需要额外的策略梯度目标；PPO 类方法的对比留到后续（见文档"已知限制"）。
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from . import data as D
from . import policy as P


@dataclass
class TrainConfig:
    epochs: int = 60
    batch_size: int = 128
    lr: float = 1e-3
    weight_decay: float = 1e-4
    patience: int = 10
    seed: int = 0
    threads: int | None = None
    split: tuple = (0.8, 0.1, 0.1)


def _to_stack(dataset: dict, stack: int) -> np.ndarray:
    if stack <= 1:
        return np.asarray(dataset["depth"], dtype=np.float32)
    return D.stack_frames(dataset, stack)


def train_policy(dataset_path, out_path, arch: str = "cnn",
                 cfg: TrainConfig | None = None, verbose: bool = True) -> dict:
    torch = P._torch()
    cfg = cfg or TrainConfig()
    if cfg.threads:
        torch.set_num_threads(int(cfg.threads))
    torch.manual_seed(cfg.seed)
    rng = np.random.default_rng(cfg.seed)

    dataset = D.load_dataset(dataset_path)
    masks = D.split_by_episode(dataset, cfg.split, cfg.seed)
    stack = P.DEPTH_STACK if arch == "cnn_stack" else 1
    depth = _to_stack(dataset, stack)
    proprio = np.asarray(dataset["proprio"], dtype=np.float32)
    label = np.asarray(dataset["label"], dtype=np.float32)

    def tensors(mask):
        return (
            torch.as_tensor(depth[mask]),
            torch.as_tensor(proprio[mask]),
            torch.as_tensor(label[mask]),
        )

    d_tr, p_tr, y_tr = tensors(masks["train"])
    d_va, p_va, y_va = tensors(masks["val"])
    d_te, p_te, y_te = tensors(masks["test"])

    module = P.build_module(arch)
    n_par = P.n_parameters(module)
    opt = torch.optim.AdamW(module.parameters(), lr=cfg.lr,
                            weight_decay=cfg.weight_decay)
    sched = torch.optim.lr_scheduler.ReduceLROnPlateau(
        opt, mode="min", factor=0.5, patience=max(1, cfg.patience // 3))
    loss_fn = torch.nn.MSELoss()

    n = d_tr.shape[0]
    best = {"epoch": -1, "val_mae": float("inf"), "state": None}
    history = []
    t0 = time.perf_counter()
    for epoch in range(cfg.epochs):
        module.train()
        order = rng.permutation(n)
        total = 0.0
        for start in range(0, n, cfg.batch_size):
            idx = torch.as_tensor(order[start:start + cfg.batch_size])
            pred = module(d_tr[idx], p_tr[idx])
            loss = loss_fn(pred, y_tr[idx])
            opt.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(module.parameters(), 5.0)
            opt.step()
            total += float(loss.item()) * idx.numel()
        train_mse = total / max(n, 1)
        module.eval()
        with torch.no_grad():
            val_mae = float((module(d_va, p_va) - y_va).abs().mean().item())
        sched.step(val_mae)
        history.append({"epoch": epoch, "train_mse": train_mse, "val_mae": val_mae})
        if val_mae < best["val_mae"] - 1e-6:
            best = {"epoch": epoch, "val_mae": val_mae,
                    "state": {k: v.detach().clone()
                              for k, v in module.state_dict().items()}}
        if epoch - best["epoch"] >= cfg.patience:
            break

    if best["state"] is not None:
        module.load_state_dict(best["state"])
    module.eval()
    with torch.no_grad():
        test_mae = float((module(d_te, p_te) - y_te).abs().mean().item())
    seconds = time.perf_counter() - t0
    meta = {
        "arch": arch,
        "n_parameters": n_par,
        "n_samples": int(depth.shape[0]),
        "n_train": int(masks["train"].sum()),
        "n_val": int(masks["val"].sum()),
        "n_test": int(masks["test"].sum()),
        "best_epoch": int(best["epoch"]),
        "best_val_mae": float(best["val_mae"]),
        "test_mae": test_mae,
        "train_seconds": round(seconds, 3),
        "config": {
            "epochs": cfg.epochs, "batch_size": cfg.batch_size, "lr": cfg.lr,
            "weight_decay": cfg.weight_decay, "patience": cfg.patience,
            "split": list(cfg.split),
        },
        "history": history,
        "torch": torch.__version__,
    }
    P.save_checkpoint(out_path, module, meta)
    if verbose:
        print(f"  结构 {arch}：{n_par} 参数，最好 epoch {best['epoch']}，"
              f"验证 MAE {best['val_mae']:.4f}，测试 MAE {test_mae:.4f}，"
              f"{seconds:.1f} s")
    return meta
