"""训练两个网络：可降落性评分（CNN，蒸馏规则老师）与制导（MLP，克隆专家律）。

两者都是纯监督回归，CPU 上几十秒到几分钟量级，不依赖 MuJoCo（数据集已经是 npz）。
"""

from __future__ import annotations

import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np

from . import data as D
from . import model as Md


@dataclass
class TrainConfig:
    epochs: int = 30
    batch_size: int = 32
    lr: float = 1e-3
    weight_decay: float = 1e-4
    patience: int = 8
    seed: int = 0
    threads: Optional[int] = None
    split: tuple = (0.8, 0.1, 0.1)


def _torch():
    return Md._torch()


def _set_seeds(seed: int) -> None:
    torch = _torch()
    np.random.seed(seed)
    torch.manual_seed(seed)


def _train_loop(module, X, Y, cfg: TrainConfig, loss_name: str, verbose: bool,
                forward) -> Dict[str, Any]:
    """通用的监督训练循环（评分与制导共用）。"""
    torch = _torch()
    nn = torch.nn
    split = D.split_samples(len(X), fractions=cfg.split, seed=cfg.seed)
    Xt, Yt = torch.as_tensor(X, dtype=torch.float32), torch.as_tensor(Y, dtype=torch.float32)
    opt = torch.optim.AdamW(module.parameters(), lr=cfg.lr, weight_decay=cfg.weight_decay)
    sched = torch.optim.lr_scheduler.ReduceLROnPlateau(
        opt, mode="min", factor=0.5, patience=max(1, cfg.patience // 3))
    loss_fn = nn.MSELoss()

    best = {"val": float("inf"), "epoch": -1, "state": None}
    history = []
    t0 = time.time()
    for epoch in range(cfg.epochs):
        module.train()
        order = np.random.permutation(split["train"])
        total, count = 0.0, 0
        for start in range(0, len(order), cfg.batch_size):
            idx = order[start:start + cfg.batch_size]
            pred = forward(module, Xt[idx])
            loss = loss_fn(pred, Yt[idx])
            opt.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(module.parameters(), 5.0)
            opt.step()
            total += float(loss.item()) * len(idx)
            count += len(idx)
        train_loss = total / max(count, 1)

        module.eval()
        with torch.no_grad():
            val_pred = forward(module, Xt[split["val"]])
            val_mae = float(torch.mean(torch.abs(val_pred - Yt[split["val"]])).item())
        sched.step(val_mae)
        history.append({"epoch": epoch + 1, "train_loss": round(train_loss, 6),
                        "val_mae": round(val_mae, 6)})
        if verbose:
            print(f"  epoch {epoch + 1:>3d}  loss {train_loss:.5f}  val_mae {val_mae:.5f}")
        if val_mae < best["val"] - 1e-6:
            best = {"val": val_mae, "epoch": epoch + 1,
                    "state": {k: v.detach().clone() for k, v in module.state_dict().items()}}
        elif epoch + 1 - best["epoch"] >= cfg.patience:
            if verbose:
                print(f"  早停：{cfg.patience} 个 epoch 没有改善")
            break

    if best["state"] is not None:
        module.load_state_dict(best["state"])
    module.eval()
    with torch.no_grad():
        test_pred = forward(module, Xt[split["test"]])
        test_mae = float(torch.mean(torch.abs(test_pred - Yt[split["test"]])).item())
    return {
        "loss": loss_name,
        "n_parameters": Md.n_parameters(module),
        "n_samples": int(len(X)),
        "n_train": int(len(split["train"])),
        "n_val": int(len(split["val"])),
        "n_test": int(len(split["test"])),
        "best_epoch": best["epoch"],
        "best_val_mae": round(best["val"], 6),
        "test_mae": round(test_mae, 6),
        "train_seconds": round(time.time() - t0, 2),
        "history": history,
        "config": asdict(cfg),
        "torch": _torch().__version__,
    }


def _forward_score(module, x):
    """评分网络：输入 (N,1,R,C) → (N,1,R,C)。"""
    import torch

    return module(x)


def _forward_guidance(module, x):
    return module(x)


def train_score(dataset_path, out_path, cfg: Optional[TrainConfig] = None,
                verbose: bool = True) -> Dict[str, Any]:
    """蒸馏规则评分：输入高度图，输出逐格可降落性。"""
    torch = _torch()
    cfg = cfg or TrainConfig()
    _set_seeds(cfg.seed)
    if cfg.threads:
        torch.set_num_threads(int(cfg.threads))
    dataset = D.load_dataset(dataset_path)
    X, Y = dataset["X"], dataset["Y"]
    module = Md.build_score_module()
    if verbose:
        print(f"评分数据集：{len(X)} 张地图，网格 {X.shape[-2]}×{X.shape[-1]}")
    meta = _train_loop(module, X, Y, cfg, "mse", verbose, _forward_score)
    meta["kind"] = "score"
    meta["grid"] = list(X.shape[-2:])
    Md.save_checkpoint(out_path, "score", module, meta)
    return meta


def train_guidance(dataset_path, out_path, cfg: Optional[TrainConfig] = None,
                   verbose: bool = True) -> Dict[str, Any]:
    """克隆专家迫降律：输入 13 维状态特征，输出期望速度。"""
    torch = _torch()
    cfg = cfg or TrainConfig()
    _set_seeds(cfg.seed)
    if cfg.threads:
        torch.set_num_threads(int(cfg.threads))
    dataset = D.load_dataset(dataset_path)
    X, Y = dataset["X"], dataset["Y"]
    module = Md.build_guidance_module()
    if verbose:
        print(f"制导数据集：{len(X)} 个状态样本，特征 {X.shape[-1]} 维")
    meta = _train_loop(module, X, Y, cfg, "mse", verbose, _forward_guidance)
    meta["kind"] = "guidance"
    from . import landing as L

    meta["features"] = list(L.GUIDANCE_FEATURES)
    Md.save_checkpoint(out_path, "guidance", module, meta)
    return meta


def main(argv=None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="训练迫降评分/制导网络")
    parser.add_argument("--score-data", default="results/score_dataset.npz")
    parser.add_argument("--guidance-data", default="results/guidance_dataset.npz")
    parser.add_argument("--out-dir", default="results")
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--threads", type=int, default=None)
    parser.add_argument("--only", choices=("score", "guidance"), default=None)
    args = parser.parse_args(argv)

    cfg = TrainConfig(epochs=args.epochs, batch_size=args.batch_size, threads=args.threads)
    out_dir = Path(args.out_dir)
    if args.only in (None, "score"):
        meta = train_score(args.score_data, out_dir / "score.pt", cfg)
        print(f"评分网络：{meta['n_parameters']} 参数，验证 MAE {meta['best_val_mae']}，"
              f"测试 MAE {meta['test_mae']}，{meta['train_seconds']} s")
    if args.only in (None, "guidance"):
        meta = train_guidance(args.guidance_data, out_dir / "guidance.pt", cfg)
        print(f"制导网络：{meta['n_parameters']} 参数，验证 MAE {meta['best_val_mae']}，"
              f"测试 MAE {meta['test_mae']}，{meta['train_seconds']} s")
    return 0
