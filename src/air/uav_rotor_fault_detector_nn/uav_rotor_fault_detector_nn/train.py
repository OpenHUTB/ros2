"""训练入口：从仿真数据集训练健康度估计网络。

在 CPU 上跑就够了——网络不到 4 万参数、窗口 100 帧、数据几千条，
一次完整训练在一台普通笔记本上两三分钟以内。不需要显卡，也不需要 ROS。
"""

from __future__ import annotations

import json
import random
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch import nn

from .data import load_dataset, split_by_episode
from .features import FEATURE_NAMES, ROTOR_COUNT
from .metrics import FAULT_GAIN_THRESHOLD, mean_absolute_error
from .model import build_model, save_checkpoint


@dataclass
class TrainConfig:
    """训练超参。默认值是在 CPU 上试出来的，不追求最优，只求稳。"""

    epochs: int = 30
    batch_size: int = 128
    lr: float = 2e-3
    weight_decay: float = 1e-4
    patience: int = 6
    seed: int = 0
    #: 含故障桨的样本在损失里的额外权重（健康样本远多于故障样本）
    fault_weight: float = 4.0
    #: CPU 训练线程数，None 表示交给 PyTorch 自己决定
    threads: int | None = None
    #: 是否按 episode 切分训练/验证/测试
    split: tuple[float, float, float] = (0.8, 0.1, 0.1)


def set_seeds(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)


def _tensor(array: np.ndarray) -> torch.Tensor:
    return torch.from_numpy(np.ascontiguousarray(array)).float()


def _sample_weights(targets: torch.Tensor, fault_weight: float) -> torch.Tensor:
    """给含故障的样本加权：``min(g)<0.9`` 的样本权重为 ``fault_weight``。"""
    faulty = (targets.min(dim=1).values < FAULT_GAIN_THRESHOLD).float()
    return 1.0 + (fault_weight - 1.0) * faulty


def _predict(model: nn.Module, features: torch.Tensor, batch_size: int) -> np.ndarray:
    model.eval()
    outputs = []
    with torch.no_grad():
        for start in range(0, features.shape[0], batch_size):
            chunk = features[start : start + batch_size]
            outputs.append(model(chunk).cpu().numpy())
    return np.concatenate(outputs, axis=0) if outputs else np.zeros((0, ROTOR_COUNT))


def evaluate_split(model: nn.Module, dataset: dict, mask: np.ndarray,
                   batch_size: int = 256) -> np.ndarray:
    """在指定子集上推理，返回 ``(n, 4)`` 的预测。"""
    if not np.any(mask):
        return np.zeros((0, ROTOR_COUNT), dtype=np.float64)
    return _predict(model, _tensor(dataset["X"][mask]), batch_size)


def train_detector(
    dataset_path: str | Path,
    out_path: str | Path,
    cfg: TrainConfig | None = None,
    verbose: bool = True,
) -> dict:
    """训练并保存检查点，返回包含各项指标的字典。"""
    cfg = cfg or TrainConfig()
    set_seeds(cfg.seed)
    if cfg.threads:
        torch.set_num_threads(int(cfg.threads))

    dataset = load_dataset(dataset_path)
    masks = split_by_episode(dataset, cfg.split)

    X = _tensor(dataset["X"])
    Y = _tensor(dataset["Y"])
    masks_t = {key: torch.from_numpy(value) for key, value in masks.items()}

    n_train = int(masks["train"].sum())
    n_val = int(masks["val"].sum())
    n_test = int(masks["test"].sum())
    if n_train == 0 or n_val == 0:
        raise RuntimeError(
            f"数据太少无法训练：train={n_train}, val={n_val}, test={n_test}；"
            "请加大 n_episodes 或调整 split"
        )

    model = build_model()
    n_params = model.n_parameters()
    optimizer = torch.optim.AdamW(model.parameters(), lr=cfg.lr, weight_decay=cfg.weight_decay)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode="min", factor=0.5, patience=2
    )

    train_idx = torch.nonzero(masks_t["train"]).squeeze(1)
    val_idx = torch.nonzero(masks_t["val"]).squeeze(1)

    best_val = float("inf")
    best_epoch = -1
    best_state = None
    history: list[dict] = []
    started = time.time()

    for epoch in range(1, cfg.epochs + 1):
        model.train()
        order = train_idx[torch.randperm(train_idx.numel())]
        total_loss = 0.0
        n_batches = 0

        for start in range(0, order.numel(), cfg.batch_size):
            index = order[start : start + cfg.batch_size]
            features = X[index]
            targets = Y[index]
            weights = _sample_weights(targets, cfg.fault_weight).unsqueeze(1)

            preds = model(features)
            loss = (nn.functional.mse_loss(preds, targets, reduction="none") * weights).mean()

            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), max_norm=5.0)
            optimizer.step()

            total_loss += float(loss.detach())
            n_batches += 1

        val_pred = _predict(model, X[val_idx], 256)
        val_target = Y[val_idx].numpy()
        val_mae = mean_absolute_error(val_pred, val_target)
        scheduler.step(val_mae)

        row = {
            "epoch": epoch,
            "train_loss": total_loss / max(n_batches, 1),
            "val_mae": val_mae,
            "lr": float(optimizer.param_groups[0]["lr"]),
        }
        history.append(row)
        if verbose:
            print(f"epoch {epoch:3d} | loss {row['train_loss']:.5f} | "
                  f"val MAE {val_mae:.4f} | lr {row['lr']:.2e}", flush=True)

        if val_mae < best_val - 1e-5:
            best_val = val_mae
            best_epoch = epoch
            best_state = {k: v.detach().clone() for k, v in model.state_dict().items()}
        elif epoch - best_epoch >= cfg.patience:
            if verbose:
                print(f"验证集 {cfg.patience} 轮没有改善，提前停止")
            break

    if best_state is not None:
        model.load_state_dict(best_state)

    test_pred = evaluate_split(model, dataset, masks["test"])
    test_target = dataset["Y"][masks["test"]]
    test_mae = mean_absolute_error(test_pred, test_target) if len(test_pred) else float("nan")

    elapsed = time.time() - started
    meta = {
        "feature_names": list(FEATURE_NAMES),
        "window": int(dataset["window"]),
        "stride": int(dataset["stride"]),
        "dt": float(dataset["dt"]),
        "n_parameters": int(n_params),
        "best_epoch": int(best_epoch),
        "best_val_mae": float(best_val),
        "test_mae": float(test_mae),
        "n_train": n_train,
        "n_val": n_val,
        "n_test": n_test,
        "train_seconds": round(elapsed, 1),
        "config": {k: (list(v) if isinstance(v, tuple) else v) for k, v in asdict(cfg).items()},
        "history": history,
        "torch": torch.__version__,
    }
    save_checkpoint(out_path, model, meta)

    if verbose:
        print(f"\n训练完成：{n_params} 参数，用时 {elapsed:.1f}s")
        print(f"最佳 epoch {best_epoch}，验证 MAE {best_val:.4f}，测试 MAE {test_mae:.4f}")
        print(f"检查点已保存：{Path(out_path)}")

    return meta


def main(argv: list[str] | None = None) -> int:
    """``python -m ... `` 之外的简易入口，供 CLI 子命令复用。"""
    import argparse

    parser = argparse.ArgumentParser(description="训练四旋翼桨故障检测网络")
    parser.add_argument("--data", default="results/detector_dataset.npz")
    parser.add_argument("--out", default="results/detector.pt")
    parser.add_argument("--epochs", type=int, default=TrainConfig.epochs)
    parser.add_argument("--batch-size", type=int, default=TrainConfig.batch_size)
    parser.add_argument("--lr", type=float, default=TrainConfig.lr)
    parser.add_argument("--patience", type=int, default=TrainConfig.patience)
    parser.add_argument("--seed", type=int, default=TrainConfig.seed)
    parser.add_argument("--threads", type=int, default=None)
    args, _ = parser.parse_known_args(argv)

    cfg = TrainConfig(
        epochs=args.epochs,
        batch_size=args.batch_size,
        lr=args.lr,
        patience=args.patience,
        seed=args.seed,
        threads=args.threads,
    )
    meta = train_detector(args.data, args.out, cfg)
    print(json.dumps({k: meta[k] for k in ("best_epoch", "best_val_mae", "test_mae")},
                     ensure_ascii=False))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
