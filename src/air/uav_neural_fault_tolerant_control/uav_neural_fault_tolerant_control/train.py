"""训练神经残差分配器：监督回归，标签是精确箱约束解与近似解的差。

数据是离线生成的（见 ``data.py``），训练时不需要仿真器，纯 CPU 几十秒。
"""

from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

from . import features as F
from . import model as M


@dataclass
class TrainConfig:
    epochs: int = 40
    batch_size: int = 256
    lr: float = 1e-3
    weight_decay: float = 1e-4
    patience: int = 8
    seed: int = 0
    threads: int | None = None
    split: tuple = (0.8, 0.1, 0.1)
    hidden: tuple = M.DEFAULT_HIDDEN


def set_seeds(seed: int) -> None:
    torch = M._torch()
    np.random.seed(int(seed))
    torch.manual_seed(int(seed))


def _split(dataset: dict, cfg: TrainConfig):
    from . import data as D
    masks = D.split_by_kind(dataset, cfg.split, seed=cfg.seed)
    x = dataset["X"]
    y = dataset["Y"]
    return {name: (x[m], y[m]) for name, m in masks.items()}


def train_residual(dataset_path, out_path, cfg: TrainConfig | None = None,
                   verbose: bool = True) -> dict:
    """训练并保存检查点，返回写进元信息的那份字典。"""
    from . import data as D

    cfg = cfg or TrainConfig()
    torch = M._torch()
    if cfg.threads:
        torch.set_num_threads(int(cfg.threads))
    set_seeds(cfg.seed)

    dataset = D.load_dataset(dataset_path)
    parts = _split(dataset, cfg)
    x_train, y_train = parts["train"]
    x_val, y_val = parts["val"]

    model = M.build_model(hidden=tuple(cfg.hidden))
    net_params = model.n_parameters()

    optimiser = torch.optim.AdamW(model.parameters(), lr=cfg.lr,
                                  weight_decay=cfg.weight_decay)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimiser, mode="min", factor=0.5, patience=max(1, cfg.patience // 3))

    x_t = torch.from_numpy(np.ascontiguousarray(x_train))
    y_t = torch.from_numpy(np.ascontiguousarray(y_train))
    x_v = torch.from_numpy(np.ascontiguousarray(x_val))
    y_v = torch.from_numpy(np.ascontiguousarray(y_val))

    batch = int(cfg.batch_size)
    n = int(x_t.shape[0])
    history: list[dict] = []
    best_mae, best_epoch, best_state = float("inf"), -1, None
    started = time.time()

    for epoch in range(1, int(cfg.epochs) + 1):
        model.train()
        order = torch.randperm(n)
        total = 0.0
        for start in range(0, n, batch):
            idx = order[start:start + batch]
            pred = model(x_t[idx])
            loss = torch.mean((pred - y_t[idx]) ** 2)
            optimiser.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
            optimiser.step()
            total += float(loss.detach()) * int(idx.numel())

        model.eval()
        with torch.no_grad():
            val_pred = model(x_v)
            val_mae = float(torch.mean(torch.abs(val_pred - y_v)))
        train_loss = total / max(n, 1)
        scheduler.step(val_mae)
        history.append({"epoch": epoch, "train_loss": train_loss,
                        "val_mae": val_mae})

        if val_mae < best_mae - 1e-6:
            best_mae = val_mae
            best_epoch = epoch
            best_state = {k: v.detach().clone() for k, v in model.state_dict().items()}
        if verbose:
            print(f"epoch {epoch:3d}  train_loss {train_loss:.6f}  "
                  f"val_mae {val_mae:.6f}")
        if epoch - best_epoch >= int(cfg.patience):
            if verbose:
                print(f"验证集连续 {cfg.patience} 轮没有改善，提前停止")
            break

    if best_state is not None:
        model.load_state_dict(best_state)
    model.eval()

    # 测试集：既报残差误差，也报"还原成指令后的误差"（乘回单桨最大推力）
    x_test, y_test = parts["test"]
    with torch.no_grad():
        test_pred = model(torch.from_numpy(np.ascontiguousarray(x_test)))
        test_mae = float(torch.mean(torch.abs(test_pred -
                                              torch.from_numpy(np.ascontiguousarray(y_test)))))

    meta = {
        "feature_names": list(F.FEATURE_NAMES),
        "n_parameters": net_params,
        "n_samples": int(dataset["X"].shape[0]),
        "n_train": int(x_train.shape[0]),
        "n_val": int(x_val.shape[0]),
        "n_test": int(x_test.shape[0]),
        "best_epoch": best_epoch,
        "best_val_mae": best_mae,
        "test_mae": test_mae,
        "train_seconds": round(time.time() - started, 2),
        "config": {**asdict(cfg), "hidden": list(cfg.hidden)},
        "history": history,
        "torch": torch.__version__,
    }
    M.save_checkpoint(out_path, model, meta)
    return meta


def main(argv=None) -> int:
    import argparse

    from . import data as D

    parser = argparse.ArgumentParser(description="训练神经残差分配器")
    parser.add_argument("--dataset", default="results/allocator_dataset.npz")
    parser.add_argument("--out", default="results/allocator.pt")
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--patience", type=int, default=8)
    parser.add_argument("--threads", type=int, default=0)
    args = parser.parse_args(argv)

    cfg = TrainConfig(epochs=args.epochs, batch_size=args.batch_size,
                      lr=args.lr, patience=args.patience,
                      threads=args.threads or None)
    dataset = D.load_dataset(args.dataset)
    print(f"数据集 {args.dataset}：{dataset['X'].shape[0]} 个样本，"
          f"{dataset['episodes']} 次飞行（其中 {dataset['crashed_episodes']} 次坠机）")
    meta = train_residual(args.dataset, args.out, cfg)
    print(json.dumps({k: meta[k] for k in ("n_parameters", "best_epoch",
                                           "best_val_mae", "test_mae",
                                           "train_seconds")},
                     ensure_ascii=False, indent=2))
    print(f"检查点已保存：{args.out}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
