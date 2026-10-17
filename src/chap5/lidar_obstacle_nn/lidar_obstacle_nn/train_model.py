"""Train the LidarMLP from a recorded .npz dataset.

Training loop:
    1. Load dataset saved by ``collect_data``.
    2. Standard 80 / 20 train / val split with class-balanced sampling.
    3. CrossEntropyLoss + Adam, early stopping on validation loss.
    4. Save best weights to ``models/lidar_mlp.pt`` along with a small
       metadata dict (input_size, accuracy, classes).

Usage:
    python -m lidar_obstacle_nn.train_model \
        --dataset /tmp/lidar_dataset.npz \
        --output  models/lidar_mlp.pt \
        --epochs  50
"""

from __future__ import annotations

import argparse
import json
import os
import time
from typing import Tuple

import numpy as np

try:
    import torch
    from torch import nn
    from torch.utils.data import DataLoader, TensorDataset
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        'PyTorch is required for training. Install with '
        "`pip install torch` (CUDA wheels recommended for RTX 5070 Ti)."
    ) from exc

from lidar_obstacle_nn.model import LidarMLP


def load_dataset(
    path: str,
) -> Tuple[np.ndarray, np.ndarray, int]:
    """Load a dataset saved by ``collect_data``.

    Returns:
        scans  : float32 array of shape (N, scan_size)
        labels : int64 array of shape (N,)
        scan_size: int
    """
    data = np.load(path, allow_pickle=True)
    scans = data['scans'].astype(np.float32)
    labels = data['labels'].astype(np.int64)
    scan_size = int(data['scan_size']) if 'scan_size' in data.files else scans.shape[1]
    return scans, labels, scan_size


def train(
    dataset_path: str,
    output_path: str,
    epochs: int = 50,
    batch_size: int = 64,
    lr: float = 1e-3,
    hidden_size: int = 128,
    patience: int = 10,
    device: str = 'auto',
) -> dict:
    """Train the model and return a metrics dict."""
    scans, labels, scan_size = load_dataset(dataset_path)

    if device == 'auto':
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
    device_t = torch.device(device)
    print('Using device: %s' % device)

    # ---- Train / val split -----------------------------------------
    n = scans.shape[0]
    perm = np.random.permutation(n)
    n_val = max(1, int(0.2 * n))
    val_idx = perm[:n_val]
    train_idx = perm[n_val:]

    x_train = torch.from_numpy(scans[train_idx]).to(device_t)
    y_train = torch.from_numpy(labels[train_idx]).to(device_t)
    x_val = torch.from_numpy(scans[val_idx]).to(device_t)
    y_val = torch.from_numpy(labels[val_idx]).to(device_t)

    # ---- Class-balanced sampler ------------------------------------
    class_counts = np.bincount(labels[train_idx], minlength=3).astype(np.float32)
    weights = (class_counts.sum() / (3 * np.maximum(class_counts, 1.0)))
    sample_weights = weights[labels[train_idx]]
    sampler = torch.utils.data.WeightedRandomSampler(
        torch.from_numpy(sample_weights).double(),
        num_samples=len(train_idx),
        replacement=True,
    )
    train_loader = DataLoader(
        TensorDataset(x_train, y_train),
        batch_size=batch_size,
        sampler=sampler,
    )

    # ---- Model / optimiser / loss ----------------------------------
    model = LidarMLP(
        input_size=scan_size,
        hidden_size=hidden_size,
    ).to(device_t)
    optim = torch.optim.Adam(model.parameters(), lr=lr)
    loss_fn = nn.CrossEntropyLoss()

    best_val_loss = float('inf')
    best_state = None
    bad_epochs = 0

    t0 = time.time()
    for epoch in range(1, epochs + 1):
        model.train()
        train_loss = 0.0
        train_correct = 0
        train_total = 0
        for xb, yb in train_loader:
            optim.zero_grad()
            logits = model(xb)
            loss = loss_fn(logits, yb)
            loss.backward()
            optim.step()
            train_loss += float(loss.item()) * xb.size(0)
            train_correct += int((logits.argmax(-1) == yb).sum().item())
            train_total += xb.size(0)

        model.eval()
        with torch.no_grad():
            val_logits = model(x_val)
            val_loss = float(loss_fn(val_logits, y_val).item())
            val_acc = float(
                (val_logits.argmax(-1) == y_val).float().mean().item()
            )

        if val_loss < best_val_loss - 1e-4:
            best_val_loss = val_loss
            best_state = {k: v.detach().clone() for k, v in model.state_dict().items()}
            bad_epochs = 0
        else:
            bad_epochs += 1

        print(
            'epoch %3d/%d  train_loss=%.4f  train_acc=%.3f  '
            'val_loss=%.4f  val_acc=%.3f  (best_val_loss=%.4f)'
            % (
                epoch,
                epochs,
                train_loss / max(train_total, 1),
                train_correct / max(train_total, 1),
                val_loss,
                val_acc,
                best_val_loss,
            )
        )

        if bad_epochs >= patience:
            print('Early stopping at epoch %d (patience=%d).' % (epoch, patience))
            break

    if best_state is not None:
        model.load_state_dict(best_state)

    os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)
    torch.save(model.state_dict(), output_path)

    # Final validation accuracy with the best weights.
    model.eval()
    with torch.no_grad():
        val_logits = model(x_val)
        final_acc = float(
            (val_logits.argmax(-1) == y_val).float().mean().item()
        )

    metrics = {
        'dataset_path': dataset_path,
        'output_path': output_path,
        'epochs_run': min(epoch, epochs),
        'final_val_accuracy': final_acc,
        'best_val_loss': best_val_loss,
        'train_samples': int(len(train_idx)),
        'val_samples': int(len(val_idx)),
        'scan_size': scan_size,
        'hidden_size': hidden_size,
        'device': str(device_t),
        'class_names': ['near', 'far', 'free'],
        'training_time_sec': round(time.time() - t0, 2),
    }
    sidecar = output_path + '.json'
    with open(sidecar, 'w', encoding='utf-8') as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)

    print('Training done. Saved model to %s' % output_path)
    print('Final validation accuracy: %.3f' % final_acc)
    print('Metrics file: %s' % sidecar)
    return metrics


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description='Train LidarMLP on a recorded .npz dataset.')
    p.add_argument('--dataset', required=True, help='Path to the .npz dataset.')
    p.add_argument(
        '--output',
        default='models/lidar_mlp.pt',
        help='Where to save the trained weights.',
    )
    p.add_argument('--epochs', type=int, default=50)
    p.add_argument('--batch-size', type=int, default=64)
    p.add_argument('--lr', type=float, default=1e-3)
    p.add_argument('--hidden-size', type=int, default=128)
    p.add_argument('--patience', type=int, default=10)
    p.add_argument(
        '--device',
        default='auto',
        choices=['auto', 'cpu', 'cuda'],
        help='Torch device to use.',
    )
    return p.parse_args()


def main() -> None:
    """Entry point for `python -m lidar_obstacle_nn.train_model`."""
    args = parse_args()
    train(
        dataset_path=args.dataset,
        output_path=args.output,
        epochs=args.epochs,
        batch_size=args.batch_size,
        lr=args.lr,
        hidden_size=args.hidden_size,
        patience=args.patience,
        device=args.device,
    )


if __name__ == '__main__':
    main()
