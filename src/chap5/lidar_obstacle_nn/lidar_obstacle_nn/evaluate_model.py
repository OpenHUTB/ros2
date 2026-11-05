"""Evaluate a trained lidar MLP checkpoint against a labelled .npz set.

Prints overall accuracy, per-class precision/recall and a 3x3 confusion
matrix (rows = true class, columns = predicted class). The dataset
format is the one written by generate_synthetic_dataset.py and
collect_data.py (``scans`` / ``labels`` / ``scan_size``).

Usage:
    ros2 run lidar_obstacle_nn evaluate_model --ros-args \\
        -p model_path:=/tmp/lidar_real.pt -p dataset_path:=/tmp/lidar_real_test.npz

or as a plain module (the ROS node wrapper is not needed for offline
evaluation):

    python3 -m lidar_obstacle_nn.evaluate_model \\
        --model /tmp/lidar_real.pt --dataset /tmp/lidar_real_test.npz
"""

from __future__ import annotations

import argparse
import sys

import numpy as np
import torch

from .model import LidarMLP
from .train_model import load_dataset

CLASS_NAMES = ['near', 'far', 'free']


def evaluate(model_path: str, dataset_path: str, device: str = 'cpu') -> int:
    scans, labels, scan_size = load_dataset(dataset_path)

    use_cuda = device in ('cuda', 'auto') and torch.cuda.is_available()
    device_t = torch.device('cuda' if use_cuda else 'cpu')

    model = LidarMLP(input_size=scan_size).to(device_t)
    state = torch.load(model_path, map_location=device_t, weights_only=True)
    model.load_state_dict(state)
    model.eval()

    x = torch.from_numpy(scans).to(device_t)
    with torch.no_grad():
        preds = model(x).argmax(dim=-1).cpu().numpy()

    n = len(labels)
    acc = float((preds == labels).mean())
    print('Model:   %s' % model_path)
    print('Dataset: %s (%d samples, device=%s)' % (dataset_path, n, device_t))
    print('Overall accuracy: %.4f (%d / %d)' % (acc, int((preds == labels).sum()), n))

    # Confusion matrix: rows = true, cols = predicted.
    cm = np.zeros((3, 3), dtype=np.int64)
    for t, p in zip(labels, preds):
        cm[int(t), int(p)] += 1

    print('\nPer-class metrics:')
    print('  class    precision recall  support')
    for i, name in enumerate(CLASS_NAMES):
        tp = int(cm[i, i])
        support = int(cm[i, :].sum())
        pred_count = int(cm[:, i].sum())
        precision = tp / pred_count if pred_count else 0.0
        recall = tp / support if support else 0.0
        print('  %-8s %6.3f    %6.3f   %5d' % (name, precision, recall, support))

    print('\nConfusion matrix (rows=true, cols=predicted):')
    print('              pred:near pred:far pred:free')
    for i, name in enumerate(CLASS_NAMES):
        print('  true:%-5s  %8d %9d %10d'
              % (name, cm[i, 0], cm[i, 1], cm[i, 2]))
    return 0


def main(argv=None) -> None:
    """Entry point usable both as a console script and via ros2 run."""
    # ros2 run passes --ros-args; argparse does not understand them, so
    # strip everything from the first --ros-args for module-style use.
    args = list(sys.argv[1:] if argv is None else argv)
    if '--ros-args' in args:
        args = args[:args.index('--ros-args')]

    p = argparse.ArgumentParser(description='Evaluate a lidar MLP checkpoint.')
    p.add_argument('--model', help='Path to lidar_mlp.pt checkpoint.')
    p.add_argument('--dataset', help='Path to the labelled .npz dataset.')
    p.add_argument('--device', default='cpu', choices=['cpu', 'cuda', 'auto'])
    ns = p.parse_args(args)

    # Also accept ROS parameters when run as `ros2 run ... --ros-args -p`.
    if ns.model is None or ns.dataset is None:
        try:
            import rclpy
            rclpy.init(args=sys.argv[1:] if argv is None else argv)
            node = rclpy.create_node('evaluate_model')
            node.declare_parameter('model_path', '')
            node.declare_parameter('dataset_path', '')
            node.declare_parameter('device', 'cpu')
            ns.model = ns.model or node.get_parameter('model_path').value
            ns.dataset = ns.dataset or node.get_parameter('dataset_path').value
            ns.device = node.get_parameter('device').value
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

    if not ns.model or not ns.dataset:
        p.error('--model and --dataset are required '
                '(or model_path / dataset_path ROS parameters).')

    raise SystemExit(evaluate(ns.model, ns.dataset, ns.device))


if __name__ == '__main__':
    main()
