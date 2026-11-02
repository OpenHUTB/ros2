"""Small end-to-end training smoke test."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import torch

from lidar_obstacle_nn.generate_synthetic_dataset import (
    generate_dataset,
    save_dataset,
)
from lidar_obstacle_nn.model import LidarMLP
from lidar_obstacle_nn.train_model import resolve_device, train


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class TestTraining(unittest.TestCase):
    def test_cuda_unavailable_falls_back_to_cpu(self) -> None:
        with patch.object(torch.cuda, 'is_available', return_value=False):
            self.assertEqual(resolve_device('cuda').type, 'cpu')

    def test_training_saves_weights_and_metrics(self) -> None:
        scans, labels = generate_dataset(
            num_samples=90,
            noise_std=0.02,
            seed=21,
        )
        with tempfile.TemporaryDirectory(dir=PACKAGE_ROOT) as tmp:
            dataset_path = Path(tmp) / 'dataset.npz'
            output_path = Path(tmp) / 'lidar_mlp.pt'
            save_dataset(
                output_path=str(dataset_path),
                scans=scans,
                labels=labels,
                scan_size=360,
                danger_distance=1.0,
                far_distance=3.0,
            )

            metrics = train(
                dataset_path=str(dataset_path),
                output_path=str(output_path),
                epochs=3,
                batch_size=16,
                patience=2,
                device='cpu',
            )

            self.assertTrue(output_path.is_file())
            sidecar = output_path.with_name(output_path.name + '.json')
            self.assertTrue(sidecar.is_file())
            with sidecar.open(encoding='utf-8') as stream:
                saved_metrics = json.load(stream)
            self.assertEqual(saved_metrics['scan_size'], 360)
            self.assertEqual(saved_metrics['train_samples'], 72)
            self.assertEqual(saved_metrics['val_samples'], 18)
            self.assertEqual(metrics['device'], 'cpu')

            state = torch.load(
                output_path,
                map_location='cpu',
                weights_only=True,
            )
            model = LidarMLP(input_size=360)
            model.load_state_dict(state)
            self.assertEqual(
                model(torch.zeros((2, 360))).shape,
                (2, 3),
            )
