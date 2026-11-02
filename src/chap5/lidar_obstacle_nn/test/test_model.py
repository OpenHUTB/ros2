"""Unit tests for the LidarMLP architecture and scan normalization."""

from __future__ import annotations

import unittest

import torch

from lidar_obstacle_nn.model import LidarMLP, normalise_scan


class TestLidarModel(unittest.TestCase):
    def test_model_returns_three_logits_and_probabilities(self) -> None:
        torch.manual_seed(7)
        model = LidarMLP(input_size=360, hidden_size=128, num_classes=3)
        model.eval()

        scans = torch.zeros((4, 360), dtype=torch.float32)
        scans[0, 0] = float('inf')
        scans[1, 0] = float('nan')
        scans[2, 0] = -float('inf')
        scans[3, :] = 30.0

        logits = model(scans)
        probabilities = model.predict_proba(scans)

        self.assertEqual(logits.shape, (4, 3))
        self.assertEqual(probabilities.shape, (4, 3))
        self.assertTrue(torch.isfinite(logits).all())
        self.assertTrue(torch.isfinite(probabilities).all())
        torch.testing.assert_close(
            probabilities.sum(dim=1),
            torch.ones(4),
            atol=1e-6,
            rtol=1e-6,
        )

    def test_scan_normalization_sanitizes_invalid_ranges(self) -> None:
        scan = torch.tensor(
            [[
                -float('inf'),
                -1.0,
                0.0,
                15.0,
                30.0,
                float('inf'),
                float('nan'),
            ]],
            dtype=torch.float32,
        )

        normalized = normalise_scan(scan)

        torch.testing.assert_close(
            normalized,
            torch.tensor([[0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0]]),
        )
