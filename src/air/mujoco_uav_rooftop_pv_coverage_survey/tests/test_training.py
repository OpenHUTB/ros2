import torch

from rooftop_pv.models import TinyMLP, TinyUNet
from rooftop_pv.training import segmentation_loss, segmentation_metrics


def test_models_have_expected_shapes():
    assert TinyUNet()(torch.zeros(2, 3, 120, 160)).shape == (2, 1, 120, 160)
    assert TinyMLP()(torch.zeros(4, 3)).shape == (4, 2)


def test_segmentation_loss_and_metrics_are_finite():
    logits = torch.zeros(1, 1, 2, 2)
    masks = torch.tensor([[[[1.0, 0.0], [1.0, 0.0]]]])
    assert torch.isfinite(segmentation_loss(logits, masks))
    metrics = segmentation_metrics(masks.bool(), masks.bool())
    assert metrics["iou"] == 1.0
    assert metrics["dice"] == 1.0

