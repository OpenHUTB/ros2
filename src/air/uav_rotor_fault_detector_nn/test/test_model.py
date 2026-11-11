"""网络结构与检查点读写的单元测试（没有 torch 时整体跳过）。"""

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from uav_rotor_fault_detector_nn.features import FEATURE_DIM, ROTOR_COUNT  # noqa: E402
from uav_rotor_fault_detector_nn.model import (  # noqa: E402
    ARCH_VERSION,
    FaultDetectorNet,
    build_model,
    load_checkpoint,
    save_checkpoint,
)


def _tiny_model() -> FaultDetectorNet:
    return FaultDetectorNet(channels=(8, 8, 12), gru_hidden=16, head_dim=12)


def test_forward_shapes_and_range():
    model = _tiny_model()
    model.eval()
    with torch.no_grad():
        out = model(torch.zeros(3, 100, FEATURE_DIM))
    assert out.shape == (3, ROTOR_COUNT)
    assert torch.all(out >= 0.0) and torch.all(out <= 1.0)


def test_forward_rejects_bad_input():
    model = _tiny_model()
    with pytest.raises(ValueError):
        model(torch.zeros(3, FEATURE_DIM))


def test_parameter_count_positive():
    model = _tiny_model()
    assert model.n_parameters() > 0
    # 小网络的参数量必须显著小于正式配置
    assert model.n_parameters() < build_model().n_parameters()


def test_build_model_accepts_overrides():
    model = build_model(channels=(4, 4, 4), gru_hidden=8, head_dim=8)
    assert isinstance(model, FaultDetectorNet)
    model.eval()
    with torch.no_grad():
        assert model(torch.zeros(1, 40, FEATURE_DIM)).shape == (1, ROTOR_COUNT)


def test_checkpoint_roundtrip(tmp_path):
    model = _tiny_model()
    meta = {"feature_names": ["a"] * FEATURE_DIM, "window": 100, "stride": 10, "dt": 0.01}
    path = save_checkpoint(tmp_path / "tiny.pt", model, meta)

    assert path.is_file()
    assert path.with_suffix(".json").is_file()

    restored, loaded_meta = load_checkpoint(path)
    assert loaded_meta == meta
    assert restored.n_parameters() == model.n_parameters()

    model.eval()
    restored.eval()
    batch = torch.randn(2, 100, FEATURE_DIM)
    with torch.no_grad():
        np.testing.assert_allclose(restored(batch).numpy(), model(batch).numpy(), rtol=1e-6, atol=1e-6)


def test_checkpoint_missing_file(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_checkpoint(tmp_path / "nope.pt")


def test_checkpoint_rejects_wrong_arch_version(tmp_path):
    path = save_checkpoint(tmp_path / "tiny.pt", _tiny_model(), {})
    payload = torch.load(path, map_location="cpu", weights_only=False)
    payload["arch_version"] = ARCH_VERSION + 1
    torch.save(payload, path)
    with pytest.raises(ValueError):
        load_checkpoint(path)
