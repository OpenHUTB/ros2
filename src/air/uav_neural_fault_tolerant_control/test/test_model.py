"""残差网络与检查点的单元测试。

torch 是可选依赖：没装时整个文件跳过，而不是失败。
"""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("torch", reason="需要 torch")

from uav_neural_fault_tolerant_control import features as F  # noqa: E402
from uav_neural_fault_tolerant_control import model as M  # noqa: E402


def test_parameter_count():
    net = M.build_model()
    assert net.n_parameters() == 5252


def test_output_layer_is_zero_initialized():
    """未训练时残差恒为 0 —— 部署上等价于近似分配。"""
    net = M.build_model()
    x = np.random.default_rng(0).normal(size=(8, F.FEATURE_DIM))
    y = M.predict_numpy(net, x)
    assert y.shape == (8, 4)
    assert np.allclose(y, 0.0, atol=1e-7)


def test_forward_shape_after_random_weights():
    net = M.build_model(hidden=(16, 8))
    # 把输出层改成非零权重，确认前向维度正确
    last = net.net[-1]
    last.weight.data.normal_(0.0, 0.1)
    last.bias.data.normal_(0.0, 0.1)
    y = M.predict_numpy(net, np.zeros((3, F.FEATURE_DIM)))
    assert y.shape == (3, 4)
    assert np.all(np.isfinite(y))


def test_checkpoint_roundtrip(tmp_path):
    net = M.build_model(hidden=(16,))
    last = net.net[-1]
    last.weight.data.normal_(0.0, 0.1)
    path = M.save_checkpoint(tmp_path / "allocator.pt", net, {"note": "测试"})

    loaded, meta = M.load_checkpoint(path)
    assert meta["note"] == "测试"
    assert loaded.n_parameters() == net.n_parameters()
    x = np.random.default_rng(1).normal(size=(5, F.FEATURE_DIM))
    assert np.allclose(M.predict_numpy(net, x), M.predict_numpy(loaded, x), atol=1e-6)
    assert path.with_suffix(".json").is_file()


def test_checkpoint_restores_non_default_arch(tmp_path):
    """结构参数要按检查点里的 arch 还原，而不是用缺省值硬编码。"""
    net = M.build_model(hidden=(24, 12), dropout=0.0)
    path = M.save_checkpoint(tmp_path / "m.pt", net, {})
    loaded, _ = M.load_checkpoint(path)
    assert loaded.hidden == (24, 12)
    assert loaded.n_parameters() == net.n_parameters()


def test_arch_version_is_recorded():
    net = M.build_model()
    assert net.arch()["arch_version"] == M.ARCH_VERSION
    assert net.arch()["feature_dim"] == F.FEATURE_DIM
