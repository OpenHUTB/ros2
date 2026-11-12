"""策略网络（三种结构）的单元测试。需要 torch，缺失时整体跳过。"""

import numpy as np
import pytest

pytest.importorskip("torch")

from uav_fault_e2e_thrust_policy import policy as P
from uav_fault_e2e_thrust_policy.obs import DEPTH_SIZE, PROPRIO_DIM


def _batch(n: int, stack: int):
    rng = np.random.default_rng(0)
    return {
        "depth": rng.uniform(0.0, 1.0, size=(n, stack, DEPTH_SIZE, DEPTH_SIZE)).astype(np.float32),
        "proprio": rng.normal(size=(n, PROPRIO_DIM)).astype(np.float32),
    }


@pytest.mark.parametrize("arch", P.ARCHS)
def test_module_output_shape_and_range(arch):
    module = P.build_module(arch)
    out = P.predict(module, _batch(3, module.stack))
    assert out.shape == (3, 4)
    assert np.all(np.isfinite(out))
    assert out.min() >= 0.0 and out.max() <= 1.0


def test_parameter_counts_are_distinct():
    counts = {a: P.n_parameters(P.build_module(a)) for a in P.ARCHS}
    assert all(v > 0 for v in counts.values())
    assert len(set(counts.values())) == len(P.ARCHS), counts


def test_cnn_smaller_than_mlp():
    assert P.n_parameters(P.build_module("cnn")) < P.n_parameters(P.build_module("mlp"))


def test_stack_depth_uses_declared_stack():
    module = P.build_module("cnn_stack")
    history = [np.full((DEPTH_SIZE, DEPTH_SIZE), i / 10.0, dtype=np.float32)
               for i in range(10)]
    stacked = P.stack_depth(history, module)
    assert stacked.shape == (1, module.stack, DEPTH_SIZE, DEPTH_SIZE)
    assert np.allclose(stacked[0, -1], history[-1])
    assert module.stack == P.DEPTH_STACK


def test_stack_depth_pads_when_history_short():
    module = P.build_module("cnn_stack")
    history = [np.zeros((DEPTH_SIZE, DEPTH_SIZE), dtype=np.float32)]
    stacked = P.stack_depth(history, module)
    assert stacked.shape == (1, module.stack, DEPTH_SIZE, DEPTH_SIZE)
    assert np.allclose(stacked, 0.0)


def test_single_frame_archs_use_one_frame():
    for arch in ("mlp", "cnn"):
        module = P.build_module(arch)
        history = [np.full((DEPTH_SIZE, DEPTH_SIZE), i / 10.0, dtype=np.float32)
                   for i in range(5)]
        assert P.stack_depth(history, module).shape[1] == 1


def test_checkpoint_roundtrip(tmp_path):
    module = P.build_module("cnn")
    path = tmp_path / "p.pt"
    P.save_checkpoint(path, module, meta={"note": "unit"})
    loaded, meta = P.load_checkpoint(path)
    assert meta.get("note") == "unit"
    batch = _batch(2, module.stack)
    assert np.allclose(P.predict(module, batch), P.predict(loaded, batch), atol=1e-6)
    assert (tmp_path / "p.json").exists()


def test_unknown_arch_rejected():
    with pytest.raises((ValueError, KeyError)):
        P.build_module("transformer")
