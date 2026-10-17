import numpy as np

from mujoco_musculoskeletal_elbow_nn_control.mlp import ActivationMLP
from mujoco_musculoskeletal_elbow_nn_control.neural_node import NeuralControllerCore


def test_neural_core_returns_safe_zero_until_all_inputs_are_fresh(tmp_path):
    model = ActivationMLP(seed=1); model.save(tmp_path / "model.json")
    core = NeuralControllerCore(tmp_path / "model.json", timeout_s=0.1)
    assert np.all(core.predict(now=10.0) == 0.0)
    core.update_target(0.8); core.update_joint(0.7, 0.1); core.update_forces([1.0, 0.5])
    output = core.predict(now=core.updated + 0.01)
    assert output.shape == (2,) and np.isfinite(output).all() and ((output >= 0) & (output <= 1)).all()
    assert np.all(core.predict(now=core.updated + 0.2) == 0.0)
