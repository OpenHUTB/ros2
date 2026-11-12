import numpy as np

from mujoco_musculoskeletal_elbow_nn_control.mlp import ActivationMLP, score


def test_mlp_bounds_and_serialization(tmp_path):
    x = np.asarray([[0.8, 0.7, 0.1, 1.0, 0.5], [1.2, 1.0, -0.2, 0.2, 0.8]])
    y = np.asarray([[0.3, 0.1], [0.1, 0.4]])
    model = ActivationMLP(seed=3); model.fit(x, y, epochs=20)
    prediction = model.predict(x)
    assert prediction.shape == (2, 2)
    assert np.isfinite(prediction).all() and ((prediction >= 0) & (prediction <= 1)).all()
    path = tmp_path / "model.json"; model.save(path)
    np.testing.assert_allclose(model.predict(x), ActivationMLP.load(path).predict(x))


def test_score_reports_metrics():
    model = ActivationMLP(seed=4); x = np.ones((4, 5)); y = np.full((4, 2), 0.5)
    metrics = score(model, x, y)
    assert metrics.mae >= 0 and metrics.rmse >= 0 and metrics.steady_state_error >= 0 and metrics.inference_ms >= 0
