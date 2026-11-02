"""MLP controller checks. MLP 控制器检查。"""
import json
from pathlib import Path

import numpy as np

from mlp import MLPRegressor
from train import evaluate


def test_saved_model_has_expected_schema():
    path = Path(__file__).with_name("models") / "mlp_controller.json"
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["input_features"] == ["sensor_range_m", "closing_speed_m_s"]
    assert payload["hidden_size"] == 16


def test_fit_and_predict_are_finite():
    features = np.asarray([[2.0, 0.2], [1.0, 0.7], [0.5, 1.0], [3.0, 0.0]])
    target = np.asarray([0.4, -0.8, -2.0, 1.0])
    model = MLPRegressor(seed=7).fit(features, target, epochs=20)
    metrics = evaluate(model, features, target)
    assert np.isfinite(model.predict(features)).all()
    assert metrics["samples"] == 4
