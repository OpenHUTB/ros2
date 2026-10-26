"""神经网络特征、权重文件和训练结果的稳定性测试。"""

import numpy as np
import pytest

from facade.learning import DenseController, control_features, train_controller
from facade.scene import build_scene
import mujoco


def test_sensor_features_have_expected_order_and_wall_error():
    model = build_scene()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    values = control_features(np.array([1., 80., -.5]), data.sensordata[:9],
                              np.array([.1, 0., -.2]), 4.)
    assert values.shape == (15,)
    np.testing.assert_allclose(values[9:12], [1., 0., -.5], atol=.03)


def test_train_save_and_reload_predicts_same_output(tmp_path):
    rng = np.random.default_rng(8)
    features = rng.normal(size=(512, 15))
    targets = np.clip(features[:, :3] * .25 - features[:, 3:6] * .1, -1., 1.)
    first = train_controller(features, targets, epochs=20, seed=19)
    second = train_controller(features, targets, epochs=20, seed=19)
    np.testing.assert_allclose(first.predict(features), second.predict(features))
    path = tmp_path / "model.json"
    first.save(path)
    np.testing.assert_allclose(DenseController.load(path).predict(features), first.predict(features))


def test_rejects_invalid_training_data():
    with pytest.raises(ValueError):
        train_controller(np.zeros((10, 15)), np.zeros((10, 3)))


def test_train_and_test_scenes_are_disjoint():
    from facade.dataset import TEST_SCENARIOS, TRAIN_SCENARIOS

    train_styles = {scene["style"] for scene in TRAIN_SCENARIOS}
    test_styles = {scene["style"] for scene in TEST_SCENARIOS}
    assert train_styles.isdisjoint(test_styles)
