import numpy as np
import pytest

from mujoco_uav_moving_pad_landing.image_controller import image_input, train_image_controller


def test_image_input_is_normalized_rgb_pixel_vector():
    image = np.full((32, 48, 3), 255, dtype=np.uint8)
    pixels = image_input(image)

    assert pixels.shape == (32 * 32 * 3,)
    assert pixels.dtype == np.float32
    np.testing.assert_array_equal(pixels, np.ones_like(pixels))


def test_image_training_uses_rendered_frames_and_episode_holdout(tmp_path):
    result = train_image_controller(tmp_path / "image_controller.joblib", samples=200)

    assert result["input"] == "32x32 RGB camera image"
    assert result["train_samples"] + result["test_samples"] == 200
    assert result["train_episodes"] + result["test_episodes"] == 10
    assert (tmp_path / "image_controller.joblib").is_file()


def test_image_training_rejects_small_dataset(tmp_path):
    with pytest.raises(ValueError, match="at least 200"):
        train_image_controller(tmp_path / "image_controller.joblib", samples=199)
