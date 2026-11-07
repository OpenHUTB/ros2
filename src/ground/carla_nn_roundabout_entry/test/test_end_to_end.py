import unittest

import numpy as np

from carla_nn_roundabout_entry.end_to_end import (
    ImageToControl,
    generate_teacher_dataset,
    render_state_observation,
    train_and_evaluate_teacher,
)


class ImageToControlTests(unittest.TestCase):
    def test_image_batch_maps_to_bounded_control(self):
        model = ImageToControl(seed=4)
        images = np.zeros((3, 16, 16), dtype=np.uint8)
        controls = model.predict(images)
        self.assertEqual(controls.shape, (3, 3))
        self.assertTrue(np.all(controls[:, 0] >= -1.0))
        self.assertTrue(np.all(controls[:, 1:] >= 0.0))
        self.assertTrue(np.all(controls[:, 1:] <= 1.0))

    def test_training_accepts_image_and_action_pairs(self):
        model = ImageToControl(seed=2)
        images = np.zeros((4, 16, 16), dtype=np.uint8)
        targets = np.array([[0.0, 0.5, 0.0]] * 4)
        model.train(images, targets, epochs=3)
        self.assertEqual(model.predict(images).shape, (4, 3))

    def test_simulated_observation_is_repeatable_and_encodes_traffic(self):
        ego = (0.0, 0.0, 0.05, 5.0)
        traffic = [(8.0, 0.5, 2.0, 0.0)]
        first = render_state_observation(ego, traffic, lane_offset_m=0.3)
        second = render_state_observation(ego, traffic, lane_offset_m=0.3)
        different_velocity = render_state_observation(
            ego, [(8.0, 0.5, 7.0, 0.0)], lane_offset_m=0.3
        )
        clear = render_state_observation(ego, [], lane_offset_m=0.3)
        self.assertEqual(first.dtype, np.uint8)
        self.assertEqual(first.shape, (16, 16))
        np.testing.assert_array_equal(first, second)
        self.assertFalse(np.array_equal(first, clear))
        self.assertFalse(np.array_equal(first, different_velocity))

    def test_teacher_dataset_is_reproducible_and_bounded(self):
        images_a, controls_a = generate_teacher_dataset(seed=19, samples=48)
        images_b, controls_b = generate_teacher_dataset(seed=19, samples=48)
        self.assertEqual(images_a.shape, (48, 16, 16))
        self.assertTrue(np.issubdtype(images_a.dtype, np.integer))
        np.testing.assert_array_equal(images_a, images_b)
        np.testing.assert_array_equal(controls_a, controls_b)
        self.assertTrue(np.all(np.isfinite(controls_a)))
        self.assertTrue(np.all(controls_a[:, 0] >= -1.0))
        self.assertTrue(np.all(controls_a[:, 0] <= 1.0))
        self.assertTrue(np.all(controls_a[:, 1:] >= 0.0))
        self.assertTrue(np.all(controls_a[:, 1:] <= 1.0))

    def test_train_and_evaluate_returns_repeatable_held_out_metrics(self):
        test_images, test_controls = _held_out_split(seed=23, samples=64)
        baseline = ImageToControl(seed=23)
        baseline_mse = baseline.evaluate(test_images, test_controls)["mse"]
        model_a, metrics_a = train_and_evaluate_teacher(seed=23, samples=64, epochs=500)
        model_b, metrics_b = train_and_evaluate_teacher(seed=23, samples=64, epochs=500)
        self.assertEqual(metrics_a, metrics_b)
        self.assertEqual(metrics_a["train_samples"], 51)
        self.assertEqual(metrics_a["test_samples"], 13)
        self.assertTrue(np.isfinite(metrics_a["test_mse"]))
        self.assertLess(metrics_a["test_mse"], baseline_mse)
        self.assertAlmostEqual(model_a.evaluate(test_images, test_controls)["mse"], metrics_a["test_mse"])
        self.assertEqual(model_b.predict(np.zeros((1, 16, 16), dtype=np.uint8)).shape, (1, 3))


def _held_out_split(seed: int, samples: int) -> tuple[np.ndarray, np.ndarray]:
    images, controls = generate_teacher_dataset(seed=seed, samples=samples)
    order = np.random.default_rng(seed + 1).permutation(samples)
    split = max(4, min(samples - 4, int(samples * 0.8)))
    return images[order[split:]], controls[order[split:]]


if __name__ == "__main__":
    unittest.main()
