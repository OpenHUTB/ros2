import unittest

import numpy as np

from carla_nn_roundabout_entry.end_to_end import ImageToControl


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


if __name__ == "__main__":
    unittest.main()
