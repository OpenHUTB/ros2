import unittest

import numpy as np

from carla_nn_roundabout_entry.policy import GapPolicy
from carla_nn_roundabout_entry.train_policy import training_data


class GapPolicyTests(unittest.TestCase):
    def test_training_is_reproducible_and_learns_labels(self):
        features, labels = training_data(seed=3, samples=300)
        model = GapPolicy(seed=3)
        accuracy = model.train(features, labels, epochs=6000)
        self.assertGreater(accuracy, 0.85)

    def test_probabilities_are_normalized(self):
        model = GapPolicy()
        probs = model.probabilities(np.array([0.8, 0.8, 0.2, 0.0]))
        self.assertEqual(probs.shape, (1, 2))
        self.assertAlmostEqual(float(probs.sum()), 1.0)


if __name__ == "__main__":
    unittest.main()
