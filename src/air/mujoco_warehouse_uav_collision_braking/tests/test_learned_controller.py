"""学习控制输入契约检查。 / Learned-controller input contract checks."""

import unittest

import numpy as np

from warehouse_braking.model import RiskSpeedMLP


class LearnedControllerTests(unittest.TestCase):
    def test_nine_range_feature_contract(self):
        model = RiskSpeedMLP(seed=1)
        features = np.zeros((1, 20), dtype=np.float32)
        probability, speed = model.predict(features)
        self.assertEqual(probability.shape, (1, 3))
        self.assertEqual(speed.shape, (1,))
        self.assertGreaterEqual(float(speed[0]), 0)
        self.assertLessEqual(float(speed[0]), 3)


if __name__ == "__main__":
    unittest.main()
