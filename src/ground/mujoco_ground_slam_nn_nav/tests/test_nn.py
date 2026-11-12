import unittest

import numpy as np

from gvnn.nn.cnn import CNN
from gvnn.nn.losses import mse
from gvnn.nn.mlp import MLP, train_model


class NNTest(unittest.TestCase):
    def test_mlp_training(self):
        rng = np.random.default_rng(0)
        X = rng.normal(size=(128, 3))
        y = (X[:, :1] * 0.5 - X[:, 1:2] * 0.2)
        model = MLP([3, 16, 1], out_activation="linear", seed=1)
        model.mean = X.mean(axis=0)
        model.std = X.std(axis=0) + 1e-6
        hist = train_model(model, X, y, epochs=40, batch_size=32, verbose=False)
        self.assertLess(hist["train_loss"][-1], hist["train_loss"][0])

    def test_cnn_shapes(self):
        spec = [
            {"type": "conv", "cin": 3, "cout": 4, "k": 3, "p": 1, "act": "relu"},
            {"type": "pool", "k": 2},
            {"type": "flatten"},
            {"type": "dense", "nin": 4 * 4 * 4, "nout": 2, "act": "tanh"},
        ]
        model = CNN(spec, seed=0)
        x = np.zeros((2, 8, 8, 3))
        y = model.forward(x)
        self.assertEqual(y.shape, (2, 2))

    def test_mlp_gradient(self):
        rng = np.random.default_rng(2)
        model = MLP([2, 3, 1], seed=3)
        x = rng.normal(size=(4, 2))
        t = rng.normal(size=(4, 1))
        pred = model.forward(x)
        _, grad = mse(t, pred)
        model.backward(grad)
        analytic = model.grads()["W0"][0, 0]
        eps = 1e-6
        w = model.layers[0].W
        old = w[0, 0]
        w[0, 0] = old + eps
        lp, _ = mse(t, model.forward(x))
        w[0, 0] = old - eps
        lm, _ = mse(t, model.forward(x))
        w[0, 0] = old
        numeric = (lp - lm) / (2 * eps)
        self.assertAlmostEqual(analytic, numeric, places=6)


if __name__ == "__main__":
    unittest.main()
