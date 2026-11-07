import numpy as np

from uav_gate_center_servo.core.mlp import TinyMLP


def test_param_count_and_forward():
    net = TinyMLP(6, 4, 3, seed=0)
    assert net.n_params() == 6 * 4 + 4 + 4 * 3 + 3
    assert net.forward(np.zeros((2, 6), np.float32)).shape == (2, 3)


def test_numerical_gradient():
    rng = np.random.default_rng(0)
    net = TinyMLP(6, 4, 3, seed=0)
    x = rng.normal(size=(5, 6)).astype(np.float32)
    y = rng.normal(size=(5, 3)).astype(np.float32)
    yh, cache = net.forward(x, cache=True)
    dy = 2.0 * (yh - y) / yh.size
    gW1, _, _, _ = net.backward(dy, cache)
    eps = 1e-6
    old = net.W1[0, 0]
    net.W1[0, 0] = old + eps
    lp = float(np.mean((net.forward(x) - y) ** 2))
    net.W1[0, 0] = old - eps
    lm = float(np.mean((net.forward(x) - y) ** 2))
    net.W1[0, 0] = old
    num = (lp - lm) / (2 * eps)
    assert abs(num - gW1[0, 0]) / max(abs(num), 1e-9) < 2e-4


def test_training_decreases_loss():
    rng = np.random.default_rng(1)
    net = TinyMLP(6, 8, 3, seed=1)
    net.set_stats(rng.normal(size=(64, 6)).astype(np.float32),
                  rng.normal(size=(64, 3)).astype(np.float32))
    x = rng.normal(size=(64, 6)).astype(np.float32)
    y = np.tile(np.array([3.0, 0.5, -0.5], np.float32), (64, 1))
    first = net.train_batch(x, y)
    for _ in range(80):
        last = net.train_batch(x, y)
    assert last < first


def test_save_load(tmp_path):
    rng = np.random.default_rng(2)
    net = TinyMLP(6, 8, 3, seed=2)
    net.set_stats(rng.normal(size=(32, 6)).astype(np.float32),
                  rng.normal(size=(32, 3)).astype(np.float32))
    path = tmp_path / "m.npz"
    net.save(str(path))
    loaded = TinyMLP.load(str(path))
    x = rng.normal(size=(4, 6)).astype(np.float32)
    assert np.allclose(net.forward(net._norm_x(x)),
                       loaded.forward(loaded._norm_x(x)), atol=1e-6)
    assert path.stat().st_size < 100 * 1024


def test_predict_bounds():
    net = TinyMLP(192, 4, 3, seed=0)
    net.set_stats(np.ones((4, 12, 16), np.float32),
                  np.zeros((4, 3), np.float32))
    pred = net.predict(np.zeros((12, 16), np.float32))
    assert pred.shape == (3,) and np.all(np.isfinite(pred))
    assert 0.0 <= pred[0] <= 10.0
    assert np.all(np.abs(pred[1:]) <= 3.0)
