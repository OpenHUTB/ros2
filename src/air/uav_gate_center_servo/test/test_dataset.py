import numpy as np

from uav_gate_center_servo.core.dataset import mirror_batch, split_by_episode
from uav_gate_center_servo.core.mlp import TinyMLP


def test_split_no_episode_leak():
    d = np.zeros((10, 12, 16), np.float32)
    l = np.zeros((10, 3), np.float32)
    e = np.array([0, 0, 0, 0, 1, 1, 1, 2, 2, 2], np.int32)
    ds = split_by_episode(d, l, e, val_frac=0.34, seed=0)
    assert not (set(ds.train_episode.tolist()) & set(ds.val_episode.tolist()))


def test_mirror_and_train_only_stats():
    orig = np.arange(2 * 12 * 16, dtype=np.float32).reshape(2, 12, 16)
    label = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], np.float32)
    d, l = mirror_batch(orig, label, np.array([True, False]))
    assert np.allclose(d[0], orig[0][:, ::-1])
    assert l[0, 1] == -2.0 and l[1, 1] == 5.0
    train = np.full((4, 12, 16), 2.0, np.float32)
    val = np.full((2, 12, 16), 5.0, np.float32)
    net = TinyMLP(192, 4, 3, seed=0)
    net.set_stats(train, np.zeros((4, 3), np.float32))
    assert abs(net.x_mean - 2.0) < 1e-6
