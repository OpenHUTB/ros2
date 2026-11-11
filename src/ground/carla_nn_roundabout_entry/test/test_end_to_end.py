"""离线单测：dataset.py 合成图像/标签规则 与 end_to_end.py 前向/训练。"""

import unittest

import numpy as np

from carla_nn_roundabout_entry.dataset import (
    IMAGE_H, IMAGE_W,
    make_dataset, rule_label, render_frame, split_dataset,
)
from carla_nn_roundabout_entry.end_to_end import ImageToControl


class TestDataset(unittest.TestCase):
    def test_render_frame_shape_and_range(self):
        img = render_frame(lateral_offset=0.0, oncoming_x=5.0, oncoming_y=1.0)
        self.assertEqual(img.shape, (IMAGE_H, IMAGE_W))
        self.assertTrue(img.dtype == np.uint8)
        self.assertTrue(img.min() >= 0 and img.max() <= 255)

    def test_rule_label_brake_when_oncoming_near(self):
        # 来车近 → 刹车
        act = rule_label(lateral_offset=0.0, oncoming_x=3.0, oncoming_y=1.0)
        self.assertAlmostEqual(act[1], 0.0, places=3)  # throttle
        self.assertAlmostEqual(act[2], 1.0, places=3)  # brake

    def test_rule_label_throttle_when_oncoming_far(self):
        act = rule_label(lateral_offset=0.0, oncoming_x=11.0, oncoming_y=0.0)
        self.assertAlmostEqual(act[1], 0.8, places=3)
        self.assertAlmostEqual(act[2], 0.0, places=3)

    def test_rule_label_coast_in_mid_range(self):
        act = rule_label(lateral_offset=0.0, oncoming_x=7.5, oncoming_y=0.0)
        self.assertTrue(0.0 < act[1] < 0.8)  # throttle 滑行
        self.assertTrue(0.0 < act[2] < 1.0)

    def test_rule_label_steer_corrects_lateral_offset(self):
        # 偏右 → 方向向左（负）
        act_right = rule_label(lateral_offset=1.0, oncoming_x=11.0, oncoming_y=0.0)
        act_left = rule_label(lateral_offset=-1.0, oncoming_x=11.0, oncoming_y=0.0)
        self.assertLess(act_right[0], 0.0)
        self.assertGreater(act_left[0], 0.0)

    def test_make_dataset_shapes(self):
        images, labels = make_dataset(n=100, seed=0)
        self.assertEqual(images.shape, (100, IMAGE_H, IMAGE_W))
        self.assertEqual(labels.shape, (100, 3))

    def test_split_dataset(self):
        images, labels = make_dataset(n=200, seed=1)
        tr_i, tr_l, te_i, te_l = split_dataset(images, labels, test_ratio=0.2, seed=1)
        self.assertEqual(len(tr_i), 160)
        self.assertEqual(len(te_i), 40)
        # 无重叠
        tr_set = set(map(tuple, tr_i.reshape(160, -1)))
        te_set = set(map(tuple, te_i.reshape(40, -1)))
        self.assertEqual(len(tr_set & te_set), 0)


class TestEndToEndModel(unittest.TestCase):
    def test_predict_output_range(self):
        model = ImageToControl(seed=4)
        imgs = np.random.randint(0, 255, (5, IMAGE_H, IMAGE_W), dtype=np.uint8)
        out = model.predict(imgs)
        self.assertEqual(out.shape, (5, 3))
        # steer ∈ [-1,1]
        self.assertTrue(np.all(out[:, 0] >= -1.0) and np.all(out[:, 0] <= 1.0))
        # throttle/brake ∈ [0,1]
        self.assertTrue(np.all(out[:, 1:] >= 0.0) and np.all(out[:, 1:] <= 1.0))

    def test_training_reduces_loss(self):
        model = ImageToControl(seed=2)
        rng = np.random.default_rng(0)
        imgs = rng.integers(0, 255, (50, IMAGE_H, IMAGE_W), dtype=np.uint8)
        # 简单标签：全零帧 → 刹车
        labels = np.tile([0.0, 0.0, 1.0], (50, 1))
        losses = model.train(imgs, labels, epochs=100, learning_rate=0.05)
        self.assertGreater(losses[0], losses[-1])

    def test_save_and_load_roundtrip(self):
        model = ImageToControl(seed=7)
        imgs = np.random.randint(0, 255, (3, IMAGE_H, IMAGE_W), dtype=np.uint8)
        expected = model.predict(imgs)
        path = "/tmp/_rb_test_model.npz"
        model.save(path)
        loaded = ImageToControl.load(path)
        actual = loaded.predict(imgs)
        np.testing.assert_allclose(expected, actual, atol=1e-6)

    def test_training_wrong_shape_raises(self):
        model = ImageToControl(seed=1)
        imgs = np.zeros((4, IMAGE_H, IMAGE_W), dtype=np.uint8)
        bad_labels = np.zeros((4, 2))  # 应该是 Nx3
        with self.assertRaises(ValueError):
            model.train(imgs, bad_labels, epochs=1)


if __name__ == "__main__":
    unittest.main()
