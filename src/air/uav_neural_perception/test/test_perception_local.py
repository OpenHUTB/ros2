"""uav_neural_perception 本地测试（不依赖 ROS / torch / 终端）。

torch 相关用例在 CI 没装 torch 时自动 skip。
跑法：python3 test/test_perception_local.py
"""
from __future__ import annotations

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from uav_neural_perception import safety as S
from uav_neural_perception import preprocess as P


class TestSanitizeAction(unittest.TestCase):
    def test_valid_clips(self):
        self.assertEqual(S.sanitize_action([5.0, -1.0, 0.9], 2.0, 0.5, 0.4),
                         (2.0, -0.5, 0.4))

    def test_within_range_unchanged(self):
        self.assertEqual(S.sanitize_action([1.0, 0.2, 0.3], 2.0, 0.5, 0.4),
                         (1.0, 0.2, 0.3))

    def test_nan_rejected(self):
        self.assertIsNone(S.sanitize_action([float("nan"), 0.0, 0.0], 2.0, 0.5, 0.4))

    def test_inf_rejected(self):
        self.assertIsNone(S.sanitize_action([float("inf"), 0.0, 0.0], 2.0, 0.5, 0.4))
        self.assertIsNone(S.sanitize_action([0.0, float("-inf"), 0.0], 2.0, 0.5, 0.4))

    def test_wrong_length(self):
        self.assertIsNone(S.sanitize_action(None, 2.0, 0.5, 0.4))
        self.assertIsNone(S.sanitize_action([1.0, 0.0], 2.0, 0.5, 0.4))
        self.assertIsNone(S.sanitize_action([1.0, 0.0, 0.0, 0.0], 2.0, 0.5, 0.4))

    def test_non_numeric(self):
        self.assertIsNone(S.sanitize_action(["a", 0.0, 0.0], 2.0, 0.5, 0.4))


class TestWeightMissing(unittest.TestCase):
    def test_missing_no_random_raises(self):
        with self.assertRaises(S.WeightMissingError):
            S.load_trained_model("/nonexistent/xyz.pth", "cpu", lambda: None,
                                  allow_random=False)

    def test_missing_with_random_returns_model(self):
        # 不真的加载 torch，只验证 allow_random 路径不抛错
        class _FakeModel:
            def to(self, d): return self
            def eval(self): return self
        m = S.load_trained_model("/nonexistent/xyz.pth", "cpu",
                                  lambda: _FakeModel(), allow_random=True)
        self.assertIsInstance(m, _FakeModel)


class TestPreprocess(unittest.TestCase):
    def test_normalize_roundtrip(self):
        import numpy as np
        a = np.array([1.0, 0.25, 0.2])
        n = P.normalize_action(a)
        back = P.denormalize_action(n)
        np.testing.assert_allclose(back, a, atol=1e-6)

    def test_decode_rgb8(self):
        import numpy as np
        # 2x2 RGB 黑图
        data = bytes(2 * 2 * 3)
        img = P.decode_image("rgb8", data, 2, 2)
        self.assertEqual(img.shape, (2, 2, 3))
        self.assertTrue((img == 0).all())

    def test_decode_bgr8_flips(self):
        import numpy as np
        # 1x1 BGR = (0,0,255) -> RGB = (255,0,0)
        data = bytes([0, 0, 255])
        img = P.decode_image("bgr8", data, 1, 1)
        self.assertEqual(img[0, 0].tolist(), [255, 0, 0])

    def test_decode_mono8_replicates(self):
        import numpy as np
        data = bytes([128])
        img = P.decode_image("mono8", data, 1, 1)
        self.assertEqual(img[0, 0].tolist(), [128, 128, 128])

    def test_decode_unsupported(self):
        with self.assertRaises(ValueError):
            P.decode_image("xyz16", b"", 1, 1)

    def test_resize_nearest(self):
        import numpy as np
        img = np.arange(10, dtype=np.uint8).reshape(10, 1)
        out = P.resize_nearest(img, 5, 1)
        self.assertEqual(out.shape, (5, 1))

    def test_preprocess_shape(self):
        import numpy as np
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        t = P.preprocess_image(img)
        self.assertEqual(t.shape, (3, P.IMG_H, P.IMG_W))


@unittest.skipUnless(_has_torch := __import__("importlib.util").util.find_spec("torch") is not None,
                      "torch not installed in CI")
class TestModelForward(unittest.TestCase):
    def test_forward_shape(self):
        import torch
        from uav_neural_perception.model import build_model
        net = build_model()
        out = net(torch.randn(2, 3, 120, 160))
        self.assertEqual(tuple(out.shape), (2, 3))


class TestPackageLayout(unittest.TestCase):
    def _read(self, rel):
        root = os.path.join(os.path.dirname(__file__), "..")
        with open(os.path.join(root, rel), encoding="utf-8") as f:
            return f.read()

    def test_ament_python(self):
        xml = self._read("package.xml")
        self.assertIn("ament_python", xml)
        self.assertNotIn("catkin", xml)
        self.assertNotIn("rospy", xml)

    def test_no_cmakelists(self):
        root = os.path.join(os.path.dirname(__file__), "..")
        self.assertFalse(os.path.exists(os.path.join(root, "CMakeLists.txt")))

    def test_node_uses_rclpy(self):
        src = self._read("uav_neural_perception/perception_node.py")
        self.assertIn("import rclpy", src)
        self.assertNotIn("rospy", src)
        self.assertNotIn("CvBridge", src)


if __name__ == "__main__":
    unittest.main(verbosity=2)
