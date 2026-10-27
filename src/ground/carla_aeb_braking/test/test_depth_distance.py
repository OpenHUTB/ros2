"""离线单元测试：深度图解码与前方距离提取（无 ROS / 无 CARLA）。"""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

from carla_aeb_braking.aeb_core import (  # noqa: E402
    AEBParams,
    decode_carla_depth,
    depth_min_distance,
    encode_carla_depth,
)

W, H = 128, 72


class TestDepthDecode(unittest.TestCase):
    def test_roundtrip(self):
        rng = np.random.default_rng(0)
        depth = rng.uniform(0.5, 40.0, size=(H, W)).astype(np.float32)
        dec = decode_carla_depth(encode_carla_depth(depth), W, H)
        # 16 位编码精度为 1/256 米
        self.assertLessEqual(np.abs(dec - depth).max(), 1.0 / 256.0)

    def test_known_value(self):
        depth = np.full((H, W), 10.0, dtype=np.float32)
        dec = decode_carla_depth(encode_carla_depth(depth), W, H)
        self.assertAlmostEqual(float(dec[0, 0]), 10.0, places=3)

    def test_raw_size(self):
        raw = encode_carla_depth(np.zeros((H, W), dtype=np.float32))
        self.assertEqual(len(raw), H * W * 4)  # CARLA 相机原始数据为 BGRA

    def test_frame_min_distance(self):
        from carla_aeb_braking.aeb_core import frame_min_distance
        depth = np.full((H, W), 100.0, dtype=np.float32)
        depth[H // 2, W // 2] = 6.0
        raw = encode_carla_depth(depth)
        self.assertAlmostEqual(frame_min_distance(raw, W, H), 6.0)
        # 全空帧 → valid_max
        empty = encode_carla_depth(np.full((H, W), 100.0, dtype=np.float32))
        self.assertEqual(frame_min_distance(empty, W, H, valid_max=80.0), 80.0)


class TestDepthMinDistance(unittest.TestCase):
    def test_no_obstacle_returns_valid_max(self):
        p = AEBParams()
        depth = np.full((H, W), 100.0, dtype=np.float32)  # 超出 valid_max
        self.assertEqual(depth_min_distance(depth, valid_max=p.valid_max), p.valid_max)

    def test_center_obstacle_detected(self):
        depth = np.full((H, W), 100.0, dtype=np.float32)
        depth[H // 2, W // 2] = 5.0
        self.assertAlmostEqual(depth_min_distance(depth), 5.0)

    def test_out_of_band_obstacle_ignored(self):
        depth = np.full((H, W), 100.0, dtype=np.float32)
        depth[2, W // 2] = 3.0        # 天空区域（行太靠上）
        depth[H - 2, W // 2] = 4.0    # 地面近端（行太靠下）
        depth[H // 2, 2] = 6.0        # 画面左侧（列太靠左）
        self.assertEqual(depth_min_distance(depth), 80.0)

    def test_body_self_filtered_by_valid_min(self):
        depth = np.full((H, W), 100.0, dtype=np.float32)
        depth[H // 2, W // 2] = 0.1   # 低于 valid_min，视为车身自身
        self.assertEqual(depth_min_distance(depth), 80.0)

    def test_returns_nearest_in_band(self):
        depth = np.full((H, W), 100.0, dtype=np.float32)
        depth[H // 2, W // 3] = 9.0
        depth[H // 2, 2 * W // 3] = 4.0
        self.assertAlmostEqual(depth_min_distance(depth), 4.0)


if __name__ == "__main__":
    unittest.main()
