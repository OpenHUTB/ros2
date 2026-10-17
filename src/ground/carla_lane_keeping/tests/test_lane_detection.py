#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车道线检测算法的单元测试（纯 numpy，不需要 OpenCV / CARLA / ROS）。

运行::

    python -m unittest discover -s tests -v
"""

import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

from lane_detection import (  # noqa: E402
    LaneDetector,
    LaneFitConfig,
    birdseye_destination_points,
    color_lane_mask,
    compute_confidence,
    compute_homography,
    directional_gradient_mask,
    estimate_curvature_m,
    evaluate_poly,
    find_lane_bases,
    gradient_magnitude_direction,
    lane_geometry_with_width,
    make_synthetic_lane_image,
    sliding_window_fit,
    sobel_gradients,
    to_grayscale,
    trapezoid_source_points,
    warp_perspective,
)


class TestBasicImageOps(unittest.TestCase):
    def test_grayscale_from_bgr_matches_weights(self):
        """纯蓝像素应得到 0.114*255。"""
        img = np.zeros((2, 2, 3), dtype=np.uint8)
        img[..., 0] = 255  # B
        gray = to_grayscale(img)
        self.assertAlmostEqual(float(gray[0, 0]), 0.114 * 255, places=4)

    def test_grayscale_from_gray_passthrough(self):
        img = np.full((3, 3), 100, dtype=np.uint8)
        np.testing.assert_allclose(to_grayscale(img), 100.0)

    def test_grayscale_rejects_bad_shape(self):
        with self.assertRaises(ValueError):
            to_grayscale(np.zeros((3, 3, 2), dtype=np.uint8))

    def test_grayscale_handles_bgra(self):
        """四通道（BGRA）输入应取前三个通道。"""
        img = np.zeros((2, 2, 4), dtype=np.uint8)
        img[..., 1] = 255  # G
        gray = to_grayscale(img)
        self.assertAlmostEqual(float(gray[0, 0]), 0.587 * 255, places=4)


class TestSobel(unittest.TestCase):
    def test_vertical_edge_gives_horizontal_gradient(self):
        """一张左右两半不同亮度的图，在交界处应产生明显的 gx、可忽略的 gy。"""
        img = np.zeros((11, 11), dtype=np.float32)
        img[:, 6:] = 100.0
        gx, gy = sobel_gradients(img)
        col = 5  # 交界附近
        self.assertGreater(abs(float(gx[5, col])), 50.0)
        self.assertLess(abs(float(gy[5, col])), 1e-6)

    def test_horizontal_edge_gives_vertical_gradient(self):
        img = np.zeros((11, 11), dtype=np.float32)
        img[6:, :] = 100.0
        gx, gy = sobel_gradients(img)
        self.assertGreater(abs(float(gy[5, 5])), 50.0)
        self.assertLess(abs(float(gx[5, 5])), 1e-6)

    def test_uniform_image_gives_zero_gradient(self):
        img = np.full((8, 8), 42.0, dtype=np.float32)
        gx, gy = sobel_gradients(img)
        np.testing.assert_allclose(gx, 0.0, atol=1e-6)
        np.testing.assert_allclose(gy, 0.0, atol=1e-6)

    def test_vertical_edge_angle_near_zero(self):
        """竖直边缘的**梯度角**接近 0 度（梯度方向水平）。

        这是本模块最容易被搞反的一点：竖直的车道线给出的是 0 度的梯度角，
        而不是 90 度。若按 90 度写判据，所有车道线都会被过滤掉。
        """
        img = np.zeros((15, 15), dtype=np.float32)
        img[:, 8:] = 200.0
        _, angle, _ = gradient_magnitude_direction(img)
        self.assertLess(float(np.degrees(angle[7, 6])), 1.0)

    def test_horizontal_edge_angle_near_90(self):
        """水平边缘的梯度角接近 90 度（取真正落在边缘上的行）。"""
        img = np.zeros((15, 15), dtype=np.float32)
        img[8:, :] = 200.0
        _, angle, _ = gradient_magnitude_direction(img)
        # 边缘在第 8 行与第 7 行之间，第 7 行才是有效响应行
        self.assertGreater(float(np.degrees(angle[7, 7])), 89.0)

    def test_rejects_non_3x3_kernel(self):
        with self.assertRaises(ValueError):
            sobel_gradients(np.zeros((5, 5), dtype=np.float32), ksize=5)


class TestMasks(unittest.TestCase):
    def test_directional_mask_keeps_vertical_edge(self):
        """竖直边缘（车道线）必须被保留。"""
        img = np.zeros((40, 40), dtype=np.float32)
        img[:, 20:] = 60.0  # 幅值落在 [25, 255] 内
        mask = directional_gradient_mask(img, sobel_min=25, sobel_max=255, angle_max_deg=45)
        self.assertTrue(mask[:, 19].any(), "竖直边缘应被保留")

    def test_directional_mask_rejects_horizontal_edge(self):
        """接近水平的边缘不是车道线，应被方向判据剔除。"""
        img = np.zeros((40, 40), dtype=np.float32)
        img[20:, :] = 60.0
        mask = directional_gradient_mask(img, sobel_min=25, sobel_max=255, angle_max_deg=45)
        self.assertFalse(mask[19, :].any(), "水平边缘不应被保留")

    def test_directional_mask_rejects_weak_gradient(self):
        img = np.zeros((40, 40), dtype=np.float32)
        img[:, 20:] = 4.0  # 太弱
        mask = directional_gradient_mask(img, sobel_min=25)
        self.assertFalse(mask.any())

    def test_directional_mask_rejects_oversaturated_gradient(self):
        """过强的梯度（如过曝边界）应被上限剔除。"""
        img = np.zeros((40, 40), dtype=np.float32)
        img[:, 20:] = 250.0  # 幅值远超上限
        mask = directional_gradient_mask(img, sobel_min=25, sobel_max=255)
        self.assertFalse(mask.any())

    def test_directional_mask_validates_angle_range(self):
        img = np.zeros((10, 10), dtype=np.float32)
        with self.assertRaises(ValueError):
            directional_gradient_mask(img, angle_max_deg=120)
        with self.assertRaises(ValueError):
            directional_gradient_mask(img, sobel_min=300, sobel_max=100)

    def test_color_mask_finds_white_line(self):
        img = np.full((20, 20, 3), 60, dtype=np.uint8)
        img[:, 10:12] = 240  # 白色
        mask = color_lane_mask(img)
        self.assertTrue(mask[:, 10].all())
        self.assertFalse(mask[:, 0].any())

    def test_color_mask_finds_yellow_line(self):
        img = np.full((20, 20, 3), 60, dtype=np.uint8)
        img[:, 10:12] = (0, 200, 230)  # BGR：B 低、G/R 高 -> 黄
        mask = color_lane_mask(img)
        self.assertTrue(mask[:, 10].all())

    def test_color_mask_ignores_gray(self):
        """灰色路面不应被误判为白色车道线。"""
        img = np.full((20, 20, 3), 150, dtype=np.uint8)
        mask = color_lane_mask(img, white_min=180)
        self.assertFalse(mask.any())

    def test_color_mask_requires_three_channels(self):
        with self.assertRaises(ValueError):
            color_lane_mask(np.zeros((5, 5), dtype=np.uint8))


class TestHomography(unittest.TestCase):
    def setUp(self):
        self.src = np.array([[100.0, 100.0], [200.0, 100.0], [200.0, 300.0], [100.0, 300.0]])
        self.dst = np.array([[0.0, 0.0], [400.0, 0.0], [400.0, 200.0], [0.0, 200.0]])

    def test_maps_source_points_to_destination(self):
        h = compute_homography(self.src, self.dst)
        for (u, v), (x, y) in zip(self.src, self.dst):
            p = h @ np.array([u, v, 1.0])
            self.assertAlmostEqual(p[0] / p[2], x, places=6)
            self.assertAlmostEqual(p[1] / p[2], y, places=6)

    def test_identity_when_points_equal(self):
        h = compute_homography(self.src, self.src)
        np.testing.assert_allclose(h / h[2, 2], np.eye(3), atol=1e-9)

    def test_warp_moves_pixel_to_expected_location(self):
        """打一个亮点，warp 后应出现在目标点对应的位置。"""
        img = np.zeros((400, 400), dtype=np.uint8)
        # 源点 (100,100) 对应目标 (0,0)
        img[100, 100] = 255
        h = compute_homography(self.src, self.dst)
        out = warp_perspective(img, h, (400, 200))
        self.assertGreater(float(out[0, 0]), 200.0)

    def test_warp_out_of_bounds_is_zero(self):
        """源图像范围之外的目标像素应为 0。"""
        img = np.full((400, 400), 255, dtype=np.uint8)
        # 用一个大平移，使目标图左上角落到源图之外
        src = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]])
        dst = np.array([[-1000.0, -1000.0], [-990.0, -1000.0], [-990.0, -990.0], [-1000.0, -990.0]])
        h = compute_homography(src, dst)
        out = warp_perspective(img, h, (50, 50))
        self.assertAlmostEqual(float(out.mean()), 0.0, places=6)

    def test_warp_identity_preserves_image(self):
        img = (np.arange(100 * 100).reshape(100, 100) % 251).astype(np.uint8)
        out = warp_perspective(img, np.eye(3), (100, 100))
        # 边缘像素因采样越界会掉，中心区域应几乎一致
        diff = np.abs(out[5:-5, 5:-5].astype(int) - img[5:-5, 5:-5].astype(int))
        self.assertLess(float(diff.max()), 2.0)

    def test_warp_preserves_channels(self):
        img = np.zeros((50, 50, 3), dtype=np.uint8)
        img[25, 25] = (10, 20, 30)
        out = warp_perspective(img, np.eye(3), (50, 50))
        self.assertEqual(out.shape, (50, 50, 3))


class TestPerspectivePoints(unittest.TestCase):
    def test_source_trapezoid_ordering(self):
        """源点顺序为 左下、左上、右上、右下。"""
        pts = trapezoid_source_points(800, 600)
        self.assertEqual(pts.shape, (4, 2))
        self.assertLess(pts[0, 0], pts[3, 0])  # 左下在右下左边
        self.assertGreater(pts[0, 1], pts[1, 1])  # 下边比上边靠下
        self.assertLess(pts[1, 0], pts[2, 0])  # 左上在右上左边

    def test_trapezoid_is_narrower_at_top(self):
        pts = trapezoid_source_points(800, 600)
        bottom_width = pts[3, 0] - pts[0, 0]
        top_width = pts[2, 0] - pts[1, 0]
        self.assertLess(top_width, bottom_width)

    def test_destination_is_rectangle(self):
        pts = birdseye_destination_points(800, 600)
        self.assertAlmostEqual(pts[0, 1], pts[3, 1], places=6)  # 下边水平
        self.assertAlmostEqual(pts[1, 1], pts[2, 1], places=6)  # 上边水平
        self.assertAlmostEqual(pts[0, 0], pts[1, 0], places=6)  # 左边竖直
        self.assertAlmostEqual(pts[2, 0], pts[3, 0], places=6)  # 右边竖直


class TestFindLaneBases(unittest.TestCase):
    def test_finds_two_peaks(self):
        hist = np.zeros(800)
        hist[200] = 50
        hist[600] = 50
        left, right = find_lane_bases(hist)
        self.assertEqual(left, 200)
        self.assertEqual(right, 600)

    def test_finds_both_peaks_when_both_in_left_half(self):
        """两条车道线的峰都落在图像左半区时，仍须同时检出并正确分左右。

        这是回归测试。早期实现按「图像左右半」各取峰值，隐含假设车道中心
        恰在图像正中；车辆偏在车道一侧、或相机视野偏置时，两条线的峰可能都在
        同一半区，那一侧就恒为空，整条车道被丢弃。

        正确的判据是按列坐标大小分左右：左车道线必然在右车道线左边。
        """
        hist = np.zeros(800)
        hist[276] = 61   # 左车道线
        hist[371] = 180  # 右车道线（更强，但不是分左右依据）
        left, right = find_lane_bases(hist)
        self.assertEqual(left, 276)
        self.assertEqual(right, 371)

    def test_left_right_by_position_regardless_of_strength(self):
        """无论哪条更强，列号小的都归左车道线。"""
        hist = np.zeros(800)
        hist[200] = 200  # 左车道线更强
        hist[600] = 100
        left, right = find_lane_bases(hist)
        self.assertEqual(left, 200)
        self.assertEqual(right, 600)

        hist2 = np.zeros(800)
        hist2[200] = 100  # 右车道线更强
        hist2[600] = 200
        left2, right2 = find_lane_bases(hist2)
        self.assertEqual(left2, 200)
        self.assertEqual(right2, 600)

    def test_returns_none_for_empty(self):
        left, right = find_lane_bases(np.zeros(800))
        self.assertIsNone(left)
        self.assertIsNone(right)

    def test_single_lane_returns_one_side(self):
        """只有一条强峰时，只应给出一侧基点。"""
        hist = np.zeros(800)
        hist[300] = 80
        left, right = find_lane_bases(hist)
        found = [v for v in (left, right) if v is not None]
        self.assertEqual(found, [300])

    def test_weak_second_peak_is_rejected(self):
        """次强峰远低于全局峰时，判为该侧无车道线。"""
        hist = np.zeros(800)
        hist[200] = 100
        hist[600] = 5  # 远低于 min_relative_peak * 全局峰
        left, right = find_lane_bases(hist, min_relative_peak=0.25)
        found = [v for v in (left, right) if v is not None]
        self.assertIn(200, found)
        self.assertNotIn(600, found)

    def test_left_right_assigned_by_center_not_by_strength(self):
        """左右归属必须由图像中线决定，而不是由峰值强弱决定。

        全局最强峰是左侧那条（值更大）时，右车道线在它**右边**，
        早期实现只在最强峰左边找另一条，导致左车道被漏掉。
        """
        hist = np.zeros(800)
        hist[200] = 200  # 最强峰，位于中线左侧
        hist[600] = 100  # 次强峰，位于中线右侧
        left, right = find_lane_bases(hist)
        self.assertEqual(left, 200)
        self.assertEqual(right, 600)

    def test_single_peak_returns_one_valid_side(self):
        """只找到一条车道线时，返回的应是「恰好一侧有效」。

        此时无法判断它属于左还是右——没有第二条线做参照，任何猜测都有一半概率
        是错的。因此实现不作猜测，直接把它放在左侧返回，由上层依据
        「两侧都必须检出」来决定是否输出横向偏差（只有一条线时不输出）。
        """
        for pos in (200, 600):
            hist = np.zeros(800)
            hist[pos] = 100
            left, right = find_lane_bases(hist)
            valid = [v for v in (left, right) if v is not None]
            self.assertEqual(valid, [pos], f"单峰在 {pos} 时结果异常")
            self.assertEqual(sum(v is None for v in (left, right)), 1)

    def test_min_separation_prevents_duplicate_peak(self):
        """同一条车道线上的相邻隆起不应被当成左右两条。"""
        hist = np.zeros(800)
        hist[400] = 100
        hist[415] = 90  # 紧邻的次强隆起
        left, right = find_lane_bases(hist, min_separation_px=40)
        found = [v for v in (left, right) if v is not None]
        self.assertEqual(found, [400])


class TestSlidingWindowFit(unittest.TestCase):
    def _mask_with_line(self, coeffs, height=200, width=400, thickness=3):
        mask = np.zeros((height, width), dtype=bool)
        v = np.arange(height)
        u = np.polyval(coeffs, v).astype(int)
        for row in range(height):
            u0 = max(0, u[row] - thickness)
            u1 = min(width, u[row] + thickness + 1)
            if u1 > u0:
                mask[row, u0:u1] = True
        return mask

    def test_recovers_straight_line(self):
        """竖直直线 u = 100 + 0.2*v，拟合后应在各高度上复现。"""
        true = np.array([0.0, 0.2, 100.0])
        mask = self._mask_with_line(true)
        cfg = LaneFitConfig(min_lane_pixels=50)
        left, right, n_left, n_right = sliding_window_fit(mask, 100.0, None, cfg)
        self.assertIsNotNone(left)
        self.assertIsNone(right)
        self.assertGreater(n_left, 50)
        for v in (0, 100, 199):
            self.assertAlmostEqual(float(np.polyval(left, v)), float(np.polyval(true, v)), delta=3.0)

    def test_returns_none_when_too_few_pixels(self):
        mask = np.zeros((200, 400), dtype=bool)
        mask[100:103, 200:203] = True
        cfg = LaneFitConfig(min_lane_pixels=1000)
        left, right, _, _ = sliding_window_fit(mask, 200.0, None, cfg)
        self.assertIsNone(left)

    def test_none_base_yields_none(self):
        mask = np.zeros((200, 400), dtype=bool)
        mask[:, 100:110] = True
        cfg = LaneFitConfig(min_lane_pixels=50)
        left, right, n_left, n_right = sliding_window_fit(mask, None, None, cfg)
        self.assertIsNone(left)
        self.assertIsNone(right)
        self.assertEqual(n_left, 0)
        self.assertEqual(n_right, 0)


class TestGeometryMetrics(unittest.TestCase):
    def setUp(self):
        self.cfg = LaneFitConfig(meters_per_pixel_x=0.05, meters_per_pixel_y=0.05)

    def test_centered_vehicle_has_zero_offset(self):
        """车道中心在图像正中时，横向偏差应为 0。"""
        left = np.array([0.0, 0.0, 300.0])
        right = np.array([0.0, 0.0, 500.0])
        center, lateral, heading, curvature = lane_geometry_with_width(
            left, right, 800, 600, self.cfg
        )
        self.assertAlmostEqual(center, 400.0, places=6)
        self.assertAlmostEqual(lateral, 0.0, places=9)

    def test_vehicle_right_of_center_gives_positive_offset(self):
        """车道中心在图像中心左侧时，车辆偏右 -> 横向偏差为正。"""
        left = np.array([0.0, 0.0, 200.0])
        right = np.array([0.0, 0.0, 400.0])
        _, lateral, _, _ = lane_geometry_with_width(left, right, 800, 600, self.cfg)
        # 车道中心 300，车辆 400，差 100 px * 0.05 = 5 m
        self.assertAlmostEqual(lateral, 5.0, places=6)
        self.assertGreater(lateral, 0.0)

    def test_straight_lane_has_zero_heading(self):
        left = np.array([0.0, 0.0, 300.0])
        right = np.array([0.0, 0.0, 500.0])
        _, _, heading, _ = lane_geometry_with_width(left, right, 800, 600, self.cfg)
        self.assertAlmostEqual(heading, 0.0, places=6)

    def test_curvature_matches_analytic_parabola(self):
        """与解析曲率公式对照，验证换算公式本身没错。

        多项式 u = A v^2 + B v + C 在物理坐标下是抛物线
            x = u * mx,   y = v * my
        令 a = 2A * mx / my^2，则 x(y) 的二阶导恒为 a，一阶导为 a*y + b。
        曲率半径的解析式为
            R = (1 + (dx/dy)^2)^1.5 / |d2x/dy2|
        这里逐点比对实现值与解析值。
        """
        cfg = LaneFitConfig(meters_per_pixel_x=0.05, meters_per_pixel_y=0.05)
        coeffs = np.array([0.002, 0.5, 400.0])
        mx, my = cfg.meters_per_pixel_x, cfg.meters_per_pixel_y

        for v in (200.0, 400.0, 599.0):
            # 物理坐标下的一阶、二阶导
            du_dv = float(np.polyval(np.polyder(coeffs, 1), v))
            d2u_dv2 = float(np.polyval(np.polyder(coeffs, 2), v))
            dx_dy = du_dv * mx / my
            d2x_dy2 = d2u_dv2 * mx / (my * my)
            expected = (1.0 + dx_dy**2) ** 1.5 / abs(d2x_dy2)

            got = estimate_curvature_m(coeffs, v, cfg)
            self.assertAlmostEqual(got, expected, places=6)

    def test_curvature_decreases_with_larger_second_derivative(self):
        """二次项系数越大（弯得越急），曲率半径越小。

        在**同一个很小的 y** 上比较，使一阶导数项的影响一致，
        这样比较的才是二阶导数（曲率）本身。
        """
        gentle = np.array([0.0005, 0.0, 400.0])
        sharp = np.array([0.004, 0.0, 400.0])
        v_eval = 10.0  # 很小的 v，一阶导数项都很小
        r_gentle = estimate_curvature_m(gentle, v_eval, self.cfg)
        r_sharp = estimate_curvature_m(sharp, v_eval, self.cfg)
        self.assertLess(r_sharp, r_gentle)

    def test_straight_lane_has_infinite_curvature(self):
        straight = np.array([0.0, 0.0, 400.0])
        r = estimate_curvature_m(straight, 599.0, self.cfg)
        self.assertTrue(math.isinf(r))

    def test_curvature_scales_with_calibration(self):
        fit = np.array([0.002, 0.0, 400.0])
        r_small = estimate_curvature_m(
            fit, 599.0, LaneFitConfig(meters_per_pixel_x=0.02, meters_per_pixel_y=0.02)
        )
        r_large = estimate_curvature_m(
            fit, 599.0, LaneFitConfig(meters_per_pixel_x=0.10, meters_per_pixel_y=0.10)
        )
        self.assertNotAlmostEqual(r_small, r_large, places=3)


class TestConfidence(unittest.TestCase):
    def test_both_sides_full(self):
        self.assertAlmostEqual(compute_confidence(300, 300, 150), 1.0, places=6)

    def test_one_side_half(self):
        self.assertAlmostEqual(compute_confidence(300, 0, 150), 0.5, places=6)

    def test_none(self):
        self.assertAlmostEqual(compute_confidence(0, 0, 150), 0.0, places=6)

    def test_partial_is_between(self):
        c = compute_confidence(75, 0, 150)
        self.assertGreater(c, 0.0)
        self.assertLess(c, 0.5)

    def test_rejects_bad_threshold(self):
        with self.assertRaises(ValueError):
            compute_confidence(10, 10, 0)


class TestEndToEndDetection(unittest.TestCase):
    """完整链路：合成图像 -> 检测 -> 车道中心与偏差。"""

    def setUp(self):
        self.width, self.height = 800, 600
        self.cfg = LaneFitConfig(
            min_lane_pixels=120,
            window_margin_px=80,
            meters_per_pixel_x=0.05,
            meters_per_pixel_y=0.05,
        )

    def test_detects_both_lanes_on_synthetic_image(self):
        img = make_synthetic_lane_image(
            width=self.width, height=self.height,
            left_x_bottom=250, left_x_top=350,
            right_x_bottom=570, right_x_top=470,
        )
        det = LaneDetector(self.cfg)
        res = det.detect(img)
        self.assertTrue(res.detected, f"未检出两侧车道线 (L={res.left_pixels}, R={res.right_pixels})")
        self.assertGreater(res.confidence, 0.9)
        self.assertIsNotNone(res.lateral_offset_m)
        self.assertIsNotNone(res.curvature_m)

    def test_slanted_lane_is_not_rejected(self):
        """倾斜的车道线不能被几何约束误杀。

        这是回归测试。车辆偏离车道中心时，车道线在鸟瞰图中是**倾斜**的，
        其包围盒会很宽（实测可达 114 px），但填充率仍然很低（约 0.27）。
        早期版本用「包围盒宽度 < 90 px」做判据，把这条真实车道线整条拒绝，
        导致横向偏差误差达 25 px。改用填充率判据后，两个方向都正确。
        """
        img = make_synthetic_lane_image(
            width=self.width, height=self.height,
            left_x_bottom=150, left_x_top=250,
            right_x_bottom=450, right_x_top=350,
            noise_std=0.0,
        )
        res = LaneDetector(self.cfg).detect(img)
        self.assertTrue(
            res.detected,
            f"倾斜车道未检出两侧 (L={res.left_pixels}, R={res.right_pixels})",
        )
        # 两侧都应有足量像素参与拟合（而非一侧被几何约束拒绝）
        self.assertGreater(res.left_pixels, 1000)
        self.assertGreater(res.right_pixels, 1000)
        # 该构型下真值车道中心在图像中心左侧 100 px（源图尺度），
        # warped 尺度约 0.725，横向偏差应约为 (100 * 0.725) * 0.05 = 3.6 m
        self.assertAlmostEqual(res.lateral_offset_m, 3.63, delta=0.6)

    def test_lane_exactly_symmetric_gives_small_offset(self):
        """左右完全对称的车道 -> 横向偏差应接近 0。"""
        img = make_synthetic_lane_image(
            width=self.width, height=self.height,
            left_x_bottom=250, left_x_top=350,
            right_x_bottom=550, right_x_top=450,
            noise_std=0.0,
        )
        det = LaneDetector(self.cfg)
        res = det.detect(img)
        self.assertTrue(res.detected)
        self.assertLess(abs(res.lateral_offset_m), 0.6)

    def test_shifting_lane_changes_offset_sign(self):
        """把整条车道向左移 -> 车辆相对偏右 -> 横向偏差应为更大的正值。"""
        base = make_synthetic_lane_image(
            width=self.width, height=self.height,
            left_x_bottom=250, left_x_top=350,
            right_x_bottom=550, right_x_top=450,
            noise_std=0.0,
        )
        shifted = make_synthetic_lane_image(
            width=self.width, height=self.height,
            left_x_bottom=150, left_x_top=250,
            right_x_bottom=450, right_x_top=350,
            noise_std=0.0,
        )
        res_base = LaneDetector(self.cfg).detect(base)
        res_shift = LaneDetector(self.cfg).detect(shifted)
        self.assertTrue(res_base.detected)
        self.assertTrue(res_shift.detected)
        self.assertGreater(res_shift.lateral_offset_m, res_base.lateral_offset_m)

    def test_missing_lane_reports_lower_confidence(self):
        """只有一条车道线时，置信度应明显低于两条都有。"""
        only_left = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        only_left[:, :] = 70

        img = make_synthetic_lane_image(
            width=self.width, height=self.height,
            left_x_bottom=250, left_x_top=350,
            right_x_bottom=550, right_x_top=450,
            noise_std=0.0,
        )
        # 抹掉右车道线区域
        img[:, 430:] = 70
        det = LaneDetector(self.cfg)
        res = det.detect(img)
        self.assertLess(res.confidence, 1.0)

    def test_dark_uniform_image_detects_nothing(self):
        img = np.full((self.height, self.width, 3), 60, dtype=np.uint8)
        det = LaneDetector(self.cfg)
        res = det.detect(img)
        self.assertFalse(res.detected)
        self.assertAlmostEqual(res.confidence, 0.0, places=6)

    def test_narrow_lane_still_detects_both_sides(self):
        """车道整体偏移/变窄时，两侧都必须仍然检出。

        这是回归测试：早期 `find_lane_bases` 有「两峰太近就抑制其一」的逻辑，
        加上透视梯形视野过窄，导致偏移后的右车道整条消失（R=0）。
        """
        img = make_synthetic_lane_image(
            width=self.width, height=self.height,
            left_x_bottom=150, left_x_top=250,
            right_x_bottom=450, right_x_top=350,
            noise_std=0.0,
        )
        res = LaneDetector(self.cfg).detect(img)
        self.assertTrue(
            res.detected,
            f"偏移后的车道未检出两侧 (L={res.left_pixels}, R={res.right_pixels})",
        )
        self.assertGreater(res.left_pixels, 500)
        self.assertGreater(res.right_pixels, 500)

    def test_warped_mask_shape_matches_input(self):
        img = make_synthetic_lane_image(width=self.width, height=self.height)
        det = LaneDetector(self.cfg)
        mask = det.warped_mask(img)
        self.assertEqual(mask.shape, (self.height, self.width))
        self.assertEqual(mask.dtype, bool)

    def test_detector_rebuilds_homography_on_size_change(self):
        """输入尺寸变化时单应矩阵必须重建，否则会错位。"""
        det = LaneDetector(self.cfg)
        img_small = make_synthetic_lane_image(width=400, height=300)
        res_small = det.detect(img_small)
        self.assertEqual(det.image_size, (400, 300))
        img_big = make_synthetic_lane_image(width=self.width, height=self.height)
        det.detect(img_big)
        self.assertEqual(det.image_size, (self.width, self.height))
        self.assertIsNotNone(res_small)


class TestPolyHelpers(unittest.TestCase):
    def test_evaluate_poly_matches_numpy(self):
        coeffs = np.array([0.01, -2.0, 300.0])
        v = np.array([0.0, 50.0, 100.0])
        np.testing.assert_allclose(evaluate_poly(coeffs, v), np.polyval(coeffs, v))

    def test_evaluate_poly_constant(self):
        np.testing.assert_allclose(evaluate_poly(np.array([5.0]), np.array([1.0, 2.0])), 5.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
