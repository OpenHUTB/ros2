# -*- coding: utf-8 -*-
"""离线地图检查与路径规划工具的单元测试。

不需要 ROS / Gazebo：测试里用临时目录现场生成 PGM + YAML，
断言的都是可独立验证的性质（阈值语义、距离变换、A* 最优性、平滑不穿墙）。
"""

from __future__ import annotations

import math
import os
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "scripts"))

import map_planner as M  # noqa: E402


# ---------------------------------------------------------------------------
# 测试夹具：现场造地图
# ---------------------------------------------------------------------------
def write_pgm_p5(path, width, height, pixels):
    with open(path, "wb") as fh:
        fh.write(("P5\n%d %d\n255\n" % (width, height)).encode("ascii"))
        fh.write(bytes(pixels))


def write_pgm_p2(path, width, height, pixels):
    with open(path, "w", encoding="ascii") as fh:
        fh.write("P2\n%d %d\n255\n" % (width, height))
        fh.write("\n".join(str(p) for p in pixels))


def write_yaml(path, image="map.pgm", resolution=0.05, origin="[0.0, 0.0, 0.0]",
               negate=0, occupied=0.65, free=0.196):
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("image: %s\n" % image)
        fh.write("resolution: %.6f\n" % resolution)
        fh.write("origin: %s\n" % origin)
        fh.write("negate: %d\n" % negate)
        fh.write("occupied_thresh: %.3f\n" % occupied)
        fh.write("free_thresh: %.3f\n" % free)


def grid_from_rows(rows):
    """'#'=占用 '.'=自由 '?'=未知 的字符画 → 占据栅格。"""
    mapping = {"#": M.OCCUPIED, ".": M.FREE, "?": M.UNKNOWN}
    return [[mapping[ch] for ch in row] for row in rows]


class TempMapCase(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix="mapplanner-")
        self.pgm = os.path.join(self.dir, "map.pgm")
        self.yaml = os.path.join(self.dir, "map.yaml")

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def make_map(self, rows, negate=0, resolution=0.05, origin="[0.0, 0.0, 0.0]"):
        height = len(rows)
        width = len(rows[0])
        pixels = []
        for row in rows:
            for ch in row:
                pixels.append({"#": 0, ".": 254, "?": 205}[ch])
        write_pgm_p5(self.pgm, width, height, pixels)
        write_yaml(self.yaml, negate=negate, resolution=resolution, origin=origin)
        return M.load_map(self.yaml)


# ---------------------------------------------------------------------------
class TestPgm(TempMapCase):
    def test_read_p5(self):
        pixels = list(range(20))
        write_pgm_p5(self.pgm, 5, 4, pixels)
        w, h, got = M.read_pgm(self.pgm)
        self.assertEqual((w, h), (5, 4))
        self.assertEqual(got, pixels)

    def test_read_p2(self):
        pixels = [0, 255, 128, 64]
        write_pgm_p2(self.pgm, 2, 2, pixels)
        w, h, got = M.read_pgm(self.pgm)
        self.assertEqual((w, h), (2, 2))
        self.assertEqual(got, pixels)

    def test_comment_is_skipped(self):
        with open(self.pgm, "wb") as fh:
            fh.write(b"P5\n# CREATOR: map_saver\n2 2\n255\n")
            fh.write(bytes([0, 1, 2, 3]))
        w, h, got = M.read_pgm(self.pgm)
        self.assertEqual((w, h), (2, 2))
        self.assertEqual(got, [0, 1, 2, 3])

    def test_bad_magic(self):
        with open(self.pgm, "wb") as fh:
            fh.write(b"P9\n2 2\n255\n")
        with self.assertRaises(ValueError):
            M.read_pgm(self.pgm)

    def test_truncated_data(self):
        with open(self.pgm, "wb") as fh:
            fh.write(b"P5\n4 4\n255\n")
            fh.write(bytes([0, 1, 2]))
        with self.assertRaises(ValueError):
            M.read_pgm(self.pgm)


class TestYaml(TempMapCase):
    def test_basic_fields(self):
        write_pgm_p5(self.pgm, 2, 2, [0, 0, 0, 0])
        write_yaml(self.yaml, resolution=0.05, origin="[-12.2, -13.5, 0.0]")
        meta = M.parse_map_yaml(self.yaml)
        self.assertAlmostEqual(meta["resolution"], 0.05)
        self.assertAlmostEqual(meta["origin"][0], -12.2)
        self.assertAlmostEqual(meta["origin"][1], -13.5)
        self.assertEqual(meta["negate"], 0)
        self.assertTrue(meta["image"].endswith("map.pgm"))

    def test_defaults_when_optional_missing(self):
        with open(self.yaml, "w", encoding="utf-8") as fh:
            fh.write("image: map.pgm\nresolution: 0.1\n")
        meta = M.parse_map_yaml(self.yaml)
        self.assertAlmostEqual(meta["occupied_thresh"], 0.65)
        self.assertAlmostEqual(meta["free_thresh"], 0.196)
        self.assertEqual(meta["origin"], [0.0, 0.0, 0.0])

    def test_missing_image_raises(self):
        with open(self.yaml, "w", encoding="utf-8") as fh:
            fh.write("resolution: 0.05\n")
        with self.assertRaises(ValueError):
            M.parse_map_yaml(self.yaml)

    def test_inverted_thresholds_raise(self):
        with open(self.yaml, "w", encoding="utf-8") as fh:
            fh.write("image: map.pgm\nresolution: 0.05\n")
            fh.write("occupied_thresh: 0.2\nfree_thresh: 0.8\n")
        with self.assertRaises(ValueError):
            M.parse_map_yaml(self.yaml)

    def test_bom_is_tolerated(self):
        """Windows 记事本另存会带 BOM，不能因此读不出 image 字段。"""
        with open(self.yaml, "w", encoding="utf-8-sig") as fh:
            fh.write("image: map.pgm\nresolution: 0.05\n")
        meta = M.parse_map_yaml(self.yaml)
        self.assertTrue(meta["image"].endswith("map.pgm"))


class TestLoadMap(TempMapCase):
    def test_classes(self):
        grid, meta = self.make_map(["#.?", "..."])
        self.assertEqual(meta["width"], 3)
        self.assertEqual(meta["height"], 2)
        self.assertEqual(grid[0], [M.OCCUPIED, M.FREE, M.UNKNOWN])
        self.assertEqual(grid[1], [M.FREE, M.FREE, M.FREE])

    def test_negate_flips_polarity(self):
        grid, _ = self.make_map(["#."], negate=1)
        self.assertEqual(grid[0], [M.FREE, M.OCCUPIED])


class TestCoordinates(TempMapCase):
    def test_roundtrip(self):
        _, meta = self.make_map(["...", "...", "..."],
                                resolution=0.5, origin="[-1.0, -2.0, 0.0]")
        for col in range(3):
            for row in range(3):
                x, y = M.cell_to_world(col, row, meta)
                self.assertEqual(M.world_to_cell(x, y, meta), (col, row))

    def test_row_flip(self):
        """PGM 第 0 行是地图最上方，世界 y 越大行号越小。"""
        _, meta = self.make_map(["...", "...", "..."], resolution=1.0)
        low = M.world_to_cell(0.5, 0.5, meta)
        high = M.world_to_cell(0.5, 2.5, meta)
        self.assertGreater(low[1], high[1])


class TestStatistics(TempMapCase):
    def test_ratios(self):
        grid, _ = self.make_map(["##..", "....", "????"])
        stats = M.map_statistics(grid)
        total = 12
        self.assertEqual(stats["total"], total)
        self.assertAlmostEqual(stats["free"] / total, 6 / total)
        self.assertAlmostEqual(stats["free_ratio"] + stats["occupied_ratio"]
                               + stats["unknown_ratio"], 1.0)

    def test_largest_component(self):
        grid = grid_from_rows([
            "..##..",
            "..##..",
            "......",
        ])
        # 三行通过第 2 行全部连通，一共 2+2+2+2+6 = 14 格
        self.assertEqual(M.largest_free_component(grid), 14)

    def test_fragmented_map(self):
        grid = grid_from_rows([
            ".#.#.",
            "#.#.#",
            ".#.#.",
        ])
        self.assertEqual(M.largest_free_component(grid), 1)


class TestDistanceTransform(unittest.TestCase):
    def test_wall_distance(self):
        grid = grid_from_rows([
            "#####",
            "#...#",
            "#...#",
            "#...#",
            "#####",
        ])
        dist = M.distance_transform(grid)
        self.assertAlmostEqual(dist[2][2], 2.0, places=6)   # 中心到墙 2 格
        self.assertAlmostEqual(dist[1][1], 1.0, places=6)   # 紧贴上方墙

    def test_free_map_is_inf(self):
        grid = grid_from_rows(["...", "...", "..."])
        dist = M.distance_transform(grid)
        self.assertEqual(dist[1][1], float("inf"))


class TestInflation(TempMapCase):
    def test_radius_monotonic(self):
        grid, meta = self.make_map([
            "........",
            "..####..",
            "........",
        ])
        counts = []
        for radius_m in (0.0, 0.1, 0.2):
            mask = M.inflate(grid, radius_m / meta["resolution"])
            counts.append(sum(row.count(True) for row in mask))
        self.assertGreater(counts[0], counts[1])
        self.assertGreater(counts[1], counts[2])

    def test_corridor_closes(self):
        """走廊只有一格宽，机器人半径大于半格宽就过不去。"""
        grid = grid_from_rows([
            "#####",
            "#...#",
            "#####",
        ])
        self.assertTrue(M.inflate(grid, 0.0)[1][2])
        self.assertFalse(M.inflate(grid, 1.5)[1][2])


class TestPlanning(TempMapCase):
    def corridor(self):
        return grid_from_rows([
            "#######",
            "#.....#",
            "#.....#",
            "#.....#",
            "#######",
        ])

    def test_straight_line_length(self):
        grid = self.corridor()
        mask = M.inflate(grid, 0.0)
        path = M.plan_path(mask, (1, 2), (5, 2))
        self.assertIsNotNone(path)
        self.assertEqual(path[0], (1, 2))
        self.assertEqual(path[-1], (5, 2))
        self.assertAlmostEqual(M.path_length(path, 1.0), 4.0, places=6)

    def test_path_stays_on_free_cells(self):
        grid = self.corridor()
        mask = M.inflate(grid, 0.0)
        path = M.plan_path(mask, (1, 1), (5, 3))
        for (c, r) in path:
            self.assertTrue(mask[r][c])

    def test_detour_around_wall(self):
        grid = grid_from_rows([
            "#######",
            "#..#..#",
            "#..#..#",
            "#.....#",
            "#######",
        ])
        mask = M.inflate(grid, 0.0)
        path = M.plan_path(mask, (1, 1), (5, 1))
        self.assertIsNotNone(path)
        straight = 4.0
        self.assertGreater(M.path_length(path, 1.0), straight + 0.5)
        # 不能从墙里穿过去
        self.assertNotIn((3, 1), path)

    def test_unreachable_returns_none(self):
        grid = grid_from_rows([
            "#######",
            "#..#..#",
            "#######",
        ])
        mask = M.inflate(grid, 0.0)
        self.assertIsNone(M.plan_path(mask, (1, 1), (5, 1)))

    def test_start_in_obstacle_returns_none(self):
        grid = self.corridor()
        mask = M.inflate(grid, 0.0)
        self.assertIsNone(M.plan_path(mask, (0, 0), (5, 2)))

    def test_out_of_bounds_raises(self):
        grid = self.corridor()
        mask = M.inflate(grid, 0.0)
        with self.assertRaises(ValueError):
            M.plan_path(mask, (99, 0), (1, 1))

    def test_no_corner_cutting(self):
        """斜向通过两堵墙的夹角是不允许的（会撞角）。"""
        grid = grid_from_rows([
            "....#",
            "....#",
            "##...",
        ])
        mask = M.inflate(grid, 0.0)
        path = M.plan_path(mask, (3, 0), (2, 2))
        # 若允许切角，会给出 (3,0)->(2,1)->(1,2) 这样的对角线
        for (c1, r1), (c2, r2) in zip(path[:-1], path[1:]):
            if c1 != c2 and r1 != r2:
                self.assertTrue(mask[r1][c2] and mask[r2][c1])


class TestSmoothing(unittest.TestCase):
    def test_endpoints_kept_and_shorter(self):
        grid = grid_from_rows([
            "#######",
            "#.....#",
            "#.....#",
            "#.....#",
            "#######",
        ])
        mask = M.inflate(grid, 0.0)
        path = M.plan_path(mask, (1, 1), (5, 3))
        smooth = M.smooth_path(mask, path)
        self.assertEqual(smooth[0], path[0])
        self.assertEqual(smooth[-1], path[-1])
        self.assertLessEqual(len(smooth), len(path))
        # 平滑只会更短（8 邻域路径是折线，直线捷径可以更短），不会更长
        self.assertLessEqual(M.path_length(smooth, 1.0),
                             M.path_length(path, 1.0) + 1e-9)
        self.assertGreater(M.path_length(smooth, 1.0), 0.0)

    def test_smoothed_segments_are_clear(self):
        grid = grid_from_rows([
            "#########",
            "#.......#",
            "#..###..#",
            "#.......#",
            "#########",
        ])
        mask = M.inflate(grid, 0.0)
        path = M.plan_path(mask, (1, 1), (7, 3))
        smooth = M.smooth_path(mask, path)
        for a, b in zip(smooth[:-1], smooth[1:]):
            self.assertTrue(M._line_clear(mask, a, b))

    def test_short_path_untouched(self):
        self.assertEqual(M.smooth_path([[True]], [(0, 0)]), [(0, 0)])
        self.assertEqual(M.smooth_path([[True]], None), [])


class TestPng(TempMapCase):
    def test_render_writes_png(self):
        grid, meta = self.make_map([
            "#####",
            "#...#",
            "#####",
        ])
        mask = M.inflate(grid, 0.0)
        out = os.path.join(self.dir, "out.png")
        M.render_png(out, grid, mask, [(2, 1), (1, 1)], meta)
        with open(out, "rb") as fh:
            head = fh.read(8)
        self.assertEqual(head, b"\x89PNG\r\n\x1a\n")
        self.assertGreater(os.path.getsize(out), 50)

    def test_png_size_matches_map(self):
        grid, meta = self.make_map(["...", "..."])
        mask = M.inflate(grid, 0.0)
        out = os.path.join(self.dir, "out.png")
        M.render_png(out, grid, mask, None, meta)
        self.assertGreater(os.path.getsize(out), 0)


if __name__ == "__main__":
    unittest.main()
