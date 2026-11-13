"""coverage（牛耕式全覆盖）的离线单元测试。"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

import coverage


class TestPolygon(unittest.TestCase):
    def test_l_shape_area(self):
        self.assertAlmostEqual(coverage.polygon_area(coverage.ROOM_L), 600.0)

    def test_point_in_polygon(self):
        poly = coverage.ROOM_L
        self.assertTrue(coverage.point_in_polygon((10.0, 10.0), poly))
        self.assertTrue(coverage.point_in_polygon((10.0, 30.0), poly))
        self.assertFalse(coverage.point_in_polygon((30.0, 30.0), poly))   # 缺角
        self.assertTrue(coverage.point_in_polygon((5.0, 5.0), poly))      # 顶点
        self.assertTrue(coverage.point_in_polygon((20.0, 20.0), poly))    # 凹角点

    def test_area_sign_independent(self):
        r1 = coverage.plan(coverage.ROOM_L, gap=2.0)
        r2 = coverage.plan(list(reversed(coverage.ROOM_L)), gap=2.0)
        self.assertAlmostEqual(r1["length"], r2["length"], delta=1e-6)
        self.assertEqual(r1["rows"], r2["rows"])


class TestPlan(unittest.TestCase):
    def test_rectangle_rows_and_length(self):
        room = [(0.0, 0.0), (30.0, 0.0), (30.0, 10.0), (0.0, 10.0)]
        r = coverage.plan(room, gap=2.0)
        self.assertEqual(r["rows"], 5)                       # 10 / 2
        # 长度预算：面积约 / gap（300/2=150），允许掉头开销
        self.assertGreater(r["length"], 145.0)
        self.assertLess(r["length"], 190.0)

    def test_rectangle_full_coverage(self):
        room = [(0.0, 0.0), (30.0, 0.0), (30.0, 10.0), (0.0, 10.0)]
        r = coverage.plan(room, gap=2.0)
        cov = coverage.coverage_stats(room, r["path"], step=0.4)
        self.assertGreaterEqual(cov, 0.99)

    def test_l_shape_coverage(self):
        r = coverage.plan(coverage.ROOM_L, gap=2.0)
        cov = coverage.coverage_stats(coverage.ROOM_L, r["path"], step=0.4)
        self.assertGreaterEqual(cov, 0.98)

    def test_l_shape_path_stays_inside(self):
        r = coverage.plan(coverage.ROOM_L, gap=2.0)
        for p in r["path"]:
            self.assertTrue(coverage.point_in_polygon(p, coverage.ROOM_L),
                            msg=str(p))

    def test_zigzag_alternates(self):
        room = [(0.0, 0.0), (30.0, 0.0), (30.0, 10.0), (0.0, 10.0)]
        r = coverage.plan(room, gap=2.0)
        # 提取扫掠行（长边线段）的方向并断言交替
        dirs = []
        p = r["path"]
        for i in range(len(p) - 1):
            if abs(p[i + 1][1] - p[i][1]) < 1e-9 and abs(p[i + 1][0] - p[i][0]) > 5.0:
                dirs.append(1 if p[i + 1][0] > p[i][0] else -1)
        self.assertGreater(len(dirs), 2)
        for a, b in zip(dirs, dirs[1:]):
            self.assertEqual(a, -b)

    def test_smaller_gap_more_length(self):
        r2 = coverage.plan(coverage.ROOM_L, gap=2.0)
        r1 = coverage.plan(coverage.ROOM_L, gap=1.0)
        self.assertGreater(r1["length"], r2["length"] * 1.8)
        self.assertLess(r1["length"], r2["length"] * 2.4)

    def test_sweep_row_ordered_spans(self):
        for y in (6.0, 15.5, 25.0, 34.0):
            for span in coverage.sweep_row(y, coverage.ROOM_L):
                self.assertLess(span[0][0], span[1][0])
                self.assertAlmostEqual(span[0][1], span[1][1])

    def test_est_time_equals_length(self):
        r = coverage.plan(coverage.ROOM_L, gap=2.0)
        self.assertAlmostEqual(r["est_time"], r["length"])


class TestTracking(unittest.TestCase):
    """plan_control_node.track_cmd 的闭环回放（无 ROS，直接积分运动学）。

    牛耕式覆盖路径掉头密集（90 度转弯），跟踪器必须在限速限转下把它走完，
    且全程不出房间边界。
    """

    def _replay(self, path, v=2.0, rate=20.0, t_max=420.0):
        import plan_control_node as pcn
        x, y, yaw = path[0][0], path[0][1], 0.0
        idx = 0
        dt = 1.0 / rate
        t = 0.0
        trace = [(x, y)]
        while t < t_max:
            v_, w, idx, done = pcn.track_cmd(path, (x, y, yaw), idx, speed=v)
            if done:
                return True, t, trace
            yaw += w * dt
            x += v_ * math.cos(yaw) * dt
            y += v_ * math.sin(yaw) * dt
            t += dt
            trace.append((x, y))
        return False, t, trace

    def test_replay_finishes_coverage(self):
        r = coverage.plan(coverage.ROOM_L, gap=2.0)
        ok, t, _ = self._replay(r["path"])
        self.assertTrue(ok)
        self.assertLess(t, 400.0)          # 路径 343 m + 掉头减速，2 m/s 足够

    def test_replay_stays_inside_room(self):
        r = coverage.plan(coverage.ROOM_L, gap=2.0)
        ok, _, trace = self._replay(r["path"])
        self.assertTrue(ok)
        for (x, y) in trace:
            # 路径本身贴边扫掠（首末行距边界 gap/2=1 m），回放在凹角切角
            # 时允许 1 m 出界余量（实测最大越界约 0.6 m，出现在 L 形凹角）
            inside = coverage.point_in_polygon((x, y), coverage.ROOM_L)
            if not inside:
                d = min(coverage.dist_to_segment((x, y), coverage.ROOM_L[i],
                                                  coverage.ROOM_L[(i + 1) % 6])
                        for i in range(6))
                self.assertLess(d, 1.0,
                                msg="回放轨迹远离房间边界 %.2f m @ (%.2f, %.2f)"
                                    % (d, x, y))


class TestDegenerateCases(unittest.TestCase):
    """评审 #1400 补充的边界回归：薄房间与退化扫掠方向。"""

    def test_thin_room_returns_empty_path(self):
        # 房间比行距还窄：没有可扫的行，返回空路径而不是参数错位崩溃
        # （旧版 _result 多传一个 0.0，poly 被顶成 float 后 TypeError）
        r = coverage.plan([(0.0, 0.0), (10.0, 0.0), (10.0, 0.5), (0.0, 0.5)],
                          gap=2.0)
        self.assertEqual(r["path"], [])
        self.assertEqual(r["rows"], 0)
        self.assertEqual(r["length"], 0.0)

    def test_theta_90_full_coverage_not_degenerate(self):
        # 旧版行中心恰好压在 x=20 内墙上，sweep_row 跳过共线水平边后该行
        # 只扫半段，漏掉 3% 还伪装成"最短方向"。修后（行起点 +epsilon
        # 偏移）：覆盖率 100%，长度不再短于 0°。
        r0 = coverage.plan(coverage.ROOM_L, gap=2.0, theta=0.0)
        r90 = coverage.plan(coverage.ROOM_L, gap=2.0, theta=math.radians(90))
        cov90 = coverage.coverage_stats(coverage.ROOM_L, r90["path"], step=0.5)
        self.assertGreaterEqual(cov90, 0.98,
                                msg="theta=90° 覆盖率 %.3f" % cov90)
        self.assertGreater(r90["length"], r0["length"],
                           msg="theta=90° 不应比 0° 更短（退化伪最优回归）")

    def test_gap_beyond_tool_width_leaves_strips(self):
        # 评审 #1400 复核：旧版判定半径 = gap*0.55 随 gap 一起放大，
        # 行距拉到 5 m 覆盖率仍"自证"100%，指标看不见漏扫。现改为与
        # 固定作业宽度绑定（w=2.2 m）：行距超过作业宽度后行间出现宽
        # gap-w 的漏扫带，覆盖率必须显著低于 100%。
        r5 = coverage.plan(coverage.ROOM_L, gap=5.0)
        cov5 = coverage.coverage_stats(coverage.ROOM_L, r5["path"], step=0.5)
        self.assertLess(cov5, 0.6, msg="gap=5 m 覆盖率 %.3f 应显著低于 100%%" % cov5)
        self.assertGreater(cov5, 0.3)
        r2 = coverage.plan(coverage.ROOM_L, gap=2.0)
        cov2 = coverage.coverage_stats(coverage.ROOM_L, r2["path"], step=0.5)
        self.assertGreaterEqual(cov2, 0.999, msg="gap=2 m 覆盖率 %.3f 应为 100%%" % cov2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
