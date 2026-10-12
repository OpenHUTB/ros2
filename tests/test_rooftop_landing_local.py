#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rooftop_landing 本地回归测试（**无需 ROS / 仿真器 / GPU**）。

运行::

    python3 tests/test_rooftop_landing_local.py

覆盖四类内容：

* **点云解析** —— PointCloud2 字段 -> (N,3)，含缺字段、空数据等异常分支；
* **几何算法** —— 平面拟合、水平性判定、凸包、面积、polylabel 最大内切圆；
* **端到端感知** —— 合成「地面 + 楼房 + 墙面 + 离群点」场景，验证能挑出屋顶并
  给出合理的降落点；
* **工程文件** —— 目录结构、入口命名、launch/package/CMake 等约定是否满足作业要求。
"""
from __future__ import annotations

import math
import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, os.pardir))
PKG = os.path.join(REPO, "src", "air", "rooftop_landing")
sys.path.insert(0, os.path.join(PKG, "scripts"))

import frames                      # noqa: E402
import rooftop_perception as rp    # noqa: E402

PASS, FAIL = [], []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print("%s %s%s" % ("[通过]" if cond else "[失败]", name,
                       ("  -- " + detail) if detail and not cond else ""))


class _Field(object):
    def __init__(self, name, offset, datatype):
        self.name = name
        self.offset = offset
        self.datatype = datatype


def build_pointcloud2(xyz):
    """把 (N,3) 数组打包成 PointCloud2 风格的 (fields, point_step, data)。"""
    fields = [_Field("x", 0, 7), _Field("y", 4, 7), _Field("z", 8, 7)]
    data = bytearray()
    for p in xyz:
        data += struct.pack("<fff", float(p[0]), float(p[1]), float(p[2]))
    return fields, 12, bytes(data)


# --------------------------------------------------------------------------- #
# 1. 点云解析
# --------------------------------------------------------------------------- #
def test_pointcloud_parsing():
    src = np.array([[1.0, 2.0, 3.0], [-4.0, 5.5, -6.25]], dtype=np.float32)
    fields, step, data = build_pointcloud2(src)
    out = frames.pointcloud2_to_xyz(fields, step, data, width=2, height=1)
    check("PointCloud2 解析出 2 个点", len(out) == 2, str(out))
    check("PointCloud2 数值正确", np.allclose(out, src, atol=1e-5), str(out))

    out0 = frames.pointcloud2_to_xyz([], 12, b"")
    check("缺字段时返回空数组而不报错", out0.shape == (0, 3), str(out0.shape))
    out1 = frames.pointcloud2_to_xyz(fields, 12, b"")
    check("空数据返回空数组", out1.shape == (0, 3), str(out1.shape))

    pts = np.array([[0, 0, 0], [np.nan, 1, 1], [2, np.inf, 2], [3, 3, 3]])
    check("NaN/Inf 被过滤", len(frames.filter_finite_xyz(pts)) == 2)


def test_frame_conversions():
    enu = np.array([1.0, 2.0, 3.0])
    ned = frames.enu_to_ned(enu)
    check("ENU->NED 公式正确", np.allclose(ned, [2.0, 1.0, -3.0]), str(ned))
    check("NED->ENU 往返一致", np.allclose(frames.ned_to_enu(ned), enu))
    check("altitude(z_up=True) 直通", frames.altitude(-7.0, True) == -7.0)
    check("altitude(z_up=False) 取反", frames.altitude(-7.0, False) == 7.0)
    q = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))  # 绕 z 转 90°
    check("四元数取偏航≈90°",
          abs(frames.yaw_from_quaternion(q) - math.pi / 2) < 1e-6)


# --------------------------------------------------------------------------- #
# 2. 几何算法
# --------------------------------------------------------------------------- #
def test_geometry():
    plane_pts = np.array([[x, y, 10.0 + np.random.default_rng(1).normal(0, 0.02)]
                          for x in np.linspace(0, 20, 30)
                          for y in np.linspace(0, 20, 30)])
    out = rp.fit_plane_ransac(plane_pts, rng=np.random.default_rng(0))
    check("RANSAC 能拟合出平面", out is not None)
    if out:
        check("拟合平面水平（倾角≈0°）", rp.plane_tilt_deg(out["normal"]) < 0.5,
              "%.3f°" % rp.plane_tilt_deg(out["normal"]))
        check("平面高度≈10 m", abs(rp.plane_height(out["normal"], out["d"]) - 10.0) < 0.05,
              "%.3f" % rp.plane_height(out["normal"], out["d"]))
    tilted = np.array([0.5, 0.0, 0.866])
    check("倾斜面倾角≈30°", abs(rp.plane_tilt_deg(tilted) - 30.0) < 0.5,
          "%.2f" % rp.plane_tilt_deg(tilted))

    square = np.array([[0, 0], [10, 0], [10, 10], [0, 10]])
    hull = rp.convex_hull_2d(np.vstack([square, [[5, 5], [3, 7]]]))
    check("凸包顶点数为 4", len(hull) == 4, str(len(hull)))
    check("凸包面积=100", abs(rp.polygon_area(hull) - 100.0) < 1e-6,
          str(rp.polygon_area(hull)))
    check("共线点凸包面积=0",
          rp.polygon_area(rp.convex_hull_2d(np.array([[0, 0], [1, 1], [2, 2]]))) == 0.0)

    (cx, cy), r = rp.polylabel([square], precision=0.05)
    check("polylabel 圆心≈(5,5)", abs(cx - 5) < 0.15 and abs(cy - 5) < 0.15,
          "(%.2f, %.2f)" % (cx, cy))
    check("polylabel 半径≈5", abs(r - 5.0) < 0.15, "%.3f" % r)

    d_in = rp.point_to_polygon_distance(5, 5, [square])
    d_out = rp.point_to_polygon_distance(-3, 5, [square])
    check("内部点距离为正", d_in > 0, str(d_in))
    check("外部点距离为负", d_out < 0, str(d_out))
    check("外部点距离绝对值≈3", abs(abs(d_out) - 3.0) < 1e-6, str(d_out))


# --------------------------------------------------------------------------- #
# 3. 端到端感知（合成屋顶场景）
# --------------------------------------------------------------------------- #
def make_city_scene():
    """地面 z=0 + 一栋楼（屋顶 z=15, x∈[10,30], y∈[5,25]）+ 墙面 + 离群点。"""
    rng = np.random.default_rng(2024)
    ground = np.column_stack([
        rng.uniform(-20, 60, 4000), rng.uniform(-20, 60, 4000),
        rng.normal(0.0, 0.02, 4000)])
    roof = np.column_stack([
        rng.uniform(11, 29, 1500), rng.uniform(6, 24, 1500),
        rng.normal(15.0, 0.02, 1500)])
    walls = []
    for _ in range(500):
        t = rng.uniform(0, 1)
        z = rng.uniform(0, 15)
        walls.append([10.0 + rng.normal(0, 0.05), 5 + t * 20, z])
        walls.append([30.0 + rng.normal(0, 0.05), 5 + t * 20, z])
    outliers = np.column_stack([
        rng.uniform(-50, 100, 200), rng.uniform(-50, 100, 200),
        rng.uniform(-30, 60, 200)])
    return np.vstack([ground, roof, np.array(walls), outliers])


def test_detect_rooftops():
    pts = make_city_scene()
    roofs = rp.detect_rooftops(pts, min_area=8.0, min_inliers=120,
                               rng=np.random.default_rng(0))
    check("检测到候选水平面", len(roofs) >= 1, "找到 %d 个" % len(roofs))
    if not roofs:
        return
    top = roofs[0]
    check("最高面高度≈15 m", abs(top["height"] - 15.0) < 0.3,
          "%.2f" % top["height"])
    check("屋顶面积接近 18x18", 250 < top["area"] < 400, "%.1f" % top["area"])
    lx, ly = top["landing_point"]
    check("降落点在屋顶内 x∈[10,30]", 10 <= lx <= 30, "%.2f" % lx)
    check("降落点在屋顶内 y∈[5,25]", 5 <= ly <= 25, "%.2f" % ly)
    check("降落点接近屋顶中心(20,15)",
          abs(lx - 20) < 3.5 and abs(ly - 15) < 3.5, "(%.2f, %.2f)" % (lx, ly))
    check("内切圆半径≈9 m", 7.0 < top["radius"] < 10.0, "%.2f" % top["radius"])

    target = rp.select_landing_target(roofs, current_xy=(0.0, 0.0),
                                      prefer="closest")
    check("closest 策略能选到目标", target is not None)
    if target:
        check("选中的是最高的屋顶", abs(target["height"] - top["height"]) < 1e-6)
    check("空输入返回 None", rp.select_landing_target([]) is None)
    check("空点云返回空列表", rp.detect_rooftops(np.zeros((0, 3))) == [])
    check("点太少返回空列表", rp.detect_rooftops(np.zeros((5, 3))) == [])


# --------------------------------------------------------------------------- #
# 4. 工程文件约定（作业要求）
# --------------------------------------------------------------------------- #
def _read(path):
    """读文本文件，缺失时返回空串（保证测试不会因缺文件而中断）。"""
    try:
        with open(path, encoding="utf-8") as fh:
            return fh.read()
    except OSError:
        return ""


def test_project_layout():
    must = ["package.xml", "CMakeLists.txt", "README.md",
            "launch/main.launch", "scripts/main.py", "config/rooftop_landing.yaml"]
    for rel in must:
        check("存在 %s" % rel, os.path.isfile(os.path.join(PKG, rel)))

    pkg = _read(os.path.join(PKG, "package.xml"))
    check("package.xml 声明了 <name>rooftop_landing</name>",
          "<name>rooftop_landing</name>" in pkg)
    check("package.xml 依赖 rospy", "rospy" in pkg)

    cml = _read(os.path.join(PKG, "CMakeLists.txt"))
    check("CMakeLists 安装 main.py 为可执行", "scripts/main.py" in cml)
    check("CMakeLists 安装 launch 目录", "launch" in cml)

    launch = _read(os.path.join(PKG, "launch", "main.launch"))
    check("launch 中节点入口为 main.py", 'type="main.py"' in launch)
    check("launch 支持 safe_altitude 参数", 'name="safe_altitude"' in launch)
    check("launch 支持 z_up 参数", 'name="z_up"' in launch)

    names = []
    for root, _dirs, files in os.walk(PKG):
        for f in files:
            names.append(f)
    check("所有文件名都是英文（无空格/中文）",
          all(all(ord(c) < 128 for c in n) and " " not in n for n in names),
          str([n for n in names if any(ord(c) >= 128 for c in n) or " " in n]))


def test_cluster_robustness():
    """凸包鲁棒性回归：平面上互不相连的点群必须先拆开再求凸包。

    不加这一步时，少数几个同高度的远处噪点就能把凸包撑开，
    「最大内切圆圆心」会落到真实屋顶之外 —— 实测：6900 个点里混进 4 个噪点，
    216 m² 的屋顶被撑成 1175 m²，降落点偏了 22 m（会掉到楼外的街上）。
    """
    rng = np.random.default_rng(7)
    roof = np.column_stack([
        rng.uniform(11, 29, 1500),
        rng.uniform(9, 21, 1500),
        rng.normal(15.0, 0.02, 1500)])
    noise = np.array([
        [-24.52, -4.05, 15.04],
        [-14.49, -4.11, 14.89],
        [6.12, -16.75, 14.94],
        [-6.92, -18.25, 14.98]])
    pts = np.vstack([roof, noise])

    roofs = rp.detect_rooftops(pts, rng=np.random.default_rng(0))
    check("混入同高度远处噪点后仍能检测到屋顶", len(roofs) >= 1,
          "检测到 %d 个" % len(roofs))
    if not roofs:
        return

    top = roofs[0]
    check("屋顶高度仍为 15 m", abs(top["height"] - 15.0) < 0.5,
          "%.2f" % top["height"])
    check("屋顶面积没有被噪点撑大（200~230 m^2）",
          200.0 <= top["area"] <= 230.0, "%.1f m^2" % top["area"])
    check("内切圆半径没被噪点放大（约 6 m）",
          abs(top["radius"] - 6.0) < 1.0, "%.2f m" % top["radius"])
    lx, ly = top["landing_point"]
    check("降落点仍在屋顶范围内 x in [11,29]", 11.0 <= lx <= 29.0, "x=%.2f" % lx)
    check("降落点仍在屋顶范围内 y in [9,21]", 9.0 <= ly <= 21.0, "y=%.2f" % ly)

    # 同高度但互不相连的两栋楼，应被拆成两个独立候选屋顶
    rng2 = np.random.default_rng(11)
    b1 = np.column_stack([rng2.uniform(0, 10, 1200), rng2.uniform(0, 10, 1200),
                          rng2.normal(12.0, 0.02, 1200)])
    b2 = np.column_stack([rng2.uniform(30, 40, 1200), rng2.uniform(0, 10, 1200),
                          rng2.normal(12.0, 0.02, 1200)])
    roofs2 = rp.detect_rooftops(np.vstack([b1, b2]),
                                rng=np.random.default_rng(0))
    check("同高度但不相连的两栋楼被拆成两个候选", len(roofs2) >= 2,
          "检测到 %d 个" % len(roofs2))


def main():
    print("=" * 68)
    print("rooftop_landing 本地回归测试（无需 ROS / 仿真器）")
    print("=" * 68)
    print("\n-- 1. 点云解析与坐标换算 --")
    test_pointcloud_parsing()
    test_frame_conversions()
    print("\n-- 2. 几何算法 --")
    test_geometry()
    print("\n-- 3. 端到端屋顶感知 --")
    test_detect_rooftops()
    print("\n-- 4. 工程文件约定 --")
    test_project_layout()
    print("\n-- 5. 凸包鲁棒性回归 --")
    test_cluster_robustness()

    total = len(PASS) + len(FAIL)
    print("\n" + "=" * 68)
    print("结果：%d/%d 通过" % (len(PASS), total))
    if FAIL:
        print("失败项：")
        for n in FAIL:
            print("  - %s" % n)
    print("=" * 68)
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
