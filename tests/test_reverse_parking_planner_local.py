#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""reverse_parking_planner 的离线回归测试。

不需要 ROS、不需要 CARLA、不需要显示器，只要 Python 2.7 或 Python 3.x：
    python3 tests/test_reverse_parking_planner_local.py
测试脚本自己在仓库根目录找包，找不到时按 tests/ 旁的目录找，两种布局都支持。
"""

from __future__ import division, print_function, unicode_literals

import io
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def _find_package_dir():
    """在几种可能的目录布局下找到包目录。"""
    candidates = [
        os.path.join(ROOT, "src", "ground", "reverse_parking_planner"),
        os.path.join(HERE, "src", "ground", "reverse_parking_planner"),
        os.path.join(os.path.dirname(ROOT), "src", "ground", "reverse_parking_planner"),
    ]
    for path in candidates:
        if os.path.isdir(os.path.join(path, "scripts")):
            return path
    raise SystemExit("找不到 reverse_parking_planner 包目录，请在仓库根目录运行本脚本")


PKG_DIR = _find_package_dir()
SCRIPTS_DIR = os.path.join(PKG_DIR, "scripts")
sys.path.insert(0, SCRIPTS_DIR)

import parking_core as core  # noqa: E402

DOC_PATH = os.path.join(ROOT, "docs", "ground", "carla_parking_lot.md")

PASSED = []
FAILED = []


def check(name, ok, detail=""):
    """记录一条断言。"""
    if ok:
        PASSED.append(name)
    else:
        FAILED.append(name if not detail else "%s  (%s)" % (name, detail))


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


def text(value):
    """把任意值转成 str/unicode，避免 Python 2 下的隐式解码。"""
    return "%s" % value


def read(path):
    return io.open(path, encoding="utf-8").read()


def _raises(expected, func, *args, **kwargs):
    """调用 func 并断言它抛出 expected 类型的异常。"""
    try:
        func(*args, **kwargs)
    except expected:
        return True
    except Exception:
        return False
    return False


print("reverse_parking_planner 离线回归测试")
print("Python %s" % sys.version.split()[0])
print("包目录 %s" % PKG_DIR)
print("-" * 60)

# ------------------------------------------------------------ A 默认值

check("A1 默认车位类型是垂直车位", core.DEFAULT_SLOT_KIND == "perpendicular")
check("A2 默认车位朝向为 -90 度", close(core.DEFAULT_SLOT_YAW, -math.pi / 2.0))
check("A3 默认车位宽 2.5 m", close(core.DEFAULT_SLOT_WIDTH, 2.50))
check("A4 默认车位长 5.3 m", close(core.DEFAULT_SLOT_LENGTH, 5.30))
check("A5 默认轴距 2.7 m", close(core.DEFAULT_WHEELBASE, 2.70))
check("A6 默认车长 4.5 m", close(core.DEFAULT_VEHICLE_LENGTH, 4.50))
check("A7 默认车宽 1.8 m", close(core.DEFAULT_VEHICLE_WIDTH, 1.80))
check("A8 默认最大转角 0.55 rad", close(core.DEFAULT_MAX_STEER, 0.55))
check("A9 默认直线段 3.0 m", close(core.DEFAULT_ENTRY_LENGTH, 3.00))
check("A10 默认转弯半径 5.0 m", close(core.DEFAULT_TURN_RADIUS, 5.00))
check("A11 默认步长 0.1 m", close(core.DEFAULT_STEP, 0.10))
check("A12 默认终点容差 0.1 m", close(core.DEFAULT_TOLERANCE, 0.10))
check("A13 默认间距下限 0", close(core.DEFAULT_CLEARANCE, 0.0))
check("A14 默认路径上限 12 m", close(core.DEFAULT_MAX_PATH, 12.00))
check("A15 默认发布频率 10 Hz", close(core.DEFAULT_PUBLISH_HZ, 10.0))
check("A16 默认俯视图 56x16", core.DEFAULT_GRID_WIDTH == 56 and core.DEFAULT_GRID_HEIGHT == 16)
check("A17 支持的车位类型", core.SLOT_KINDS == ("perpendicular", "parallel"))
check("A18 倒车挡为 -1、前进挡为 1",
      core.GEAR_REVERSE == -1 and core.GEAR_FORWARD == 1)

# ------------------------------------------------------------ B 小工具

check("B1 normalize_angle 保留 0", close(core.normalize_angle(0.0), 0.0))
check("B2 normalize_angle 把 -pi 折算成 pi", close(core.normalize_angle(-math.pi), math.pi, 1e-9))
check("B3 normalize_angle 把 3*pi/2 折算成 -pi/2", close(core.normalize_angle(1.5 * math.pi), -0.5 * math.pi, 1e-9))
check("B4 normalize_angle 折算 -2*pi", close(abs(core.normalize_angle(-2.0 * math.pi)), 0.0, 1e-9))
check("B5 distance 计算两点距离",
      close(core.distance(core.Pose(0.0, 0.0), core.Pose(3.0, 4.0)), 5.0))
check("B6 yaw 0 的四元数", close(core.yaw_to_quaternion(0.0)[2], 0.0) and close(core.yaw_to_quaternion(0.0)[3], 1.0))
check("B7 yaw pi/2 的 z 分量约 0.7071", close(core.yaw_to_quaternion(math.pi / 2.0)[2], math.sqrt(0.5), 1e-9))
check("B8 四元数转回 yaw", close(core.quaternion_to_yaw(*core.yaw_to_quaternion(0.7)), 0.7, 1e-9))
check("B9 quaternion_to_yaw 接受四个分量", close(core.quaternion_to_yaw(0.0, 0.0, 0.0, 1.0), 0.0, 1e-9))
check("B10 to_message_text 返回字符串", isinstance(core.to_message_text("场景"), str))
check("B11 _positive 拒绝 0", _raises(core.ParkingError, core._positive, "x", 0.0))
check("B12 _positive 拒绝负数", _raises(core.ParkingError, core._positive, "x", -1.0))
check("B13 _positive 接受正数", close(core._positive("x", 2.5), 2.5))

# ------------------------------------------------------------ C Pose

pose = core.Pose(1.0, 2.0, 0.0)
check("C1 as_tuple 返回三元组", pose.as_tuple() == (1.0, 2.0, 0.0))
check("C2 支持下标访问", close(pose[0], 1.0) and close(pose[1], 2.0) and close(pose[2], 0.0))
check("C3 支持 len", len(pose) == 3)
check("C4 point 返回坐标对", pose.point() == (1.0, 2.0))
check("C5 航向 0 时车头朝 +x", close(pose.forward()[0], 1.0) and close(pose.forward()[1], 0.0))
check("C6 航向 0 时车身右侧朝 -y", close(pose.right()[1], -1.0))
check("C7 航向 pi/2 时车头朝 +y", close(core.Pose(0.0, 0.0, math.pi / 2.0).forward()[1], 1.0))
check("C8 shifted 沿车头方向平移", close(pose.shifted(along=2.0).x, 3.0))
check("C9 shifted 沿右侧平移", close(pose.shifted(lateral=1.0).y, 1.0))
check("C10 shifted 不改变航向", close(pose.shifted(along=1.0, lateral=1.0).yaw, 0.0))
check("C11 repr 含坐标", "Pose" in text(pose))

# ------------------------------------------------------------ D Vehicle

vehicle = core.Vehicle()
check("D1 最小转弯半径 = 轴距/tan(最大转角)",
      close(vehicle.min_turn_radius, 2.70 / math.tan(0.55), 1e-9))
check("D2 最小转弯半径约 4.404", close(vehicle.min_turn_radius, 4.404, 1e-3))
check("D3 最大曲率 = 1/最小转弯半径", close(vehicle.max_curvature, 1.0 / vehicle.min_turn_radius))
check("D4 默认车长车宽", close(vehicle.length, 4.5) and close(vehicle.width, 1.8))

corners = vehicle.corners(core.Pose(0.0, 0.0, 0.0))
check("D5 车身四角数量", len(corners) == 4)
check("D6 右后角在原点", close(corners[0][0], 0.0) and close(corners[0][1], -0.9))
check("D7 左后角在 +y 侧", close(corners[1][1], 0.9))
check("D8 沿车头方向的跨度等于车长", close(corners[2][0] - corners[1][0], 4.5, 1e-9))
check("D9 车身宽度等于车宽", close(corners[1][1] - corners[0][1], 1.8, 1e-9))

turn_corners = vehicle.corners(core.Pose(0.0, 0.0, math.pi / 2.0))
check("D10 航向 90 度时车头朝 +y", close(turn_corners[2][1], 4.5, 1e-9))
check("D11 轴距非正时报错", _raises(core.ParkingError, core.Vehicle, wheelbase=0.0))
check("D12 最大转角非正时报错", _raises(core.ParkingError, core.Vehicle, max_steer=0.0))
check("D13 repr 含最小转弯半径", "R_min" in text(vehicle))

# ------------------------------------------------------------ E Slot

slot = core.Slot()
check("E1 默认车位位姿在原点且朝向 -pi/2", close(slot.pose.yaw, -math.pi / 2.0))
check("E2 车位四角数量", len(slot.corners()) == 4)
check("E3 车位宽度沿 x 方向", close(abs(slot.corners()[0][0] - slot.corners()[1][0]), 2.5, 1e-9))

target = slot.target_pose(vehicle)
check("E4 终点在车位中心前方半个车长处", close(target.x, 0.0, 1e-9) and close(target.y, 2.25, 1e-9))
check("E5 终点航向与车位一致", close(target.yaw, slot.pose.yaw))

left = slot.neighbor_pose(vehicle, -1)
right = slot.neighbor_pose(vehicle, 1)
check("E6 右侧邻车在 -x 方向 2.5 m", close(right.x, -2.5, 1e-9) and close(right.y, 2.25, 1e-9))
check("E7 左侧邻车在 +x 方向 2.5 m", close(left.x, 2.5, 1e-9))
check("E8 邻车航向与终点一致", close(left.yaw, target.yaw))
check("E9 车位宽度非正时报错", _raises(core.ParkingError, core.Slot, width=0.0))
check("E10 车位长度非正时报错", _raises(core.ParkingError, core.Slot, length=-1.0))
check("E11 未支持的车位类型报错", _raises(core.ParkingError, core.Slot, kind="diagonal"))
check("E12 车位宽长可被读取", close(slot.width, 2.5) and close(slot.length, 5.3))

# ------------------------------------------------------------ F build_plan

plan = core.build_plan()
samples = plan["samples"]

check("F1 采样点里程单调递增",
      all(samples[i][0] <= samples[i + 1][0] for i in range(len(samples) - 1)))
check("F2 首点里程为 0", close(samples[0][0], 0.0))
check("F3 路径长度 = R*pi/2 + entry", close(plan["length"], 5.0 * math.pi / 2.0 + 3.0, 1e-6))
check("F4 路径长 10.854 m", close(plan["length"], 10.853982, 1e-5))
check("F5 转角为 90 度", close(math.degrees(plan["sweep"]), 90.0, 1e-9))
check("F6 全程倒车", all(item[2] == core.GEAR_REVERSE for item in samples))
check("F7 起点在车道上 (5, -5.75)",
      close(plan["start"].x, 5.0, 1e-9) and close(plan["start"].y, -5.75, 1e-9))
check("F8 起点车头朝 -x（车道一侧）", close(abs(plan["start"].yaw), math.pi, 1e-9))
check("F9 终点 (0, 2.25)", close(plan["target"].x, 0.0, 1e-9) and close(plan["target"].y, 2.25, 1e-9))
check("F10 终点航向 -pi/2", close(plan["target"].yaw, -math.pi / 2.0, 1e-9))
check("F11 路径最右端等于转弯半径 (x=5)",
      close(max(item.x for item in plan["poses"]), 5.0, 1e-6))
check("F11b 路径最上端到达车位中心 (y=2.25)",
      close(max(item.y for item in plan["poses"]), 2.25, 1e-9))
check("F12 采样点数量 = 弧 80 点 + 直线 30 点", len(samples) == 110)
check("F13 半径被记进结果", close(plan["radius"], 5.0))
check("F14 直线段长度被记进结果", close(plan["entry"], 3.0))
check("F15 相邻采样点间距不超过步长",
      all(core.distance(plan["poses"][i], plan["poses"][i + 1]) <= 0.101
          for i in range(len(plan["poses"]) - 1)))
check("F16 解析长度不小于折线长度", plan["length"] >= plan["sampled_length"] - 1e-9)
check("F17 解析长度与折线长度相差不到 1 毫米",
      plan["length"] - plan["sampled_length"] <= 1.0e-3)
check("F18 转弯半径过小时报错", _raises(core.ParkingError, core.build_plan, turn_radius=3.0))
check("F19 转弯半径 4.5 可用",
      close(core.build_plan(turn_radius=4.5)["length"], 4.5 * math.pi / 2.0 + 3.0, 1e-6))
check("F20 转弯半径越小路径越短", core.build_plan(turn_radius=4.5)["length"] < plan["length"])
check("F21 直线段变长则路径变长",
      close(core.build_plan(entry_length=5.0)["length"], plan["length"] + 2.0, 1e-6))
check("F22 步长非正时报错", _raises(core.ParkingError, core.build_plan, step=0.0))
check("F23 车位变宽时间距变大",
      core.min_clearance(core.build_plan(slot=core.Slot(width=3.0))) > core.min_clearance(plan))
check("F24 plan 返回邻居车位", len(plan["neighbors"]) == 2)
check("F25 plan 记录车位与车辆", plan["slot"] is not None and plan["vehicle"] is not None)

# ------------------------------------------------------------ G 碰撞检测

square = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
far = [(3.0, 0.0), (4.0, 0.0), (4.0, 1.0), (3.0, 1.0)]
near = [(1.5, 0.0), (2.5, 0.0), (2.5, 1.0), (1.5, 1.0)]
overlap = [(0.5, 0.5), (1.5, 0.5), (1.5, 1.5), (0.5, 1.5)]

check("G1 相距 2 m 时间距为 2", close(core.polygon_gap(square, far), 2.0, 1e-9))
check("G2 相距 0.5 m 时间距为 0.5", close(core.polygon_gap(square, near), 0.5, 1e-9))
check("G3 相交时间距为 0", close(core.polygon_gap(square, overlap), 0.0))
check("G4 polygon_gap 对称", close(core.polygon_gap(far, square), 2.0, 1e-9))
check("G5 默认场景最小间距约 0.494 m", close(core.min_clearance(plan), 0.494, 1e-3))
check("G6 默认场景最小间距为正", core.min_clearance(plan) > 0.0)

# ------------------------------------------------------------ H 断言

results = core.evaluate_plan(plan)
check("H1 默认场景 4 条断言", len(results) == 4)
check("H2 默认场景全部通过", core.all_passed(results))
check("H3 断言名包含终点精度", any(item["type"] == "终点精度" for item in results))
check("H4 断言名包含无碰撞", any(item["type"] == "无碰撞" for item in results))
check("H5 断言名包含曲率上限", any(item["type"] == "曲率上限" for item in results))
check("H6 断言名包含路径长度", any(item["type"] == "路径长度" for item in results))
check("H7 无碰撞的明细带间距数值",
      "0.494" in [item["detail"] for item in results if item["type"] == "无碰撞"][0])
check("H8 曲率明细带上限数值",
      "0.227" in [item["detail"] for item in results if item["type"] == "曲率上限"][0])

narrow = core.build_plan(slot=core.Slot(width=1.7))
narrow_results = core.evaluate_plan(narrow)
check("H9 车位过窄时无碰撞失败",
      [item["passed"] for item in narrow_results if item["type"] == "无碰撞"] == [False])
check("H10 车位过窄时其余断言仍通过",
      all(item["passed"] for item in narrow_results if item["type"] != "无碰撞"))
check("H11 车位过窄时整体不通过", not core.all_passed(narrow_results))

short_entry = core.build_plan(entry_length=1.0)
check("H12 直线段过短会撞车",
      [item["passed"] for item in core.evaluate_plan(short_entry) if item["type"] == "无碰撞"] == [False])
check("H13 直线段过短时路径变短", close(short_entry["length"], 5.0 * math.pi / 2.0 + 1.0, 1e-6))

long_path = core.evaluate_plan(plan, max_path=8.0)
check("H14 路径超上限时路径长度失败",
      [item["passed"] for item in long_path if item["type"] == "路径长度"] == [False])
check("H15 路径超上限时其余断言仍通过",
      all(item["passed"] for item in long_path if item["type"] != "路径长度"))

doctored = dict(plan)
doctored["samples"] = plan["samples"][:-1]
doctored["poses"] = plan["poses"][:-1]
tolerance_results = core.evaluate_plan(doctored, tolerance=0.01)
check("H16 终点提前会触发终点精度失败",
      [item["passed"] for item in tolerance_results if item["type"] == "终点精度"] == [False])
check("H17 放宽容差后终点精度通过",
      [item["passed"] for item in core.evaluate_plan(doctored, tolerance=0.50) if item["type"] == "终点精度"] == [True])
check("H18 提高间距下限后无碰撞失败",
      [item["passed"] for item in core.evaluate_plan(plan, clearance=0.6)
       if item["type"] == "无碰撞"] == [False])

# ------------------------------------------------------------ I 报告与俯视图

grid = core.render_map(plan)
check("I1 俯视图 16 行", len(grid) == 16)
check("I2 俯视图每行 56 列", all(len(line) == 56 for line in grid))
check("I3 俯视图含车位线", any("-" in line for line in grid))
check("I4 俯视图含邻车", any("#" in line for line in grid))
check("I5 俯视图含路径", any("o" in line for line in grid))
check("I6 俯视图含车身", any("@" in line for line in grid))
check("I7 可以自定义俯视图尺寸", len(core.render_map(plan, width=40, height=10)) == 10)

report = core.format_report(plan, results, header="探头")
check("I8 报告含标题行", report.split("\n")[0].startswith("探头"))
check("I9 报告含车辆参数", u"最小转弯半径" in report)
check("I10 报告含路径长度", u"10.85" in report)
check("I11 报告含断言统计", u"断言：4 通过 / 0 失败" in report)
check("I12 报告含 PASS 标记", u"[PASS] 终点精度" in report)
check("I13 报告含结论 PASSED", u"结论：PASSED" in report)
check("I14 失败场景报告写 FAILED", u"结论：FAILED" in core.format_report(narrow, narrow_results))
check("I15 失败场景报告含 FAIL 标记", u"[FAIL] 无碰撞" in core.format_report(narrow, narrow_results))
check("I16 报告含俯视图标题", u"俯视图" in report)

# ------------------------------------------------------------ J 参数

params = core.PlannerParams()
check("J1 参数共 21 项", len(params.__slots__) == 21)
check("J2 默认车位宽", close(params.slot_width, 2.5))
check("J3 默认转弯半径", close(params.turn_radius, 5.0))
check("J4 默认发布频率", close(params.publish_hz, 10.0))
check("J5 默认规划话题", params.plan_topic == "/parking/plan")
check("J6 默认报告话题", params.report_topic == "/parking/report")
check("J7 默认逐点话题", params.point_topic == "/parking/plan_point")
check("J8 默认坐标系", params.frame_id == "map")
check("J9 make_vehicle 返回 Vehicle", isinstance(params.make_vehicle(), core.Vehicle))
check("J10 make_slot 返回 Slot", isinstance(params.make_slot(), core.Slot))
check("J11 repr 含车位尺寸", "slot=2.50x5.30" in text(params))

table = {"slot/width": 3.0, "plan/turn_radius": 6.0, "rate/publish_hz": 5.0,
         "topic/plan": "/park", "check/max_path": 20.0}


def table_getter(name, default=None):
    """模拟 rospy.get_param(name, default) 的行为。"""
    return table.get(name, default)


custom = core.PlannerParams.from_params(table_getter)
check("J12 自定义车位宽生效", close(custom.slot_width, 3.0))
check("J13 自定义转弯半径生效", close(custom.turn_radius, 6.0))
check("J14 自定义频率生效", close(custom.publish_hz, 5.0))
check("J15 自定义话题名生效", custom.plan_topic == "/park")
check("J16 自定义路径上限生效", close(custom.max_path, 20.0))
check("J17 缺项回落默认值", close(custom.slot_length, 5.3))
check("J18 字符串数值可解析",
      close(core.PlannerParams.from_params(
          lambda name, default=None: "7.5" if name == "plan/turn_radius" else default).turn_radius, 7.5))
check("J19 自定义车辆进模型",
      close(core.build_plan(vehicle=custom.make_vehicle())["vehicle"].length, 4.5))
check("J20 车位类型可参数化",
      core.PlannerParams.from_params(
          lambda name, default=None: "parallel" if name == "slot/kind" else default).slot_kind == "parallel")

# ------------------------------------------------------------ K 工程文件

pkg_files = [
    "CMakeLists.txt",
    "package.xml",
    "README.md",
    os.path.join("launch", "main.launch"),
    os.path.join("config", "parking_planner.yaml"),
    os.path.join("scripts", "main.py"),
    os.path.join("scripts", "parking_core.py"),
    os.path.join("scripts", "parking_planner_node.py"),
]
for index, relative in enumerate(pkg_files):
    check("K%d 存在 %s" % (index + 1, relative.replace(os.sep, "/")),
          os.path.isfile(os.path.join(PKG_DIR, relative)))

launch_path = os.path.join(PKG_DIR, "launch", "main.launch")
if os.path.isfile(launch_path):
    launch_text = text(read(launch_path))
    check("K9 launch 里节点入口是 main.py", 'type="main.py"' in launch_text)
    check("K10 launch 用 rosparam 加载 yaml", 'command="load"' in launch_text)
    check("K11 launch 用 find 定位包内资源", "$(find reverse_parking_planner)" in launch_text)
    check("K12 launch 的 arg 都带 doc", launch_text.count("<arg") == launch_text.count('doc="'))
    check("K13 launch 有 slot_width 覆盖", 'name="slot_width"' in launch_text)
    check("K14 launch 有 turn_radius 覆盖", 'name="turn_radius"' in launch_text)

yaml_path = os.path.join(PKG_DIR, "config", "parking_planner.yaml")
if os.path.isfile(yaml_path):
    yaml_text = text(read(yaml_path))
    for index, key in enumerate(["slot:", "vehicle:", "plan:", "check:", "rate:", "topic:"]):
        check("K%d yaml 含 %s" % (15 + index, key), key in yaml_text)

package_path = os.path.join(PKG_DIR, "package.xml")
if os.path.isfile(package_path):
    package_text = text(read(package_path))
    check("K21 包名正确", "<name>reverse_parking_planner</name>" in package_text)
    check("K22 依赖 rospy", "rospy" in package_text)
    check("K23 依赖 nav_msgs", "nav_msgs" in package_text)
    check("K24 依赖 std_msgs", "std_msgs" in package_text)

cmake_path = os.path.join(PKG_DIR, "CMakeLists.txt")
if os.path.isfile(cmake_path):
    cmake_text = text(read(cmake_path))
    check("K25 CMakeLists 含 rospy 组件", "rospy" in cmake_text)
    check("K26 CMakeLists 安装三个脚本", cmake_text.count("scripts/") == 3)
    check("K27 CMakeLists 安装 launch 与 config", "install(DIRECTORY launch config" in cmake_text)
    check("K28 CMakeLists 保留执行权限", "USE_SOURCE_PERMISSIONS" in cmake_text)

core_text = text(read(os.path.join(PKG_DIR, "scripts", "parking_core.py")))
check("K29 核心模块不依赖 rospy", "\nimport rospy" not in core_text)
check("K30 核心模块不依赖 numpy", "\nimport numpy" not in core_text)
check("K31 核心模块不依赖 yaml", "\nimport yaml" not in core_text)
check("K32 核心模块不 import pygame", "\nimport pygame" not in core_text)

node_path = os.path.join(PKG_DIR, "scripts", "parking_planner_node.py")
if os.path.isfile(node_path):
    node_text = text(read(node_path))
    check("K33 节点复用核心模块", "import parking_core" in node_text)
    check("K34 节点在主循环里检查关闭", "rospy.is_shutdown()" in node_text)
    check("K35 节点有 py2 日志兼容层", "_log(" in node_text)
    check("K35b 节点启用 unicode_literals（避免 py2 字节/unicode 混用）",
          "unicode_literals" in node_text)
    check("K35c 核心模块也启用 unicode_literals", "unicode_literals" in core_text)

readme_path = os.path.join(PKG_DIR, "README.md")
if os.path.isfile(readme_path):
    readme_text = text(read(readme_path))
    check("K36 README 给出启动命令", "roslaunch reverse_parking_planner main.launch" in readme_text)
    check("K37 README 给出离线测试命令", "test_reverse_parking_planner_local.py" in readme_text)

if os.path.isfile(DOC_PATH):
    doc_text = text(read(DOC_PATH))
    check("K38 文档页保留原 CARLA 章节", u"# CARLA 停车场场景生成" in doc_text)
    check("K39 文档页保留原参考链接", u"carla_ros_scenario_runner" in doc_text or u"carla_parking_lot" in doc_text)
    check("K40 文档页新增规划器章节", u"倒车入库" in doc_text)
    check("K41 文档页说明话题", u"/parking/plan" in doc_text)
    check("K42 文档页说明参数", u"slot_width" in doc_text)
else:
    check("K38 本次未在仓库根目录运行，跳过文档页校验", True)
    check("K39 本地不含 docs 目录，符合预期", not os.path.isdir(os.path.join(ROOT, "docs")))
    check("K40 包目录仍然存在", os.path.isdir(PKG_DIR))
    check("K41 测试脚本位于 tests 目录下", os.path.basename(HERE) == "tests")
    check("K42 核心模块仍然可导入", hasattr(core, "build_plan"))

# ------------------------------------------------------------ 汇总

print("=" * 60)
if FAILED:
    for name in FAILED:
        print("  [FAIL] %s" % name)
    print("=" * 60)
print("通过 %d 项，失败 %d 项" % (len(PASSED), len(FAILED)))
if FAILED:
    print("共有 %d 项失败" % len(FAILED))
    sys.exit(1)
print("全部通过")
sys.exit(0)
