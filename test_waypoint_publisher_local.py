#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""路径点发布模块本地测试（不需要 ROS / CARLA / 仿真器）.

分层之后核心逻辑都在 ``scripts/waypoint_core.py``（不依赖 rospy 与 PyYAML），
所以这个测试只用标准库，任何装了 Python 的机器都能直接跑：

    python3 tests/test_waypoint_publisher_local.py
    python2 tests/test_waypoint_publisher_local.py

覆盖范围：
A. PublisherParams —— 默认值、参数服务器覆盖、模式校验、布尔容错
B. parse_route_text —— 路线文本解析（注释/空行/逗号/3 列）与各类错误
C. 路线校验 —— 内置示例路线、config/demo_route.txt 一致性、NaN / 重合 / 点数
D. TargetCursor —— 顺序 / 循环 / 手动三种推进、越界切换、就近吸附、结束判定
E. 几何 —— 距离、到达判定、最近点、模拟车辆步进
F. 四元数 —— yaw 与 (z, w) 的互转往返
G. 工程文件 —— package.xml / CMakeLists / main.launch / config / README / 文档页
"""
# 与 waypoint_core 一致：同时兼容 Python 2.7（Melodic）与 Python 3.x（Noetic）
from __future__ import absolute_import, division, print_function

import io
import math
import os
import sys
import xml.etree.ElementTree as ET

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "src", "ground", "waypoint_publisher")
SCRIPTS = os.path.join(PKG, "scripts")
CONFIG = os.path.join(PKG, "config", "waypoint_publisher.yaml")
DEMO_TXT = os.path.join(PKG, "config", "demo_route.txt")
LAUNCH = os.path.join(PKG, "launch", "main.launch")
DOC = os.path.join(ROOT, "docs", "ground", "waypoint.md")
sys.path.insert(0, SCRIPTS)

import waypoint_core as tc  # noqa: E402


# ---------------------------------------------------------------- 工具
def fake_get_param(values):
    """充当 rospy.get_param(name, default) 的最小替身。"""
    def get_param(name, default=None):
        return values.get(name, default)
    return get_param


def expect_value_error(fn):
    """确认 fn() 抛出 ValueError（参数写错时应该报错而不是静默跑）。"""
    try:
        fn()
    except ValueError:
        return True
    return False


def read(path):
    # 用 io.open 而不是内置 open：Python 2 的内置 open 不接受 encoding 参数
    with io.open(path, encoding="utf-8") as handle:
        return handle.read()


try:  # Python 2：文本类型是 unicode，str 是字节串
    _TEXT_TYPES = unicode  # noqa: F821
except NameError:  # Python 3：str 就是文本类型
    _TEXT_TYPES = (str,)


def text(value):
    """统一成文本类型，避免 Python 2 下字节串与 unicode 比较时隐式 ASCII 解码。"""
    if isinstance(value, _TEXT_TYPES):
        return value
    return value.decode("utf-8")


# ================================================================= A. 参数
def test_params():
    print("== A. PublisherParams 参数 ==")
    p = tc.PublisherParams()
    check(p.publish_hz == 10.0, "默认 publish_hz = 10.0")
    check(p.frame_id == "map", "默认 frame_id = map")
    check(p.mode == "sequential", "默认 mode = sequential")
    check(p.target_tolerance == 1.0, "默认 target_tolerance = 1.0")
    check(p.start_index == 0, "默认 start_index = 0")
    check(p.autostart is True, "默认 autostart = True")
    check(p.route_file == "", "默认 route_file 为空（用内置路线）")
    check(p.use_mock_odom is False, "默认 use_mock_odom = False")
    check(p.mock_speed == 2.0, "默认 mock_speed = 2.0")
    check(p.waypoints_topic == "/waypoints", "默认 waypoints_topic = /waypoints")
    check(p.target_topic == "/waypoint/target", "默认 target_topic = /waypoint/target")
    check(p.index_topic == "/waypoint/current_index", "默认 index_topic 正确")
    check(p.set_index_topic == "/waypoint/set_index", "默认 set_index_topic 正确")
    check(p.odom_topic == "/odom", "默认 odom_topic = /odom")

    # 参数服务器给值 → 覆盖；未给 → 回落默认值
    p2 = tc.PublisherParams.from_params(fake_get_param({
        "rate/publish_hz": 25,
        "control/mode": "loop",
        "control/target_tolerance": "0.5",
        "control/start_index": 3,
        "mock/enabled": "true",
        "topic/target": "/car/target",
    }))
    check(p2.publish_hz == 25.0, "from_params 覆盖 rate/publish_hz")
    check(p2.mode == "loop", "from_params 覆盖 control/mode")
    check(p2.target_tolerance == 0.5, "from_params 接受字符串数值（rosparam 兼容）")
    check(p2.start_index == 3, "from_params 覆盖 control/start_index")
    check(p2.use_mock_odom is True, "from_params 接受字符串 true")
    check(p2.target_topic == "/car/target", "from_params 覆盖 topic/target")
    check(p2.frame_id == "map", "未提供的参数回落默认值（frame/id）")
    check(p2.mock_speed == 2.0, "未提供的参数回落默认值（mock/speed）")

    # 布尔容错
    check(tc.as_bool("false") is False, "as_bool 接受字符串 false")
    check(tc.as_bool("on") is True, "as_bool 接受字符串 on")
    check(tc.as_bool(0) is False, "as_bool 接受数字 0")
    check(tc.as_bool(True) is True, "as_bool 接受布尔 True")

    # 模式写错要报错，不能静默跑
    check(expect_value_error(lambda: tc.PublisherParams(mode="bad")),
          "非法的 control/mode 抛 ValueError")

    # 所有参数名都从 yaml 里的键读取
    names = []

    def spy(name, default=None):
        names.append(name)
        return default

    tc.PublisherParams.from_params(spy)
    for key in ("rate/publish_hz", "frame/id", "control/mode", "control/target_tolerance",
                "control/start_index", "control/autostart", "route/file", "mock/enabled",
                "mock/speed", "topic/waypoints", "topic/target", "topic/current_index",
                "topic/set_index", "topic/odom"):
        check(key in names, "from_params 读取 %s" % key)


# ================================================================= B. 解析
def test_parse_route_text():
    print("== B. parse_route_text 路线解析 ==")
    points = tc.parse_route_text("0 0\n5 0\n")
    check(len(points) == 2, "两行解析出 2 个点")
    check(points[0] == tc.make_waypoint(0, 0), "第一个点 (0,0)，yaw 默认 0")
    check(points[1].x == 5.0, "第二个点 x = 5.0")

    points = tc.parse_route_text("# 注释行\n\n1,2,3.14\n")
    check(len(points) == 1, "注释与空行被跳过")
    check(close(points[0].yaw, 3.14), "第三列被当作 yaw")

    points = tc.parse_route_text("0\t0\t0\n1 1 1\n")
    check(len(points) == 2, "制表符与空格都当分隔符")

    points = tc.parse_route_text("0 0 0  # 行尾注释\n1 1\n")
    check(len(points) == 2, "行尾注释不影响解析")

    check(expect_value_error(lambda: tc.parse_route_text("")), "空文本报错")
    check(expect_value_error(lambda: tc.parse_route_text("# 只有注释\n")), "只有注释报错")
    check(expect_value_error(lambda: tc.parse_route_text("0 0\n1\n")), "少一个数报错")
    check(expect_value_error(lambda: tc.parse_route_text("0 0 0 0\n")), "多一个数报错")
    check(expect_value_error(lambda: tc.parse_route_text("0 0\na b\n")), "非数字报错")

    try:
        tc.parse_route_text("0 0\n1\n", source="r.txt")
        check(False, "报错信息应带行号与文件名")
    except ValueError as exc:
        message = str(exc)
        check("r.txt" in message, "报错信息带文件名")
        check("第 2 行" in message, "报错信息带出错行号")

    try:
        tc.parse_route_text("")
    except ValueError as exc:
        check("没有解析到任何路径点" in str(exc), "空文本的报错说明原因")


# ================================================================= C. 路线校验
def test_route_validation():
    print("== C. 路线校验 ==")
    route = tc.demo_route()
    check(len(route) == 8, "内置路线有 8 个点")
    check(route[0] == tc.make_waypoint(0.0, 0.0, 0.0), "内置路线从 (0,0) 开始")
    check(route[4] == tc.make_waypoint(10.0, 10.0, 0.0), "内置路线第 5 个点是 (10,10)")
    check(tc.validate_waypoints(route) == [], "内置路线通过校验")

    # 内置路线与随包的示例文件必须完全一致（防止文档/配置漂移）
    file_points = tc.parse_route_text(read(DEMO_TXT), source="demo_route.txt")
    check(file_points == route, "config/demo_route.txt 与内置路线一致")

    check(tc.validate_waypoints([]) != [], "空路线被检出")
    check(tc.validate_waypoints([tc.make_waypoint(0, 0)]) != [], "只有一个点被检出")

    duplicated = [tc.make_waypoint(0, 0), tc.make_waypoint(0, 0), tc.make_waypoint(1, 0)]
    problems = tc.validate_waypoints(duplicated)
    check(any("重合" in p for p in problems), "相邻点重合被检出")

    nan_route = [tc.make_waypoint(float("nan"), 0.0), tc.make_waypoint(1.0, 0.0)]
    check(any("NaN" in p for p in tc.validate_waypoints(nan_route)), "NaN 坐标被检出")

    inf_route = [tc.make_waypoint(0.0, 0.0), tc.make_waypoint(float("inf"), 0.0)]
    check(any("NaN" in p for p in tc.validate_waypoints(inf_route)), "Inf 坐标被检出")


# ================================================================= D. 游标
def test_target_cursor():
    print("== D. TargetCursor 目标点游标 ==")
    cursor = tc.TargetCursor(8)
    check(cursor.current() == 0, "初始目标点是第 1 个")
    check(cursor.mode == "sequential", "默认顺序推进")
    check(cursor.is_finished() is False, "初始未结束")

    # sequential：顺序推进，到底结束
    cursor = tc.TargetCursor(3)
    check(cursor.advance() == 1, "sequential 推进到下标 1")
    check(cursor.advance() == 2, "sequential 推进到下标 2")
    check(cursor.at_last() is True, "到达最后一个路径点（at_last）")
    check(cursor.is_finished() is False, "刚到最后一个点还不算结束")
    check(cursor.advance() == 2, "结束后不再前进")
    check(cursor.is_finished() is True, "再推进一次确认走完，标记结束")

    # loop：回绕
    cursor = tc.TargetCursor(3, mode="loop")
    cursor.advance()
    cursor.advance()
    check(cursor.current() == 2, "loop 走到最后一个点")
    check(cursor.advance() == 0, "loop 回绕到第 1 个")
    check(cursor.is_finished() is False, "loop 永不结束")
    check(cursor.advance() == 1, "loop 回绕后继续前进")

    # manual：不自动推进，但可以被指令推进
    cursor = tc.TargetCursor(3, mode="manual")
    check(cursor.advance() == 0, "manual 不自动推进")
    check(cursor.force_next() == 1, "manual 下 force_next 仍前进")
    check(cursor.is_finished() is False, "manual 不会自己结束")

    # 动态切换：按序号跳转
    cursor = tc.TargetCursor(8)
    check(cursor.set_index(5) is True, "set_index 接受合法下标")
    check(cursor.current() == 5, "切换后当前下标为 5")
    check(cursor.set_index(8) is False, "越界下标被拒绝")
    check(cursor.current() == 5, "被拒绝时不改变当前下标")
    check(cursor.set_index(-1) is False, "负数下标被拒绝")
    check(cursor.set_index("3") is True and cursor.current() == 3, "字符串下标可接受")
    check(cursor.set_index(None) is False, "None 被拒绝")
    check(cursor.set_index(3.9) is True and cursor.current() == 3, "浮点下标被截断取整")

    # 切换会清掉结束标记（允许走完后再从头指定目标）
    cursor = tc.TargetCursor(2)
    check(cursor.advance() == 1, "两点路线推进到最后一个点")
    check(cursor.at_last() is True, "两点路线到达末尾")
    check(cursor.advance() == 1, "再推进仍停在最后一个点")
    check(cursor.is_finished() is True, "两点路线确认走完")
    cursor.set_index(0)
    check(cursor.is_finished() is False, "重新指定目标后结束标记被清除")

    # start_index 超范围要被夹住而不是报错
    check(tc.TargetCursor(4, start_index=99).current() == 3, "start_index 超上界被夹到最后一个")
    check(tc.TargetCursor(4, start_index=-5).current() == 0, "start_index 超下界被夹到 0")

    # 非法构造
    check(expect_value_error(lambda: tc.TargetCursor(0)), "count=0 抛 ValueError")
    check(expect_value_error(lambda: tc.TargetCursor(3, mode="bad")),
          "非法 mode 抛 ValueError")

    # 就近吸附
    route = tc.demo_route()
    cursor = tc.TargetCursor(len(route))
    check(cursor.jump_to_nearest((10.0, 10.0), route) is True, "就近吸附成功")
    check(cursor.current() == 4, "吸附到 (10,10)，即第 5 个点")
    check(cursor.jump_to_nearest((0.0, 0.0), []) is False, "空路线吸附失败")

    check("TargetCursor" in repr(cursor), "游标有可读的 repr")


# ================================================================= E. 几何
def test_geometry():
    print("== E. 几何与模拟车辆 ==")
    check(close(tc.distance((0, 0), (3, 4)), 5.0), "distance 用平面欧氏距离")
    check(close(tc.distance(tc.make_waypoint(0, 0), tc.make_waypoint(0, 2)), 2.0),
          "distance 也接受 Waypoint")

    check(tc.reached((0, 0, 0), (0.5, 0), 1.0) is True, "阈值内判定到达")
    check(tc.reached((0, 0, 0), (1.5, 0), 1.0) is False, "阈值外判定未到达")
    check(tc.reached((0, 0, 0), (0, 0), 1.0) is True, "与目标重合判定到达")
    check(tc.reached((0, 0, 0), (0, 0), 0.0) is False,
          "tolerance<=0 关闭自动到达判定（只能靠指令切换）")
    check(tc.reached((0, 0, 0), (0.1, 0), -1.0) is False, "负阈值同样关闭自动判定")

    route = tc.demo_route()
    check(tc.nearest_index((0.1, 0.1), route) == 0, "最近点：靠近原点时是第 1 个")
    check(tc.nearest_index((10.2, 9.8), route) == 4, "最近点：靠近 (10,10) 时是第 5 个")
    check(tc.nearest_index((0, 0), []) == -1, "空路线返回 -1")

    # 模拟车辆步进
    pose = tc.step_towards((0, 0, 0), (10, 0), 2.0)
    check(close(pose[0], 2.0) and close(pose[1], 0.0), "步长 2m：x 前进 2m、y 不变")
    check(close(pose[2], 0.0), "朝向目标在 +x 方向时 yaw=0")

    pose = tc.step_towards((9, 0, 0), (10, 0), 5.0)
    check(close(pose[0], 10.0), "最后一步不会越过目标点")

    pose = tc.step_towards((0, 0, 0), (0, 10), 3.0)
    check(close(pose[2], math.pi / 2.0), "目标在 +y 方向时 yaw=90°")

    pose = tc.step_towards((1, 1, 0.5), (1, 1), 5.0)
    check(pose == (1.0, 1.0, 0.5), "已重合时不移动、保持原朝向")

    pose = tc.step_towards((0, 0, 0), (10, 0), 0.0)
    check(pose == (0.0, 0.0, 0.0), "step<=0 时不移动")


# ================================================================= F. 四元数
def test_quaternion():
    print("== F. 四元数互转 ==")
    qz, qw = tc.yaw_to_quaternion(0.0)
    check(close(qz, 0.0) and close(qw, 1.0), "yaw=0 对应四元数 (z=0, w=1)")
    qz, qw = tc.yaw_to_quaternion(math.pi / 2.0)
    check(close(qz, math.sin(math.pi / 4.0)), "yaw=90° 的 z 分量为 sin(45°)")
    check(close(qw, math.cos(math.pi / 4.0)), "yaw=90° 的 w 分量为 cos(45°)")

    for yaw in (0.0, 0.5, -1.2, math.pi):
        qz, qw = tc.yaw_to_quaternion(yaw)
        back = tc.quaternion_to_yaw(0.0, 0.0, qz, qw)
        delta = math.atan2(math.sin(back - yaw), math.cos(back - yaw))
        check(close(delta, 0.0), "yaw 与四元数往返一致（yaw=%.2f）" % yaw)


# ================================================================= G. 工程文件
def test_project_files():
    print("== G. 工程文件 ==")
    pkg = ET.parse(os.path.join(PKG, "package.xml")).getroot()
    check(pkg.findtext("name").strip() == "waypoint_publisher", "package.xml 包名正确")
    depends = [d.text.strip() for d in pkg.findall("exec_depend")]
    for name in ("rospy", "geometry_msgs", "nav_msgs", "std_msgs"):
        check(name in depends, "package.xml 声明 exec_depend %s" % name)

    cmake = read(os.path.join(PKG, "CMakeLists.txt"))
    for name in ("main.py", "waypoint_core.py", "waypoint_node.py"):
        check(name in cmake, "CMakeLists 安装 %s" % name)
    check("install(DIRECTORY launch config" in cmake, "CMakeLists 安装 launch 与 config")

    launch_text = read(LAUNCH)
    launch = ET.parse(LAUNCH).getroot()
    nodes = [n.get("type") for n in launch.findall("node")]
    check("main.py" in nodes, "main.launch 启动 main.py（入口符合 main.* 约定）")
    check("config/waypoint_publisher.yaml" in launch_text, "main.launch 加载 config yaml")
    args = [a.get("name") for a in launch.findall("arg")]
    for name in ("publish_hz", "frame_id", "mode", "target_tolerance", "start_index",
                 "autostart", "route_file", "mock_enabled", "mock_speed",
                 "waypoints_topic", "target_topic", "odom_topic"):
        check(name in args, "main.launch 暴露参数 %s" % name)
    check(all(a.get("doc") for a in launch.findall("arg")), "main.launch 每个参数都有 doc")
    params = [p.get("name") for p in launch.findall("param")]
    for name in ("rate/publish_hz", "control/mode", "control/target_tolerance",
                 "route/file", "mock/enabled", "topic/target"):
        check(name in params, "main.launch 把 <arg> 覆盖到 %s" % name)

    yaml_text = read(CONFIG)
    for key in ("publish_hz", "mode", "target_tolerance", "start_index", "autostart",
                "file", "enabled", "speed", "waypoints", "target", "current_index",
                "set_index", "odom"):
        check(key in yaml_text, "config yaml 含 %s" % key)

    # 分层：核心逻辑不依赖 ROS，入口是薄壳
    core_src = read(os.path.join(SCRIPTS, "waypoint_core.py"))
    core_imports = [line.strip() for line in core_src.splitlines()]
    check(not any(line.startswith(("import rospy", "from rospy")) for line in core_imports),
          "waypoint_core 不依赖 rospy（可离线测试）")
    check(not any(line.startswith(("import yaml", "from yaml")) for line in core_imports),
          "waypoint_core 不依赖 PyYAML（路线用纯文本）")
    node_src = read(os.path.join(SCRIPTS, "waypoint_node.py"))
    check("import waypoint_core" in node_src, "节点复用 waypoint_core 的逻辑")
    entry_src = read(os.path.join(SCRIPTS, "main.py"))
    check("from waypoint_node import main" in entry_src, "main.py 是复用节点逻辑的入口")

    # 随包资源
    check(os.path.isfile(DEMO_TXT), "config/demo_route.txt 随包提供")
    readme = read(os.path.join(PKG, "README.md"))
    for word in ("路径点", "动态目标点切换", "main.launch", "离线测试"):
        check(text(word) in readme, "README 提到 %s" % word)

    # 文档页：本模块是「路径点发布器」这一页的自主实现部分
    check(os.path.isfile(DOC), "文档页 docs/ground/waypoint.md 存在")
    doc_text = read(DOC)
    for word in ("waypoint_publisher", "动态目标点切换", "mock_enabled", "离线测试"):
        check(text(word) in doc_text, "文档页提到 %s" % word)


def main():
    test_params()
    test_parse_route_text()
    test_route_validation()
    test_target_cursor()
    test_geometry()
    test_quaternion()
    test_project_files()
    print("\n==================== 结果 ====================")
    print("通过 %d 项，失败 %d 项" % (len(PASS), len(FAIL)))
    if FAIL:
        for item in FAIL:
            print("  FAIL: %s" % item)
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
