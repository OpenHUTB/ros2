#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``gesture_control_bridge`` 本地回归测试（**无需 ROS / 摄像头 / AirSim / 显示器**）。

运行::

    python3 tests/test_gesture_bridge_local.py

覆盖内容
--------
1. **语法与静态约定** —— 脚本可编译、没有硬编码绝对路径、入口是 ``main.py``。
2. **词表契约** —— 与 ``gesture_control_drone/scripts/gesture_map.py`` 的指令集**逐项比对**
   （防止两个包各写一份 token 表而漂移），并校验分类互斥/全覆盖。
3. **映射正确性** —— 机体系速度的符号（前/上/下/左/右）、``home`` 目标点、急停零速。
4. **桥接层契约** —— 直接读 ``carlair_ros_bridge`` 的 ``bridge.yaml``：
   指令流周期必须小于 ``cmd_timeout``、速度不超过桥接层限速、动作通道存在。
5. **节点行为** —— 用替身 PUB/SLEEP：指令流条数与节拍、冷却、未知指令、退出安全。
6. **工程文件约定** —— 目录、launch、CMakeLists、package.xml、文档与 mkdocs 登记。
"""
from __future__ import annotations

import io
import os
import re
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, os.pardir))
PKG = os.path.join(REPO, "src", "air", "gesture_control_bridge")
SCRIPTS = os.path.join(PKG, "scripts")
BRIDGE = os.path.join(REPO, "src", "air", "carlair_ros_bridge")
GESTURE = os.path.join(REPO, "src", "air", "gesture_control_drone")

# 被测脚本所在目录（离线测试直接从源码 import，与 catkin 安装无关）
sys.path.insert(0, SCRIPTS)

PASSED, FAILED = [], []


def check(name, cond, detail=""):
    (PASSED if cond else FAILED).append(name)
    print("%s %s%s" % ("[通过]" if cond else "[失败]", name,
                       ("  -- " + detail) if detail and not cond else ""))


def read(path):
    with io.open(path, "r", encoding="utf-8") as fh:
        return fh.read()


# --------------------------------------------------------------------------- #
# ROS 替身（必须在 import 被测代码之前安装）
# --------------------------------------------------------------------------- #
PARAMS = {}
PUBLISHED = []


def _mod(name):
    m = types.ModuleType(name)
    sys.modules[name] = m
    return m


def install_stubs():
    rospy = _mod("rospy")
    rospy.get_param = lambda name, default=None: PARAMS.get(name, default)
    rospy.init_node = lambda *a, **k: None
    rospy.loginfo = lambda *a, **k: None
    rospy.logwarn = lambda *a, **k: None
    rospy.logerr = lambda *a, **k: None
    rospy.logwarn_throttle = lambda *a, **k: None
    rospy.loginfo_throttle = lambda *a, **k: None
    rospy.on_shutdown = lambda fn: setattr(rospy, "shutdown_hook", fn)
    rospy.spin = lambda: None
    rospy.is_shutdown = lambda: False
    rospy.get_time = lambda: 0.0
    rospy.ROSInterruptException = type("ROSInterruptException", (Exception,), {})

    class Publisher(object):
        def __init__(self, topic, msg_type, queue_size=10, **k):
            self.topic = topic
            self.msg_type = msg_type

        def publish(self, msg):
            PUBLISHED.append((self.topic, msg))

    class Subscriber(object):
        def __init__(self, topic, msg_type, callback=None, queue_size=1, **k):
            self.topic = topic
            self.msg_type = msg_type
            self.callback = callback

    rospy.Publisher = Publisher
    rospy.Subscriber = Subscriber
    rospy.Rate = lambda hz: types.SimpleNamespace(sleep=lambda: None)

    # ---- geometry_msgs / nav_msgs / std_msgs ----
    geo = _mod("geometry_msgs")
    geo_msg = _mod("geometry_msgs.msg")
    geo.msg = geo_msg

    class _V(object):
        def __init__(self):
            self.x = self.y = self.z = 0.0

    class Twist(object):
        def __init__(self):
            self.linear = _V()
            self.angular = _V()

        def __repr__(self):
            return ("Twist(lin=(%.2f,%.2f,%.2f), ang=%.2f)"
                    % (self.linear.x, self.linear.y, self.linear.z,
                       self.angular.z))

    class Point(object):
        def __init__(self):
            self.x = self.y = self.z = 0.0

        def __repr__(self):
            return "Point(%.2f,%.2f,%.2f)" % (self.x, self.y, self.z)

    geo_msg.Twist = Twist
    geo_msg.Point = Point

    nav = _mod("nav_msgs")
    nav_msg = _mod("nav_msgs.msg")
    nav.msg = nav_msg
    nav_msg.Odometry = type("Odometry", (object,), {})

    std = _mod("std_msgs")
    std_msg = _mod("std_msgs.msg")
    std.msg = std_msg

    class String(object):
        def __init__(self, data=""):
            self.data = data

        def __repr__(self):
            return "String(%r)" % (self.data,)

    std_msg.String = String


# --------------------------------------------------------------------------- #
# 1. 语法与静态约定
# --------------------------------------------------------------------------- #
def test_syntax_and_static():
    import py_compile
    ok, bad = 0, []
    for name in sorted(os.listdir(SCRIPTS)):
        if not name.endswith(".py"):
            continue
        path = os.path.join(SCRIPTS, name)
        try:
            py_compile.compile(path, doraise=True)
            ok += 1
        except Exception as exc:                      # noqa: BLE001
            bad.append((name, str(exc)[:120]))
    check("3 个脚本全部可编译", ok == 3 and not bad, str(bad))

    for name in ("main.py", "gesture_bridge_node.py", "bridge_cmd.py"):
        check("存在 %s" % name, os.path.isfile(os.path.join(SCRIPTS, name)))

    for name in sorted(os.listdir(SCRIPTS)):
        if not name.endswith(".py"):
            continue
        src = read(os.path.join(SCRIPTS, name))
        bad_path = re.search(r"['\"](?:/home/|/Users/|C:\\\\)", src)
        check("%s 没有硬编码绝对路径" % name, bad_path is None,
              bad_path.group(0) if bad_path else "")

    main_src = read(os.path.join(SCRIPTS, "main.py"))
    check("main.py 只做入口转发（import 节点 main）",
          "from gesture_bridge_node import main" in main_src)
    bc_src = read(os.path.join(SCRIPTS, "bridge_cmd.py"))
    check("bridge_cmd.py 不 import rospy（可离线单测）",
          re.search(r"^\s*(?:import|from)\s+rospy", bc_src, re.M) is None)


# --------------------------------------------------------------------------- #
# 2. 词表契约：与 gesture_control_drone 的 gesture_map.py 对齐
# --------------------------------------------------------------------------- #
def parse_gesture_commands():
    """从 gesture_map.py 源码里解析出 ALL_COMMANDS 的 token 列表（不 import 对方）。"""
    src = read(os.path.join(GESTURE, "scripts", "gesture_map.py"))
    consts = dict(re.findall(r'^(CMD_[A-Z_]+)\s*=\s*"([^"]+)"', src, re.M))
    m = re.search(r"ALL_COMMANDS\s*=\s*\(([^)]*)\)", src, re.S)
    if not m:
        return None
    names = [x.strip() for x in m.group(1).replace("\n", " ").split(",") if x.strip()]
    return [consts.get(n, n) for n in names]


def test_vocabulary_contract():
    other = parse_gesture_commands()
    check("能从 gesture_map.py 解析出指令表", bool(other), str(other))
    import bridge_cmd as bc
    check("指令集与 gesture_control_drone 完全一致",
          set(other) == set(bc.ALL_COMMANDS),
          "对方=%s 本模块=%s" % (sorted(other), sorted(bc.ALL_COMMANDS)))
    check("指令表没有重复项", len(bc.ALL_COMMANDS) == len(set(bc.ALL_COMMANDS)))
    check("动作/速度/目标三类互不重叠且覆盖除 estop 外全部指令",
          (set(bc.ACTION_COMMANDS) | set(bc.VELOCITY_COMMANDS) | set(bc.GOAL_COMMANDS))
          == set(bc.ALL_COMMANDS) - {bc.CMD_ESTOP})
    check("三类本身互不相交",
          not (set(bc.ACTION_COMMANDS) & set(bc.VELOCITY_COMMANDS))
          and not (set(bc.ACTION_COMMANDS) & set(bc.GOAL_COMMANDS))
          and not (set(bc.VELOCITY_COMMANDS) & set(bc.GOAL_COMMANDS)))
    for command in bc.ALL_COMMANDS:
        check("kind_of(%s) 有分类" % command, bc.kind_of(command) is not None)


# --------------------------------------------------------------------------- #
# 3. 纯逻辑映射
# --------------------------------------------------------------------------- #
def test_mapping():
    import bridge_cmd as bc

    check("forward -> +x", bc.body_velocity("forward", speed=1.0) == (1.0, 0.0, 0.0, 0.0))
    check("up -> +z（ROS 机体上为正）", bc.body_velocity("up", speed=1.0) == (0.0, 0.0, 1.0, 0.0))
    check("down -> -z", bc.body_velocity("down", speed=1.0) == (0.0, 0.0, -1.0, 0.0))
    check("yaw_left -> +angular.z（逆时针=左）",
          bc.body_velocity("yaw_left", yaw_rate=0.5) == (0.0, 0.0, 0.0, 0.5))
    check("yaw_right -> -angular.z",
          bc.body_velocity("yaw_right", yaw_rate=0.5) == (0.0, 0.0, 0.0, -0.5))
    check("estop -> 全零", bc.body_velocity("estop") == (0.0, 0.0, 0.0, 0.0))
    check("动作类没有速度", bc.body_velocity("takeoff") is None)
    check("speed/yaw 取绝对值（避免符号写错）",
          bc.body_velocity("forward", speed=-1.0) == (1.0, 0.0, 0.0, 0.0)
          and bc.body_velocity("yaw_left", yaw_rate=-2.0) == (0.0, 0.0, 0.0, 2.0))

    check("action_for(takeoff)==takeoff", bc.action_for("takeoff") == "takeoff")
    check("action_for(estop) 默认 hover", bc.action_for("estop") == "hover")
    check("action_for(estop, land)", bc.action_for("estop", estop_action="land") == "land")
    try:
        bc.action_for("estop", estop_action="crash")
        ok = False
    except ValueError:
        ok = True
    check("非法 estop_action 直接报错", ok)

    check("home 目标点是 (0,0,高度)", bc.goal_for(3.0) == (0.0, 0.0, 3.0))
    check("home 高度取绝对值", bc.goal_for(-2.0) == (0.0, 0.0, 2.0))

    check("stream_count(1.0, 20) == 20", bc.stream_count(1.0, 20.0) == 20)
    check("stream_count 至少 1 条", bc.stream_count(0.01, 20.0) == 1)
    try:
        bc.stream_count(1.0, 0)
        ok = False
    except ValueError:
        ok = True
    check("stream_hz<=0 直接报错", ok)

    plan = bc.plan("forward", speed=0.8, duration=1.0, stream_hz=20.0)
    check("plan(forward) 生成 20 条同速指令",
          len(plan["velocities"]) == 20
          and all(v == (0.8, 0.0, 0.0, 0.0) for v in plan["velocities"]))
    check("plan(forward) 没有动作/目标", plan["action"] is None and plan["goal"] is None)
    check("plan(forward) 带 hz（发送间隔 1/hz）", plan["hz"] == 20.0)
    plan = bc.plan("estop", estop_zeros=4)
    check("plan(estop) 先 4 条零速再发动作",
          len(plan["velocities"]) == 4
          and all(v == (0.0, 0.0, 0.0, 0.0) for v in plan["velocities"])
          and plan["action"] == "hover")
    check("plan(未知指令) 返回 None", bc.plan("bogus") is None)


# --------------------------------------------------------------------------- #
# 4. 桥接层契约（直接读对方 yaml / 源码）
# --------------------------------------------------------------------------- #
def bridge_param(path, key, default=None):
    src = read(path)
    m = re.search(r"^\s*%s:\s*([-\d.]+)" % re.escape(key), src, re.M)
    return float(m.group(1)) if m else default


def test_bridge_contract():
    yaml_path = os.path.join(BRIDGE, "config", "bridge.yaml")
    check("桥接层 bridge.yaml 存在", os.path.isfile(yaml_path))
    cmd_timeout = bridge_param(yaml_path, "cmd_timeout")
    cmd_duration = bridge_param(yaml_path, "cmd_duration")
    max_h = bridge_param(yaml_path, "max_horiz_speed")
    max_v = bridge_param(yaml_path, "max_vert_speed")
    max_yaw = bridge_param(yaml_path, "max_yaw_rate")
    check("读到桥接层 cmd_timeout", cmd_timeout is not None, str(cmd_timeout))
    check("读到桥接层 cmd_duration", cmd_duration is not None, str(cmd_duration))

    cfg = read(os.path.join(PKG, "config", "gesture_bridge.yaml"))
    hz = float(re.search(r"^stream_hz:\s*([\d.]+)", cfg, re.M).group(1))
    duration = float(re.search(r"^action_duration:\s*([\d.]+)", cfg, re.M).group(1))
    speed = float(re.search(r"^speed:\s*([\d.]+)", cfg, re.M).group(1))
    yaw_rate = float(re.search(r"^yaw_rate:\s*([\d.]+)", cfg, re.M).group(1))

    check("指令流周期 1/hz < 桥接层 cmd_timeout（否则会被判失效自动悬停）",
          1.0 / hz < cmd_timeout, "1/%.1f=%.3fs vs %.3fs" % (hz, 1.0 / hz, cmd_timeout))
    check("单次动作时长短于 cmd_timeout 会导致只发 1 条；这里 duration > cmd_duration",
          duration > cmd_duration, "%.2f vs %.2f" % (duration, cmd_duration))
    check("我们的 speed ≤ 桥接层 max_horiz_speed", speed <= max_h, "%s vs %s" % (speed, max_h))
    check("我们的 speed ≤ 桥接层 max_vert_speed", speed <= max_v, "%s vs %s" % (speed, max_v))
    check("我们的 yaw_rate ≤ 桥接层 max_yaw_rate", yaw_rate <= max_yaw, "%s vs %s" % (yaw_rate, max_yaw))

    bridge_src = read(os.path.join(BRIDGE, "scripts", "cmd_vel_sub.py"))
    check("桥接层已支持 /uav/action（本 PR 补的动作通道）",
          "topic/action" in bridge_src and "cb_action" in bridge_src)
    for action in ("takeoff", "land", "hover"):
        check("桥接层动作通道支持 %s" % action, '"%s"' % action in bridge_src)
    check("桥接层 bridge.yaml 登记了 topic/action", re.search(r"^\s*action:", read(yaml_path), re.M) is not None)
    check("动作执行期间速度通道让位（避免互相打断）",
          "action_active" in bridge_src and "self.action_active" in bridge_src)


# --------------------------------------------------------------------------- #
# 5. 节点行为（替身 PUB/SLEEP）
# --------------------------------------------------------------------------- #
def make_node(**params):
    import gesture_bridge_node as node_mod
    PARAMS.clear()
    for key, value in params.items():
        PARAMS["~" + key] = value
    return node_mod


def collect():
    """把 PUBLISHED 压成 (kind, value) 便于断言。"""
    out = []
    for topic, msg in PUBLISHED:
        if hasattr(msg, "linear"):
            out.append(("cmd", (round(msg.linear.x, 4), round(msg.linear.y, 4),
                                round(msg.linear.z, 4), round(msg.angular.z, 4))))
        elif hasattr(msg, "data"):
            out.append(("action" if topic.endswith("/action") else "state", msg.data))
        else:
            out.append(("goal", (round(msg.x, 4), round(msg.y, 4), round(msg.z, 4))))
    return out


def test_node_behavior():
    node_mod = make_node(speed=0.8, yaw_rate=0.5, action_duration=1.0,
                         stream_hz=20.0, home_altitude=3.0,
                         estop_action="hover", estop_zeros=5,
                         command_cooldown=0.0)
    sleeps = []
    node = node_mod.GestureBridgeNode(publish=None, sleep=sleeps.append,
                                      clock=lambda: 0.0)
    check("节点从参数读到 speed=0.8", abs(node.speed - 0.8) < 1e-9)
    check("节点从参数读到 stream_hz=20", abs(node.stream_hz - 20.0) < 1e-9)

    del PUBLISHED[:]
    summary = node.handle_command("forward", now=0.0)
    items = collect()
    cmds = [v for k, v in items if k == "cmd"]
    check("forward 下发 20 条 cmd_vel", len(cmds) == 20, str(len(cmds)))
    check("forward 的 20 条都是 (0.8,0,0,0)",
          all(v == (0.8, 0.0, 0.0, 0.0) for v in cmds), str(cmds[:1]))
    check("forward 的节拍是 19 次 sleep(0.05)",
          len(sleeps) == 19 and all(abs(s - 0.05) < 1e-9 for s in sleeps), str(sleeps[:3]))
    check("forward 只发 cmd_vel，不发 action/goal",
          all(k == "cmd" or k == "state" for k, _ in items), str(items[-2:]))
    check("forward 有状态回显", any(k == "state" and "cmd_vel×20" in v for k, v in items),
          summary)

    for command, expected in (("takeoff", "takeoff"), ("land", "land"), ("hover", "hover")):
        del PUBLISHED[:]
        node.handle_command(command, now=100.0)
        items = collect()
        actions = [v for k, v in items if k == "action"]
        check("%s 只发一条 action=%s" % (command, expected),
              actions == [expected], str(actions))
        check("%s 不发 cmd_vel" % command, not [1 for k, _ in items if k == "cmd"])

    del PUBLISHED[:]
    node.handle_command("home", now=200.0)
    goals = [v for k, v in collect() if k == "goal"]
    check("home 发 /uav/goal (0,0,3.0)", goals == [(0.0, 0.0, 3.0)], str(goals))

    del PUBLISHED[:]
    sleeps[:] = []
    node.handle_command("estop", now=300.0)
    items = collect()
    cmds = [v for k, v in items if k == "cmd"]
    actions = [v for k, v in items if k == "action"]
    check("estop 先发 5 条零速", cmds == [(0.0, 0.0, 0.0, 0.0)] * 5, str(cmds))
    check("estop 再发 action=hover", actions == ["hover"], str(actions))
    check("estop 的零速在动作之前",
          [k for k, _ in items][:5] == ["cmd"] * 5, str([k for k, _ in items]))

    # 冷却
    node2_mod = make_node(speed=0.8, stream_hz=20.0, action_duration=1.0,
                          command_cooldown=5.0)
    node2 = node2_mod.GestureBridgeNode(sleep=lambda s: None, clock=lambda: 0.0)
    node2.handle_command("takeoff", now=0.0)
    del PUBLISHED[:]
    check("冷却期内重复指令被忽略", node2.handle_command("takeoff", now=1.0) is None)
    check("冷却期内没有新消息", collect() == [], str(collect()[:2]))
    check("冷却期后可以再次执行", node2.handle_command("takeoff", now=9.0) is not None)

    check("未知指令返回 None", node2.handle_command("bogus", now=100.0) is None)

    # 退出：零速 + hover
    del PUBLISHED[:]
    node2.shutdown()
    items = collect()
    check("退出时先发零速", [k for k, _ in items].count("cmd") >= 1 and items[0][0] == "cmd")
    check("退出时发 hover", ("action", "hover") in items, str(items[-2:]))


# --------------------------------------------------------------------------- #
# 6. 工程 / 文档约定
# --------------------------------------------------------------------------- #
def test_project_layout():
    check("包位于 src/air/gesture_control_bridge",
          os.path.isdir(PKG))
    for name in ("package.xml", "CMakeLists.txt", "README.md",
                 "config/gesture_bridge.yaml", "launch/main.launch"):
        check("存在 %s" % name, os.path.isfile(os.path.join(PKG, name)))

    pkg_xml = read(os.path.join(PKG, "package.xml"))
    check("package.xml 没有 TODO/todo 占位符",
          "TODO" not in pkg_xml and "todo.todo" not in pkg_xml)
    for dep in ("gesture_control_drone", "carlair_ros_bridge"):
        check("package.xml 声明运行时依赖 %s" % dep,
              "<exec_depend>%s</exec_depend>" % dep in pkg_xml)

    cmake = read(os.path.join(PKG, "CMakeLists.txt"))
    check("CMakeLists 使用 catkin_install_python", "catkin_install_python" in cmake)
    block = re.search(r"catkin_install_python\(PROGRAMS(.*?)\)", cmake, re.S)
    listed = block.group(1) if block else ""
    check("入口只装 main.py / gesture_bridge_node.py",
          "main.py" in listed and "gesture_bridge_node.py" in listed
          and "bridge_cmd.py" not in listed, listed.replace("\n", " "))
    check("辅助模块走 install(FILES)（devel 里是启动器，不能当模块 import）",
          "install(FILES" in cmake and "scripts/bridge_cmd.py" in cmake)
    check("CMakeLists 安装 launch 与 config",
          "install(DIRECTORY launch config" in cmake)

    launch = read(os.path.join(PKG, "launch", "main.launch"))
    check('launch 用 type="main.py" 启动入口', 'type="main.py"' in launch)
    check("launch 暴露 host 参数", '<arg name="host"' in launch)
    check("launch 关掉桥接层自动起飞（auto_takeoff:=false）",
          '<arg name="auto_takeoff"   value="false"/>' in launch
          or 'name="auto_takeoff"' in launch and 'value="false"' in launch)
    check("launch 关掉 gesture_control_drone 的直连执行节点",
          'name="start_drone_control"' in launch and 'value="false"' in launch)
    check("launch 在纯 AirSim 环境可关掉相机/雷达发布",
          'name="publish_image"' in launch and 'name="publish_lidar"' in launch)

    cfg = read(os.path.join(PKG, "config", "gesture_bridge.yaml"))
    for key in ("stream_hz", "action_duration", "speed", "yaw_rate",
                "home_altitude", "estop_action", "estop_zeros", "command_topic"):
        check("config 含 %s" % key, re.search(r"^%s:" % key, cfg, re.M) is not None)
    check("config 里 estop_action 合法",
          re.search(r"^estop_action:\s*(hover|land)\s*(?:#.*)?$", cfg, re.M) is not None)

    doc_path = os.path.join(REPO, "docs", "air", "gesture_control_bridge.md")
    check("文档位于 docs/air/gesture_control_bridge.md", os.path.isfile(doc_path))
    if os.path.isfile(doc_path):
        doc = read(doc_path)
        check("文档代码围栏成对", doc.count("```") % 2 == 0)
        check("文档列出三条通道", all(t in doc for t in
                                 ("/uav/cmd_vel", "/uav/action", "/uav/goal")))
        check("文档包含大模型使用声明", "大模型" in doc)
        check("文档没有 TODO 占位符", "TODO" not in doc)

    mkdocs = read(os.path.join(REPO, "mkdocs.yml"))
    check("mkdocs.yml 已登记本文档", "air/gesture_control_bridge.md" in mkdocs)
    index = read(os.path.join(REPO, "docs", "index.md"))
    check("docs/index.md 有首页链接", "air/gesture_control_bridge.md" in index)

    readme = read(os.path.join(PKG, "README.md"))
    check("README 说明 auto_takeoff 必须为 false", "auto_takeoff" in readme and "false" in readme)
    check("README 说明速度指令要按帧连续下发", "cmd_duration" in readme)


# --------------------------------------------------------------------------- #
def main():
    install_stubs()
    print("=" * 70)
    print("gesture_control_bridge 本地回归测试（无需 ROS / 摄像头 / AirSim）")
    print("=" * 70)
    for title, fn in [
            ("1. 语法与静态约定", test_syntax_and_static),
            ("2. 词表契约（与 gesture_control_drone 对齐）", test_vocabulary_contract),
            ("3. 指令 -> 桥接层消息映射", test_mapping),
            ("4. 桥接层契约（cmd_timeout / 限速 / 动作通道）", test_bridge_contract),
            ("5. 节点行为（指令流 / 冷却 / 退出安全）", test_node_behavior),
            ("6. 工程文件与文档约定", test_project_layout)]:
        print("\n-- %s --" % title)
        fn()

    print("\n" + "=" * 70)
    total = len(PASSED) + len(FAILED)
    if FAILED:
        print("失败项：")
        for name in FAILED:
            print("  - %s" % name)
    print("结果：%d/%d 通过" % (len(PASSED), total))
    print("=" * 70)
    return 0 if not FAILED else 1


if __name__ == "__main__":
    sys.exit(main())
