#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gesture_control_drone 本地回归测试（**无需 ROS / 摄像头 / AirSim / 显示器**）。

运行::

    python3 tests/test_gesture_control_local.py

覆盖内容
--------
1. **语法与静态约定** —— 每个脚本都能编译；没有硬编码绝对路径；只有一个识别入口。
2. **词表一致性** —— 7 种手势 -> 7 个动作全覆盖；10 条指令都有 AirSim 实现；
   发布端只发指令 token、订阅端用同一份词表校验。
3. **手势分类** —— 用合成关键点验证 7 种手势（含 OK、拇指向上/向下）。
4. **指令闸门** —— 去抖、边沿触发、冷却、"手势结束"判定。
5. **摄像头** —— 按平台选后端；自动扫描索引；失败给出可读错误。
6. **AirSim 封装** —— 解锁/定高起飞/降落/悬停/前进/上升/下降/偏航/返航/急停；
   安全限幅（限速、限高、限低）；RPC 超时保护；NED->ENU 状态转换；缺依赖报错。
7. **端到端联调** —— 手势 token -> 指令 token -> AirSim API，七种手势全链路。
8. **两个节点的契约** —— 感知节点边沿触发；控制节点冷却/拦截/状态上报；
   以及 ``nav_msgs/Odometry`` 状态回传（ENU）。
9. **叠加层不丢帧** —— ``gesture_demo.draw_overlay`` 必须把画好的帧返回并被接收。
10. **工程文件约定** —— 目录、launch、CMakeLists、package.xml、requirements、
    文档与 mkdocs 登记。

说明：ROS / OpenCV / MediaPipe / pygame / AirSim 全部用替身模块，被测代码一字未改。
"""
from __future__ import annotations

import ast
import os
import re
import sys
import threading
import time
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, os.pardir))
PKG = os.path.join(REPO, "src", "air", "gesture_control_drone")
SCRIPTS = os.path.join(PKG, "scripts")

PASSED, FAILED = [], []


def check(name, cond, detail=""):
    (PASSED if cond else FAILED).append(name)
    print("%s %s%s" % ("[通过]" if cond else "[失败]", name,
                       ("  -- " + detail) if detail and not cond else ""))


# --------------------------------------------------------------------------- #
# 替身模块（必须在 import 被测代码之前安装）
# --------------------------------------------------------------------------- #
CALLS = {"flip": 0, "imshow": 0}
CAM_STATE = {"open": set()}


def _mod(name):
    m = types.ModuleType(name)
    sys.modules[name] = m
    return m


def install_stubs():
    # ---------------- cv2 ----------------
    cv2 = _mod("cv2")
    for k, v in dict(COLOR_BGR2RGB=4, COLOR_RGB2BGR=5, CAP_DSHOW=700,
                     CAP_ANY=0, CAP_V4L2=200, CAP_PROP_FRAME_WIDTH=3,
                     CAP_PROP_FRAME_HEIGHT=4, FONT_HERSHEY_SIMPLEX=0).items():
        setattr(cv2, k, v)

    class _Capture(object):
        def __init__(self, index=0, backend=0):
            self.index = index
            self.backend = backend
            self._open = index in CAM_STATE["open"]

        def isOpened(self):
            return self._open

        def set(self, *a):
            return True

        def read(self):
            if not self._open:
                return False, None
            return True, np.zeros((8, 8, 3), dtype=np.uint8)

        def release(self):
            self._open = False

    def _flip(frame, code):
        CALLS["flip"] += 1
        return frame.copy()

    cv2.VideoCapture = _Capture
    cv2.cvtColor = lambda img, code: np.array(img, copy=True)
    cv2.flip = _flip
    cv2.putText = lambda *a, **k: None
    cv2.imshow = lambda *a, **k: CALLS.__setitem__("imshow", CALLS["imshow"] + 1)
    cv2.waitKey = lambda ms: -1
    cv2.destroyAllWindows = lambda: None

    # ---------------- mediapipe ----------------
    mp = _mod("mediapipe")
    mp.solutions = types.SimpleNamespace()
    mp.solutions.hands = types.SimpleNamespace(
        Hands=lambda **k: types.SimpleNamespace(
            process=lambda img: types.SimpleNamespace(multi_hand_landmarks=None),
            close=lambda: None),
        HAND_CONNECTIONS=[])
    mp.solutions.drawing_utils = types.SimpleNamespace(
        DrawingSpec=lambda **k: object(),
        draw_landmarks=lambda *a, **k: None)

    # ---------------- PIL ----------------
    pil = _mod("PIL")
    image = _mod("PIL.Image")
    image.fromarray = lambda arr: arr
    image_draw = _mod("PIL.ImageDraw")
    image_draw.Draw = lambda im: types.SimpleNamespace(text=lambda *a, **k: None)

    class _Font(object):
        pass

    image_font = _mod("PIL.ImageFont")
    image_font.FreeTypeFont = _Font
    image_font.truetype = lambda *a, **k: _Font()
    image_font.load_default = lambda: _Font()
    pil.Image, pil.ImageDraw, pil.ImageFont = image, image_draw, image_font

    # ---------------- pygame ----------------
    pg = _mod("pygame")
    pg.error = type("error", (Exception,), {})
    pg.QUIT = 256
    pg.SRCALPHA = 65536

    class _Surface(object):
        def __init__(self, *a, **k):
            pass

    pg.Surface = _Surface
    pg.init = lambda: None
    pg.quit = lambda: None
    pg.display = types.SimpleNamespace(
        set_mode=lambda size, *a: _Surface(), set_caption=lambda s: None,
        flip=lambda: None, get_surface=lambda: _Surface())
    pg.draw = types.SimpleNamespace(rect=lambda *a, **k: None,
                                    circle=lambda *a, **k: None)
    pg.event = types.SimpleNamespace(get=lambda: [])
    pg.surfarray = types.SimpleNamespace(
        array3d=lambda s: np.zeros((8, 8, 3), dtype=np.uint8),
        blit_array=lambda *a: None)
    pg.locals = types.SimpleNamespace(QUIT=256, KEYDOWN=768)

    # ---------------- rospy / std_msgs ----------------
    rospy = _mod("rospy")
    rospy.ROSInterruptException = type("ROSInterruptException", (Exception,), {})
    rospy.NODE_STATE = {}
    rospy.PUBLISHED = []
    rospy.SUBSCRIBED = []
    rospy.LOGS = []

    class _Publisher(object):
        def __init__(self, topic, msg_type, **k):
            self.topic, self.msg_type, self.sent = topic, msg_type, []
            rospy.PUBLISHED.append(self)

        def publish(self, msg):
            if self.msg_type is not None and not isinstance(msg, self.msg_type):
                raise TypeError("publish() 期望 %s，收到 %r"
                                % (self.msg_type, type(msg).__name__))
            self.sent.append(msg)

    rospy.Publisher = _Publisher
    rospy.init_node = lambda name, **k: rospy.NODE_STATE.setdefault("name", name)
    rospy.get_param = lambda name, default=None: default
    rospy.Subscriber = lambda topic, typ, cb, **k: rospy.SUBSCRIBED.append(
        (topic, cb))
    rospy.loginfo = lambda msg, *a, **k: rospy.LOGS.append(("info", msg))
    rospy.logwarn = lambda msg, *a, **k: rospy.LOGS.append(("warn", msg))
    rospy.logerr = lambda msg, *a, **k: rospy.LOGS.append(("err", msg))
    rospy.loginfo_throttle = rospy.logwarn_throttle = \
        lambda *a, **k: rospy.LOGS.append(("throttled", a[1] if len(a) > 1 else ""))
    rospy.on_shutdown = lambda cb: None
    rospy.is_shutdown = lambda: False
    rospy.signal_shutdown = lambda why="": None
    rospy.spin = lambda: None
    rospy.Rate = lambda hz: types.SimpleNamespace(sleep=lambda: None)
    rospy.Time = types.SimpleNamespace(now=lambda: 0.0)

    std_msgs = _mod("std_msgs")
    std_msgs_msg = _mod("std_msgs.msg")

    class String(object):
        def __init__(self, data=""):
            self.data = data

        def __repr__(self):
            return "String(%r)" % self.data

    std_msgs_msg.String = String
    std_msgs.msg = std_msgs_msg

    # ---------------- nav_msgs（状态回传）----------------
    nav = _mod("nav_msgs")
    nav_msg = _mod("nav_msgs.msg")

    class _Vector3(object):
        def __init__(self):
            self.x = self.y = self.z = 0.0

    class _Quaternion(object):
        def __init__(self):
            self.x = self.y = self.z = 0.0
            self.w = 1.0

    class _Pose(object):
        def __init__(self):
            self.position = _Vector3()
            self.orientation = _Quaternion()

    class _PoseWC(object):
        def __init__(self):
            self.pose = _Pose()

    class _Twist(object):
        def __init__(self):
            self.linear = _Vector3()
            self.angular = _Vector3()

    class _TwistWC(object):
        def __init__(self):
            self.twist = _Twist()

    class _Header(object):
        def __init__(self):
            self.stamp = 0.0
            self.frame_id = ""

    class Odometry(object):
        def __init__(self):
            self.header = _Header()
            self.pose = _PoseWC()
            self.twist = _TwistWC()

    nav_msg.Odometry = Odometry
    nav.msg = nav_msg
    return String, Odometry


String, Odometry = install_stubs()
sys.path.insert(0, SCRIPTS)

import camera_utils             # noqa: E402
import drone_client             # noqa: E402
import gesture_demo             # noqa: E402
import gesture_detection as gd  # noqa: E402
import gesture_gate             # noqa: E402
import gesture_map as gm        # noqa: E402
import gesture_ros_node         # noqa: E402
import drone_control_node       # noqa: E402

FakeString = String


def script_files():
    return sorted(f for f in os.listdir(SCRIPTS) if f.endswith(".py"))


def read(path):
    try:
        with open(path, encoding="utf-8") as fh:
            return fh.read()
    except OSError:
        return ""


# --------------------------------------------------------------------------- #
# 1. 语法与静态约定
# --------------------------------------------------------------------------- #
def test_syntax_and_static():
    for name in script_files():
        src = read(os.path.join(SCRIPTS, name))
        try:
            compile(src, name, "exec")
            check("%s 语法正确" % name, True)
        except SyntaxError as exc:
            check("%s 语法正确" % name, False, "%s: %s (line %s)"
                  % (type(exc).__name__, exc.msg, exc.lineno))

    bad = [n for n in script_files()
           if re.search(r"sys\.path\.append\(|/home/[a-z]",
                        read(os.path.join(SCRIPTS, n)))]
    check("没有硬编码的本机绝对路径", not bad, str(bad))

    node_src = read(os.path.join(SCRIPTS, "gesture_ros_node.py"))
    check("感知节点调用 detector.detect(...)", "detector.detect(" in node_src)
    check("感知节点不再调用不存在的 detect_gestures(...)",
          "detector.detect_gestures(" not in node_src)
    check("GestureDetector 只暴露一个公共识别方法",
          hasattr(gd.GestureDetector, "detect"))

    for name in script_files():
        src = read(os.path.join(SCRIPTS, name))
        tree = ast.parse(src)
        unused = False
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for a in node.names:
                    if a.name == "sys":
                        body = "\n".join(
                            ln for ln in src.split("\n")
                            if not ln.strip().startswith("import "))
                        unused = not re.search(r"\bsys\.", body)
        check("%s 没有未使用的 sys 导入" % name, not unused)


# --------------------------------------------------------------------------- #
# 2. 词表一致性（本轮最重要的回归）
# --------------------------------------------------------------------------- #
def test_vocabulary_contract():
    check("每个手势都能映射到指令",
          all(gm.gesture_to_command(g) in gm.ALL_COMMANDS
              for g in gm.GESTURE_TO_COMMAND))
    check("GESTURE_TO_COMMAND 的取值都在 ALL_COMMANDS 里",
          set(gm.GESTURE_TO_COMMAND.values()) <= set(gm.ALL_COMMANDS))
    check("GESTURE_NONE 不会映射出指令",
          gm.gesture_to_command(gm.GESTURE_NONE) == gm.CMD_NONE)
    check("手势表覆盖 7 个可用手势",
          len(gm.GESTURE_TO_COMMAND) == 7, str(len(gm.GESTURE_TO_COMMAND)))
    check("反向映射与正向映射一致",
          all(gm.COMMAND_TO_GESTURE[c] == g
              for g, c in gm.GESTURE_TO_COMMAND.items()))

    drone = drone_client.AirSimDrone()
    check("每条指令都有 AirSim 实现",
          all(c in drone._handlers for c in gm.ALL_COMMANDS),
          str(sorted(set(gm.ALL_COMMANDS) - set(drone._handlers))))

    node_src = read(os.path.join(SCRIPTS, "gesture_ros_node.py"))
    check("感知节点发布 result.command（词表产出）",
          "result.command" in node_src or "gesture_to_command" in node_src)
    check("感知节点发布 String 而不是元组",
          "String(data=command)" in node_src)

    ctrl_src = read(os.path.join(SCRIPTS, "drone_control_node.py"))
    check("控制节点用 gesture_map 校验指令", "gm.is_command(" in ctrl_src)
    check("控制节点不再比较英文硬编码指令",
          '"open"' not in ctrl_src and '"fist"' not in ctrl_src)
    check("两端共用同一个指令话题参数名",
          "command_topic" in node_src and "command_topic" in ctrl_src)


# --------------------------------------------------------------------------- #
# 3. 手势分类（合成关键点）
# --------------------------------------------------------------------------- #
FINGER_X = {"index": 60, "middle": 100, "ring": 140, "pinky": 180}
WRIST = (100, 300)


def make_landmarks(extended, thumb_extended=True, ok=False, thumb_dir=None):
    """按"哪些手指伸出"造 21 个关键点（图像坐标，y 向下）。"""
    pts = [(0, 0)] * 21
    pts[gd.WRIST] = WRIST
    pts[gd.THUMB_CMC] = (140, 245)
    pts[gd.THUMB_MCP] = (165, 230)
    pts[gd.THUMB_IP] = (185, 220)
    pts[gd.THUMB_TIP] = (215, 205) if thumb_extended else (150, 205)
    if thumb_dir == "up":
        pts[gd.THUMB_TIP] = (250, 175)
    elif thumb_dir == "down":
        pts[gd.THUMB_TIP] = (250, 255)
    for name, x in FINGER_X.items():
        up = name in extended
        mcp = {"index": gd.INDEX_MCP, "middle": gd.MIDDLE_MCP,
               "ring": gd.RING_MCP, "pinky": gd.PINKY_MCP}[name]
        pip = {"index": gd.INDEX_PIP, "middle": gd.MIDDLE_PIP,
               "ring": gd.RING_PIP, "pinky": gd.PINKY_PIP}[name]
        dip = {"index": gd.INDEX_DIP, "middle": gd.MIDDLE_DIP,
               "ring": gd.RING_DIP, "pinky": gd.PINKY_DIP}[name]
        tip = {"index": gd.INDEX_TIP, "middle": gd.MIDDLE_TIP,
               "ring": gd.RING_TIP, "pinky": gd.PINKY_TIP}[name]
        pts[mcp] = (x, 200)
        pts[pip] = (x, 165)
        pts[dip] = (x, 140 if up else 175)
        pts[tip] = (x, 110) if up else (x, 195)
    if ok:
        pts[gd.THUMB_TIP] = pts[gd.INDEX_TIP]
    return pts


def test_classification():
    cases = [
        ("张开手掌", gm.GESTURE_PALM,
         make_landmarks({"index", "middle", "ring", "pinky"}, True)),
        ("握拳", gm.GESTURE_FIST, make_landmarks(set(), False)),
        ("食指指向", gm.GESTURE_POINT, make_landmarks({"index"}, False)),
        ("胜利手势", gm.GESTURE_VICTORY,
         make_landmarks({"index", "middle"}, False)),
        ("拇指向上", gm.GESTURE_THUMB_UP,
         make_landmarks(set(), True, thumb_dir="up")),
        ("拇指向下", gm.GESTURE_THUMB_DOWN,
         make_landmarks(set(), True, thumb_dir="down")),
        ("OK 手势", gm.GESTURE_OK,
         make_landmarks({"middle", "ring", "pinky"}, False, ok=True)),
        ("其他组合 -> 未识别", gm.GESTURE_NONE,
         make_landmarks({"index", "middle", "ring"}, False)),
    ]
    for label, expected, pts in cases:
        got = gd.classify_landmarks(pts)
        check("%s 识别为 %s" % (label, expected), got == expected,
              "实际 %s" % got)

    check("关键点不足时返回 NONE",
          gd.classify_landmarks([(0, 0)] * 5) == gm.GESTURE_NONE)
    check("空关键点返回 NONE", gd.classify_landmarks([]) == gm.GESTURE_NONE)
    check("手掌尺度用腕部到中指根的距离",
          abs(gd.palm_scale(make_landmarks(set())) - 100.0) < 1e-6,
          str(gd.palm_scale(make_landmarks(set()))))
    check("OK 手势阈值与手掌尺度成比例（不是固定 30 像素）",
          gd.OK_DISTANCE_RATIO < 0.5)
    check("拇指朝上/朝下用与手掌尺度成比例的竖直阈值",
          gd.THUMB_DIRECTION_RATIO < 0.5)
    check("拇指朝上判定函数", gd.thumb_direction(
        make_landmarks(set(), True, thumb_dir="up")) == gm.GESTURE_THUMB_UP)
    check("拇指向下判定函数", gd.thumb_direction(
        make_landmarks(set(), True, thumb_dir="down")) == gm.GESTURE_THUMB_DOWN)
    check("拇指横指时不判方向",
          gd.thumb_direction(make_landmarks(set(), True)) is None)


# --------------------------------------------------------------------------- #
# 4. 指令闸门
# --------------------------------------------------------------------------- #
def test_command_gate():
    gate = gesture_gate.CommandGate(stable_frames=3, cooldown=1.0,
                                    release_frames=2, clock=lambda: 0.0)
    outs = [gate.update(gm.CMD_TAKEOFF, now=t) for t in (0.0, 0.1, 0.2)]
    check("去抖：前两帧不发布", outs[:2] == [None, None], str(outs))
    check("去抖：第 3 帧发布一次", outs[2] == gm.CMD_TAKEOFF, str(outs))

    held = [gate.update(gm.CMD_TAKEOFF, now=0.3 + i * 0.1) for i in range(5)]
    check("边沿触发：一直举着不重复发布", all(o is None for o in held), str(held))

    check("无手势帧不发布", gate.update(gm.CMD_NONE, now=5.0) is None)
    check("一个无手势帧不足以重置边沿",
          gate.update(gm.CMD_TAKEOFF, now=5.1) is None)
    gate.update(gm.CMD_NONE, now=5.2)
    gate.update(gm.CMD_NONE, now=5.3)
    again = [gate.update(gm.CMD_TAKEOFF, now=5.4 + i * 0.1) for i in range(3)]
    check("手势结束（连续无手势）后可以再次触发",
          again == [None, None, gm.CMD_TAKEOFF], str(again))

    gate2 = gesture_gate.CommandGate(stable_frames=1, cooldown=10.0)
    check("冷却：首条发布", gate2.update(gm.CMD_LAND, now=0.0) == gm.CMD_LAND)
    gate2.update(gm.CMD_NONE, now=0.1)
    gate2.update(gm.CMD_NONE, now=0.2)
    check("冷却：间隔不足时不再发布",
          gate2.update(gm.CMD_LAND, now=1.0) is None)
    check("冷却：超过冷却时间后可以再次发布",
          gate2.update(gm.CMD_LAND, now=11.0) == gm.CMD_LAND)


# --------------------------------------------------------------------------- #
# 5. 摄像头
# --------------------------------------------------------------------------- #
def test_camera():
    check("Windows 用 DirectShow 后端",
          camera_utils.choose_backend("win32") == camera_utils.cv2.CAP_DSHOW)
    for plat in ("linux", "darwin"):
        check("%s 不用 DirectShow 后端" % plat,
              camera_utils.choose_backend(plat) != camera_utils.cv2.CAP_DSHOW)

    CAM_STATE["open"] = {0}
    cam = camera_utils.Camera(camera_index=0)
    check("索引 0 可用时直接打开", cam.initialize() is True)
    CALLS["flip"] = 0
    frame = cam.read_frame()
    check("read_frame 返回图像", frame is not None and frame.shape == (8, 8, 3))
    check("read_frame 做了水平镜像", CALLS["flip"] == 1, str(CALLS["flip"]))
    cam.release()
    check("release 之后不再持有句柄", cam.cap is None)

    CAM_STATE["open"] = {2}
    cam2 = camera_utils.Camera(camera_index=0)
    check("索引 0 不可用时自动扫描到 2",
          cam2.initialize() is True and cam2.camera_index == 2,
          str(cam2.camera_index))
    cam2.release()

    CAM_STATE["open"] = set()
    cam_src = read(os.path.join(SCRIPTS, "camera_utils.py"))
    check("open_capture 默认设 MJPG（VM 透传摄像头默认格式读不到帧）",
          "CAP_PROP_FOURCC" in cam_src and "MJPG" in cam_src)
    check("支持 GESTURE_CAMERA_FOURCC 覆盖像素格式",
          "GESTURE_CAMERA_FOURCC" in cam_src)

    cam3 = camera_utils.Camera(camera_index=0)
    try:
        cam3.initialize()
        check("无可用摄像头时抛 RuntimeError", False, "没有抛异常")
    except RuntimeError as exc:
        check("无可用摄像头时抛 RuntimeError", True)
        check("报错信息包含排查提示",
              "摄像头" in str(exc) and "索引" in str(exc), str(exc))
    CAM_STATE["open"] = {0}


# --------------------------------------------------------------------------- #
# 6. AirSim 封装
# --------------------------------------------------------------------------- #
class FakeAirSim(object):
    """记录调用，并维护一个可操控的"载具状态"（NED：z 向上为负）。"""

    def __init__(self, confirm=True, altitude=-2.0, landed=1, block=0.0,
                 **_k):
        self.confirm = confirm
        self.calls = []
        self.altitude = altitude
        self.landed = landed
        self.collided = False
        self.block = block

    def _rec(self, name, *args):
        self.calls.append((name,) + args)

    def names(self):
        return [c[0] for c in self.calls]

    def confirmConnection(self, **k):
        self._rec("confirmConnection")
        return self.confirm

    def enableApiControl(self, value=True, **k):
        self._rec("enableApiControl", value)

    def armDisarm(self, value, **k):
        self._rec("armDisarm", value)
        return True

    def isApiControlEnabled(self, **k):
        return True

    def takeoffAsync(self, **k):
        self._rec("takeoffAsync")
        self.landed, self.altitude = 1, -2.0      # 1 = Flying（官方语义）
        return self

    def landAsync(self, **k):
        self._rec("landAsync")
        self.landed, self.altitude = 0, 0.0       # 0 = Landed
        return self

    def hoverAsync(self, **k):
        self._rec("hoverAsync")
        return self

    def moveToZAsync(self, z, velocity, **k):
        self._rec("moveToZAsync", z, velocity)
        self.altitude = z
        return self

    def moveByVelocityBodyFrameAsync(self, vx, vy, vz, duration, **k):
        self._rec("move", vx, vy, vz, duration)
        return self

    def rotateByYawRateAsync(self, yaw_rate, duration, **k):
        self._rec("yaw", yaw_rate, duration)
        return self

    def moveToPositionAsync(self, x, y, z, velocity, **k):
        self._rec("moveToPosition", x, y, z, velocity)
        return self

    def goHomeAsync(self, velocity, **k):
        self._rec("goHomeAsync", velocity)
        return self

    def getMultirotorState(self, **k):
        self._rec("getState")
        pos = types.SimpleNamespace(x_val=1.0, y_val=2.0, z_val=self.altitude)
        vel = types.SimpleNamespace(x_val=0.1, y_val=0.2, z_val=-0.3)
        return types.SimpleNamespace(
            kinematics_estimated=types.SimpleNamespace(
                position=pos, linear_velocity=vel),
            landed_state=self.landed)

    def simGetCollisionInfo(self, **k):
        return types.SimpleNamespace(has_collided=self.collided)

    def join(self):
        if self.block:
            time.sleep(self.block)
        return self


def make_drone(fake=None, **kw):
    fake = fake or FakeAirSim()
    drone = drone_client.AirSimDrone(
        client_factory=lambda **k: fake, log=lambda m: None,
        clock=lambda: 0.0, **kw)
    return drone, fake


def test_drone_client():
    drone, fake = make_drone()
    check("连接成功", drone.connect() is True)
    check("连接后接管 API 控制", ("enableApiControl", True) in fake.calls)
    n = len(fake.calls)
    drone.connect()
    check("重复 connect 不会重复建连/重复接管", len(fake.calls) == n)

    # ---- 起飞：解锁 -> takeoff -> 定高 -> 悬停 ----
    drone, fake = make_drone(takeoff_altitude=3.0, speed=1.0)
    check("起飞执行成功", drone.execute(gm.CMD_TAKEOFF) is True)
    names = fake.names()
    order_ok = ("armDisarm" in names and "takeoffAsync" in names
                and "moveToZAsync" in names and "hoverAsync" in names
                and names.index("armDisarm") < names.index("takeoffAsync")
                < names.index("moveToZAsync") <= names.index("hoverAsync"))
    check("起飞顺序：解锁 -> 起飞 -> 定高 -> 悬停", order_ok, str(names))
    check("解锁参数为 True", ("armDisarm", True) in fake.calls, str(fake.calls))
    z_args = [c for c in fake.calls if c[0] == "moveToZAsync"]
    check("定高目标为 -起飞高度（NED 向上为负）",
          z_args and abs(z_args[0][1] + 3.0) < 1e-6, str(z_args))
    check("起飞后记录 armed", drone.armed is True)

    # ---- 降落 / 悬停 ----
    for command, api in ((gm.CMD_LAND, "landAsync"),
                         (gm.CMD_HOVER, "hoverAsync")):
        d, f = make_drone()
        check("%s 执行成功" % command, d.execute(command) is True)
        check("%s 调用了 %s" % (command, api), api in f.names(), str(f.names()))

    # ---- 前进 / 上升 / 下降 ----
    d, f = make_drone(speed=1.0)
    d.execute(gm.CMD_FORWARD)
    moves = [c for c in f.calls if c[0] == "move"]
    check("前进：机体系 +x", moves and moves[0][1] > 0, str(moves))

    d, f = make_drone(speed=1.0)
    d.execute(gm.CMD_UP)
    moves = [c for c in f.calls if c[0] == "move"]
    check("上升：NED 下 -z", moves and moves[0][3] < 0, str(moves))

    d, f = make_drone(speed=1.0)
    d.execute(gm.CMD_DOWN)
    moves = [c for c in f.calls if c[0] == "move"]
    check("下降：NED 下 +z", moves and moves[0][3] > 0, str(moves))

    # ---- 偏航 ----
    d, f = make_drone(yaw_rate=30.0)
    d.execute(gm.CMD_YAW_LEFT)
    yaws = [c for c in f.calls if c[0] == "yaw"]
    check("左转：偏航速率为负（NED 顺时针为正）",
          yaws and yaws[0][1] < 0, str(yaws))
    d, f = make_drone(yaw_rate=30.0)
    d.execute(gm.CMD_YAW_RIGHT)
    yaws = [c for c in f.calls if c[0] == "yaw"]
    check("右转：偏航速率为正", yaws and yaws[0][1] > 0, str(yaws))

    # ---- 返航（含老客户端回退）----
    d, f = make_drone()
    check("返航优先用 goHomeAsync",
          d.execute(gm.CMD_HOME) is True and "goHomeAsync" in f.names(),
          str(f.names()))
    check("返航第一个参数是超时 60s（不是速度）",
          ("goHomeAsync", 60.0) in f.calls, str(f.calls))
    d, f = make_drone()
    f.goHomeAsync = None
    check("没有 goHomeAsync 时回退到 moveToPositionAsync",
          d.execute(gm.CMD_HOME) is True and "moveToPosition" in f.names(),
          str(f.names()))

    # ---- 急停三种语义 ----
    for action, api in (("hover", "hoverAsync"), ("land", "landAsync")):
        d, f = make_drone(estop_action=action)
        d.execute(gm.CMD_ESTOP)
        check("estop_action=%s 调用 %s" % (action, api), api in f.names(),
              str(f.names()))
    d, f = make_drone(estop_action="disarm")
    d.execute(gm.CMD_ESTOP)
    check("estop_action=disarm 才锁桨（默认不锁桨）",
          ("armDisarm", False) in f.calls, str(f.calls))
    try:
        drone_client.AirSimDrone(estop_action="nope")
        check("非法 estop_action 被拒绝", False)
    except ValueError:
        check("非法 estop_action 被拒绝", True)

    # ---- 安全限幅 ----
    d, f = make_drone(speed=1.0, max_speed=2.0, move_duration=1.0)
    d.execute(gm.CMD_FORWARD)
    moves = [c for c in f.calls if c[0] == "move"]
    check("速度不超过 max_speed", moves and abs(moves[0][1]) <= 2.0 + 1e-9,
          str(moves))

    d, f = make_drone(takeoff_altitude=3.0, max_altitude=30.0)
    try:
        drone_client.AirSimDrone(takeoff_altitude=40.0, max_altitude=30.0)
        check("起飞高度超过上限时拒绝构造", False)
    except ValueError:
        check("起飞高度超过上限时拒绝构造", True)
    d.takeoff_altitude = 40.0                # 运行期被改大也要拦住
    check("运行期起飞高度超限时拒绝执行", d.execute(gm.CMD_TAKEOFF) is False)
    check("拒绝原因说明超限", "上限" in d.last_error, d.last_error)

    d, f = make_drone(max_altitude=30.0)
    f.altitude = -30.0
    check("到限高后拒绝继续上升", d.execute(gm.CMD_UP) is False)
    check("限高原因可读", "最大高度" in d.last_error, d.last_error)

    d, f = make_drone(min_altitude=0.5)
    f.altitude = -0.5
    check("到限低后拒绝继续下降", d.execute(gm.CMD_DOWN) is False)
    check("限低原因提示改用降落", "降落" in d.last_error, d.last_error)

    # ---- 指令超时保护 ----
    # 起飞/降落现在各有独立上限（见评审第 2 条），这里把它们也压到 0.05 s，
    # 才能测到 command_timeout 的超时保护本身。
    d, f = make_drone(FakeAirSim(block=0.5), command_timeout=0.05,
                      takeoff_timeout=0.05, land_timeout=0.05)
    started = time.monotonic()
    ok = d.execute(gm.CMD_TAKEOFF)
    elapsed = time.monotonic() - started
    check("RPC 卡住时不会永远阻塞（超时返回失败）", ok is False)
    check("超时在 command_timeout 附近返回", elapsed < 0.45, "%.2fs" % elapsed)
    check("超时原因可读", "超过" in d.last_error, d.last_error)

    # ---- 未知指令 / 连接失败 / 缺依赖 ----
    d, f = make_drone()
    check("未知指令返回 False", d.execute("bogus") is False)
    check("未知指令不产生动作调用",
          not [c for c in f.calls if c[0] != "confirmConnection"],
          str(f.calls))

    d, f = make_drone(FakeAirSim(confirm=False))
    check("连接失败时 execute 返回 False", d.execute(gm.CMD_TAKEOFF) is False)
    check("连接失败会记录错误信息", bool(d.last_error), d.last_error)

    saved = sys.modules.get("airsim", "MISSING")
    sys.modules["airsim"] = None
    try:
        d2 = drone_client.AirSimDrone(log=lambda m: None)
        check("缺 airsim 依赖时安全失败", d2.execute(gm.CMD_TAKEOFF) is False)
        check("缺 airsim 依赖时提示 pip install airsim",
              "airsim" in d2.last_error, d2.last_error)
    finally:
        if saved == "MISSING":
            sys.modules.pop("airsim", None)
        else:
            sys.modules["airsim"] = saved

    # ---- 状态回传（NED -> ENU）----
    d, f = make_drone()
    d.connect()
    st = d.state()
    check("位置 NED -> ENU 交换 x/y 且 z 取负",
          abs(st.position_enu[0] - 2.0) < 1e-6
          and abs(st.position_enu[1] - 1.0) < 1e-6
          and abs(st.position_enu[2] - 2.0) < 1e-6, str(st.position_enu))
    check("速度同样转换",
          abs(st.velocity_enu[0] - 0.2) < 1e-6
          and abs(st.velocity_enu[2] - 0.3) < 1e-6, str(st.velocity_enu))
    check("落地状态可读（1 -> Flying）", st.landed_state == "Flying",
          st.landed_state)
    check("高度取 ENU 的 z", abs(st.altitude - 2.0) < 1e-6, str(st.altitude))
    check("api_control 如实反映", st.api_control is True)
    f.collided = True
    check("碰撞状态如实反映", d.state().collision is True)
    check("状态单行摘要包含高度与状态",
          "alt=" in st.summary() and "Flying" in st.summary(), st.summary())

    # ---- 关闭流程 ----
    d, f = make_drone()
    d.connect()
    f.calls.clear()
    d.close()
    check("关闭时先悬停", "hoverAsync" in f.names(), str(f.names()))
    check("关闭时交还 API 控制",
          ("enableApiControl", False) in f.calls, str(f.calls))
    check("关闭后 connected=False", d.connected is False)


# --------------------------------------------------------------------------- #
# 6.5 AirSim API 兼容：async build 与 sync build 都要能用
# --------------------------------------------------------------------------- #
class FakeSyncOnlyAirSim(object):
    """只提供同步接口的假客户端（模拟本项目实测的 UE4.27 Blocks build）。

    刻意**没有** takeoffAsync / landAsync / hoverAsync，
    并且 takeoff 会阻塞 0.3 s —— 用来验证"发完即弃、不阻塞调用方"。
    """

    def __init__(self, paused=False, z=-2.0, **_k):
        self.calls = []
        self.paused = paused
        self.z = z
        self.landed = 1                      # 1 = Flying

    def _rec(self, name, *args):
        self.calls.append((name,) + args)

    def names(self):
        return [c[0] for c in self.calls]

    # -- 连接 / 控制权 --
    def confirmConnection(self, **k):
        self._rec("confirmConnection")
        return True

    def enableApiControl(self, value=True, vehicle_name=""):
        self._rec("enableApiControl", value)

    def isApiControlEnabled(self, vehicle_name=""):
        return True

    def armDisarm(self, value, vehicle_name=""):
        self._rec("armDisarm", value)
        return True

    # -- 状态（landed_state 刻意恒为 1，与真实 build 一致）--
    def getMultirotorState(self, vehicle_name=""):
        pos = types.SimpleNamespace(x_val=0.0, y_val=0.0, z_val=self.z)
        vel = types.SimpleNamespace(x_val=0.0, y_val=0.0, z_val=0.0)
        return types.SimpleNamespace(
            kinematics_estimated=types.SimpleNamespace(position=pos,
                                                       linear_velocity=vel),
            landed_state=self.landed)

    def simGetCollisionInfo(self, vehicle_name=""):
        return types.SimpleNamespace(has_collided=False)

    def simIsPaused(self):
        self._rec("simIsPaused")
        return self.paused

    def simPause(self, value):
        self._rec("simPause", value)
        self.paused = bool(value)

    # -- 同步动作（会阻塞）--
    def takeoff(self, timeout_sec=20, vehicle_name=""):
        self._rec("takeoff", timeout_sec, vehicle_name)
        time.sleep(0.3)                       # 模拟同步阻塞
        self.landed, self.z = 1, -2.0        # 起飞后 = Flying

    def land(self, timeout_sec=60, vehicle_name=""):
        self._rec("land", timeout_sec, vehicle_name)
        self.landed, self.z = 0, 0.0         # 落地后 = Landed

    def hover(self, vehicle_name=""):
        self._rec("hover", vehicle_name)


def test_api_compat():
    # ---- 支持 async 的 build：仍走原路径 ----
    d, f = make_drone()
    d.connect()
    check("支持 *Async 的 build 识别为 async 模式", d.api_mode == "async",
          d.api_mode)
    f.calls.clear()
    d.execute(gm.CMD_TAKEOFF)
    check("async build 仍走 takeoffAsync", "takeoffAsync" in f.names(),
          str(f.names()))
    check("async build 仍会定高（moveToZAsync）", "moveToZAsync" in f.names(),
          str(f.names()))

    # ---- 只有同步接口的 build：自动回退 ----
    fake = FakeSyncOnlyAirSim()
    d2 = drone_client.AirSimDrone(client_factory=lambda **k: fake,
                                 log=lambda m: None, clock=lambda: 0.0)
    check("只有同步接口时连接成功", d2.connect() is True)
    check("sync-only build 被识别为 sync 模式", d2.api_mode == "sync",
          d2.api_mode)

    t0 = time.monotonic()
    ok = d2.execute(gm.CMD_TAKEOFF)
    elapsed = time.monotonic() - t0
    check("sync 模式下 takeoff 执行成功", ok is True)
    check("sync 模式不阻塞调用方（同步 takeoff 在后台跑）", elapsed < 0.25,
          "%.2fs" % elapsed)
    time.sleep(0.6)
    check("sync 模式确实调用了 takeoff()（独立连接）",
          "takeoff" in fake.names(), str(fake.names()))
    check("sync 模式不会调用不存在的 takeoffAsync",
          "takeoffAsync" not in fake.names(), str(fake.names()))

    fake.calls.clear()
    check("sync 模式 hover 执行成功", d2.execute(gm.CMD_HOVER) is True)
    check("sync 模式 hover 走同步接口", "hover" in fake.names(),
          str(fake.names()))

    fake.calls.clear()
    check("sync 模式 land 执行成功", d2.execute(gm.CMD_LAND) is True)
    time.sleep(0.3)
    check("sync 模式 land 走同步接口", "land" in fake.names(), str(fake.names()))

    # ---- 仿真暂停时自动取消暂停 ----
    paused = FakeSyncOnlyAirSim(paused=True)
    logs = []
    d3 = drone_client.AirSimDrone(client_factory=lambda **k: paused,
                                 log=logs.append, clock=lambda: 0.0)
    d3.connect()
    check("连接时检测到仿真暂停", ("simPause", False) in paused.calls,
          str(paused.calls))
    check("自动取消暂停并打日志",
          any("暂停" in str(m) for m in logs), str(logs[-3:]))
    check("取消后 simIsPaused 为 False", paused.paused is False)

    # ---- LandedState 用官方语义：0 = Landed，1 = Flying ----
    check("LandedState 0 -> Landed（官方语义）",
          drone_client.landed_state_name(0) == "Landed",
          drone_client.landed_state_name(0))
    check("LandedState 1 -> Flying（官方语义）",
          drone_client.landed_state_name(1) == "Flying",
          drone_client.landed_state_name(1))

    fake2 = FakeSyncOnlyAirSim(z=-2.0)          # ENU z = 2.0
    fake2.landed = 1                            # 在飞
    d5 = drone_client.AirSimDrone(client_factory=lambda **k: fake2,
                                 log=lambda m: None, clock=lambda: 0.0)
    d5.connect()
    st = d5.state()
    check("状态里的 landed_state 与高度互不干扰",
          st.landed_state == "Flying" and abs(st.altitude - 2.0) < 1e-6,
          "%s / %s" % (st.landed_state, st.altitude))

    # ---- 并发保护：状态定时器与指令线程共用连接，必须串行化 ----
    check("drone_client 用 RLock 串行化对同一客户端的调用",
          isinstance(d5._api_lock, type(threading.RLock())),
          type(getattr(d5, "_api_lock", None)).__name__)



def test_land_slow_descent():
    """3 m 降落约需 14 s，不能套用全局 command_timeout(10 s)。"""

    class SlowLandAirSim(FakeAirSim):
        def landAsync(self, **k):
            self._rec("landAsync")
            time.sleep(0.4)                 # 模拟慢降落
            self.landed, self.altitude = 0, 0.0
            return self

    fake = SlowLandAirSim()
    drone = drone_client.AirSimDrone(
        client_factory=lambda **k: fake, log=lambda m: None,
        command_timeout=0.05, land_timeout=5.0)
    drone.connect()
    check("慢降落不会被 command_timeout 误判为超时",
          drone.execute(gm.CMD_LAND) is True, drone.last_error)
    check("land 真的等到了落地（landed=Flying->Landed）",
          fake.landed == 0, fake.landed)


def test_sync_move_signature():
    """同步回退必须按服务端签名传参，否则 forward/up/down/yaw 会失败。"""

    class SyncMoveAirSim(FakeSyncOnlyAirSim):
        def moveByVelocityBodyFrame(self, vx, vy, vz, duration, drivetrain=0,
                                    yaw_mode=None, vehicle_name=""):
            self._rec("moveByVelocityBodyFrame", round(vx, 2), round(vy, 2),
                      round(vz, 2), round(duration, 2), vehicle_name)

        def rotateByYawRate(self, yaw_rate, duration, vehicle_name=""):
            self._rec("rotateByYawRate", yaw_rate, duration, vehicle_name)

    fake = SyncMoveAirSim(z=-2.0)
    d = drone_client.AirSimDrone(client_factory=lambda **k: fake,
                                log=lambda m: None, clock=lambda: 0.0)
    d.connect()
    check("sync 模式 forward 能执行", d.execute(gm.CMD_FORWARD) is True,
          d.last_error)
    mv = [c for c in fake.calls if c[0] == "moveByVelocityBodyFrame"]
    check("forward 按 7 参数签名调用（4 个位置 + vehicle_name）",
          bool(mv) and len(mv[0]) == 6, str(mv[:1]))
    check("sync 模式 yaw_left 能执行", d.execute(gm.CMD_YAW_LEFT) is True,
          d.last_error)
    yr = [c for c in fake.calls if c[0] == "rotateByYawRate"]
    check("yaw 按 3 参数签名调用（rate, duration, vehicle_name）",
          bool(yr) and len(yr[0]) == 4, str(yr[:1]))


# --------------------------------------------------------------------------- #
# 7. 端到端：手势 -> 指令 -> AirSim API
# --------------------------------------------------------------------------- #
def gesture_landmarks(gesture):
    return {
        gm.GESTURE_PALM: make_landmarks({"index", "middle", "ring", "pinky"},
                                        True),
        gm.GESTURE_FIST: make_landmarks(set(), False),
        gm.GESTURE_POINT: make_landmarks({"index"}, False),
        gm.GESTURE_VICTORY: make_landmarks({"index", "middle"}, False),
        gm.GESTURE_THUMB_UP: make_landmarks(set(), True, thumb_dir="up"),
        gm.GESTURE_THUMB_DOWN: make_landmarks(set(), True, thumb_dir="down"),
        gm.GESTURE_OK: make_landmarks({"middle", "ring", "pinky"}, False,
                                      ok=True),
    }[gesture]


E2E_EXPECT = {
    gm.GESTURE_PALM: (gm.CMD_TAKEOFF, "takeoffAsync"),
    gm.GESTURE_FIST: (gm.CMD_LAND, "landAsync"),
    gm.GESTURE_POINT: (gm.CMD_FORWARD, "move_x+"),
    gm.GESTURE_VICTORY: (gm.CMD_UP, "move_z-"),
    gm.GESTURE_THUMB_DOWN: (gm.CMD_DOWN, "move_z+"),
    gm.GESTURE_THUMB_UP: (gm.CMD_HOVER, "hoverAsync"),
    gm.GESTURE_OK: (gm.CMD_ESTOP, "hoverAsync"),
}


def _hit(fake, spec):
    if spec == "move_x+":
        return any(c[0] == "move" and c[1] > 0 for c in fake.calls)
    if spec == "move_z-":
        return any(c[0] == "move" and c[3] < 0 for c in fake.calls)
    if spec == "move_z+":
        return any(c[0] == "move" and c[3] > 0 for c in fake.calls)
    return spec in fake.names()


def test_end_to_end():
    for gesture, (expect_cmd, spec) in E2E_EXPECT.items():
        got = gd.classify_landmarks(gesture_landmarks(gesture))
        command = gm.gesture_to_command(got)
        drone, fake = make_drone()
        ok = drone.execute(command)
        check("端到端：%s -> %s -> AirSim(%s)"
              % (gm.gesture_cn(gesture), gm.command_cn(expect_cmd), spec),
              got == gesture and command == expect_cmd and ok and _hit(fake, spec),
              "gesture=%s command=%s calls=%s" % (got, command, fake.names()))


# --------------------------------------------------------------------------- #
# 8. 两个节点的契约 + 状态回传
# --------------------------------------------------------------------------- #
class FakeCamera(object):
    camera_index = 0

    def initialize(self):
        return True

    def read_frame(self):
        return np.zeros((8, 8, 3), dtype=np.uint8)

    def release(self):
        pass


class FakeDetector(object):
    def __init__(self, gesture):
        self.gesture = gesture

    def detect(self, frame):
        return gd.GestureResult(frame=frame, gesture=self.gesture,
                                command=gm.gesture_to_command(self.gesture))

    def close(self):
        pass


class FakeDroneExecutor(object):
    """给 drone_control_node 用的假执行器。"""

    ip, port, estop_action = "127.0.0.1", 41451, "hover"
    takeoff_altitude, max_speed, max_altitude = 3.0, 3.0, 30.0

    def __init__(self):
        self.executed = []
        self.last_error = ""
        self.counts = {}
        self.closed = False
        self._state = drone_client.DroneState(
            position_enu=(1.0, 2.0, 3.0), velocity_enu=(0.1, 0.0, -0.2),
            landed_state="Flying", api_control=True, collision=False)

    def execute(self, command):
        self.executed.append(command)
        self.counts[command] = self.counts.get(command, 0) + 1
        return True

    def state(self):
        return self._state

    def close(self):
        self.closed = True


def test_nodes():
    node = gesture_ros_node.GestureControlNode(
        camera=FakeCamera(), detector=FakeDetector(gm.GESTURE_PALM))

    sent = [node.handle_result(
        gd.GestureResult(gesture=gm.GESTURE_PALM, command=gm.CMD_TAKEOFF),
        now=float(i)) for i in range(3)]
    check("感知节点：去抖后只发一条指令",
          sent == [None, None, gm.CMD_TAKEOFF], str(sent))
    check("感知节点：发布的是 String",
          all(isinstance(m, FakeString) for m in node.pub_cmd.sent),
          str(node.pub_cmd.sent))
    check("感知节点：发布的数据是词表里的指令 token",
          all(isinstance(m.data, str) and gm.is_command(m.data)
              for m in node.pub_cmd.sent),
          str([m.data for m in node.pub_cmd.sent]))

    held = [node.handle_result(
        gd.GestureResult(gesture=gm.GESTURE_PALM, command=gm.CMD_TAKEOFF),
        now=3.0 + i) for i in range(3)]
    check("感知节点：一直举着不会刷屏重复发布",
          all(h is None for h in held), str(held))

    executor = FakeDroneExecutor()
    ctrl = drone_control_node.DroneControlNode(drone=executor)
    ctrl.pub_state.sent.clear()
    check("控制节点：首条指令执行成功",
          ctrl.handle_command(gm.CMD_TAKEOFF, now=0.0) is True)
    check("控制节点：冷却期内不重复执行",
          ctrl.handle_command(gm.CMD_TAKEOFF, now=0.1) is False)
    check("控制节点：真的调用了执行器一次",
          executor.executed == [gm.CMD_TAKEOFF], str(executor.executed))
    check("控制节点：上报状态 takeoff OK",
          any(m.data == "takeoff OK" for m in ctrl.pub_state.sent),
          str([m.data for m in ctrl.pub_state.sent]))

    executor.executed.clear()
    ctrl.on_command(FakeString(data="open"))
    check("控制节点：忽略词表外的指令", executor.executed == [])
    ctrl.on_command(FakeString(data=gm.CMD_LAND))
    check("控制节点：词表内的指令进入队列", ctrl._queue.qsize() == 1)

    # ---- 状态回传 ----
    ctrl.pub_odom.sent.clear()
    st = ctrl.publish_odom()
    check("控制节点：发布 nav_msgs/Odometry",
          len(ctrl.pub_odom.sent) == 1, str(ctrl.pub_odom.sent))
    odom = ctrl.pub_odom.sent[0] if ctrl.pub_odom.sent else None
    check("Odometry 是 nav_msgs 类型", isinstance(odom, Odometry))
    check("Odometry 位置为 ENU",
          odom is not None and abs(odom.pose.pose.position.x - 1.0) < 1e-6
          and abs(odom.pose.pose.position.z - 3.0) < 1e-6,
          "" if odom is None else str(odom.pose.pose.position.__dict__))
    check("Odometry 速度已转换",
          odom is not None and abs(odom.twist.twist.linear.z + 0.2) < 1e-6,
          "" if odom is None else str(odom.twist.twist.linear.__dict__))
    check("Odometry frame_id 正确",
          odom is not None and odom.header.frame_id == "world",
          "" if odom is None else odom.header.frame_id)
    check("publish_odom 返回状态快照", isinstance(st, drone_client.DroneState))

    check("控制节点：状态话题与手势状态话题不同名",
          ctrl.state_topic != node.state_topic,
          "%s vs %s" % (ctrl.state_topic, node.state_topic))
    check("两节点共用同一个指令话题",
          ctrl.command_topic == node.command_topic)


# --------------------------------------------------------------------------- #
# 9. 叠加层必须真的画上去
# --------------------------------------------------------------------------- #
def test_overlay_not_lost():
    demo = gesture_demo.GestureDemo.__new__(gesture_demo.GestureDemo)
    demo.drone = types.SimpleNamespace(is_flying=False, altitude=0.0)

    marker = [0]
    original = gesture_demo.put_chinese

    def fake_put(frame, text, position, font_size=24, color=(0, 255, 0)):
        out = frame.copy()
        out[0, 0, 0] = 1
        marker[0] = 1
        return out

    gesture_demo.put_chinese = fake_put
    try:
        src = np.zeros((16, 16, 3), dtype=np.uint8)
        out = demo.draw_overlay(src, "张开手掌", "起飞", 30.0)
        check("draw_overlay 返回新帧（不是 None）", out is not None)
        check("draw_overlay 确实把字画在了返回的帧上",
              out is not None and out[0, 0, 0] == 1,
              str(out[0, 0, 0] if out is not None else None))
        check("draw_overlay 不就地修改入参（与原实现不同）", src[0, 0, 0] == 0)
        check("draw_overlay 里调用了中文绘制", marker[0] == 1)
    finally:
        gesture_demo.put_chinese = original

    src_text = read(os.path.join(SCRIPTS, "gesture_demo.py"))
    check("调用处接住了返回值 frame = ...draw_overlay(...)",
          re.search(r"frame\s*=\s*self\.draw_overlay\(", src_text) is not None)

    demo2 = gesture_demo.GestureDemo.__new__(gesture_demo.GestureDemo)
    demo2.drone = types.SimpleNamespace(execute_command=lambda c: True)
    demo2.stable = 3
    demo2._candidate = gm.GESTURE_NONE
    demo2._candidate_count = 0
    demo2._last_command = gm.CMD_NONE
    demo2._last_command_at = None
    outs = [demo2.update(gm.GESTURE_PALM, now=t) for t in (0.0, 0.1, 0.2)]
    check("演示程序同样去抖：第 3 帧才执行",
          outs == [gm.CMD_NONE, gm.CMD_NONE, gm.CMD_TAKEOFF], str(outs))


# --------------------------------------------------------------------------- #
# 10. 工程文件约定
# --------------------------------------------------------------------------- #
def test_project_layout():
    check("包位于 src/air/gesture_control_drone", os.path.isdir(PKG))
    for rel in ("package.xml", "CMakeLists.txt", "README.md",
                "requirements.txt", "launch/main.launch",
                "config/gesture_control.yaml", "scripts/main.py"):
        check("存在 %s" % rel, os.path.isfile(os.path.join(PKG, rel)))

    launch_path = os.path.join(PKG, "launch", "main.launch")
    launch = read(launch_path)
    import xml.etree.ElementTree as ET
    try:
        root = ET.parse(launch_path).getroot()
        check("launch/main.launch 是合法 XML", True)
    except ET.ParseError as exc:
        root = None
        check("launch/main.launch 是合法 XML", False, str(exc))
    if root is not None:
        types_ = [n.get("type") for n in root.findall("node")]
        check('launch 用 type="main.py" 启动入口', "main.py" in types_,
              str(types_))
        check("launch 里没有带路径的 type",
              all("/" not in t for t in types_), str(types_))
        for t in types_:
            check("launch 引用的 %s 存在于 scripts/" % t,
                  os.path.isfile(os.path.join(SCRIPTS, t)))
        check("launch 暴露 airsim_ip 参数", 'name="airsim_ip"' in launch)

    cml = read(os.path.join(PKG, "CMakeLists.txt"))
    check("CMakeLists 使用 catkin_install_python",
          "catkin_install_python" in cml)
    active = "\n".join(ln for ln in cml.split("\n")
                       if not ln.strip().startswith("#"))
    missing = [n for n in script_files() if "scripts/" + n not in active]
    check("CMakeLists 安装了 scripts/ 下的全部脚本", not missing, str(missing))
    check("CMakeLists 安装 launch 与 config 目录",
          "install(DIRECTORY launch config" in active.replace("\n", " "))

    # catkin_install_python 在 devel 空间生成的是启动器，辅助模块不能这样装，
    # 否则 `from drone_client import AirSimDrone` 会 ImportError。
    block = re.search(r"catkin_install_python\(PROGRAMS(.*?)\)", active, re.S)
    listed = block.group(1) if block else ""
    helpers = ("camera_utils.py", "gesture_detection.py", "gesture_gate.py",
               "gesture_map.py", "drone_client.py")
    bad = [h for h in helpers if h in listed]
    check("辅助模块不交给 catkin_install_python（否则 devel 里变成启动器）",
          not bad, str(bad))
    check("辅助模块改用 install(FILES)",
          "install(FILES" in active and all(h in active for h in helpers))

    pkg = read(os.path.join(PKG, "package.xml"))
    check("package.xml 没有 TODO 占位符",
          "TODO" not in pkg and "user@todo.todo" not in pkg)
    check("package.xml 声明 nav_msgs 依赖（状态回传）", "nav_msgs" in pkg)
    check("package.xml 声明 python3-opencv 运行依赖", "python3-opencv" in pkg)
    check("package.xml 声明 std_msgs 依赖", "std_msgs" in pkg)

    req = read(os.path.join(PKG, "requirements.txt"))
    for dep in ("mediapipe", "airsim"):
        listed = re.search(r"^%s\b" % dep, req, re.M) is not None
        if dep == "mediapipe":
            # mediapipe 1.x 移除了 mp.solutions（老 Hands 接口），必须钉 <1.0
            listed = listed and "<1.0" in req.replace(" ", "")
        check("requirements.txt 列出 %s（mediapipe 需钉 <1.0）" % dep, listed)

    cfg = read(os.path.join(PKG, "config", "gesture_control.yaml"))
    for key in ("camera_index", "airsim", "estop_action", "command_cooldown",
                "stable_frames", "command_topic", "takeoff_altitude",
                "max_altitude", "min_altitude", "state_rate", "odom_topic"):
        check("config 里包含 %s" % key, key in cfg)
    try:
        import yaml
        data = yaml.safe_load(cfg)
        check("config 是合法 YAML", isinstance(data, dict))
        check("config 的 airsim.ip 可配置",
              isinstance(data.get("airsim"), dict) and "ip" in data["airsim"])
        check("config 的起飞高度是数值",
              isinstance(data.get("takeoff_altitude"), (int, float)))
    except ImportError:
        print("[跳过] 未安装 PyYAML，跳过 YAML 解析检查")

    doc = os.path.join(REPO, "docs", "air", "gesture_control_drone.md")
    check("文档位于 docs/air/gesture_control_drone.md", os.path.isfile(doc))
    text = read(doc)
    fences = [ln for ln in text.split("\n") if ln.strip().startswith("```")]
    check("文档的代码围栏成对", len(fences) % 2 == 0, "围栏数 %d" % len(fences))
    check("文档写明 7 种手势与指令对照表",
          all(c in text for c in ("takeoff", "land", "hover", "forward", "up",
                                  "down", "estop")))
    check("文档包含大模型使用声明", "大模型" in text and "声明" in text)
    check("文档没有 TODO 占位符", "TODO" not in text)

    mk = os.path.join(REPO, "mkdocs.yml")
    snippet = os.path.join(REPO, "mkdocs_nav_snippet.yml")
    if os.path.isfile(mk):
        check("mkdocs.yml 已登记本文档",
              "gesture_control_drone.md" in read(mk))
    else:
        check("提供了 mkdocs nav 片段且指向本文档",
              "gesture_control_drone.md" in read(snippet))

    readme = read(os.path.join(PKG, "README.md"))
    check("模块 README 说明了 airsim_ip 跨机配置", "airsim_ip" in readme)
    check("模块 README 写明状态回传话题", "/drone/odom" in readme)


def main():
    print("=" * 70)
    print("gesture_control_drone 本地回归测试（无需 ROS / 摄像头 / AirSim）")
    print("=" * 70)
    for title, fn in [
            ("1. 语法与静态约定", test_syntax_and_static),
            ("2. 词表一致性", test_vocabulary_contract),
            ("3. 手势分类", test_classification),
            ("4. 指令闸门（去抖/边沿/冷却）", test_command_gate),
            ("5. 摄像头（跨平台后端）", test_camera),
            ("6. AirSim 封装（动作/限幅/超时/状态）", test_drone_client),
            ("6.5 AirSim API 兼容（async / sync 自动回退）", test_api_compat),
            ("6.6 降落超时（慢下降不误判）", test_land_slow_descent),
            ("6.7 同步回退的参数签名", test_sync_move_signature),
            ("7. 端到端：手势 -> 指令 -> AirSim", test_end_to_end),
            ("8. 两个节点的契约 + 状态回传", test_nodes),
            ("9. 叠加层不丢帧", test_overlay_not_lost),
            ("10. 工程文件约定", test_project_layout)]:
        print("\n-- %s --" % title)
        fn()

    total = len(PASSED) + len(FAILED)
    print("\n" + "=" * 70)
    print("结果：%d/%d 通过" % (len(PASSED), total))
    if FAILED:
        print("失败项：")
        for name in FAILED:
            print("  - %s" % name)
    print("=" * 70)
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
