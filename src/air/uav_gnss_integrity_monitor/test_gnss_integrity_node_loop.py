"""ROS 2 节点闭环测试：把消息喂进 /drone/gnss，检查三个话题到底发了什么。

为什么单独一个文件：`test_gnss_integrity_monitor.py` 测的是 `integrity_math` 的纯函数，
测不到「订阅进来的 NavSatFix 有没有真的变成判据的输入、门控结论有没有真的发出去、
完全收不到消息时看门狗会不会自己进 HOLD」这一层。这一层只有把节点真跑起来才测得到。

本机没有 ROS 2，所以这里用一层进程内最小 rclpy 兼容层
（init / Node / Publisher / Subscription / Timer / Parameter / Clock / 消息类），
节点代码一行不改地运行：话题在进程内投递，定时器与时钟由测试手工推进，
逐拍确定、不联网、不开端口。

兼容层不是 ROS 的替代品：真实 DDS 收发、`colcon build` 与 `ros2 launch` 仍要在装了
ROS 2 的机器上验证，命令见 `docs/air/uav_gnss_integrity_monitor.md` 第 6 节。

运行（在本模块目录下）：
    python -m unittest -v test_gnss_integrity_node_loop.py
    python -m pytest -v test_gnss_integrity_node_loop.py   # 如已安装 pytest
"""

from __future__ import annotations

import ast
import json
import sys
import types
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

MODULE_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(MODULE_DIR))

GNSS_TOPIC = "/drone/gnss"
SAT_TOPIC = "/drone/gnss_satellites"
POSE_TOPIC = "/drone/pose_validated"
VALID_TOPIC = "/drone/gnss_valid"
STATUS_TOPIC = "/drone/gnss_status"


# --------------------------------------------------------------- 最小 rclpy 兼容层


class Clock:
    """可控时钟：测试自己决定「现在几点」，看门狗才能逐拍验证。"""

    def __init__(self) -> None:
        self.seconds = 0.0

    def now(self) -> "Time":
        return Time(self.seconds)

    def advance(self, dt: float) -> None:
        self.seconds += dt


class Time:
    def __init__(self, seconds: float) -> None:
        self._seconds = float(seconds)

    @property
    def nanoseconds(self) -> int:
        return int(round(self._seconds * 1e9))

    def to_msg(self) -> "TimeMsg":
        sec = int(self._seconds)
        return TimeMsg(sec, int(round((self._seconds - sec) * 1e9)))


class TimeMsg:
    def __init__(self, sec: int = 0, nanosec: int = 0) -> None:
        self.sec = sec
        self.nanosec = nanosec


class Header:
    def __init__(self, stamp: TimeMsg | None = None, frame_id: str = "") -> None:
        self.stamp = stamp if stamp is not None else TimeMsg()
        self.frame_id = frame_id


class Vector3:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Quaternion:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 0.0


class Pose:
    def __init__(self) -> None:
        self.position = Vector3()
        self.orientation = Quaternion()


class PoseStamped:
    def __init__(self) -> None:
        self.header = Header()
        self.pose = Pose()


class NavSatStatus:
    STATUS_NO_FIX = -1
    STATUS_FIX = 0
    STATUS_SBAS_FIX = 1
    STATUS_GBAS_FIX = 2

    def __init__(self, status: int = 0, service: int = 1) -> None:
        self.status = status
        self.service = service


class NavSatFix:
    COVARIANCE_TYPE_UNKNOWN = 0
    COVARIANCE_TYPE_APPROXIMATED = 1
    COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
    COVARIANCE_TYPE_KNOWN = 3

    def __init__(
        self,
        latitude: float = 0.0,
        longitude: float = 0.0,
        altitude: float = 0.0,
        status: NavSatStatus | None = None,
        position_covariance: list | None = None,
        position_covariance_type: int = 2,
        header: Header | None = None,
    ) -> None:
        self.header = header if header is not None else Header()
        self.status = status if status is not None else NavSatStatus()
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.position_covariance = (
            position_covariance
            if position_covariance is not None
            else [0.25, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.25]
        )
        self.position_covariance_type = position_covariance_type


class Bool:
    def __init__(self, data: bool = False) -> None:
        self.data = data


class Int32:
    def __init__(self, data: int = 0) -> None:
        self.data = data


class String:
    def __init__(self, data: str = "") -> None:
        self.data = data


class Logger:
    def __init__(self) -> None:
        self.records: list[tuple[str, str]] = []

    def _record(self, level: str, message: str) -> None:
        self.records.append((level, message))

    def info(self, message: str) -> None:
        self._record("info", message)

    def warn(self, message: str) -> None:
        self._record("warn", message)

    def error(self, message: str) -> None:
        self._record("error", message)

    def debug(self, message: str) -> None:
        self._record("debug", message)


class Parameter:
    def __init__(self, name: str, value) -> None:
        self.name = name
        self.value = value


class Publisher:
    def __init__(self, msg_type, topic: str) -> None:
        self.msg_type = msg_type
        self.topic = topic
        self.sent: list = []

    def publish(self, message) -> None:
        self.sent.append(message)


class Subscription:
    def __init__(self, msg_type, topic: str, callback) -> None:
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback

    def deliver(self, message) -> None:
        """模拟 DDS 收到一条消息。"""
        self.callback(message)


class Timer:
    def __init__(self, period: float, callback) -> None:
        self.period = period
        self.callback = callback
        self.fired = 0

    def fire(self) -> None:
        self.fired += 1
        self.callback()


CLOCK = Clock()
OVERRIDES: dict[str, object] = {}


class StubNode:
    """只实现本模块节点真正用到的那些原语。"""

    def __init__(self, name: str) -> None:
        self.node_name = name
        self._parameters: dict[str, Parameter] = {}
        self._publishers: list[Publisher] = []
        self._subscriptions: list[Subscription] = []
        self._timers: list[Timer] = []
        self._logger = Logger()

    # -- 参数：declare_parameter 时用 launch 想传的值覆盖默认值
    def declare_parameter(self, name: str, default=None):
        value = OVERRIDES.get(name, default)
        self._parameters.setdefault(name, Parameter(name, value))
        return self._parameters[name]

    def get_parameter(self, name: str) -> Parameter:
        if name not in self._parameters:
            raise KeyError(f"未声明的参数：{name}")
        return self._parameters[name]

    def set_parameter_value(self, name: str, value) -> None:
        self._parameters[name] = Parameter(name, value)

    # -- 收发
    def create_publisher(self, msg_type, topic: str, qos) -> Publisher:
        publisher = Publisher(msg_type, topic)
        self._publishers.append(publisher)
        return publisher

    def create_subscription(self, msg_type, topic: str, callback, qos) -> Subscription:
        subscription = Subscription(msg_type, topic, callback)
        self._subscriptions.append(subscription)
        return subscription

    def create_timer(self, period: float, callback) -> Timer:
        timer = Timer(period, callback)
        self._timers.append(timer)
        return timer

    def get_logger(self) -> Logger:
        return self._logger

    def get_clock(self) -> Clock:
        return CLOCK

    def destroy_node(self) -> None:
        pass


def install_rclpy_stub() -> None:
    """把 rclpy 与三个消息包装进 sys.modules，节点才能被导入。"""
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda args=None: None
    rclpy.ok = lambda: True
    rclpy.shutdown = lambda: None
    rclpy.spin = lambda node: None
    rclpy.spin_once = lambda node, timeout_sec=0.0: None

    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = StubNode
    rclpy.node = rclpy_node

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.PoseStamped = PoseStamped
    geometry_msgs.msg = geometry_msgs_msg

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.NavSatFix = NavSatFix
    sensor_msgs_msg.NavSatStatus = NavSatStatus
    sensor_msgs.msg = sensor_msgs_msg

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = Bool
    std_msgs_msg.Int32 = Int32
    std_msgs_msg.String = String
    std_msgs.msg = std_msgs_msg

    for name, module in (
        ("rclpy", rclpy),
        ("rclpy.node", rclpy_node),
        ("geometry_msgs", geometry_msgs),
        ("geometry_msgs.msg", geometry_msgs_msg),
        ("sensor_msgs", sensor_msgs),
        ("sensor_msgs.msg", sensor_msgs_msg),
        ("std_msgs", std_msgs),
        ("std_msgs.msg", std_msgs_msg),
    ):
        sys.modules[name] = module


install_rclpy_stub()

from uav_gnss_integrity_monitor.gnss_integrity_node import (  # noqa: E402
    GnssIntegrityNode,
    main as node_main,
)

# --------------------------------------------------------------- 测试用的小工具


def make_node(**overrides) -> GnssIntegrityNode:
    """按 launch 传参的方式构造节点，并把时钟与发布记录清空。"""
    OVERRIDES.clear()
    OVERRIDES.update(overrides)
    CLOCK.seconds = 0.0
    return GnssIntegrityNode()


def publisher(node: GnssIntegrityNode, topic: str) -> Publisher:
    for item in node._publishers:
        if item.topic == topic:
            return item
    raise AssertionError(f"没有注册话题 {topic}")


def subscription(node: GnssIntegrityNode, topic: str) -> Subscription:
    for item in node._subscriptions:
        if item.topic == topic:
            return item
    raise AssertionError(f"没有订阅话题 {topic}")


def timer(node: GnssIntegrityNode) -> Timer:
    assert len(node._timers) == 1, node._timers
    return node._timers[0]


def navigation_fix(
    t: float,
    latitude: float = 0.0,
    longitude: float = 0.0,
    altitude: float = 5.0,
    status: int = NavSatStatus.STATUS_FIX,
    covariance: list | None = None,
    covariance_type: int = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN,
    frame_id: str = "map",
) -> NavSatFix:
    """构造一帧 NavSatFix；t 为秒，写进 header.stamp。"""
    sec = int(t)
    msg = NavSatFix(
        latitude=latitude,
        longitude=longitude,
        altitude=altitude,
        status=NavSatStatus(status, 1),
        position_covariance=covariance,
        position_covariance_type=covariance_type,
        header=Header(TimeMsg(sec, int(round((t - sec) * 1e9))), frame_id),
    )
    return msg


def feed(node: GnssIntegrityNode, msg: NavSatFix) -> None:
    """把一帧定位投递给节点，等价于 DDS 送来一条 /drone/gnss。"""
    subscription(node, GNSS_TOPIC).deliver(msg)


def feed_satellites(node: GnssIntegrityNode, count: int) -> None:
    subscription(node, SAT_TOPIC).deliver(Int32(count))


def last_pose(node: GnssIntegrityNode) -> PoseStamped:
    return publisher(node, POSE_TOPIC).sent[-1]


def last_valid(node: GnssIntegrityNode) -> bool:
    return publisher(node, VALID_TOPIC).sent[-1].data


def last_status(node: GnssIntegrityNode) -> dict:
    return json.loads(publisher(node, STATUS_TOPIC).sent[-1].data)


def lat_for(y_meters: float) -> float:
    """把「北向多少米」换成纬度偏移，便于按米描述位移。"""
    return y_meters / 110540.0


# --------------------------------------------------------------- A. 节点装配


class TestNodeWiring(unittest.TestCase):
    def test_node_name_and_topics(self):
        node = make_node()
        self.assertEqual(node.node_name, "gnss_integrity_monitor")
        topics = sorted(item.topic for item in node._publishers)
        self.assertEqual(
            topics, sorted([POSE_TOPIC, VALID_TOPIC, STATUS_TOPIC])
        )
        types_by_topic = {item.topic: item.msg_type for item in node._publishers}
        self.assertIs(types_by_topic[POSE_TOPIC], PoseStamped)
        self.assertIs(types_by_topic[VALID_TOPIC], Bool)
        self.assertIs(types_by_topic[STATUS_TOPIC], String)

    def test_subscriptions(self):
        node = make_node()
        subs = {item.topic: item.msg_type for item in node._subscriptions}
        self.assertEqual(sorted(subs), sorted([GNSS_TOPIC, SAT_TOPIC]))
        self.assertIs(subs[GNSS_TOPIC], NavSatFix)
        self.assertIs(subs[SAT_TOPIC], Int32)

    def test_watchdog_timer_is_10hz(self):
        node = make_node()
        self.assertAlmostEqual(timer(node).period, 0.1)

    def test_launch_parameters_reach_config(self):
        node = make_node(max_speed=8.0, stale_timeout=2.0, suspect_limit=3,
                         recovery_streak=2, reacquisition_radius=1.0)
        self.assertEqual(node._cfg.max_speed, 8.0)
        self.assertEqual(node._cfg.stale_timeout, 2.0)
        self.assertEqual(node._cfg.suspect_limit, 3)
        self.assertEqual(node._cfg.recovery_streak, 2)
        self.assertEqual(node._cfg.reacquisition_radius, 1.0)

    def test_startup_log_reports_config(self):
        node = make_node(max_speed=9.5)
        logs = " ".join(message for _, message in node._logger.records)
        self.assertIn("9.5", logs)


# --------------------------------------------------------------- B. 一帧消息走完全链路


class TestMessageToTopic(unittest.TestCase):
    def test_first_fix_publishes_pose_and_valid(self):
        node = make_node()
        feed(node, navigation_fix(1.0, latitude=0.0, longitude=0.0, altitude=5.0))

        self.assertEqual(len(publisher(node, POSE_TOPIC).sent), 1)
        self.assertEqual(len(publisher(node, VALID_TOPIC).sent), 1)
        self.assertEqual(len(publisher(node, STATUS_TOPIC).sent), 1)

        pose = last_pose(node)
        self.assertAlmostEqual(pose.pose.position.x, 0.0)
        self.assertAlmostEqual(pose.pose.position.y, 0.0)
        self.assertAlmostEqual(pose.pose.position.z, 5.0)
        self.assertAlmostEqual(pose.pose.orientation.w, 1.0)
        self.assertTrue(last_valid(node))

        status = last_status(node)
        self.assertEqual(status["state"], "TRACKING")
        self.assertTrue(status["valid"])
        self.assertEqual(status["action"], "track")
        self.assertEqual(status["reason"], "quality_ok")

    def test_header_stamp_and_frame_passthrough(self):
        node = make_node()
        feed(node, navigation_fix(1.25, frame_id="drone/map"))
        pose = last_pose(node)
        self.assertEqual(pose.header.stamp.sec, 1)
        self.assertEqual(pose.header.stamp.nanosec, 250000000)
        self.assertEqual(pose.header.frame_id, "drone/map")

    def test_empty_frame_id_falls_back_to_parameter(self):
        node = make_node(frame_id="world")
        feed(node, navigation_fix(1.0, frame_id=""))
        self.assertEqual(last_pose(node).header.frame_id, "world")

    def test_hold_keeps_frame_of_last_good_fix(self):
        """冻结位姿必须沿用可信帧的坐标系，不能跟着坏帧的 header 走。

        冻结下来的是 last_good 的 x/y/z（在该帧坐标系下算出的米制偏移）。
        如果 HOLD 时改用当前坏帧的 frame_id 或参数默认值，同一组数值会被
        解释到另一个坐标系，下游看到的位姿会整体平移。
        """
        node = make_node(frame_id="world", suspect_limit=1)
        feed(node, navigation_fix(1.0, frame_id="map"))
        self.assertEqual(last_pose(node).header.frame_id, "map")

        # 跳变帧换了坐标系
        feed(node, navigation_fix(1.1, latitude=0.001, frame_id="gps/other"))
        self.assertFalse(last_valid(node))
        self.assertEqual(last_status(node)["state"], "HOLD")
        self.assertEqual(last_pose(node).header.frame_id, "map")

        # 跳变帧连坐标系都没填
        feed(node, navigation_fix(1.2, latitude=0.002, frame_id=""))
        self.assertEqual(last_pose(node).header.frame_id, "map")

    def test_watchdog_keeps_frame_of_last_good_fix(self):
        node = make_node(frame_id="world", stale_timeout=1.0)
        feed(node, navigation_fix(1.0, frame_id="drone/map"))
        CLOCK.advance(5.0)
        timer(node).fire()
        self.assertEqual(last_status(node)["state"], "HOLD")
        self.assertEqual(last_status(node)["reason"], "stream_timeout")
        # 看门狗手里没有当前消息，只能沿用上一次可信帧的坐标系
        self.assertEqual(last_pose(node).header.frame_id, "drone/map")

    def test_local_xy_uses_origin_parameters(self):
        node = make_node(origin_lat=0.0, origin_lon=0.0, origin_alt=10.0)
        # 纬度 +0.0001° ≈ 北向 11.054 m；经度 +0.0002° ≈ 东向 22.264 m（赤道）
        feed(node, navigation_fix(1.0, latitude=0.0001, longitude=0.0002,
                                 altitude=15.0))
        pose = last_pose(node)
        self.assertAlmostEqual(pose.pose.position.x, 0.0002 * 111320.0, places=3)
        self.assertAlmostEqual(pose.pose.position.y, 0.0001 * 110540.0, places=3)
        self.assertAlmostEqual(pose.pose.position.z, 5.0, places=6)


# --------------------------------------------------------------- C. 精度与定位类型


class TestQualityMapping(unittest.TestCase):
    def test_covariance_becomes_accuracy(self):
        # cov[0]=0.25 → σ=0.5 m > accuracy_limit 0.4 → 判为精度不合格
        node = make_node(accuracy_limit=0.4)
        feed(node, navigation_fix(1.0, covariance=[0.25, 0, 0, 0, 0.25, 0, 0, 0, 0.25]))
        self.assertFalse(last_valid(node))
        self.assertEqual(last_status(node)["reason"], "accuracy")

    def test_unknown_covariance_falls_back_to_default_accuracy(self):
        node = make_node(accuracy_limit=0.4, default_accuracy=0.2)
        feed(node, navigation_fix(
            1.0,
            covariance=[99.0, 0, 0, 0, 99.0, 0, 0, 0, 99.0],
            covariance_type=NavSatFix.COVARIANCE_TYPE_UNKNOWN,
        ))
        # 协方差类型未知时不采信协方差，退回 default_accuracy=0.2
        self.assertTrue(last_valid(node))

    def test_no_fix_status_is_rejected(self):
        node = make_node()
        feed(node, navigation_fix(1.0, status=NavSatStatus.STATUS_NO_FIX))
        self.assertFalse(last_valid(node))
        self.assertEqual(last_status(node)["reason"], "fix_type")

    def test_gbas_fix_is_accepted(self):
        node = make_node()
        feed(node, navigation_fix(1.0, status=NavSatStatus.STATUS_GBAS_FIX))
        self.assertTrue(last_valid(node))
        self.assertEqual(last_status(node)["state"], "TRACKING")

    def test_satellites_topic_overrides_default(self):
        node = make_node(min_sats=8)
        # 上游没发卫星数时按 min_sats 处理，刚好达标
        feed(node, navigation_fix(1.0))
        self.assertTrue(last_valid(node))

        feed_satellites(node, 3)
        feed(node, navigation_fix(1.1, latitude=lat_for(0.05)))
        self.assertFalse(last_valid(node))
        self.assertEqual(last_status(node)["reason"], "low_sats")

    def test_satellites_topic_can_restore_quality(self):
        node = make_node(min_sats=8)
        feed(node, navigation_fix(1.0))
        feed_satellites(node, 2)
        feed(node, navigation_fix(1.1, latitude=lat_for(0.05)))
        self.assertFalse(last_valid(node))
        feed_satellites(node, 12)
        feed(node, navigation_fix(1.2, latitude=lat_for(0.10)))
        self.assertTrue(last_valid(node))


# --------------------------------------------------------------- D. 故障注入端到端


class TestFaultInjection(unittest.TestCase):
    def test_position_jump_is_not_forwarded(self):
        node = make_node()
        feed(node, navigation_fix(1.0))
        feed(node, navigation_fix(1.1, latitude=lat_for(25.0)))  # 北向突然 +25 m

        self.assertFalse(last_valid(node))
        self.assertEqual(last_status(node)["state"], "SUSPECT")
        self.assertEqual(last_status(node)["reason"], "position_jump")
        # 输出的位姿必须是冻结的最后可信位姿，不是坏帧
        self.assertAlmostEqual(last_pose(node).pose.position.y, 0.0, places=6)

    def test_two_bad_frames_force_hold(self):
        node = make_node()
        feed(node, navigation_fix(1.0))
        feed(node, navigation_fix(1.1, latitude=lat_for(25.0)))
        feed(node, navigation_fix(1.2, latitude=lat_for(25.0)))

        status = last_status(node)
        self.assertEqual(status["state"], "HOLD")
        self.assertEqual(status["action"], "hover")
        self.assertFalse(last_valid(node))
        self.assertAlmostEqual(last_pose(node).pose.position.y, 0.0, places=6)

    def test_recovery_needs_full_streak(self):
        node = make_node(suspect_limit=1, recovery_streak=3,
                         reacquisition_radius=3.0)
        feed(node, navigation_fix(1.0))
        feed(node, navigation_fix(1.1, latitude=lat_for(25.0)))
        self.assertEqual(last_status(node)["state"], "HOLD")

        for i in range(2):
            feed(node, navigation_fix(1.2 + 0.1 * i, latitude=lat_for(0.02)))
            self.assertEqual(last_status(node)["state"], "HOLD")
            self.assertFalse(last_valid(node))

        feed(node, navigation_fix(1.4, latitude=lat_for(0.04)))
        status = last_status(node)
        self.assertEqual(status["state"], "TRACKING")
        self.assertEqual(status["reason"], "reacquired")
        self.assertTrue(last_valid(node))

    def test_stream_gap_enters_hold(self):
        node = make_node(stale_timeout=1.5)
        feed(node, navigation_fix(1.0))
        feed(node, navigation_fix(3.0))  # 断流 2 s，本帧数值本身合法
        status = last_status(node)
        self.assertEqual(status["state"], "HOLD")
        self.assertEqual(status["reason"], "stream_gap")
        self.assertFalse(last_valid(node))

    def test_output_stays_finite_under_bad_input(self):
        node = make_node()
        feed(node, navigation_fix(1.0))
        feed(node, navigation_fix(1.1, latitude=float("nan")))
        self.assertEqual(last_status(node)["reason"], "non_finite")
        pose = last_pose(node)
        self.assertTrue(all(
            value == value  # NaN != NaN
            for value in (pose.pose.position.x, pose.pose.position.y,
                          pose.pose.position.z)
        ))


# --------------------------------------------------------------- E. 看门狗


class TestWatchdog(unittest.TestCase):
    def test_idle_before_first_fix(self):
        node = make_node()
        for _ in range(5):
            CLOCK.advance(1.0)
            timer(node).fire()
        # 还没收到过任何定位，看门狗不该自己发东西
        self.assertEqual(publisher(node, POSE_TOPIC).sent, [])
        self.assertEqual(publisher(node, VALID_TOPIC).sent, [])

    def test_receive_time_used_instead_of_message_stamp(self):
        """看门狗必须用「本机收到的时刻」判超时，不能拿消息时间戳比节点时钟。

        真实场景里定位时间戳可能来自 GPS 周内秒或仿真时钟，与节点时钟不同源；
        若直接拿消息时间戳减节点时钟，第一次看门狗就会误判成超时并永久 HOLD。
        """
        node = make_node(stale_timeout=1.5)
        CLOCK.seconds = 1000.0
        feed(node, navigation_fix(1.0))  # 消息时间戳 1.0 s，节点时钟 1000.0 s

        CLOCK.advance(0.2)
        timer(node).fire()
        self.assertEqual(last_status(node)["state"], "TRACKING")
        self.assertTrue(last_valid(node))
        self.assertEqual(len(publisher(node, POSE_TOPIC).sent), 1)

    def test_timeout_publishes_hold_once(self):
        node = make_node(stale_timeout=1.5)
        CLOCK.seconds = 1000.0
        feed(node, navigation_fix(1.0))

        CLOCK.advance(1.0)
        timer(node).fire()
        self.assertEqual(len(publisher(node, POSE_TOPIC).sent), 1)
        self.assertTrue(last_valid(node))

        CLOCK.advance(1.0)  # 累计 2.0 s > stale_timeout
        timer(node).fire()
        self.assertEqual(len(publisher(node, POSE_TOPIC).sent), 2)
        self.assertFalse(last_valid(node))
        status = last_status(node)
        self.assertEqual(status["state"], "HOLD")
        self.assertEqual(status["reason"], "stream_timeout")
        self.assertTrue(any(
            level == "warn" for level, _ in node._logger.records
        ))

        CLOCK.advance(1.0)  # 已经在 HOLD，不应重复刷屏
        timer(node).fire()
        self.assertEqual(len(publisher(node, POSE_TOPIC).sent), 2)

    def test_new_fix_after_timeout_recovers(self):
        node = make_node(stale_timeout=1.5, recovery_streak=2,
                         reacquisition_radius=3.0)
        CLOCK.seconds = 1000.0
        feed(node, navigation_fix(1.0))
        CLOCK.advance(5.0)
        timer(node).fire()
        self.assertEqual(last_status(node)["state"], "HOLD")

        CLOCK.seconds = 1003.0
        feed(node, navigation_fix(3.0, latitude=lat_for(0.02)))
        feed(node, navigation_fix(3.1, latitude=lat_for(0.04)))
        self.assertEqual(last_status(node)["state"], "TRACKING")
        self.assertTrue(last_valid(node))


# --------------------------------------------------------------- F. 打包一致性


class TestPackaging(unittest.TestCase):
    """`colcon build` 会因为这些点失败，这里先用静态检查兜住。

    本机没有 ROS 2，跑不了真正的 colcon；这组用例覆盖的是打包配置本身
    （入口点、资源文件、依赖声明、launch 目标）与被启动代码是否对得上。
    """

    def test_entry_point_matches_node_main(self):
        text = (MODULE_DIR / "setup.py").read_text(encoding="utf-8")
        self.assertRegex(
            text,
            r"'gnss_integrity_node\s*=\s*'\s*\n?\s*'?"
            r"uav_gnss_integrity_monitor\.gnss_integrity_node:main",
        )
        self.assertTrue(callable(node_main))

    def test_package_xml_matches_directory_and_dependencies(self):
        tree = ET.parse(MODULE_DIR / "package.xml")
        root = tree.getroot()
        self.assertEqual(root.findtext("name"), MODULE_DIR.name)
        self.assertEqual(root.findtext("version"), "0.0.1")

        declared = {
            (element.text or "").strip()
            for element in root
            if element.tag in {"depend", "exec_depend"}
        }
        # 从节点源码里提取实际 import 的第三方包，逐一核对是否声明
        source = (MODULE_DIR / "uav_gnss_integrity_monitor"
                  / "gnss_integrity_node.py").read_text(encoding="utf-8")
        imported = set()
        for node in ast.walk(ast.parse(source)):
            if isinstance(node, ast.Import):
                imported.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.level == 0:
                imported.add((node.module or "").split(".")[0])
        ros_packages = {
            name for name in imported
            if name.split("_")[0] in {"rclpy", "sensor", "geometry", "std"}
        }
        self.assertEqual(
            ros_packages - declared, set(),
            f"package.xml 缺少依赖：{ros_packages - declared}",
        )

    def test_launch_targets_declared_executable(self):
        launch_path = (MODULE_DIR / "launch"
                       / "uav_gnss_integrity_monitor.launch.py")
        tree = ast.parse(launch_path.read_text(encoding="utf-8"))
        values: dict[str, list[str]] = {}
        parameter_keys: set[str] = set()
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            for keyword in node.keywords:
                if isinstance(keyword.value, ast.Constant) and isinstance(
                    keyword.value.value, str
                ):
                    values.setdefault(keyword.arg, []).append(keyword.value.value)
                if keyword.arg == "parameters" and isinstance(
                    keyword.value, ast.List
                ):
                    for item in keyword.value.elts:
                        if isinstance(item, ast.Dict):
                            for key in item.keys:
                                if isinstance(key, ast.Constant):
                                    parameter_keys.add(key.value)

        self.assertIn(MODULE_DIR.name, values.get("package", []))
        self.assertIn("gnss_integrity_node", values.get("executable", []))
        # launch 传的参数必须都是节点声明过的，不能有拼错的键
        declared = {"max_speed", "max_vertical_speed", "position_margin",
                    "accuracy_limit", "min_fix_type", "min_sats",
                    "stale_timeout", "max_extrapolate_dt", "suspect_limit",
                    "recovery_streak", "reacquisition_radius", "origin_lat",
                    "origin_lon", "origin_alt", "default_accuracy", "frame_id"}
        self.assertEqual(parameter_keys - declared, set())
        self.assertEqual(declared - parameter_keys, set())

    def test_resource_and_setup_cfg_present(self):
        resource = MODULE_DIR / "resource" / MODULE_DIR.name
        self.assertTrue(resource.is_file())
        cfg = (MODULE_DIR / "setup.cfg").read_text(encoding="utf-8")
        self.assertIn(f"lib/{MODULE_DIR.name}", cfg)


class TestModuleMain(unittest.TestCase):
    def test_main_runs_with_stub_runtime(self):
        # 兼容层里的 init/spin/shutdown 都是空操作，这里只验证入口不会抛异常
        self.assertEqual(node_main(), 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
