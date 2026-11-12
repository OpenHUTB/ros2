#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""passability_check ROS 2 节点的闭环冒烟测试（不需要装 ROS）。

为什么要有这个文件：只测纯函数测不到「节点到底有没有把结果发出去、收到 evaluate 会不会重算」。
本文件用一层进程内的最小 rclpy 兼容层把 `PassabilityNode` 真跑起来：

    参数 → 核算 → 发布 report/summary → 收到 evaluate → 重算 → 再发布

兼容层只实现节点用到的原语（init / Node / Publisher / Subscription / Timer /
Parameter / Logger / QoSProfile），话题在进程内投递、定时器手工触发，所以逐拍确定、无网络、无端口。
真实 rclpy 在线运行需要在装了 ROS 2 的机器上做，命令见 docs/ground/passability_check.md 第 8 节。

运行：
    python3 test/test_passability_node_loop.py       # 在本模块目录下
    python3 tests/test_passability_node_loop.py      # 也能从仓库根目录跑
"""

from __future__ import print_function

import json
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(os.path.dirname(HERE), "src", "ground", "passability_check")
if not os.path.isdir(PKG):
    PKG = os.path.dirname(HERE)
sys.path.insert(0, PKG)

PASSED = []
FAILED = []


def check(name, condition, detail=""):
    if condition:
        PASSED.append(name)
    else:
        FAILED.append("%s %s" % (name, detail))
    return bool(condition)


# ------------------------------------------------------------------ 最小 rclpy 兼容层


class _Logger(object):
    def __init__(self):
        self.records = []

    def _record(self, level, message):
        self.records.append((level, message))

    def info(self, message):
        self._record("info", message)

    def warn(self, message):
        self._record("warn", message)

    def error(self, message):
        self._record("error", message)

    def debug(self, message):
        self._record("debug", message)


class _Parameter(object):
    def __init__(self, name, value):
        self.name = name
        self.value = value


class _Publisher(object):
    def __init__(self, topic, msg_type, qos, bus):
        self.topic = topic
        self.msg_type = msg_type
        self.qos = qos
        self.bus = bus
        self.sent = []

    def publish(self, message):
        self.sent.append(message)
        self.bus.setdefault(self.topic, []).append(message)


class _Subscription(object):
    def __init__(self, msg_type, topic, callback, qos, bus):
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self.qos = qos
        self.bus = bus
        bus.setdefault("__subs__", []).append(self)


class _Timer(object):
    def __init__(self, period, callback):
        self.period = period
        self.callback = callback
        self.fired = 0

    def fire(self):
        self.fired += 1
        self.callback()


BUS = {}
OVERRIDES = {}


class _Node(object):
    """只实现本模块节点真正用到的那些方法。

    话题投递用模块级 `BUS`，参数默认值用 `OVERRIDES` 覆盖（模拟 launch 传参）。
    """

    def __init__(self, name, bus=None):
        self.node_name = name
        self._parameters = {}
        self._publishers = []
        self._subscriptions = []
        self._timers = []
        self._logger = _Logger()
        self.bus = bus if bus is not None else BUS

    # -- 参数
    def declare_parameter(self, name, default=None):
        value = OVERRIDES.get(name, default)
        self._parameters.setdefault(name, _Parameter(name, value))
        return self._parameters[name]

    def get_parameter(self, name):
        if name not in self._parameters:
            raise KeyError("未声明的参数：%s" % name)
        return self._parameters[name]

    def set_parameter_value(self, name, value):
        self._parameters[name] = _Parameter(name, value)

    # -- 收发
    def create_publisher(self, msg_type, topic, qos):
        publisher = _Publisher(topic, msg_type, qos, self.bus)
        self._publishers.append(publisher)
        return publisher

    def create_subscription(self, msg_type, topic, callback, qos):
        subscription = _Subscription(msg_type, topic, callback, qos, self.bus)
        self._subscriptions.append(subscription)
        return subscription

    def create_timer(self, period, callback):
        timer = _Timer(period, callback)
        self._timers.append(timer)
        return timer

    def get_logger(self):
        return self._logger

    def destroy_node(self):
        pass


class _QoSProfile(object):
    def __init__(self, depth=10, history=None, reliability=None, durability=None):
        self.depth = depth
        self.history = history
        self.reliability = reliability
        self.durability = durability


def install_rclpy_stub():
    """把 rclpy / rclpy.node / rclpy.qos / std_msgs.msg 装进 sys.modules。"""
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda args=None: None
    rclpy.ok = lambda: True
    rclpy.shutdown = lambda: None
    rclpy.spin = lambda node: None
    rclpy.spin_once = lambda node, timeout_sec=0.0: None

    node_module = types.ModuleType("rclpy.node")
    node_module.Node = _Node
    rclpy.node = node_module

    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = _QoSProfile
    qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL="transient_local",
                                                VOLATILE="volatile")
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST="keep_last", KEEP_ALL="keep_all")
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE="reliable", BEST_EFFORT="best_effort")
    rclpy.qos = qos

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    class _String(object):
        def __init__(self, data=""):
            self.data = data

    class _Empty(object):
        def __init__(self):
            pass

    std_msgs_msg.String = _String
    std_msgs_msg.Empty = _Empty
    std_msgs.msg = std_msgs_msg

    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = node_module
    sys.modules["rclpy.qos"] = qos
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg
    return rclpy


# ------------------------------------------------------------------ 场景


def build_node(**overrides):
    """按 launch 传参的方式构造节点（兼容层把 OVERRIDES 当成 launch 的参数注入）。"""
    from passability_check.node import PassabilityNode

    # 默认关掉定时器，让每一步都由测试自己驱动，逐拍确定
    overrides.setdefault("publish_hz", 0.0)
    OVERRIDES.clear()
    OVERRIDES.update(overrides)
    BUS.clear()
    node = PassabilityNode()
    check("A03 节点名", node.node_name == "passability_check", node.node_name)
    return node


def run_checks():
    # 0. 兼容层本身可用
    stub = install_rclpy_stub()
    check("A01 兼容层装上 rclpy", "rclpy" in sys.modules)
    check("A02 兼容层有 QoSProfile", hasattr(stub.qos, "QoSProfile"))

    # 1. 构造：参数 → 核算 → 发布就绪
    node = build_node()
    check("B01 报告已生成", bool(node.report_text), node.report_text[:40])
    check("B02 报告含结论", "结论：" in node.report_text)
    check("B03 结论为 FAILED（轿车 / 地库坡道）", node.result == "FAILED", node.result)
    check("B04 两个发布者", len(node._publishers) == 2, len(node._publishers))
    check("B05 主题名正确",
          sorted(item.topic for item in node._publishers) == ["report", "summary"],
          [item.topic for item in node._publishers])
    check("B06 一个订阅者", len(node._subscriptions) == 1, len(node._subscriptions))
    check("B07 订阅 evaluate", node._subscriptions[0].topic == "evaluate",
          node._subscriptions[0].topic)
    check("B08 publish_hz=0 时不起定时器", not node._timers, node._timers)
    check("B09 日志里有结论", any("FAILED" in message for _, message in node._logger.records),
          node._logger.records)

    # 2. 发布：内容与 JSON 结构
    node.publish()
    bus = node.bus
    check("C01 发布了 report", len(bus.get("report", [])) == 1, len(bus.get("report", [])))
    check("C02 发布了 summary", len(bus.get("summary", [])) == 1)
    text = bus["report"][0].data
    payload = json.loads(bus["summary"][0].data)
    check("C03 report 与节点文本一致", text == node.report_text)
    check("C04 summary 是合法 JSON", isinstance(payload, dict))
    check("C05 summary 有 5 条断言", len(payload["assertions"]) == 5,
          len(payload.get("assertions", [])))
    check("C06 summary 结论一致", payload["result"] == node.result)
    check("C07 summary 带场景名", payload["scenario"]["name"] == "garage_ramp",
          payload["scenario"]["name"])

    # 3. 收到 evaluate：重算并重发
    subscriber = node._subscriptions[0]
    subscriber.callback(sys.modules["std_msgs.msg"].Empty())
    check("D01 重发了一次 report", len(bus["report"]) == 2, len(bus["report"]))
    check("D02 重算日志", any("重新核算" in message for _, message in node._logger.records))

    # 4. 运行时换车型：SUV 应当 PASSED
    node.set_parameter_value("vehicle", "suv")
    subscriber.callback(sys.modules["std_msgs.msg"].Empty())
    check("E01 换 SUV 后结论变 PASSED", node.result == "PASSED", node.result)
    check("E02 文本同步更新", "PASSED" in node.report_text and "FAILED" not in node.report_text)
    payload = json.loads(node.bus["summary"][-1].data)
    check("E03 JSON 结论同步", payload["result"] == "PASSED", payload["result"])

    # 5. 换场景 + 安全余量：同一台车也能被判不通过
    node.set_parameter_value("scenario", "speed_bump")
    node.set_parameter_value("safety_margin_m", 0.5)
    subscriber.callback(sys.modules["std_msgs.msg"].Empty())
    check("F01 余量抬高后不通过", node.result == "FAILED", node.result)
    failed = [item["type"] for item in json.loads(node.bus["summary"][-1].data)["assertions"]
              if not item["passed"]]
    check("F02 失败项包含离地间隙与托底", "离地间隙" in failed and "全程不托底" in failed, failed)

    # 6. 参数错误要变成 ERROR，而不是抛异常把节点带崩
    node.set_parameter_value("vehicle", "truck")
    subscriber.callback(sys.modules["std_msgs.msg"].Empty())
    check("G01 未知车型 → ERROR", node.result == "ERROR", node.result)
    check("G02 错误写进日志", any("参数错误" in message for _, message in node._logger.records))
    check("G03 出错不发布脏数据", len(node.bus["report"]) == 4, len(node.bus["report"]))

    # 7. 定时器：给 publish_hz 就应当建定时器，触发时重发
    node2 = build_node(publish_hz=2.0)
    check("H01 有定时器", len(node2._timers) == 1, node2._timers)
    check("H02 周期正确", abs(node2._timers[0].period - 0.5) < 1e-9, node2._timers[0].period)
    node2._timers[0].fire()
    check("H03 定时器触发后发布", len(node2.bus.get("report", [])) == 1,
          len(node2.bus.get("report", [])))

    # 8. 直接读场景文件路径
    scenario_file = os.path.join(PKG, "config", "scenarios", "mountain_crest.json")
    node3 = build_node(scenario_file=scenario_file)
    check("I01 scenario_file 优先", node3.summary_text.find("mountain_crest") > 0,
          node3.result)


def run_all():
    run_checks()
    print("通过 %d 项，失败 %d 项" % (len(PASSED), len(FAILED)))
    if FAILED:
        for item in FAILED:
            print("  [FAIL] %s" % item)
        print("存在失败项")
        return 1
    print("全部通过")
    return 0


def test_node_loop():
    before = len(FAILED)
    run_checks()
    new = FAILED[before:]
    assert not new, "\n".join(new)


if __name__ == "__main__":
    sys.exit(run_all())
