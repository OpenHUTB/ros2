#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""carlair_ros_bridge 诊断工具：检查关键话题频率和 TF 是否正常.

用法：
    rosrun carlair_ros_bridge bridge_doctor.py
    rosrun carlair_ros_bridge bridge_doctor.py --duration 8
    rosrun carlair_ros_bridge bridge_doctor.py --skip-tf

退出码：0 全部通过；1 有 FAIL。
"""
from __future__ import annotations

import argparse
import sys

import rospy
from tf2_ros import Buffer, TransformListener

try:
    from rospy import AnyMsg
except ImportError:  # 老版本 rospy
    AnyMsg = None


DEFAULT_TOPICS = (
    ("topic/odom", "/uav/odom", 5.0),
    ("topic/image", "/camera/image_raw", 1.0),
    ("topic/depth", "/camera/depth", 1.0),
    ("topic/lidar", "/lidar/points", 1.0),
)


def rate_status(actual_hz, min_hz):
    """返回 PASS / LOW / NO_DATA."""
    if actual_hz <= 0.0:
        return "NO_DATA"
    if actual_hz < min_hz:
        return "LOW"
    return "PASS"


def format_table(rows):
    """rows: [(topic, min_hz, actual_hz, status)] -> 对齐的文本表格."""
    header = ("TOPIC", "MIN_HZ", "ACTUAL_HZ", "STATUS")
    data = [header] + [
        (str(t), "%.1f" % float(m), "%.1f" % float(a), str(s))
        for t, m, a, s in rows
    ]
    widths = [max(len(r[i]) for r in data) for i in range(4)]
    lines = []
    for idx, row in enumerate(data):
        line = "  ".join(row[i].ljust(widths[i]) for i in range(4))
        lines.append(line)
        if idx == 0:
            lines.append("  ".join("-" * w for w in widths))
    return "\n".join(lines)


def _topic_specs():
    specs = []
    for param, default, min_hz in DEFAULT_TOPICS:
        name = rospy.get_param(param, default)
        specs.append((name, float(min_hz)))
    return specs


class BridgeDoctor(object):
    def __init__(self, duration=5.0, skip_tf=False):
        self.duration = float(duration)
        self.skip_tf = bool(skip_tf)
        self.counts = {}
        self.subs = []
        for name, _ in _topic_specs():
            self.counts[name] = 0
            self.subs.append(rospy.Subscriber(
                name, AnyMsg, self._make_cb(name), queue_size=10))
        self.tf_buffer = None
        self.tf_listener = None
        if not self.skip_tf:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer)

    def _make_cb(self, name):
        def _cb(_msg):
            self.counts[name] += 1
        return _cb

    def collect(self):
        rospy.sleep(self.duration)
        rows = []
        failed = False
        for name, min_hz in _topic_specs():
            actual = self.counts.get(name, 0) / max(self.duration, 1e-6)
            status = rate_status(actual, min_hz)
            if status != "PASS":
                failed = True
            rows.append((name, min_hz, actual, status))
        return rows, failed

    def check_tf(self):
        if self.skip_tf or self.tf_buffer is None:
            return None
        try:
            self.tf_buffer.lookup_transform(
                "world", "base_link", rospy.Time(0), rospy.Duration(1.0))
            return "PASS"
        except Exception:  # noqa: BLE001 - 诊断工具只关心成功/失败
            return "FAIL"

    def run(self):
        rows, failed = self.collect()
        print(format_table(rows))
        if not self.skip_tf:
            tf_status = self.check_tf()
            print("\nTF world -> base_link: %s" % tf_status)
            if tf_status != "PASS":
                failed = True
        if failed:
            print("\n结果：有 FAIL，请检查仿真是否启动、话题是否发布。")
            return 1
        print("\n结果：全部通过。")
        return 0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="carlair_ros_bridge 诊断工具")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="每个话题的采样时长（秒），默认 5")
    parser.add_argument("--skip-tf", action="store_true",
                        help="跳过 world -> base_link TF 检查")
    return parser.parse_known_args(argv)[0]


def main(argv=None):
    args = parse_args(argv)
    rospy.init_node("bridge_doctor", anonymous=True)
    return BridgeDoctor(duration=args.duration, skip_tf=args.skip_tf).run()


if __name__ == "__main__":
    sys.exit(main())
