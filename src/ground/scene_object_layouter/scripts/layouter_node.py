#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""scene_object_layouter 的 ROS 节点。

它做的事很少：读参数 → 算一次布局 → 打印报告 → 写 CSV/JSON → 把结果发到话题。
真正的几何逻辑全在 layouter_core.py 里，那边不依赖 ROS，可以单独做单元测试。

Python 2.7 / 3.x 都能跑（Melodic 与 Noetic 通用）。
"""

from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import io
import os
import sys
import traceback

import rospy
from std_msgs.msg import Empty
from std_msgs.msg import String

import layouter_core as core


#: 内置演示场景：不做任何配置直接 roslaunch 也能看到结果。


def _log(level, message):
    """Melodic（Python 2.7）上给 rospy 的日志消息必须是 byte str。

    rosgraph 的格式化对非 ASCII unicode 调 str() 会抛 UnicodeEncodeError，
    所以这里统一先编码成 utf-8 再交给 rospy。
    """
    text = message
    if sys.version_info[0] < 3:
        if isinstance(text, unicode):  # noqa: F821  (Python 2 才有 unicode)
            text = text.encode("utf-8")
    else:
        text = "%s" % text
    getattr(rospy, level)(text)


class SceneLayouter(object):
    """把场景摆一遍、校验一遍、发出去。"""

    def __init__(self, params=None):
        self.params = params or core.LayouterParams.from_params(rospy.get_param)
        self.scene = None
        self.placed = []
        self.results = []
        self.report = ""
        self.csv_text = ""
        self.json_text = ""
        self._publishers_ready = False

        self.pub_objects = None
        self.pub_summary = None
        self.timer = None

    # ---------------------------------------------------------------- 场景

    def load(self):
        """优先读 scene/file；没有配置就用内置演示场景。"""
        path = self.params.scene_file
        if path:
            path = os.path.expanduser(path)
            if not os.path.isfile(path):
                raise core.LayouterError("场景文件不存在：%s" % path)
            scene = core.load_scene(path)
        else:
            scene = core.parse_scene_text(core.DEMO_SCENE_TEXT, source="builtin:demo_parking_row")

        # launch / 命令行给的场地尺寸优先于文件里的
        scene.field = core.Field(self.params.field_width, self.params.field_length)
        scene.min_gap = self.params.min_gap
        self.scene = scene

    def layout(self):
        """算布局 + 跑四条断言 + 生成三种文本输出。"""
        self.placed = core.layout_scene(self.scene)
        self.results = core.check_layout(self.placed, self.scene.field,
                                         self.scene.min_gap,
                                         expected_count=self.scene.total_objects())
        self.report = core.format_report(self.scene, self.placed, self.results)
        self.csv_text = core.to_csv(self.placed)
        self.json_text = core.to_json(self.scene, self.placed, self.results)

    def write_files(self):
        """按参数把 CSV / JSON 落盘（io/write_files 为 false 时跳过）。"""
        if not self.params.write_files:
            return []
        written = []
        for key, text in (("csv_path", self.csv_text), ("json_path", self.json_text)):
            path = getattr(self.params, key)
            if not path:
                continue
            path = os.path.expanduser(path)
            folder = os.path.dirname(path)
            if folder and not os.path.isdir(folder):
                os.makedirs(folder)
            with io.open(path, "w", encoding="utf-8", newline="") as handle:
                handle.write(text)
            written.append((path, len(text)))
        return written

    # ---------------------------------------------------------------- ROS

    def setup_publishers(self):
        if self._publishers_ready:
            return
        self.pub_objects = rospy.Publisher(self.params.topic_objects, String,
                                           queue_size=1, latch=True)
        self.pub_summary = rospy.Publisher(self.params.topic_summary, String,
                                           queue_size=1, latch=True)
        self._publishers_ready = True

    def publish(self):
        self.setup_publishers()
        self.pub_objects.publish(String(data=core.to_message_text(self.csv_text)))
        self.pub_summary.publish(String(data=core.to_message_text(self.json_text)))

    def on_trigger(self, _message=None):
        """收到 /scene/layout 时重新算一遍（方便改完文件不重启节点）。"""
        try:
            self.load()
            self.layout()
            written = self.write_files()
            self.publish()
            _log("loginfo", "已按触发重算：%d 个物体，%s，写出 %d 个文件"
                 % (len(self.placed),
                    "PASSED" if core.all_passed(self.results) else "FAILED",
                    len(written)))
        except Exception:
            _log("logerr", "触发重算失败：\n%s" % traceback.format_exc())

    def run(self):
        try:
            self.load()
            self.layout()
        except Exception:
            _log("logerr", "场景载入失败：\n%s" % traceback.format_exc())
            return 1

        _log("loginfo", "场景物体布置节点已启动：%d 个物体组，共 %d 个物体"
             % (len(self.scene.groups), len(self.placed)))

        self.setup_publishers()
        self.publish()
        written = self.write_files()
        for path, size in written:
            _log("loginfo", "已写出 %s（%d 字节）" % (path, size))

        print(self.report)
        _log("loginfo", "结论：%s（%d/%d 条断言通过）"
             % ("PASSED" if core.all_passed(self.results) else "FAILED",
                len([item for item in self.results if item["passed"]]),
                len(self.results)))
        _log("loginfo", "订阅 %s 可重新计算；按 Ctrl+C 退出"
             % self.params.topic_trigger)

        rospy.Subscriber(self.params.topic_trigger, Empty, self.on_trigger, queue_size=1)
        rospy.on_shutdown(self.on_shutdown)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.params.publish_hz),
                                  lambda _event: self.publish())
        rospy.spin()
        return 0

    def on_shutdown(self):
        try:
            _log("loginfo", "场景物体布置节点退出")
        except Exception:
            pass


def main():
    rospy.init_node("scene_object_layouter", anonymous=False)
    try:
        node = SceneLayouter()
    except Exception:
        _log("logerr", "参数读取失败：\n%s" % traceback.format_exc())
        return 1
    return node.run()


if __name__ == "__main__":
    sys.exit(main())
