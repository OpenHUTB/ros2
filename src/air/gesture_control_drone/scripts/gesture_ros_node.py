#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手势感知节点：摄像头 -> MediaPipe -> 指令 token -> ``/gesture_command``。

发布什么
--------
发布的是 :mod:`gesture_map` 里的**指令 token**（``takeoff``/``land``/…），
而不是中文手势名——这是修复"两端词表不相交"的关键：消费端
``drone_control_node`` 只认这几个 token。

什么时候发布
------------
经过 :class:`gesture_gate.CommandGate` 的去抖 + 边沿触发 + 冷却：
同一个手势一直举着只会发一条指令，不会以 10 Hz 刷屏（原实现会）。

同时把"当前识别到的手势"发到 ``/gesture_state``（变化时才发），方便
``rostopic echo`` 观察识别效果、排查"为什么没触发"。
"""
from __future__ import annotations

import os
import sys

# catkin_install_python 在 devel 空间给每个脚本生成的是"启动器"，
# 它只保证被启动的文件能跑，不保证同目录的辅助模块能被 import；
# 所以把本文件所在的源码目录放到 sys.path 最前面。
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import cv2
import rospy
from std_msgs.msg import String

import gesture_map as gm
import gesture_mlp_classifier as gmc
from camera_utils import Camera, DEFAULT_HEIGHT, DEFAULT_WIDTH
from gesture_detection import GestureDetector
from gesture_gate import CommandGate

NODE_NAME = "gesture_ros_node"


def _p(name, default):
    return rospy.get_param("~" + name, default)


def describe_model_path(value):
    """日志里怎么描述"要找的模型路径"：留空时别打出空括号。"""
    return value or "自动查找 <包>/models/ 与 install/share/<包>/models/"


class GestureControlNode(object):
    def __init__(self, camera=None, detector=None):
        rospy.init_node(NODE_NAME, anonymous=False)

        self.camera_index = int(_p("camera_index", 0))
        self.width = int(_p("width", DEFAULT_WIDTH))
        self.height = int(_p("height", DEFAULT_HEIGHT))
        self.publish_rate = max(1.0, float(_p("publish_rate", 10.0)))
        self.show_window = bool(_p("show_window", False))
        self.command_topic = _p("command_topic", "/gesture_command")
        self.state_topic = _p("gesture_state_topic", "/gesture_state")

        self.gate = CommandGate(
            stable_frames=_p("stable_frames", 3),
            cooldown=_p("command_cooldown", 1.5))

        # 分类器实现：rules（几何规则，默认）/ mlp（gesture_train.py 训出来的模型）
        self.classifier_name = str(_p("classifier", "rules")).strip().lower()
        self.mlp = None
        self.mlp_threshold = float(_p("mlp/threshold", gmc.DEFAULT_THRESHOLD))

        self.camera = camera or Camera(
            camera_index=self.camera_index, width=self.width, height=self.height)
        self.detector = detector or GestureDetector(
            max_hands=_p("max_hands", 1),
            detection_confidence=_p("detection_confidence", 0.7),
            tracking_confidence=_p("tracking_confidence", 0.5))

        self.pub_cmd = rospy.Publisher(self.command_topic, String, queue_size=10)
        self.pub_state = rospy.Publisher(self.state_topic, String, queue_size=10)

        self._last_gesture = None
        self._frames = 0
        rospy.on_shutdown(self.shutdown)
        # 可能加载失败（还没训练过模型），所以放在日志之前构建
        self._maybe_build_mlp()

        rospy.loginfo("[gesture] 已启动：摄像头索引 %d，%dx%d，%.1f Hz"
                      % (self.camera_index, self.width, self.height,
                         self.publish_rate))
        rospy.loginfo("[gesture] 手势 -> 指令：%s" % gm.describe_map())
        rospy.loginfo("[gesture] 发布 %s（指令）与 %s（识别状态）"
                      % (self.command_topic, self.state_topic))

    # ------------------------------------------------------------------ #
    def _maybe_build_mlp(self):
        """按 ``classifier`` 参数加载 MLP；**出任何问题都回退到规则法**。

        模型缺失是最常见的情况（刚 clone 下来还没训练）；但也要挡住"文件在、
        内容坏了"——0 字节 npz 会抛 EOFError、截断的 npz 会抛 BadZipFile，
        这些都不该让节点起不来。
        """
        if self.classifier_name in ("", "rules", "rule"):
            self.classifier_name = "rules"
            return
        if self.classifier_name not in ("mlp", "model", "nn"):
            rospy.logwarn("[gesture] 未知 classifier=%r，回退到 rules"
                          % self.classifier_name)
            self.classifier_name = "rules"
            return

        # 留空 = 自动按布局找（源码树 / install 的 share/<pkg>/models/）
        model_path = _p("mlp/model_path", "")
        found = gmc.find_model_path(model_path)
        if not found:
            rospy.logwarn("[gesture] classifier=mlp 但没找到模型（%s），回退到 rules；"
                          "先用 gesture_train.py 训练一份"
                          % describe_model_path(model_path))
            self.classifier_name = "rules"
            return
        try:
            self.mlp = gmc.MLPClassifier.from_file(
                found, threshold=self.mlp_threshold)
        except Exception as exc:                       # noqa: BLE001
            # 这里刻意捕获所有异常：加载模型失败的唯一后果应该是"退回规则法"，
            # 而不是让整个感知节点崩掉（断点/坏文件/依赖缺失都算）。
            rospy.logwarn("[gesture] 加载模型失败（%s，%s: %s），回退到 rules"
                          % (found, type(exc).__name__, exc))
            self.classifier_name = "rules"
            return
        self.classifier_name = "mlp"
        rospy.loginfo("[gesture] 分类器 = mlp（%s，阈值 %.2f，类别 %d 个）"
                      % (found, self.mlp_threshold, len(self.mlp.classes)))

    # ------------------------------------------------------------------ #
    def handle_result(self, result, now=None):
        """处理一次识别结果，返回本次**实际发布**的指令（没有则 None）。

        抽成独立方法是为了能脱离 ROS 单测（见 tests/）。
        """
        # classifier=mlp 时，用模型重新判一次（关键点相同，判定逻辑不同）
        if self.mlp is not None:
            gesture, confidence = self.mlp.predict(result.landmarks)
            result.gesture = gesture
            result.command = gm.gesture_to_command(gesture)
            result.confidence = float(confidence)

        if result.gesture != self._last_gesture:
            self._last_gesture = result.gesture
            if result.gesture != gm.GESTURE_NONE:
                rospy.loginfo("[gesture] 识别到 %s" % gm.gesture_cn(result.gesture))
            self.pub_state.publish(String(data=result.gesture))

        command = self.gate.update(result.command, now=now)
        if command is None:
            return None
        self.pub_cmd.publish(String(data=command))
        rospy.loginfo("[gesture] 发布指令 %s（%s）"
                      % (gm.command_cn(command), command))
        return command

    def spin(self):
        camera = self.camera
        try:
            camera.initialize()
        except RuntimeError as exc:
            rospy.logerr("[gesture] %s" % exc)
            rospy.signal_shutdown("camera unavailable")
            return

        rospy.loginfo("[gesture] 摄像头已打开（索引 %d），开始识别"
                      % camera.camera_index)
        rate = rospy.Rate(self.publish_rate)
        misses = 0
        while not rospy.is_shutdown():
            frame = camera.read_frame()
            if frame is None:
                misses += 1
                if misses % 10 == 1:
                    rospy.logwarn("[gesture] 读取摄像头失败 %d 次，继续重试"
                                  % misses)
                rate.sleep()
                continue
            misses = 0
            self._frames += 1

            result = self.detector.detect(frame)
            self.handle_result(result)

            if self.show_window and result.frame is not None:
                cv2.imshow("gesture (press q to quit)", result.frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    rospy.signal_shutdown("user quit")
                    break
            rate.sleep()

    def shutdown(self):
        try:
            self.camera.release()
        finally:
            cv2.destroyAllWindows()
        rospy.loginfo("[gesture] 已退出（处理 %d 帧）" % self._frames)


def main():
    node = GestureControlNode()
    node.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
