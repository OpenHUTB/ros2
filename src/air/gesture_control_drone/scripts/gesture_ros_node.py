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

import cv2
import rospy
from std_msgs.msg import String

import gesture_map as gm
from camera_utils import Camera, DEFAULT_HEIGHT, DEFAULT_WIDTH
from gesture_detection import GestureDetector
from gesture_gate import CommandGate

NODE_NAME = "gesture_ros_node"


def _p(name, default):
    return rospy.get_param("~" + name, default)


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

        rospy.loginfo("[gesture] 已启动：摄像头索引 %d，%dx%d，%.1f Hz"
                      % (self.camera_index, self.width, self.height,
                         self.publish_rate))
        rospy.loginfo("[gesture] 手势 -> 指令：%s" % gm.describe_map())
        rospy.loginfo("[gesture] 发布 %s（指令）与 %s（识别状态）"
                      % (self.command_topic, self.state_topic))

    # ------------------------------------------------------------------ #
    def handle_result(self, result, now=None):
        """处理一次识别结果，返回本次**实际发布**的指令（没有则 None）。

        抽成独立方法是为了能脱离 ROS 单测（见 tests/）。
        """
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
