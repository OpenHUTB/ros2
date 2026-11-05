#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""基于 MediaPipe 的手势识别 —— **全模块唯一的一份实现**。

修复点
------
1. 原 ``gesture_ros_node.py`` 调用的是 ``detector.detect(frame)``，而类里只有
   ``detect_gestures``，节点第一帧就 ``AttributeError`` 崩掉。现在公共方法
   **只有一个名字** :meth:`GestureDetector.detect`，调用方不会再猜错。
2. 原 ``detect_gestures`` 返回 3 元组，调用方直接把元组发进
   ``std_msgs/String``。现在返回 :class:`GestureResult` 数据类，
   ``.command`` 是可直接发布的字符串，从类型上杜绝这个错误。
3. 原 ``main.py`` 与 ``gesture_detection.py`` 各有一套识别逻辑（阈值、
   手势集合、词表都不同）。现在只保留这一份，``main.py``（现
   ``gesture_demo.py``）改为 import 本模块。
4. 分类逻辑抽成纯函数 :func:`classify_landmarks`，无需摄像头/MediaPipe
   即可离线单测。

需要说明的局限：手指伸展判定用的是"指尖在 PIP 关节上方"的经典启发式，
对**手心朝向镜头、手指大致朝上**的手势最可靠；手旋转 90° 以上时准确率下降。
真机上建议配合 ``stable_frames``（连续多帧一致才发布）使用。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from gesture_map import (CMD_NONE, GESTURE_FIST, GESTURE_NONE, GESTURE_OK,
                         GESTURE_PALM, GESTURE_POINT, GESTURE_THUMB_DOWN,
                         GESTURE_THUMB_UP, GESTURE_VICTORY, gesture_to_command)

# MediaPipe 手部关键点索引
WRIST = 0
THUMB_CMC, THUMB_MCP, THUMB_IP, THUMB_TIP = 1, 2, 3, 4
INDEX_MCP, INDEX_PIP, INDEX_DIP, INDEX_TIP = 5, 6, 7, 8
MIDDLE_MCP, MIDDLE_PIP, MIDDLE_DIP, MIDDLE_TIP = 9, 10, 11, 12
RING_MCP, RING_PIP, RING_DIP, RING_TIP = 13, 14, 15, 16
PINKY_MCP, PINKY_PIP, PINKY_DIP, PINKY_TIP = 17, 18, 19, 20

#: OK 手势判定：拇指尖与食指尖的距离小于"手掌尺度"的这个比例
OK_DISTANCE_RATIO = 0.35
#: 拇指"伸出"判定：拇指尖相对腕部的水平偏移超过手掌尺度的这个比例
THUMB_OFFSET_RATIO = 0.25
#: 拇指朝上/朝下判定：拇指尖相对 IP 关节的竖直偏移超过手掌尺度的这个比例
THUMB_DIRECTION_RATIO = 0.15

Landmarks = List[Tuple[int, int]]


@dataclass
class GestureResult(object):
    """一次识别的结果。

    ``command`` 是可以直接发布/执行的指令 token（见 :mod:`gesture_map`）；
    未识别出手势时为 ``CMD_NONE``，此时 ``bool(result)`` 为 False。
    """

    frame: Optional[np.ndarray] = None
    gesture: str = GESTURE_NONE
    command: str = CMD_NONE
    landmarks: Optional[Landmarks] = field(default=None)

    def __bool__(self):
        return self.command != CMD_NONE


# --------------------------------------------------------------------------- #
# 纯函数：几何与分类（可离线单测，不需要摄像头）
# --------------------------------------------------------------------------- #
def _dist(a, b):
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def palm_scale(landmarks):
    """手掌尺度（腕部 -> 中指根），用来把阈值变成与分辨率/远近无关的比例。

    原实现用固定的 30 像素判 OK 手势，换个分辨率或离镜头远近就失效。
    """
    return max(1.0, _dist(landmarks[WRIST], landmarks[MIDDLE_MCP]))


def finger_extended(landmarks, tip_idx, pip_idx):
    """非拇指手指是否伸出：指尖在 PIP 关节**上方**（图像 y 轴向下）。"""
    return landmarks[tip_idx][1] < landmarks[pip_idx][1]


def thumb_extended(landmarks, handedness=None):
    """拇指是否伸出。

    判据：拇指尖相对**拇指 IP 关节**是否继续向"外侧"延伸（带一个与手掌尺度成
    比例的余量）。方向由 ``handedness`` 指定；不指定时按"拇指尖相对腕部在
    哪一侧"自适应 —— 因为画面做了水平镜像，MediaPipe 给出的左右手标签可能与
    实际操作相反，自适应更稳。

    ``handedness`` 为 ``"Left"``/``"Right"`` 时按解剖学方向判断，供已知
    自身相机朝向的调用方使用。
    """
    scale = palm_scale(landmarks)
    margin = THUMB_OFFSET_RATIO * scale
    wrist_x = landmarks[WRIST][0]
    tip_x = landmarks[THUMB_TIP][0]
    ip_x = landmarks[THUMB_IP][0]
    if handedness == "Left":
        return (ip_x - tip_x) > margin
    if handedness == "Right":
        return (tip_x - ip_x) > margin
    if tip_x >= wrist_x:
        return (tip_x - ip_x) > margin
    return (ip_x - tip_x) > margin


def finger_states(landmarks, handedness=None):
    """返回 ``[thumb, index, middle, ring, pinky]`` 的伸出状态。"""
    return [
        thumb_extended(landmarks, handedness),
        finger_extended(landmarks, INDEX_TIP, INDEX_PIP),
        finger_extended(landmarks, MIDDLE_TIP, MIDDLE_PIP),
        finger_extended(landmarks, RING_TIP, RING_PIP),
        finger_extended(landmarks, PINKY_TIP, PINKY_PIP),
    ]


def is_ok_gesture(landmarks, states=None):
    """OK 手势：拇指尖与食指尖相触，同时中指/无名指/小指伸出。"""
    if states is None:
        states = finger_states(landmarks)
    if not (states[2] and states[3] and states[4]):
        return False
    return _dist(landmarks[THUMB_TIP], landmarks[INDEX_TIP]) < \
        OK_DISTANCE_RATIO * palm_scale(landmarks)


def thumb_direction(landmarks):
    """只有拇指伸出时，判断它朝上还是朝下。

    以拇指 IP 关节为参照、用竖直偏移与手掌尺度成比例的阈值判断；
    方向不明确（手指横指等）返回 ``None``。
    """
    margin = THUMB_DIRECTION_RATIO * palm_scale(landmarks)
    tip_y = landmarks[THUMB_TIP][1]
    ip_y = landmarks[THUMB_IP][1]
    if tip_y < ip_y - margin:      # 图像 y 轴向下，越小越高
        return GESTURE_THUMB_UP
    if tip_y > ip_y + margin:
        return GESTURE_THUMB_DOWN
    return None


def classify_landmarks(landmarks, handedness=None):
    """把 21 个像素坐标关键点分类成手势 token（见 :mod:`gesture_map`）。

    判断顺序：OK 手势 -> 只有拇指伸出（朝上/朝下）-> 按伸出指数分类。
    返回 ``GESTURE_NONE`` 表示"没识别出可用手势"（含关键点不足的情况）。
    """
    if not landmarks or len(landmarks) < 21:
        return GESTURE_NONE

    states = finger_states(landmarks, handedness)
    extended = sum(1 for s in states if s)

    if is_ok_gesture(landmarks, states):
        return GESTURE_OK
    # 只伸出拇指：按朝向区分"悬停/下降"
    if extended == 1 and states[0]:
        return thumb_direction(landmarks) or GESTURE_NONE
    if extended == 5:
        return GESTURE_PALM
    if extended == 0:
        return GESTURE_FIST
    if extended == 1 and states[1]:
        return GESTURE_POINT
    if extended == 2 and states[1] and states[2]:
        return GESTURE_VICTORY
    return GESTURE_NONE


# --------------------------------------------------------------------------- #
# MediaPipe 封装
# --------------------------------------------------------------------------- #
class GestureDetector(object):
    """从视频帧里识别手势。公共方法只有 :meth:`detect`。"""

    def __init__(self, max_hands=1, detection_confidence=0.7,
                 tracking_confidence=0.5):
        try:
            import mediapipe as mp
        except ImportError as exc:      # pragma: no cover - 依赖缺失时的提示
            raise RuntimeError(
                "缺少 mediapipe：pip install mediapipe"
                "（或见 requirements.txt）") from exc
        self._mp = mp
        self._hands_mod = mp.solutions.hands
        self._drawing = mp.solutions.drawing_utils
        self._hands = self._hands_mod.Hands(
            static_image_mode=False,
            max_num_hands=int(max_hands),
            min_detection_confidence=float(detection_confidence),
            min_tracking_confidence=float(tracking_confidence),
        )
        self._draw_spec = self._drawing.DrawingSpec(
            color=(0, 255, 0), thickness=2, circle_radius=2)

    # -- 对外唯一接口 ------------------------------------------------------ #
    def detect(self, frame):
        """识别一帧。返回 :class:`GestureResult`（永不返回元组）。"""
        if frame is None or getattr(frame, "size", 0) == 0:
            return GestureResult(frame=frame)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        results = self._hands.process(rgb)
        rgb.flags.writeable = True

        if not results.multi_hand_landmarks:
            return GestureResult(frame=frame)

        hand = results.multi_hand_landmarks[0]
        self._drawing.draw_landmarks(
            frame, hand, self._hands_mod.HAND_CONNECTIONS,
            self._draw_spec, self._draw_spec)

        landmarks = self.to_pixels(hand, frame.shape)
        # 刻意不传 handedness：画面做了水平镜像，MediaPipe 的左右手标签可能与
        # 实际操作相反，用几何自适应判据更稳（见 thumb_extended 的说明）。
        gesture = classify_landmarks(landmarks)
        return GestureResult(frame=frame, gesture=gesture,
                             command=gesture_to_command(gesture),
                             landmarks=landmarks)

    @staticmethod
    def to_pixels(hand_landmarks, frame_shape):
        """归一化关键点 -> 像素坐标（并裁剪到画面内）。"""
        h, w = int(frame_shape[0]), int(frame_shape[1])
        out = []
        for lm in hand_landmarks.landmark:
            out.append((int(np.clip(lm.x * w, 0, w - 1)),
                        int(np.clip(lm.y * h, 0, h - 1))))
        return out

    def close(self):
        hands = getattr(self, "_hands", None)
        if hands is not None:
            try:
                hands.close()
            except Exception:      # pragma: no cover - 关闭失败无需影响退出
                pass


#: 兼容旧名字，避免"方法名对不上"的老问题再次发生（只暴露一个入口）
