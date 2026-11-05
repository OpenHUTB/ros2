#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跨平台摄像头工具。

修复点（原 ``camera_utils.py``）
--------------------------------
原实现**无条件**使用 ``cv2.CAP_DSHOW``：那是 Windows 的 DirectShow 后端，
在文档声明的 Ubuntu 20.04 环境下不可用，``isOpened()`` 恒为 False，
5 个索引全部失败后 ``raise``，节点一启动就退出。
现在按平台选择后端（Windows 用 DirectShow，Linux/macOS 用默认 V4L2/AVFoundation），
并允许用参数或环境变量覆盖。

本文件不 ``import rospy``，可以脱离 ROS 单独测试。
"""
from __future__ import annotations

import os
import sys

import cv2

DEFAULT_INDICES = (0, 1, 2, 3, 4)
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480


def choose_backend(platform_name=None):
    """返回当前平台应使用的 OpenCV 采集后端常量。

    * Windows -> ``cv2.CAP_DSHOW``（DirectShow，避免默认 MSMF 的延迟/占用问题）
    * 其它    -> ``cv2.CAP_ANY``（Linux 走 V4L2，macOS 走 AVFoundation）

    可用环境变量 ``GESTURE_CAMERA_BACKEND`` 覆盖（填 ``CAP_DSHOW`` /
    ``CAP_V4L2`` / ``CAP_ANY`` 等名字）。
    """
    override = os.environ.get("GESTURE_CAMERA_BACKEND", "").strip()
    if override:
        value = getattr(cv2, override, None)
        if isinstance(value, int):
            return value
    if platform_name is None:
        platform_name = sys.platform
    if str(platform_name).startswith("win"):
        return getattr(cv2, "CAP_DSHOW", 700)
    return getattr(cv2, "CAP_ANY", 0)


def open_capture(index, backend=None, width=DEFAULT_WIDTH,
                 height=DEFAULT_HEIGHT, fourcc=None):
    """按指定后端打开摄像头并配置格式/分辨率；打开失败返回 ``None``。

    默认把像素格式设成 **MJPG**：VMware/VirtualBox 透传的摄像头（以及不少
    UVC 摄像头）在默认的 YUYV 720p 下带宽不够，``cap.read()`` 会一直
    ``select() timeout``（设备能 open、但读不到帧）。MJPG 是压缩帧，
    带宽只需 1/5~1/10，实测 640x480 MJPG 立刻出画面。

    可用环境变量 ``GESTURE_CAMERA_FOURCC`` 覆盖（例如 ``YUYV``）。
    """
    if backend is None:
        backend = choose_backend()
    if fourcc is None:
        fourcc = os.environ.get("GESTURE_CAMERA_FOURCC", "MJPG").strip() or None
    cap = cv2.VideoCapture(int(index), backend)
    if not cap.isOpened():
        cap.release()
        return None
    if fourcc:
        try:
            cap.set(cv2.CAP_PROP_FOURCC,
                    cv2.VideoWriter_fourcc(*fourcc[:4].ljust(4)))
        except Exception:               # 后端不支持时忽略，不影响后续设置
            pass
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap


class Camera(object):
    """摄像头封装：自动扫描可用索引、镜像翻转、安全释放。"""

    def __init__(self, camera_index=0, width=DEFAULT_WIDTH,
                 height=DEFAULT_HEIGHT, indices=None, backend=None):
        self.camera_index = int(camera_index)
        self.width = int(width)
        self.height = int(height)
        self.indices = tuple(indices) if indices else DEFAULT_INDICES
        self.backend = backend
        self.cap = None

    def initialize(self):
        """打开摄像头；成功返回 True，失败抛 ``RuntimeError``（含排查提示）。"""
        order = [self.camera_index] + [i for i in self.indices
                                       if i != self.camera_index]
        tried = []
        for idx in order:
            cap = open_capture(idx, backend=self.backend, width=self.width,
                               height=self.height)
            tried.append(idx)
            if cap is not None:
                self.cap = cap
                self.camera_index = idx
                return True
        raise RuntimeError(
            "无法打开摄像头（已尝试索引 %s，后端=%s）。请检查："
            "1) 摄像头是否被其它程序占用；2) 设备是否已连接（Linux: ls /dev/video*）；"
            "3) 容器/虚拟机是否已把 USB 摄像头透传进来；"
            "4) 可用 GESTURE_CAMERA_BACKEND 指定后端。" % (tried, self.backend))

    def is_opened(self):
        return self.cap is not None and self.cap.isOpened()

    def read_frame(self):
        """读取一帧并做水平镜像；失败返回 ``None``（不再抛异常，交给调用方重试）。"""
        if self.cap is None:
            raise RuntimeError("摄像头未初始化，请先调用 initialize()")
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return None
        return cv2.flip(frame, 1)

    def release(self):
        if self.cap is not None:
            try:
                self.cap.release()
            finally:
                self.cap = None


def main():
    """命令行自检：打开摄像头打开窗口，按 q 退出。"""
    camera = Camera()
    try:
        camera.initialize()
        print("摄像头已打开，索引 %d，按 q 退出" % camera.camera_index)
        while True:
            frame = camera.read_frame()
            if frame is None:
                print("读取失败，重试…")
                continue
            cv2.imshow("camera", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except RuntimeError as exc:
        print("错误：%s" % exc)
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
