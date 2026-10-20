#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""无 ROS 的单机演示：摄像头 + 手势识别 + pygame「虚拟无人机」。

作用：不装 ROS、没有 AirSim 时也能验证手势识别与词表是否正确
（答辩/自测用）。真正的 ROS 链路请用 ``launch/main.launch``。

修复点
------
1. 原 ``main.py`` 自己又实现了一套手势识别（``GestureRecognizer``），
   阈值、手势集合、词表都和 ``gesture_detection.py`` 不同。现在统一 import
   :class:`gesture_detection.GestureDetector` 与 :mod:`gesture_map`。
2. 原 ``_display_info`` 在函数内重新绑定 ``frame`` 后**不返回**，而
   ``put_chinese_on_opencv`` 内部 ``cv2.cvtColor`` 返回的是新数组 ——
   结果"手势/命令/状态/FPS"这些文字**一个都没画到画面上**。
   现在 :meth:`GestureDemo.draw_overlay` 明确 ``return frame``，
   调用处也写成 ``frame = self.draw_overlay(...)``。
3. 中文字体每次绘制都重新 ``truetype`` 加载（一帧 4 次）。现在按字号缓存。
4. 删掉未使用的导入（``KEYDOWN`` / ``Dict`` / ``Union``）。
"""
from __future__ import annotations

import os
import sys
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np

import gesture_map as gm
from camera_utils import Camera
from gesture_detection import GestureDetector

try:                                     # pygame 只在本演示里需要
    import pygame
    from PIL import Image, ImageDraw, ImageFont
except ImportError as _exc:              # pragma: no cover
    pygame = None
    _PYGAME_IMPORT_ERROR = _exc
else:
    _PYGAME_IMPORT_ERROR = None

ColorType = Tuple[int, int, int]
PositionType = List[int]

_FONT_CACHE = {}
_FONT_CANDIDATES = (
    "C:/Windows/Fonts/simhei.ttf",
    "simhei.ttf",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/wqy-zenhei/wqy-zenhei.ttc",
    "/System/Library/Fonts/PingFang.ttc",
)


def get_chinese_font(font_size: int = 24):
    """按字号缓存地获取中文字体（原实现每帧都重新加载）。"""
    key = int(font_size)
    if key in _FONT_CACHE:
        return _FONT_CACHE[key]
    if pygame is None:
        raise RuntimeError("缺少 pygame/Pillow：%s" % _PYGAME_IMPORT_ERROR)
    font = None
    for path in _FONT_CANDIDATES:
        try:
            font = ImageFont.truetype(path, key, encoding="utf-8")
            break
        except (OSError, IOError):
            continue
    if font is None:
        print("[warn] 未找到中文字体，中文可能显示为方块")
        font = ImageFont.load_default()
    _FONT_CACHE[key] = font
    return font


def put_chinese(frame: np.ndarray, text: str, position: Tuple[int, int],
                font_size: int = 24,
                color: Tuple[int, int, int] = (0, 255, 0)) -> np.ndarray:
    """在 OpenCV 帧上写中文。

    **返回一个新数组**（``cv2.cvtColor`` 的语义），调用方必须接收返回值。
    """
    pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    ImageDraw.Draw(pil).text(position, text, font=get_chinese_font(font_size),
                             fill=(color[2], color[1], color[0]))
    return cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)


class VirtualDrone(object):
    """pygame 里的一台"虚拟无人机"，用来在没有 AirSim 时演示动作。"""

    WINDOW_WIDTH = 420
    WINDOW_HEIGHT = 320
    SPEED = 3
    BG_COLOR: ColorType = (30, 30, 50)
    GROUND_COLOR: ColorType = (50, 50, 70)
    FLYING_COLOR: ColorType = (0, 220, 120)
    GROUNDED_COLOR: ColorType = (230, 120, 120)
    TEXT_COLOR: ColorType = (240, 240, 240)

    def __init__(self):
        if pygame is None:
            raise RuntimeError("缺少 pygame：%s" % _PYGAME_IMPORT_ERROR)
        pygame.init()
        self.screen = pygame.display.set_mode((self.WINDOW_WIDTH,
                                               self.WINDOW_HEIGHT))
        pygame.display.set_caption("虚拟无人机（gesture demo）")
        self.position: PositionType = [self.WINDOW_WIDTH // 2, 160]
        self.altitude = 0.0
        self.battery = 100.0
        self.is_flying = False
        self.yaw = 0                          # 仅用于演示偏航/返航
        self.last_command = gm.CMD_NONE

    def execute_command(self, command) -> bool:
        """按词表指令 token 更新虚拟状态；返回是否真的产生了动作。"""
        if command == gm.CMD_TAKEOFF and not self.is_flying:
            self.is_flying, self.altitude = True, 10.0
        elif command == gm.CMD_LAND and self.is_flying:
            self.is_flying, self.altitude = False, 0.0
        elif command == gm.CMD_FORWARD and self.is_flying:
            self.position[1] = max(60, self.position[1] - self.SPEED)
            self.altitude = min(50.0, self.altitude + 0.5)
        elif command == gm.CMD_UP and self.is_flying:
            self.altitude = min(100.0, self.altitude + 10.0)
        elif command == gm.CMD_DOWN and self.is_flying:
            self.altitude = max(0.0, self.altitude - 10.0)
            if self.altitude <= 0.0:
                self.is_flying = False
        elif command == gm.CMD_HOVER and self.is_flying:
            pass                                  # 虚拟无人机本来就"停得住"
        elif command in (gm.CMD_YAW_LEFT, gm.CMD_YAW_RIGHT) and self.is_flying:
            self.yaw += -15 if command == gm.CMD_YAW_LEFT else 15
        elif command == gm.CMD_HOME and self.is_flying:
            self.position[1] = 160
            self.yaw = 0
        elif command == gm.CMD_ESTOP:
            self.is_flying, self.altitude = False, 0.0
        else:
            return False
        self.last_command = command
        if self.is_flying:
            self.battery = max(0.0, self.battery - 0.05)
        return True

    def pump_events(self) -> bool:
        """处理窗口事件；返回 False 表示用户关窗。"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
        return True

    def draw(self, gesture_cn: str, command_cn: str) -> None:
        self.screen.fill(self.BG_COLOR)
        pygame.draw.rect(self.screen, self.GROUND_COLOR,
                         (0, self.WINDOW_HEIGHT - 80, self.WINDOW_WIDTH, 80))
        drone_y = self.WINDOW_HEIGHT - 100 - int(self.altitude * 1.8)
        color = self.FLYING_COLOR if self.is_flying else self.GROUNDED_COLOR
        pygame.draw.circle(self.screen, color,
                           (self.position[0], drone_y), 14)
        for dx, dy in ((-20, -12), (20, -12), (-20, 12), (20, 12)):
            pygame.draw.circle(self.screen, (200, 200, 200),
                               (self.position[0] + dx, drone_y + dy), 5)
        # pygame 里同样用 PIL 写中文；写成"先画到临时 Surface 再 blit"，
        # 避免原实现 surfarray 往返导致的文字方向/颜色问题。
        rows = [
            "手势: %s" % gesture_cn,
            "指令: %s" % command_cn,
            "状态: %s" % ("飞行中" if self.is_flying else "在地面"),
            "高度: %.1f m    电池: %.1f%%" % (self.altitude, self.battery),
        ]
        self._blit_chinese(rows, (10, 8))
        tips = ["手势 -> 动作（7 种）：",
                "张开手掌 起飞 / 握拳 降落 / OK 手势 急停",
                "食指指向 前进 / 胜利手势 上升",
                "拇指向上 悬停 / 拇指向下 下降      q 退出"]
        self._blit_chinese(tips, (10, self.WINDOW_HEIGHT - 92))
        pygame.display.flip()

    def _blit_chinese(self, rows, position) -> None:
        font = get_chinese_font(18)
        surf = pygame.Surface((self.WINDOW_WIDTH, 22 * len(rows) + 8),
                              pygame.SRCALPHA)
        surf.fill((0, 0, 0, 0))
        pil = Image.fromarray(pygame.surfarray.array3d(surf).swapaxes(0, 1))
        draw = ImageDraw.Draw(pil)
        for i, text in enumerate(rows):
            draw.text((4, 4 + i * 22), text, font=font, fill=self.TEXT_COLOR)
        pygame.surfarray.blit_array(
            surf, np.array(pil).swapaxes(0, 1)[:, :, :3])
        self.screen.blit(surf, position)

    def shutdown(self) -> None:
        pygame.quit()


class GestureDemo(object):
    """演示主循环：摄像头 -> 识别 -> 画叠加层 -> 更新虚拟无人机。"""

    COMMAND_INTERVAL = 0.8
    WINDOW_NAME = "gesture demo (press q to quit)"

    def __init__(self, camera_index=0, stable_frames=3,
                 show_window: bool = True):
        self.camera = Camera(camera_index=camera_index)
        self.detector = GestureDetector()
        self.drone = VirtualDrone()
        self.stable = max(1, int(stable_frames))
        self.show_window = show_window
        self._last_gesture = gm.GESTURE_NONE
        self._candidate = gm.GESTURE_NONE
        self._candidate_count = 0
        # None 表示"还没执行过任何指令"；用 0.0 会让首条指令被冷却吃掉
        self._last_command_at = None
        self._last_command = gm.CMD_NONE

    def draw_overlay(self, frame: np.ndarray, gesture_cn: str, command_cn: str,
                     fps: float) -> np.ndarray:
        """把所有文字画上去并**返回新帧**（原实现丢掉了返回值）。"""
        frame = put_chinese(frame, u"手势: %s" % gesture_cn, (10, 10),
                            color=(0, 255, 0))
        frame = put_chinese(frame, u"指令: %s" % command_cn, (10, 40),
                            color=(0, 255, 255))
        frame = put_chinese(frame, u"虚拟无人机: %s %.1f m"
                            % ("飞行中" if self.drone.is_flying else "在地面",
                               self.drone.altitude), (10, 70),
                            color=(255, 255, 0))
        cv2.putText(frame, "FPS: %.1f" % fps, (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, "press q to quit", (10, frame.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return frame

    def update(self, gesture, now=None) -> str:
        """去抖 + 边沿触发 + 冷却；返回本次执行的指令 token。"""
        if now is None:
            now = time.monotonic()
        if gesture == self._candidate:
            self._candidate_count += 1
        else:
            self._candidate = gesture
            self._candidate_count = 1
        if self._candidate_count < self.stable or gesture == gm.GESTURE_NONE:
            return gm.CMD_NONE
        command = gm.gesture_to_command(gesture)
        if command == gm.CMD_NONE or command == self._last_command:
            return gm.CMD_NONE
        if self._last_command_at is not None and \
                (now - self._last_command_at) < self.COMMAND_INTERVAL:
            return gm.CMD_NONE
        self._last_command_at = now
        self._last_command = command
        self.drone.execute_command(command)
        return command

    def run(self) -> int:
        self.camera.initialize()
        print("手势演示已启动：%s" % "，".join(
            "%s->%s" % (gm.gesture_cn(g), gm.command_cn(c))
            for g, c in sorted(gm.GESTURE_TO_COMMAND.items())))
        frame_count, t0, last_t = 0, time.monotonic(), time.monotonic()
        fps = 0.0
        try:
            while True:
                if not self.drone.pump_events():
                    break
                frame = self.camera.read_frame()
                if frame is None:
                    time.sleep(0.05)
                    continue
                frame_count += 1
                now = time.monotonic()
                if now - last_t >= 0.5:
                    fps = frame_count / max(1e-6, now - t0)
                    last_t = now

                result = self.detector.detect(frame)
                command = self.update(result.gesture, now=now)
                if command != gm.CMD_NONE:
                    print("执行指令：%s" % gm.command_cn(command))

                # ★ 关键修复：叠加层的返回值必须接住，否则文字全丢
                frame = self.draw_overlay(
                    frame, gm.gesture_cn(result.gesture),
                    gm.command_cn(command if command != gm.CMD_NONE
                                  else self._last_command), fps)
                if self.show_window:
                    cv2.imshow(self.WINDOW_NAME, frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
        finally:
            self.camera.release()
            self.detector.close()
            self.drone.shutdown()
            cv2.destroyAllWindows()
        print("演示结束：共处理 %d 帧，平均 %.1f FPS"
              % (frame_count, frame_count / max(1e-6, time.monotonic() - t0)))
        return 0


def main():
    camera_index = int(os.environ.get("GESTURE_CAMERA_INDEX", "0"))
    demo = GestureDemo(camera_index=camera_index)
    try:
        return demo.run()
    except RuntimeError as exc:
        print("启动失败：%s" % exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())
