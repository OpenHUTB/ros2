#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""指令闸门：去抖 + 边沿触发 + 冷却（纯逻辑，可离线单测）。

为什么需要它
------------
原感知节点以固定 10 Hz **无条件**把识别结果发出去：只要举着"张开手掌"，
控制节点每秒就会收到 10 条指令、发起 10 次起飞。等词表修好之后这会立刻变成
灾难。这里把"什么情况下才值得发一条指令"收敛成三件事：

1. **去抖**：连续 ``stable_frames`` 帧识别到同一个手势才算数（滤掉抖动）；
2. **边沿触发**：同一条指令只在"从无到有"时发一次，一直举着不会重复发；
3. **冷却**：同一条指令两次之间至少间隔 ``cooldown`` 秒（防手抖/防误触）。

:meth:`CommandGate.update` 返回"这一次需要发布的指令"，不需要发布时返回 ``None``。
"""
from __future__ import annotations

import time

from gesture_map import CMD_NONE


class CommandGate(object):
    def __init__(self, stable_frames=3, cooldown=1.5, command_none=CMD_NONE,
                 clock=None, release_frames=2):
        self.stable_frames = max(1, int(stable_frames))
        self.cooldown = max(0.0, float(cooldown))
        self.command_none = command_none
        # 连续多少帧"没有手势"才认为手势结束（避免中途漏检导致重复触发）
        self.release_frames = max(1, int(release_frames))
        self._clock = clock if clock is not None else time.monotonic

        self._candidate = command_none
        self._candidate_count = 0
        self._none_count = 0
        self._emitted = command_none          # 已经发布过的指令（边沿状态）
        self._last_emit_at = None

    def update(self, command, now=None):
        """喂入一次识别结果，返回需要发布的指令 token（或 ``None``）。"""
        if now is None:
            now = self._clock()
        if command is None:
            command = self.command_none

        # ---- 1) "手势结束"判定：用**原始输入**，不受去抖影响 ----
        # 否则 release_frames 会被 stable_frames 挡住（要先攒够 stable_frames 帧
        # 无手势才会走到这里），"手放下"要等更久才能再次触发。
        if command == self.command_none:
            self._none_count += 1
            if self._none_count >= self.release_frames:
                self._emitted = self.command_none
            # 无手势帧会打断去抖计数：手势被中途漏检就得重新攒
            self._candidate = self.command_none
            self._candidate_count = 0
            return None
        self._none_count = 0

        # ---- 2) 去抖：连续 stable_frames 帧同一手势才算数 ----
        if command == self._candidate:
            self._candidate_count += 1
        else:
            self._candidate = command
            self._candidate_count = 1
        if self._candidate_count < self.stable_frames:
            return None

        # ---- 3) 边沿触发：同一条指令已经发过就不重复 ----
        if command == self._emitted:
            return None

        # ---- 4) 冷却 ----
        if self._last_emit_at is not None and \
                (now - self._last_emit_at) < self.cooldown:
            return None

        self._emitted = command
        self._last_emit_at = now
        return command

    def reset(self):
        self._candidate = self.command_none
        self._candidate_count = 0
        self._none_count = 0
        self._emitted = self.command_none
        self._last_emit_at = None

    @property
    def emitted(self):
        """当前已经发布过的指令（边沿状态），供日志/测试查看。"""
        return self._emitted
