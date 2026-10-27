#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘策略：按键 -> 肌肉激活叠加 / 动作切换。

两类按键：
  1. 手动兴奋键（自锁：按一次开、再按关）：直接兴奋指定肌肉，叠加在
     反射层输出上；
  2. 动作键（按一次切换）：站立 / 踢腿 / 蹲起 / 行走 / 放松 / 步频增减。

被 ROS 键盘节点与离线演示脚本共用。
"""
import numpy as np

from msk_human_core import MUSCLE_ID, N_MUSCLE

# 手动兴奋键：按键 -> [(肌肉名, 激活值), ...]
# 终端无按键抬起事件，采用自锁语义：按一次开启、再按一次关闭
HOLD_MAP = {
    "q": [("hipfl_l", 0.6)], "w": [("hipe_l", 0.6)],
    "a": [("kneef_l", 0.6)], "s": [("kneee_l", 0.6)],
    "z": [("ankd_l", 0.5)], "x": [("ankp_l", 0.6)],
    "u": [("hipfl_r", 0.6)], "i": [("hipe_r", 0.6)],
    "j": [("kneef_r", 0.6)], "k": [("kneee_r", 0.6)],
    "n": [("ankd_r", 0.5)], "m": [("ankp_r", 0.6)],
    "r": [("shof_l", 0.5), ("shof_r", 0.5)],
    "t": [("shoe_l", 0.5), ("shoe_r", 0.5)],
    "f": [("elbf_l", 0.5), ("elbf_r", 0.5)],
    "g": [("elbe_l", 0.5), ("elbe_r", 0.5)],
}

# 动作键：按一次触发一次（返回给上层）
ACTION_KEYS = {
    "1": "stand",
    "2": "kick",
    "3": "squat",
    "4": "walk",
    "0": "relax",
    "+": "faster",
    "=": "faster",
    "-": "slower",
}

KEY_HELP = """==================== 键盘控制说明 ====================
[动作切换]  1 站立 | 2 踢腿(右) | 3 蹲起 | 4 行走 | 0 全身放松
[步频调节]  + 加快 | - 减慢（仅行走动作）
[手动兴奋]  (按一次开启 / 再按一次关闭, 叠加在姿势反射之上)
   左腿:  Q/W 髋屈/伸   A/S 膝屈/伸   Z/X 踝背屈/跖屈
   右腿:  U/I 髋屈/伸   J/K 膝屈/伸   N/M 踝背屈/跖屈
   双臂:  R/T 肩前摆/后摆   F/G 肘屈/伸
[其他]  H 帮助 | ESC 退出
提示: 手动兴奋会与当前动作叠加, 例如行走时按 U 抬右腿跨障。
======================================================"""


class KeyboardPolicy:
    """维护按键状态，输出 22 维手动激活向量与动作事件。"""

    def __init__(self):
        self.held = set()
        self.manual = np.zeros(N_MUSCLE)

    # ---------- 按键事件 ----------
    def press(self, key):
        """返回 action 字符串或 None（手动键在内部自锁生效）。"""
        if key in HOLD_MAP:
            if key in self.held:          # 自锁切换：再按一次关闭
                self.held.discard(key)
            else:
                self.held.add(key)
            self._apply_holds()
            return None
        return ACTION_KEYS.get(key)

    def release(self, key):
        if key in self.held:
            self.held.discard(key)
            self._apply_holds()

    def _apply_holds(self):
        self.manual[:] = 0.0
        for k in self.held:
            for name, val in HOLD_MAP[k]:
                self.manual[MUSCLE_ID[name]] = max(
                    self.manual[MUSCLE_ID[name]], val)

    def manual_vector(self):
        return self.manual.copy()
