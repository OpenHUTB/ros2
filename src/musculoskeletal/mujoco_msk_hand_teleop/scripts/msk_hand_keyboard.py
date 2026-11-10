#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘遥操作：按键 → 11 维肌肉激活目标（切换式），含一阶平滑。

按键映射（切换式：按一次开、再按一次关）：
    q/a  食指 伸展/屈曲        w/s  中指 伸展/屈曲
    e/d  拇指 伸展/屈曲        r    拇指 对掌(内收)
    u/j  手腕 屈曲(下压)/伸展(上抬)
    k/l  手腕 左摆/右摆
    空格  全部放松               ESC  退出

本模块只负责"按键状态 → 平滑激活目标"的策略部分，不含 ROS 依赖，
可被 ROS 节点复用，也支持脚本化按键序列（用于无头测试与 GIF 录制）。
"""
import numpy as np

# 键 -> 肌肉名（与 msk_hand_core.MUSCLE_NAMES 对应）
KEY_MUSCLE_MAP = {
    "q": "m_index_ext",
    "a": "m_index_flex",
    "w": "m_middle_ext",
    "s": "m_middle_flex",
    "e": "m_thumb_ext",
    "d": "m_thumb_flex",
    "r": "m_thumb_add",
    "u": "m_wrist_flex",
    "j": "m_wrist_ext",
    "k": "m_wrist_yaw_l",
    "l": "m_wrist_yaw_r",
}

KEY_HELP = (
    "按键(切换式): q/a 食指伸/屈  w/s 中指伸/屈  e/d 拇指伸/屈  r 拇指对掌\n"
    "             u/j 腕屈/腕伸   k/l 腕左摆/右摆  空格 全部放松  ESC 退出"
)


class KeyboardPolicy:
    """切换式键盘策略：维护按键集合，输出经一阶低通平滑的激活向量。

    target : 0/1 目标激活（由按键集合决定）
    cmd    : 实际下发的平滑激活，ctrl = ctrl + (target - ctrl) * dt / tau
    """

    def __init__(self, tau=0.12, n_muscle=11):
        self.tau = tau
        self.n = n_muscle
        self.held = set()          # 当前"开启"的键
        self._cmd = np.zeros(self.n)

    # ------------------------------------------------------------------ #
    def toggle(self, key):
        """按一次键：在开/关之间切换。返回 None 或生效的肌肉名。"""
        if key == " ":
            self.held.clear()
            return "*relax*"
        m = KEY_MUSCLE_MAP.get(key)
        if m is None:
            return None
        if key in self.held:
            self.held.discard(key)
        else:
            self.held.add(key)
        return m

    def set_target_raw(self, keys):
        """直接设置按键集合（脚本化演示用）。"""
        valid = set(k for k in keys if k in KEY_MUSCLE_MAP)
        self.held = valid

    # ------------------------------------------------------------------ #
    def target_vector(self):
        """当前目标激活向量（0/1）。"""
        t = np.zeros(self.n)
        for k in self.held:
            i = self._muscle_index(KEY_MUSCLE_MAP[k])
            if i >= 0:
                t[i] = 1.0
        return t

    def update(self, dt):
        """向目标做一阶平滑，返回平滑后的激活向量（作为 ctrl 下发）。"""
        t = self.target_vector()
        self._cmd += (t - self._cmd) * min(1.0, dt / self.tau)
        return self._cmd.copy()

    def reset(self):
        self.held.clear()
        self._cmd[:] = 0.0

    # ------------------------------------------------------------------ #
    @staticmethod
    def _muscle_index(name):
        from msk_hand_core import MUSCLE_NAMES
        try:
            return MUSCLE_NAMES.index(name)
        except ValueError:
            return -1


# ------------------------------ 自检 ---------------------------------- #
if __name__ == "__main__":
    p = KeyboardPolicy()
    # 模拟按键序列：握拳 -> 张开 -> 腕屈
    script = [("a", 1.0), ("a", 1.0), ("q", 0.8), ("u", 1.2)]
    t = 0.0
    for key, hold in script:
        p.toggle(key)
        for _ in range(int(hold / 0.01)):
            c = p.update(0.01)
            t += 0.01
        print("t=%.2f held=%s cmd=%s" %
              (t, sorted(p.held), np.round(c, 2)))
