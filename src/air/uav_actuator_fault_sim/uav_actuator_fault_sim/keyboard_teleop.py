# -*- coding: utf-8 -*-
"""终端键盘直飞：任务①「通过键盘对机器人进行运动控制」的入口。

按键::

    W / S    前进 / 后退
    A / D    左移 / 右移
    R / F    上升 / 下降
    Q / E    左偏航 / 右偏航
    空格     速度清零，原地悬停
    Esc      退出

本模块**不依赖 ROS**，直接驱动底座 ``Quadrotor``；同时可以叠加故障
（``--fault single_loss``），因此"一边飞一边把桨打坏"的演示也走这条路径。
"""

from __future__ import annotations

import os
import sys
import time
from typing import List, Optional

import numpy as np

from ._compat import load_base
from .fault import FaultInjector
from .runner import RunConfig, _make_scene
from .scenario import make_case

#: 按键 → (线速度增量方向, 偏航角速度增量)，均为"按下时"的期望速度
KEY_BINDINGS = {
    "w": ((1.0, 0.0, 0.0), 0.0),
    "s": ((-1.0, 0.0, 0.0), 0.0),
    "a": ((0.0, 1.0, 0.0), 0.0),
    "d": ((0.0, -1.0, 0.0), 0.0),
    "r": ((0.0, 0.0, 1.0), 0.0),
    "f": ((0.0, 0.0, -1.0), 0.0),
    "q": ((0.0, 0.0, 0.0), 1.0),
    "e": ((0.0, 0.0, 0.0), -1.0),
}

#: 最大水平速度 / 垂直速度（m/s）与偏航角速度（rad/s）
MAX_HSPEED = 1.2
MAX_VSPEED = 0.8
MAX_YAW_RATE = np.radians(60.0)


class _KeyReader:
    """非阻塞单键读取，Windows 用 msvcrt，POSIX 用 termios + select。"""

    def __init__(self) -> None:
        self._is_windows = os.name == "nt"
        self._fd = None
        self._old = None
        if not self._is_windows:
            import termios
            import tty

            self._fd = sys.stdin.fileno()
            self._old = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
            self._termios = termios

    def poll(self) -> Optional[str]:
        if self._is_windows:
            import msvcrt

            if msvcrt.kbhit():
                ch = msvcrt.getwch()
                # 方向键是两字节前缀序列，这里只关心单字符
                return ch.lower() if ch and ch != "\x00" else None
            return None

        import select

        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready:
            return None
        ch = sys.stdin.read(1)
        return ch.lower() if ch else None

    def close(self) -> None:
        if not self._is_windows and self._fd is not None and self._old is not None:
            self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._old)


def run_keyboard_teleop(
    scene: str = "open_ground",
    seed: int = 0,
    seconds: float = 60.0,
    fault_scenario: Optional[str] = None,
    rotor: int = 0,
    onset: float = 5.0,
    severity: Optional[float] = None,
    gif_path: Optional[str] = None,
    width: int = 320,
    height: int = 240,
    quality: str = "fast",
) -> int:
    """键盘直飞主循环。返回进程退出码。"""
    api = load_base()

    scene_obj = _make_scene(api, scene, seed)
    quad = api.Quadrotor(scene=scene_obj, dt=0.01, substeps=4)
    start = np.array([0.0, 0.0, 1.5])
    quad.reset(pos=start)
    ctrl = api.GeometricController(quad.params)

    injector = None
    if fault_scenario and fault_scenario != "healthy":
        case = make_case(scene, fault_scenario, seed=seed, rotor=rotor,
                         onset=onset, severity=severity)
        injector = FaultInjector(case.build_spec(), max_thrust=float(quad.params.max_thrust))

    visual = None
    frames: List[np.ndarray] = []
    if gif_path:
        cam = api.CameraConfig(width=width, height=height, quality=quality)
        visual = api.VisionSensors(quad.model, cam)

    print(__doc__)
    print(f"场景 {scene}（seed={seed}），dt=0.01，按 Esc 退出")
    if injector is not None:
        print(f"已挂载故障：{injector.spec.describe()}，将在 {onset:.1f}s 注入")
    print("等待按键…（终端需要焦点）")

    reader = _KeyReader()
    p_d = start.copy()
    v_des = np.zeros(3)
    yaw = 0.0
    dt = 0.01
    steps = int(round(seconds / dt))
    t0 = time.time()
    crash_t = None

    try:
        for k in range(steps):
            t = k * dt
            # ---- 1. 读键，更新期望速度 ----
            ch = reader.poll()
            if ch is not None:
                if ch in ("\x1b",):
                    print("\n收到 Esc，退出")
                    break
                if ch == " ":
                    v_des[:] = 0.0
                    yaw = 0.0
                elif ch in KEY_BINDINGS:
                    direction, dyaw = KEY_BINDINGS[ch]
                    v_des += np.array(direction, dtype=float)
                    v_des[0] = np.clip(v_des[0], -1, 1) * MAX_HSPEED
                    v_des[1] = np.clip(v_des[1], -1, 1) * MAX_HSPEED
                    v_des[2] = np.clip(v_des[2], -1, 1) * MAX_VSPEED
                    yaw += dyaw * MAX_YAW_RATE
                else:
                    continue

            # ---- 2. 期望位置积分 + 几何控制器 ----
            p_d = p_d + v_des * dt
            p_d[2] = max(0.3, p_d[2])          # 别撞地
            u = np.asarray(ctrl.act(quad, p_d, v_des, np.zeros(3), yaw), dtype=float)
            if injector is not None:
                u = injector.apply(u, t)
            quad.apply_rotor_thrust(u)
            quad.step(1)
            st = quad.state()

            if visual is not None and k % 5 == 0:
                frames.append(visual.rgb(quad.data).copy())

            if crash_t is None and st.pos[2] < 0.15:
                crash_t = t - (onset if injector is not None else 0.0)
                print(f"\n[!] 高度跌破 0.15 m，t={t:.2f}s"
                      + (f"（故障后 {crash_t:.2f}s）" if injector is not None else ""))

            if k % 50 == 0:
                print(f"  t={t:5.2f}s  pos=({st.pos[0]:6.2f},{st.pos[1]:6.2f},"
                      f"{st.pos[2]:5.2f})  v={np.linalg.norm(st.vel):4.2f} m/s"
                      f"  yaw={np.degrees(st.yaw):6.1f}°")
    finally:
        reader.close()
        if visual is not None:
            visual.close()

    elapsed = time.time() - t0
    print(f"\n结束：仿真 {seconds if crash_t is None else '提前中止'} s，"
          f"实际耗时 {elapsed:.1f} s（{int(round(seconds / dt)) / max(elapsed, 1e-6):.0f}× 实时）")

    if gif_path and frames:
        from PIL import Image

        parent = os.path.dirname(os.path.abspath(gif_path))
        if parent:
            os.makedirs(parent, exist_ok=True)
        imgs = [Image.fromarray(f) for f in frames]
        imgs[0].save(gif_path, save_all=True, append_images=imgs[1:],
                     duration=20, loop=0, optimize=True)
        print(f"已录制键盘遥控动图：{gif_path}（{len(imgs)} 帧）")
    return 0
