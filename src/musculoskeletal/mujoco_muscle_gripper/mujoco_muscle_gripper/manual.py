"""终端键盘控制肌肉激活与抬升，支持同一映射的可重复按键序列。"""

import json
import sys

import mujoco
import numpy as np

from .core import DT, HillMuscle, muscle_force
from .runs import _finger_contact, _make


def command(key, activation, height):
    if key == "w":
        activation += 0.05
    elif key == "s":
        activation -= 0.05
    elif key == "r":
        height += 0.01
    elif key == "f":
        height -= 0.01
    elif key == " ":
        activation = 0.0
    return float(np.clip(activation, 0, 1)), float(np.clip(height, 0, 0.15))


def _read_key():
    if sys.platform == "win32":
        import msvcrt
        return msvcrt.getwch().lower()
    import termios
    import tty
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1).lower()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)


def run(keys=None):
    if keys is None and not sys.stdin.isatty():
        raise ValueError("交互控制需要终端；非交互环境请传入 --keys 按键序列。")
    model, data = _make()
    muscles = [HillMuscle(), HillMuscle()]
    joints = [model.joint(name) for name in ("flj", "frj")]
    activation, height = 0.0, 0.0
    iterator = iter(keys) if keys is not None else None
    records = []
    print("W/S 增减肌肉激活，R/F 升降，空格松开，Q 退出。", flush=True)
    while True:
        key = next(iterator, "q") if iterator is not None else _read_key()
        if key == "q":
            break
        activation, height = command(key, activation, height)
        for _ in range(round(0.1 / DT)):
            data.ctrl[0] = height
            for joint, muscle in zip(joints, muscles):
                qi, vi = int(joint.qposadr[0]), int(joint.dofadr[0])
                muscle.update_activation(activation, DT)
                data.qfrc_applied[vi] = muscle_force(muscle, data.qpos[qi], data.qvel[vi])
            mujoco.mj_step(model, data)
        contacts, force = _finger_contact(model, data)
        row = {"key": key, "time_s": float(data.time), "activation": activation,
               "height_target_m": height, "contact_points": contacts, "grip_force_n": force}
        records.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
    return records
