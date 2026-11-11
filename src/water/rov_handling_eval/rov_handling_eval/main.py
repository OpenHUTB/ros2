# -*- coding: utf-8 -*-
"""单进程终端模式（模式 A）：不依赖 ROS，能直接在虚拟机/无显卡环境跑。

用途有三个：
1. **快速验证物理**：``--check`` 跑 3 s 自由漂移，把实测漂移与解析解对比，
   符号/帧搞反了会立刻暴露（这是最容易被忽略、又最耗时的 bug）；
2. **无 ROS 演示**：``python3 main.py`` 直接键控，适合课堂投影；
3. **3D 查看**：``--gui`` 打开 MuJoCo 原生视窗（需要 OpenGL 3.3+）。

与 ROS 版本共用同一套 ``dynamics.py`` 与 MJCF，所以两边手感必须一致；
若不一致，说明 ROS 节点里有额外的符号/坐标处理，优先怀疑那里。
"""

import argparse
import math
import sys
import time

import numpy as np

from .current_model import CurrentField, CurrentParams
from .dynamics import (VehicleParams, buoyancy_wrench, current_authority,
                       drag_force, drag_torque, velocity_control)
from .keys import KeyReader, drain_stdin
from .model import MJCF, quat_to_matrix

KEY_AXES = {
    "w": (0, +1.0), "s": (0, -1.0),      # surge
    "a": (1, +1.0), "d": (1, -1.0),      # sway
    "r": (2, +1.0), "f": (2, -1.0),      # heave
    "j": (3, +1.0), "l": (3, -1.0),      # roll
    "i": (4, +1.0), "k": (4, -1.0),      # pitch
    "q": (5, +1.0), "e": (5, -1.0),      # yaw
}
NAMES = ("纵荡", "横荡", "升沉", "横滚", "俯仰", "转向")
LIMITS = np.array([0.6, 0.5, 0.4, 0.6, 0.5, 0.8])


def build_env(params: VehicleParams, start_depth: float):
    import mujoco
    model = mujoco.MjModel.from_xml_string(MJCF.format(mass=params.mass))
    data = mujoco.MjData(model)
    data.qpos[2] = -start_depth
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    mujoco.mj_forward(model, data)
    return mujoco, model, data


def _say(text=""):
    """在 GBK 等窄编码控制台上安全打印。

    Windows 默认控制台是 GBK，遇到 `²`、`η` 这类字符会抛 UnicodeEncodeError。
    这里捕获后用 replace 降级，保证自检不会因为一个单位符号而崩掉。
    """
    try:
        print(text)
    except UnicodeEncodeError:
        enc = getattr(sys.stdout, "encoding", None) or "ascii"
        print(text.encode(enc, "replace").decode(enc))


def step(mujoco, model, data, body_id, params, cmd, current, dt, depth_hold, R_out,
         use_velocity_control=True):
    """推进一个物理步。

    ``use_velocity_control=False`` 是**自由漂移**模式：不加任何控制力/力矩，
    只留阻力与浮力。自检必须走这条路径——否则速度环会主动把"上浮"刹住，
    测到的就不是终端速度，而是"速度环把漂移压到多小"。
    """
    q = data.qpos[3:7]
    R = quat_to_matrix(q[0], q[1], q[2], q[3])
    R_out[0] = R
    v_body = R.T @ np.asarray(data.qvel[0:3], dtype=float)
    w_body = np.asarray(data.qvel[3:6], dtype=float)
    v_rel = v_body - R.T @ current
    if use_velocity_control:
        f_body, tau_body = velocity_control(cmd, v_body, w_body, params)
    else:
        f_body, tau_body = np.zeros(3), np.zeros(3)
    f_body = f_body + drag_force(v_rel, params.k())
    tau_body = tau_body + drag_torque(w_body, params.drag_angular)
    f_world = R @ f_body
    tau_world = R @ tau_body
    fb, tb = buoyancy_wrench(params, R)
    data.xfrc_applied[body_id, 0:3] = f_world + fb
    data.xfrc_applied[body_id, 3:6] = tau_world + tb
    mujoco.mj_step(model, data)
    return v_body, w_body


def run_check(params: VehicleParams, start_depth: float, seconds: float = 3.0, dt: float = 0.002):
    """自由漂移自检：把实测漂移速度与解析解对比，顺带检查符号。

    注意走的是 ``use_velocity_control=False``：只保留阻力与浮力，不接管速度。
    """
    mujoco, model, data = build_env(params, start_depth)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rov")
    R = [np.eye(3)]
    zero = np.zeros(6)
    cur = np.zeros(3)
    n = int(seconds / dt)
    for _ in range(n):
        step(mujoco, model, data, body, params, zero, cur, dt, False, R,
             use_velocity_control=False)

    z0, z1 = -start_depth, float(data.qpos[2])
    measured = (z1 - z0) / (n * dt)             # 正 = 上浮
    predicted = math.copysign(params.terminal_speed(2), params.net_buoyancy())

    _say("=== 自由漂移自检（无速度环）===")
    _say("净浮力        : %+.2f N (about %+.2f kg)" % (params.net_buoyancy(),
                                                       params.net_buoyancy() / 9.81))
    _say("解析终端速度  : %+.3f m/s" % predicted)
    _say("实测平均速度  : %+.3f m/s (%.1f s, 位移 %+.3f m)"
         % (measured, seconds, z1 - z0))
    ok = True
    if abs(predicted) > 0.02:
        if predicted * measured <= 0:
            _say("[FAIL] 方向相反：浮力/重力的符号或 z 轴方向错了")
            ok = False
        else:
            err = abs(measured - predicted) / abs(predicted)
            _say("相对误差      : %.1f%% (判据 <= 25%%: %s)"
                 % (err * 100, "通过" if err <= 0.25 else "不通过"))
            ok = err <= 0.25
    _say("整流垂向阻力  : %.2f N s^2/m^2" % params.k()[2])
    return ok


def run_interactive(params: VehicleParams, start_depth: float, gui: bool,
                    current_param: CurrentParams, dt: float = 0.002, hud_hz: float = 5.0):
    mujoco, model, data = build_env(params, start_depth)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rov")
    current = CurrentField(current_param)
    R = [np.eye(3)]
    cmd = np.zeros(6)
    target = np.zeros(6)
    last_seen = {}
    scale = 1.0
    hold = False
    depth_target = -start_depth
    reader = None

    print("模式 A：终端交互（Ctrl-C 退出，H 定深，空格急停）")
    if gui:
        try:
            from mujoco import viewer
            viewer.launch(model, data)
        except Exception as exc:
            print("打开 3D 视窗失败（%s），回退到纯终端" % exc)
            gui = False

    nxt = time.perf_counter()
    last_hud = 0.0
    try:
        with KeyReader() as kr:
            reader = kr
            while True:
                now = time.perf_counter()
                for ch in reader.poll(0.0):
                    low = ch.lower()
                    if ch == " ":
                        last_seen.clear()
                        target[:] = 0.0
                        cmd[:] = 0.0
                    elif low == "h":
                        hold = not hold
                        depth_target = float(data.qpos[2])
                    elif ch in ("+", "="):
                        scale = min(1.5, scale + 0.1)
                    elif ch == "-":
                        scale = max(0.2, scale - 0.1)
                    elif low in KEY_AXES:
                        last_seen[low] = now

                target[:] = 0.0
                for k, stamp in list(last_seen.items()):
                    if now - stamp > 0.35:
                        del last_seen[k]
                        continue
                    axis, sign = KEY_AXES[k]
                    target[axis] += sign
                target = np.clip(target, -1.0, 1.0) * scale * LIMITS
                if hold:
                    target[2] = float(np.clip(1.5 * (-depth_target - (-float(data.qpos[2]))), -0.4, 0.4))
                cmd += (target - cmd) * min(1.0, dt / 0.12)

                cur = current.step(dt, float(data.qpos[2]))
                v_body, w_body = step(mujoco, model, data, body, params, cmd, cur,
                                      dt, hold, R)

                if gui:
                    try:
                        viewer.sync()
                    except Exception:
                        gui = False

                if now - last_hud > 1.0 / hud_hz:
                    last_hud = now
                    eta = current_authority(params, float(np.linalg.norm(cur[:2])))
                    line = ("\r深度 %5.2f m | v(%.2f,%.2f,%.2f) | 倍率 %.1f %s | "
                            "eta(x,y,z)=%.2f,%.2f,%.2f   "
                            % (-float(data.qpos[2]), v_body[0], v_body[1], v_body[2],
                               scale, "定深" if hold else "   ",
                               eta[0], eta[1], eta[2]))
                    try:
                        sys.stdout.write(line)
                    except UnicodeEncodeError:      # GBK 控制台：去掉无法编码的字符
                        enc = getattr(sys.stdout, "encoding", None) or "ascii"
                        sys.stdout.write(line.encode(enc, "replace").decode(enc))
                    sys.stdout.flush()

                # 实时节流（500 Hz 物理，按真实时间推进）
                nxt += dt
                rest = nxt - time.perf_counter()
                if rest > 0:
                    time.sleep(rest)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n退出。")


def main(argv=None):
    ap = argparse.ArgumentParser(description="ROV 手操评测仿真（终端模式）")
    ap.add_argument("--gui", action="store_true", help="打开 MuJoCo 3D 视窗")
    ap.add_argument("--check", action="store_true", help="只做自由漂移自检")
    ap.add_argument("--depth", type=float, default=8.0, help="初始深度 m")
    ap.add_argument("--ballast", type=float, default=0.0, help="额外配重 kg")
    ap.add_argument("--current", type=float, default=0.0, help="平均流速 m/s")
    ap.add_argument("--current-dir", type=float, default=90.0, help="流向 deg")
    args = ap.parse_args(argv)

    params = VehicleParams(ballast=args.ballast)
    if args.check:
        ok = run_check(params, args.depth)
        return 0 if ok else 1

    run_interactive(params, args.depth, args.gui,
                    CurrentParams(mean_speed=args.current, direction_deg=args.current_dir))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
