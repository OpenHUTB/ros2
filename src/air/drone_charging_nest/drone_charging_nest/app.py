#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 无人机自主充电/机巢演示（ROS 2 ament_python 包入口）。

任务流程：停机待命 → 起飞 → 飞往巡检点 → 悬停 → 返航 → 降落机巢 →
自动充电 → 充满后再次起飞 → 降落完成。主视图 + 机巢相机画中画 + HUD
（状态 / 电量 / 高度），可录制帧序列生成截图、GIF 和 MP4。

控制：四旋翼简化模型（4 个旋翼推力经 xfrc_applied 施加），
级联位置→姿态 PID + 推力分配；电池/充电为演示模型（非真实电化学模型）。
"""
import argparse
import json
import math
import os

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
MODULE_ROOT = os.path.dirname(PACKAGE_DIR)


def _share_dir():
    """安装为 ROS 2 ament_python 包时从 share 目录读取模型。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("drone_charging_nest")
    except Exception:  # 源码/venv 直接运行时回退到模块根目录
        return MODULE_ROOT


SHARE_DIR = _share_dir()
SCENE = os.path.join(SHARE_DIR, "models", "charging_nest.xml")

MASS = 1.0
G = 9.81
ARM = 0.17
KAPPA = 0.05          # rotor drag yaw coefficient
F_MAX = 6.0           # per-rotor thrust limit
C_D = 0.28            # linear drag
C_ANG = 0.015         # angular drag

FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_MONO = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"

PAD = np.array([0.0, 0.0])
PAD_Z = 0.14
CRUISE_Z = 1.15
INSPECT = np.array([1.75, 1.15, CRUISE_Z])

PHASES = [
    # (name, duration, target position, thrust-enabled)
    ("STANDBY", 1.0, np.array([0.0, 0.0, PAD_Z]), False),
    ("TAKEOFF", 2.0, np.array([0.0, 0.0, CRUISE_Z]), True),
    ("CRUISE", 3.0, INSPECT.copy(), True),
    ("INSPECT", 1.2, INSPECT.copy(), True),
    ("RETURN", 3.0, np.array([0.0, 0.0, CRUISE_Z]), True),
    ("LANDING", 2.6, np.array([0.0, 0.0, PAD_Z]), True),
    ("CHARGING", 7.0, np.array([0.0, 0.0, PAD_Z]), False),
    ("TAKEOFF2", 1.6, np.array([0.0, 0.0, 0.9]), True),
    ("LAND2", 2.2, np.array([0.0, 0.0, PAD_Z]), True),
    ("DONE", 1.2, np.array([0.0, 0.0, PAD_Z]), False),
]


def smoothstep(t):
    t = min(max(t, 0.0), 1.0)
    return t * t * (3.0 - 2.0 * t)


def smoothstep_d(t, dur):
    t = min(max(t / dur, 0.0), 1.0)
    return 6.0 * t * (1.0 - t) / dur


def quat_to_R(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def vee(S):
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


def load_fonts():
    try:
        return (ImageFont.truetype(FONT_PATH, 17), ImageFont.truetype(FONT_MONO, 14),
                ImageFont.truetype(FONT_PATH, 13))
    except Exception:
        f = ImageFont.load_default()
        return f, f, f


class DroneChargingSim:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(SCENE)
        self.data = mujoco.MjData(self.model)
        self.drone_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "drone")
        self.dt = self.model.opt.timestep
        self.rng = np.random.RandomState(0)
        # settle on the pad
        self.data.qpos[:7] = [0, 0, 0.16, 1, 0, 0, 0]
        self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)
        for _ in range(int(1.0 / self.dt)):
            self.data.xfrc_applied[self.drone_id, :] = 0
            mujoco.mj_step(self.model, self.data)

        self.battery = 100.0
        self.thrust_enabled = False
        self.phase = 0
        self.phase_t = 0.0
        self.ref = np.array([self.data.qpos[0], self.data.qpos[1], self.data.qpos[2]])
        self.ref_start = self.ref.copy()
        self.log = []
        self.frames = []

    # ---------------- state ----------------
    def state(self):
        p = self.data.qpos[0:3].copy()
        v = self.data.qvel[0:3].copy()
        R = quat_to_R(self.data.qpos[3:7])
        w = self.data.qvel[3:6].copy()
        return p, v, R, w

    def phase_name(self):
        return PHASES[min(self.phase, len(PHASES) - 1)][0]

    def status_text(self):
        names = {
            "STANDBY": "STANDBY ON NEST",
            "TAKEOFF": "TAKEOFF",
            "CRUISE": "CRUISE TO INSPECTION",
            "INSPECT": "INSPECTING TARGET",
            "RETURN": "RETURN TO NEST",
            "LANDING": "LANDING",
            "CHARGING": "CHARGING",
            "TAKEOFF2": "TAKEOFF (READY)",
            "LAND2": "LANDING",
            "DONE": "MISSION COMPLETE",
        }
        return names.get(self.phase_name(), self.phase_name())

    # ---------------- controller ----------------
    def control(self, p_ref, v_ref, yaw_ref=0.0):
        if not self.thrust_enabled:
            self.data.xfrc_applied[self.drone_id, :] = 0
            return
        p, v, R, w = self.state()
        e_p = p_ref - p
        e_v = v_ref - v
        kp = np.array([5.5, 5.5, 7.0])
        kd = np.array([4.2, 4.2, 5.2])
        a = kp * e_p + kd * e_v
        a[2] += G
        a[0:2] = np.clip(a[0:2], -7.0, 7.0)
        a[2] = np.clip(a[2], -3.0, 13.0)

        F_des = MASS * a
        F_norm = np.linalg.norm(F_des)
        if F_norm < 1e-6:
            F_norm = 1e-6
        z_des = F_des / F_norm
        b1 = np.array([math.cos(yaw_ref), math.sin(yaw_ref), 0.0])
        b1 = b1 - z_des * np.dot(b1, z_des)
        if np.linalg.norm(b1) < 1e-6:
            b1 = np.array([1.0, 0.0, 0.0])
        b1 /= np.linalg.norm(b1)
        b2 = np.cross(z_des, b1)
        R_des = np.column_stack([b1, b2, z_des])

        R_err = R_des.T @ R
        e_R = 0.5 * vee(R_err - R_err.T)
        T = max(0.0, float(np.dot(F_des, R[:, 2])))
        tau = -np.array([2.8, 2.8, 1.6]) * e_R - np.array([0.45, 0.45, 0.30]) * w

        A = np.array([
            [1.0, 1.0, 1.0, 1.0],
            [ARM, -ARM, ARM, -ARM],
            [-ARM, -ARM, ARM, ARM],
            [KAPPA, -KAPPA, -KAPPA, KAPPA],
        ])
        try:
            f = np.linalg.solve(A, np.array([T, tau[0], tau[1], tau[2]]))
        except np.linalg.LinAlgError:
            f = np.array([T / 4.0] * 4)
        f = np.clip(f, 0.0, F_MAX)
        T_act = float(np.sum(f))
        tau_act = A[1:, :] @ f

        F_world = R @ np.array([0.0, 0.0, T_act]) - C_D * v
        tau_world = R @ tau_act - C_ANG * w
        self.data.xfrc_applied[self.drone_id, :3] = F_world
        self.data.xfrc_applied[self.drone_id, 3:] = tau_world

    # ---------------- mission ----------------
    def step(self):
        name, dur, target, thrust = PHASES[self.phase]
        phase_started = self.phase_t == 0.0
        if phase_started:
            self.ref_start = self.ref.copy()
            self.thrust_enabled = thrust
        self.phase_t += self.dt

        s = smoothstep(min(self.phase_t / dur, 1.0))
        self.ref = self.ref_start + (target - self.ref_start) * s
        v_ref = (target - self.ref_start) * smoothstep_d(min(self.phase_t, dur), dur)
        self.control(self.ref, v_ref)

        mujoco.mj_step(self.model, self.data)
        p, v, R, w = self.state()

        # battery model
        airborne = self.phase_name() not in ("STANDBY", "CHARGING", "DONE")
        if airborne:
            self.battery = max(0.0, self.battery - 1.8 * self.dt)
        elif self.phase_name() == "CHARGING" and self.phase_t < dur:
            self.battery = min(95.0, self.battery + 2.6 * self.dt)

        self.log.append({
            "t": round(sum(x[1] for x in PHASES[:self.phase]) + self.phase_t, 3),
            "phase": self.phase_name(),
            "x": round(float(p[0]), 4), "y": round(float(p[1]), 4), "z": round(float(p[2]), 4),
            "vx": round(float(v[0]), 4), "vy": round(float(v[1]), 4), "vz": round(float(v[2]), 4),
            "battery": round(self.battery, 2),
        })

        if self.phase_t >= dur:
            self.phase += 1
            self.phase_t = 0.0
            if self.phase >= len(PHASES):
                self.phase = len(PHASES) - 1
                self.phase_t = PHASES[-1][1]
                return name, False
            return name, False
        return name, True

    def done(self):
        return self.phase == len(PHASES) - 1 and self.phase_t >= PHASES[-1][1]


# ---------------- rendering ----------------
def draw_hud(frame, sim):
    img = Image.fromarray(frame)
    draw = ImageDraw.Draw(img, "RGBA")
    font, mono, small = load_fonts()
    p, v, R, w = sim.state()
    name = sim.phase_name()
    charging = name == "CHARGING"
    color = (30, 210, 110) if charging else (80, 170, 255)

    # panel
    draw.rectangle([10, 10, 330, 128], fill=(8, 12, 18, 165), outline=(90, 110, 130, 220), width=1)
    draw.text((22, 18), "AUTO CHARGING NEST", font=font, fill=(230, 240, 250))
    draw.text((22, 44), "STATUS: " + sim.status_text(), font=font, fill=color)

    # battery bar
    bx, by, bw, bh = 22, 76, 170, 15
    draw.rectangle([bx, by, bx + bw, by + bh], fill=(35, 40, 46), outline=(160, 175, 190))
    ratio = max(0.0, min(1.0, sim.battery / 100.0))
    fillc = (40, 200, 90) if ratio > 0.35 else (230, 170, 40)
    draw.rectangle([bx + 2, by + 2, bx + 2 + (bw - 4) * ratio, by + bh - 2], fill=fillc)
    draw.text((bx + bw + 10, by - 2), "%d%%" % int(round(sim.battery)), font=mono, fill=(240, 245, 250))

    # telemetry
    draw.text((22, 100), "ALT %5.2f m    POS (%5.2f, %5.2f)" % (p[2], p[0], p[1]),
              font=small, fill=(200, 210, 220))

    # charging indicator
    if charging:
        draw.ellipse([310, 38, 320, 48], fill=(60, 230, 140))

    # inset nest camera
    img = img.convert("RGB")
    return np.array(img)


def render_frame(sim, main_renderer, pad_renderer, fps):
    main_renderer.update_scene(sim.data, camera="overview")
    main = main_renderer.render()
    pad_renderer.update_scene(sim.data, camera="padcam")
    pad = pad_renderer.render()
    inset = np.array(Image.fromarray(pad).resize((220, 165)))
    frame = main.copy()
    x0 = frame.shape[1] - inset.shape[1] - 10
    y0 = 10
    frame[y0:y0 + inset.shape[0], x0:x0 + inset.shape[1]] = inset
    frame[y0 - 2:y0, x0 - 2:x0 + inset.shape[1] + 2] = 220
    frame[y0:y0 + inset.shape[0], x0 - 2:x0] = 220
    frame[y0:y0 + inset.shape[0], x0 + inset.shape[1]:x0 + inset.shape[1] + 2] = 220
    frame[y0 + inset.shape[0]:y0 + inset.shape[0] + 2, x0 - 2:x0 + inset.shape[1] + 2] = 220
    return draw_hud(frame, sim)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--record", default="", help="directory to write rendered frames")
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--log", default="results/mission_log.json")
    ap.add_argument("--stills", default="results")
    args, _ = ap.parse_known_args()  # 兼容 ros2 launch 追加的 --ros-args 等参数

    sim = DroneChargingSim()
    os.makedirs(args.stills, exist_ok=True)

    renderer = pad_renderer = None
    if args.record:
        os.makedirs(args.record, exist_ok=True)
        for f in os.listdir(args.record):
            if f.endswith(".png"):
                os.remove(os.path.join(args.record, f))
        renderer = mujoco.Renderer(sim.model, 480, 640)
        pad_renderer = mujoco.Renderer(sim.model, 240, 320)

    total_dur = sum(p[1] for p in PHASES)
    frame_every = max(1, int(round(1.0 / args.fps / sim.dt)))
    step_count = 0
    still_times = {"01_standby": 0.5, "02_takeoff": 2.2, "03_cruise": 5.0,
                   "04_inspect": 7.6, "05_return": 9.5, "06_landing": 11.8,
                   "07_charging": 15.5, "08_complete": total_dur - 0.3}
    stills_taken = set()
    frames = 0
    while sum(p[1] for p in PHASES[:sim.phase]) + sim.phase_t < total_dur and not sim.done():
        sim.step()
        step_count += 1
        t = sum(p[1] for p in PHASES[:sim.phase]) + sim.phase_t
        if renderer and step_count % frame_every == 0:
            frame = render_frame(sim, renderer, pad_renderer, args.fps)
            Image.fromarray(frame).save(os.path.join(args.record, "f_%05d.png" % frames))
            frames += 1
        for name, when in still_times.items():
            if name not in stills_taken and t >= when:
                if renderer:
                    frame = render_frame(sim, renderer, pad_renderer, args.fps)
                    Image.fromarray(frame).save(os.path.join(args.stills, name + ".png"))
                stills_taken.add(name)

    p, v, R, w = sim.state()
    print("simulation done at t=%.2f, drone pos=" % total_dur, np.round(p, 3),
          "battery=%.1f" % sim.battery)
    print("frames:", frames)
    os.makedirs(os.path.dirname(args.log) or ".", exist_ok=True)
    with open(args.log, "w", encoding="utf-8") as f:
        json.dump(sim.log, f, ensure_ascii=False, indent=1)
    result = {
        "final_pos": [round(float(x), 4) for x in p],
        "final_battery": round(sim.battery, 2),
        "frames": frames,
        "log": args.log,
        "still_files": sorted(os.path.join(args.stills, n + ".png") for n in still_times),
    }
    print("RESULT_JSON=" + json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
