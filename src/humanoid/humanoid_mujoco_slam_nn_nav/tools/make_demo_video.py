"""生成演示视频：真实 MuJoCo 仿真画面 + 终端命令回放（合成演示，非录屏）。

用法::

    python tools/make_demo_video.py --out media/desktop_demo.mp4

输出 1280×720 / 25 fps 的 MP4，并在 `media/` 下生成同名 GIF 预览。
真实的 Ubuntu 桌面录屏请用 `tools/record_desktop_demo.sh`。
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from humanoid_mujoco_slam_nn_nav.gait import GaitParams, Walker              # noqa: E402
from humanoid_mujoco_slam_nn_nav.nav import NavConfig, Navigation            # noqa: E402
from humanoid_mujoco_slam_nn_nav.sim import HumanoidSim                      # noqa: E402

W, H = 1280, 720
FPS = 25
TERM_BG = (48, 10, 36)
TERM_FG = (238, 238, 236)
PROMPT = (138, 226, 52)
INFO = (114, 159, 207)
WARN = (252, 233, 79)
FONT_PATHS = [
    "C:/Windows/Fonts/consola.ttf", "C:/Windows/Fonts/msyh.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
]


def font(size: int, cjk: bool = False):
    candidates = (["C:/Windows/Fonts/msyh.ttc", "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"]
                  if cjk else FONT_PATHS)
    for p in candidates:
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                continue
    return ImageFont.load_default()


def sim_frame(sim, w=480, h=360):
    return Image.fromarray(sim.follow_view(w, h, distance=2.5, azimuth=125, elevation=-12))


class Timeline:
    """把"打字 -> 输出 -> 等待"编排成一条时间线。"""

    def __init__(self):
        self.events = []      # (kind, payload)
        self.t = 0.0

    def type(self, text, cps=45.0, pause=0.5):
        self.events.append(("type", (text, cps, pause)))

    def line(self, text, color=None, hold=0.35):
        self.events.append(("line", (text, color, hold)))

    def wait(self, seconds):
        self.events.append(("wait", seconds))


def terminal_image(lines, typed, w, h, font_obj, title="lxy@ubuntu: ~/humanoid_ws", cjk=None):
    img = Image.new("RGB", (w, h), TERM_BG)
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, w, 30], fill=(60, 60, 60))
    d.text((12, 7), title, font=font_obj, fill=(220, 220, 220))
    for i, c in enumerate([(255, 95, 86), (255, 189, 46), (39, 201, 63)]):
        d.ellipse([w - 70 + i * 20, 10, w - 60 + i * 20, 20], fill=c)
    y = 44
    visible = lines[-19:]
    cjk = cjk or font_obj
    for text, color in visible:
        fnt = cjk if any(ord(ch) > 127 for ch in text) else font_obj
        d.text((14, y), text, font=fnt, fill=color or TERM_FG)
        y += 22
    if typed is not None:
        fnt = cjk if any(ord(ch) > 127 for ch in typed) else font_obj
        d.text((14, y), typed, font=fnt, fill=TERM_FG)
        d.rectangle([14 + d.textlength(typed, font=fnt), y - 2,
                     14 + d.textlength(typed, font=fnt) + 9, y + 18], fill=(200, 200, 200))
    return img


def compose(term, view, task, idx, total, t):
    img = Image.new("RGB", (W, H), (30, 30, 34))
    d = ImageDraw.Draw(img)
    # 顶栏
    d.rectangle([0, 0, W, 30], fill=(20, 20, 22))
    d.text((16, 6), "活动", font=font(14, True), fill=(210, 210, 210))
    d.text((W // 2 - 120, 6), "humanoid_mujoco_slam_nn_nav — 终端", font=font(14, True), fill=(210, 210, 210))
    clock = "14:%02d" % (int(32 + t / 60) % 60)
    d.text((W - 70, 6), clock, font=font(14), fill=(210, 210, 210))
    # 终端
    img.paste(term, (28, 46))
    # 仿真窗口
    d.rectangle([796, 46, 796 + 484, 46 + 30], fill=(60, 60, 60))
    d.text((808, 53), "MuJoCo · humanoid_mujoco_slam_nn_nav", font=font(14), fill=(225, 225, 225))
    img.paste(view, (798, 76))
    d.rectangle([798, 76, 798 + 480, 76 + 360], outline=(90, 90, 96))
    # 底部状态
    d.rectangle([0, H - 44, W, H], fill=(20, 20, 22))
    d.text((28, H - 34), f"片段 {idx}/{total} · {task}", font=font(15, True), fill=(240, 240, 240))
    d.text((W - 470, H - 34), "合成演示：终端为命令回放，仿真画面为真实 MuJoCo 渲染",
           font=font(13, True), fill=(150, 150, 150))
    return np.asarray(img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="media/desktop_demo.mp4")
    ap.add_argument("--fps", type=int, default=FPS)
    args = ap.parse_args()

    import imageio_ffmpeg
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    ff = imageio_ffmpeg.get_ffmpeg_exe()
    writer = imageio_ffmpeg.write_frames(args.out, (W, H), fps=args.fps, codec="libx264",
                                         pix_fmt_in="rgb24", pix_fmt_out="yuv420p",
                                         output_params=["-crf", "23", "-preset", "medium"])
    writer.send(None)

    fterm = font(15)
    fcjk = font(15, True)
    t0 = time.time()
    frames = 0

    def emit(arr):
        nonlocal frames
        writer.send(arr.tobytes())
        frames += 1

    # ------------------------------------------------------------------
    gp = GaitParams(pelvis_mode="feet", pelvis_height=0.70, duty=0.78, period=1.15,
                    side_shift=0.02, foot_spread=0.07, step_height=0.08,
                    capture_gain=0.35, capture_max=0.08, com_kp=1.0, com_kd=0.1,
                    com_max=0.04, roll_gain=0.0, pitch_gain=0.0, ankle_kp=0.0, ankle_kd=0.0)
    cfg = NavConfig(settle_period=9.0, settle_time=1.4, v_max=0.25, inflation=0.30)

    # ---------------- 片段 1：键盘遥控 ----------------
    sim = HumanoidSim(scene="warehouse", render=True)
    sx, sy, syaw = sim.layout.start
    walker = Walker(gp, sx, sy, syaw)
    sim.reset()
    lines = [("lxy@ubuntu:~$ ", None)]
    term = terminal_image(lines, "", 740, 560, fterm, cjk=fcjk)

    cmd1 = "roslaunch humanoid_mujoco_slam_nn_nav teleop.launch scene:=warehouse"
    for i in range(1, len(cmd1) + 1):
        term = terminal_image([("lxy@ubuntu:~$ ", PROMPT)], cmd1[:i], 740, 560, fterm, cjk=fcjk)
        emit(compose(term, sim_frame(sim), "键盘遥控", 1, 3, frames / args.fps))
    out1 = [
        ("... logging to /home/lxy/.ros/log/...", None),
        ("[INFO] [humanoid_sim]: MuJoCo 3.x, 15 joints, scene=warehouse", INFO),
        ("[INFO] [humanoid_teleop]: 键盘遥控已启动 W/S 前后 A/D 转向", INFO),
        ("[INFO] [humanoid_sim]: /scan 10Hz  /camera/depth 5Hz  /joint_states 100Hz", INFO),
    ]
    for text, color in out1:
        lines.append((text, color))
        for k in range(3):
            emit(compose(terminal_image([("lxy@ubuntu:~$ " + cmd1, PROMPT)] + lines[1:] + [("", None)],
                                        "", 740, 560, fterm, cjk=fcjk),
                         sim_frame(sim), "键盘遥控", 1, 3, frames / args.fps))

    # 行走 20 s
    dt = 0.01                   # 控制周期 10 ms
    control_per_frame = 5       # 每帧 5 次控制 = 0.05 s 仿真
    frames_needed = 20 * args.fps
    for f in range(frames_needed):
        tt = f * 0.05
        v = 0.20 if 2.0 < tt < 17.0 else 0.0
        w = 0.32 if 9.0 < tt < 11.5 else 0.0
        walker.command(v, w)
        for _ in range(control_per_frame):
            st = sim.state()
            feet = {"l": tuple(sim.ankle_pos("l")[:2]), "r": tuple(sim.ankle_pos("r")[:2])}
            q = walker.update(dt, actual_pos=st["pos"], actual_yaw=st["yaw"], roll=st["roll"],
                              pitch=st["pitch"], vel=st["vel"], feet_xy=feet)
            sim.set_targets(q)
            sim.step(5)
        if f % 2 == 0:
            hud = f"v={v:.2f} m/s  ω={w:+.2f} rad/s"
            term = terminal_image([("lxy@ubuntu:~$ " + cmd1, PROMPT)] + lines[1:] + [(hud, WARN)],
                                  "", 740, 560, fterm, cjk=fcjk)
            emit(compose(term, sim_frame(sim), "键盘遥控", 1, 3, frames / args.fps))

    # ---------------- 片段 2：建图导航 ----------------
    cmd2 = "roslaunch humanoid_mujoco_slam_nn_nav slam_nav.launch scene:=warehouse"
    term = terminal_image([("lxy@ubuntu:~$ " + cmd1, PROMPT)] + lines[1:] + [("", None)],
                          "", 740, 560, fterm, cjk=fcjk)
    for i in range(1, len(cmd2) + 1):
        term2 = terminal_image([("lxy@ubuntu:~$ " + cmd1, PROMPT)] + lines[1:] + [("", None)],
                               cmd2[:i], 740, 560, fterm, cjk=fcjk)
        emit(compose(term2, sim_frame(sim), "建图 + 导航", 2, 3, frames / args.fps))
    lines2 = lines + [("lxy@ubuntu:~$ " + cmd2, PROMPT),
                      ("[INFO] [humanoid_nav]: 导航节点启动，目标 (2.40, 0.00)", INFO),
                      ("[INFO] [humanoid_sim]: scene=warehouse seed=0", INFO),
                      ("[INFO] [humanoid_metrics]: 等待 /humanoid/ground_truth ...", INFO)]

    sim2 = HumanoidSim(scene="warehouse", render=True)
    nav = Navigation(sim2, cfg, policy="classical")
    nframes = 30 * args.fps
    for f in range(nframes):
        for _ in range(6):
            nav.step()
        if f % 2 == 0:
            st = sim2.state()
            status = (f"nav_status: {nav.mode}  |  SLAM ({nav.slam.pose[0]:+.2f},"
                      f"{nav.slam.pose[1]:+.2f})  |  map {nav.grid.coverage()*100:.0f}%")
            term = terminal_image(lines2 + [(status, WARN)], "", 740, 560, fterm, cjk=fcjk)
            emit(compose(term, sim_frame(sim2), "建图 + 导航", 2, 3, frames / args.fps))
        if nav.mode == "done":
            break

    done = nav.summary()
    lines3 = lines2 + [
        ("[INFO] [humanoid_metrics]: 结束 success=%s time=%.1fs ate=%.3fm collisions=%d"
         % (done["success"], done["time_s"], done["ate_rmse_m"], done["collisions"]), INFO)]

    # ---------------- 片段 3：端到端 ----------------
    cmd3 = "roslaunch humanoid_mujoco_slam_nn_nav e2e.launch scene:=corridor"
    for i in range(1, len(cmd3) + 1):
        term = terminal_image(lines3 + [("", None)], cmd3[:i], 740, 560, fterm, cjk=fcjk)
        emit(compose(term, sim_frame(sim2), "端到端 CNN", 3, 3, frames / args.fps))

    lines4 = lines3 + [("lxy@ubuntu:~$ " + cmd3, PROMPT),
                       ("[INFO] [humanoid_e2e]: 端到端 CNN 已加载 (24x32 深度图)", INFO)]
    sim3 = HumanoidSim(scene="corridor", render=True)
    nav3 = Navigation(sim3, cfg, policy="e2e", with_depth=True)
    for f in range(24 * args.fps):
        for _ in range(6):
            nav3.step()
        if f % 2 == 0:
            st = sim3.state()
            status = f"e2e: v={nav3.v:.2f} m/s  ω={nav3.w:+.2f} rad/s  |  {nav3.mode}"
            term = terminal_image(lines4 + [(status, WARN)], "", 740, 560, fterm, cjk=fcjk)
            emit(compose(term, sim_frame(sim3), "端到端 CNN", 3, 3, frames / args.fps))
        if nav3.mode == "done":
            break

    writer.close()
    print(f"saved {args.out}: {frames} frames, {os.path.getsize(args.out)/1e6:.1f} MB, "
          f"{time.time()-t0:.0f}s wall")


if __name__ == "__main__":
    main()
