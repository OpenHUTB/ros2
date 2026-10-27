#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""人形足迹下的局部避障算法对比平台。

用法（功能包根目录）：
    python3 main.py --version                 # 版本
    python3 main.py --test                    # 自检
    python3 main.py --headless                # 三种规划器各跑一次，出对比表
    python3 main.py --headless --planner dwa  # 只跑一种
    python3 main.py --demo --planner dwa      # 打开视窗看轨迹

退出码：0 全部规划器到达；1 有未到达/碰撞；2 参数错误。
"""

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import humanoid_local_planner_bench.config as config          # noqa: E402
from humanoid_local_planner_bench.metrics import RunRecorder, plot_compare  # noqa: E402
from humanoid_local_planner_bench.planners import PLANNERS    # noqa: E402
from humanoid_local_planner_bench.sim_core import CorridorSim  # noqa: E402


def positive_float(text):
    """argparse 类型校验：必须是正数，否则按参数错误退出（码 2）。"""
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError("需要数字，收到 {!r}".format(text))
    if value <= 0:
        raise argparse.ArgumentTypeError("必须为正数，收到 {}".format(value))
    return value


def build_parser():
    p = argparse.ArgumentParser(description="人形局部避障算法对比")
    p.add_argument("--planner", default="all", choices=["all", *PLANNERS.keys()],
                   help="选择规划器（默认全部）")
    p.add_argument("--headless", action="store_true", help="无窗口运行并导出结果")
    p.add_argument("--demo", action="store_true", help="打开视窗回放")
    p.add_argument("--gif", action="store_true", help="离屏渲染并导出演示动图")
    p.add_argument("--gif-width", type=int, default=480,
                   help="动图输出宽度（默认 %(default)s，按 4:3 计算高度）")
    p.add_argument("--gif-colors", type=int, default=48,
                   help="动图调色板颜色数（默认 %(default)s，越小体积越小）")
    p.add_argument("--figures", action="store_true", help="导出四宫格场景图与单图")
    p.add_argument("--shot", action="store_true", help="打开指定机位的视窗并保持（用于桌面截图）")
    p.add_argument("--view", default="persp",
                   choices=["overview", "persp", "door", "obstacle", "start"],
                   help="--shot 使用的机位")
    p.add_argument("--hold", type=positive_float, default=20.0, help="--shot 保持秒数")
    p.add_argument("--test", action="store_true", help="运行自检")
    p.add_argument("--control-hz", type=positive_float, default=config.CONTROL_HZ)
    p.add_argument("--time-limit", type=positive_float, default=config.TIME_LIMIT)
    p.add_argument("--save-dir", default=config.DEFAULT_OUTPUT_DIR)
    p.add_argument("--version", action="version",
                   version="humanoid_local_planner_bench {}".format(config.VERSION))
    return p


def run_one(name, args, sim=None, viewer=None, on_step=None, pace=False):
    """跑一次实验，返回 (recorder, summary)。"""
    sim = sim or CorridorSim()
    sim.reset(*config.START)
    planner = PLANNERS[name]()
    planner.reset()
    rec = RunRecorder()
    dt = 1.0 / args.control_hz
    goal = config.GOAL

    # 先记录初始状态（t=0），否则第一条样本是"走完第一步之后"的位置
    rec.add(0.0, sim.pose(), 0.0, 0.0, sim.clearance(), sim.collided())
    if on_step is not None:
        on_step(rec.samples[-1])
    if viewer is not None:
        viewer.sync()

    t = 0.0
    phase = 0.0
    while t < args.time_limit:
        pose = sim.pose()
        scan = sim.scan()
        v, w = planner.step(pose, scan, goal, dt)

        x = pose[0] + v * math.cos(pose[2]) * dt
        y = pose[1] + v * math.sin(pose[2]) * dt
        yaw = pose[2] + w * dt
        phase += (v * dt) / 0.7 * 2.0 * math.pi      # 步幅 0.7 m
        sim.set_pose(x, y, yaw, phase)

        clr = sim.clearance()
        hit = sim.collided()
        t += dt
        rec.add(t, sim.pose(), v, w, clr, hit)
        if on_step is not None:
            on_step(rec.samples[-1])
        if viewer is not None:
            viewer.sync()
        if math.hypot(x - goal[0], y - goal[1]) <= config.GOAL_TOL:
            break
        if pace:                              # 演示时按控制周期节流，否则仿真几秒就跑完
            time.sleep(dt)
    return rec, rec.summary(goal, config.GOAL_TOL)


def print_table(rows):
    print("\n{:<8}{:>6}{:>8}{:>10}{:>10}{:>12}{:>10}{:>9}".format(
        "规划器", "到达", "时间(s)", "路径(m)", "直线(m)", "最小间距(m)", "碰撞步", "平均|w|"))
    print("-" * 79)
    for name, m in rows:
        print("{:<8}{:>6}{:>8.2f}{:>10.2f}{:>10.2f}{:>12.3f}{:>10d}{:>9.2f}".format(
            name, "是" if m["reached"] else "否", m["time"], m["path_len"],
            m["straight_len"], m["min_clearance"], m["collision_steps"], m["mean_w"]))


def cmd_headless(args):
    names = list(PLANNERS.keys()) if args.planner == "all" else [args.planner]
    save = Path(args.save_dir)
    rows, recs, ok = [], [], True
    for name in names:
        print("[运行] {}".format(name))
        rec, m = run_one(name, args)
        rec.write_csv(save / "{}_run.csv".format(name))
        rows.append((name, m))
        recs.append(rec)
        if not m["reached"] or m["collision_steps"] > 0:
            ok = False
    print_table(rows)
    print("\n[导出] CSV 写入 {}".format(save.resolve()))
    png = save / "compare.png"
    if plot_compare(list(zip(names, recs)), config.GOAL, png):
        print("[导出] {}".format(png.resolve()))
    print("[结论] {}".format("全部到达且无碰撞" if ok else "存在未到达或碰撞"))
    return 0 if ok else 1


def cmd_demo(args):
    import mujoco.viewer

    sim = CorridorSim()
    name = "dwa" if args.planner == "all" else args.planner
    viewer = mujoco.viewer.launch_passive(sim.model, sim.data)
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    viewer.cam.lookat[:] = [4.0, 0.0, 0.0]
    viewer.cam.distance = 17.0
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -85
    print("[演示] 规划器 {}，关闭窗口退出".format(name))

    # —— 轨迹拖尾 + 终点标记：用 viewer.user_scn 画，不改模型 ——
    import numpy as np
    trail = []
    scn = viewer.user_scn
    scn.ngeom = 0
    mujoco.mjv_initGeom(scn.geoms[0], mujoco.mjtGeom.mjGEOM_SPHERE,
                        np.array([0.12, 0.0, 0.0]), np.array([config.GOAL[0], config.GOAL[1], 0.12]),
                        np.eye(3).flatten(), np.array([0.1, 1.0, 0.2, 1.0]))
    scn.ngeom = 1
    n_goal = scn.ngeom

    def draw_trail(st):
        trail.append((st["x"], st["y"]))
        if len(trail) > 600:
            trail.pop(0)
        scn.ngeom = n_goal
        for tx, ty in trail[::2]:
            if scn.ngeom >= scn.maxgeom:
                break
            mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                                np.array([0.045, 0.0, 0.0]), np.array([tx, ty, 0.06]),
                                np.eye(3).flatten(), np.array([0.15, 0.55, 1.0, 1.0]))
            scn.ngeom += 1

    rec, m = run_one(name, args, sim=sim, viewer=viewer, on_step=draw_trail, pace=True)
    print("  到达={} 时间={:.2f}s 最小间距={:.3f}m".format(
        m["reached"], m["time"], m["min_clearance"]))
    while viewer.is_running():
        viewer.sync()
    viewer.close()
    return 0


def cmd_gif(args):
    """离屏渲染逐帧并合成 GIF（跟拍机器人，看得到过门与绕障）。"""
    import mujoco
    from PIL import Image

    sim = CorridorSim()
    sim.reset(*config.START)
    planner = PLANNERS["dwa" if args.planner == "all" else args.planner]()
    planner.reset()
    w = min(640, sim.model.vis.global_.offwidth)
    h = min(426, sim.model.vis.global_.offheight)
    renderer = mujoco.Renderer(sim.model, height=h, width=w)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(sim.model, cam)
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = sim.body
    cam.distance = 5.0
    cam.azimuth = 55
    cam.elevation = -32

    frames = []
    dt = 1.0 / args.control_hz
    t, k, phase = 0.0, 0, 0.0
    while t < args.time_limit:
        pose = sim.pose()
        scan = sim.scan()
        v, w_ang = planner.step(pose, scan, config.GOAL, dt)
        x = pose[0] + v * math.cos(pose[2]) * dt
        y = pose[1] + v * math.sin(pose[2]) * dt
        yaw = pose[2] + w_ang * dt
        phase += (v * dt) / 0.7 * 2.0 * math.pi
        sim.set_pose(x, y, yaw, phase)
        if k % 10 == 0:
            renderer.update_scene(sim.data, camera=cam)
            frames.append(Image.fromarray(renderer.render().copy()))
        k += 1
        if math.hypot(x - config.GOAL[0], y - config.GOAL[1]) <= config.GOAL_TOL:
            break
        t += dt
    if not frames:
        print("[动图] 没有采到帧")
        return 1
    out = Path(args.save_dir) / "planner_demo.gif"
    out.parent.mkdir(parents=True, exist_ok=True)
    # 统一调色板 + 抽帧，避免逐帧独立量化带来的体积膨胀和抖动
    step = max(1, round(len(frames) / 30.0))
    picked = frames[::step]
    gw = max(160, min(args.gif_width, w))
    gh = int(round(gw * h / w))
    picked = [f.resize((gw, gh), Image.LANCZOS) for f in picked]
    first = picked[0].quantize(colors=max(8, min(256, args.gif_colors)),
                               method=Image.MEDIANCUT)
    q = [first] + [f.quantize(palette=first, dither=Image.FLOYDSTEINBERG)
                   for f in picked[1:]]
    q[0].save(out, save_all=True, append_images=q[1:],
              duration=90 * step, loop=0, optimize=True, disposal=2)
    frames = picked
    size = out.stat().st_size / 1024
    print("[动图] {}  帧数={}（每 {} 帧取 1）  尺寸={}x{}  {} 色  {:.0f} KB".format(
        out.resolve(), len(q), step, gw, gh, args.gif_colors, size))
    del renderer
    return 0


SHOT_VIEWS = {
    "overview": ((4.0, 0.0, 0.55), 3.4, 90, -86, (4.0, 0.0, 0.0), math.pi * 0.35),
    "persp":    ((4.0, 0.0, 0.7), 9.2, 118, -20, (0.0, 0.0, 0.0), math.pi * 0.5),
    "door":     ((4.6, 0.0, 0.9), 3.4, 133, -22, (4.0, 0.0, 0.9), math.pi * 0.4),
    "obstacle": ((2.45, 0.66, 0.9), 2.5, 140, -20, (2.45, 0.66, 0.0), math.pi * 0.6),
    "start":    ((1.2, 0.0, 0.8), 4.5, 125, -18, (0.6, 0.0, 0.8), math.pi * 0.5),
}


def cmd_shot(args):
    """按指定机位打开视窗并保持，供外部截图使用。"""
    import mujoco
    import mujoco.viewer

    sim = CorridorSim()
    lookat, dist, az, el, pose, phase = SHOT_VIEWS[args.view]
    sim.set_pose(*pose, phase=phase)
    viewer = mujoco.viewer.launch_passive(sim.model, sim.data)
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    viewer.cam.lookat[:] = lookat
    viewer.cam.distance = dist
    viewer.cam.azimuth = az
    viewer.cam.elevation = el
    print("[截图模式] 机位={}，保持 {} 秒".format(args.view, args.hold))
    # 不推进物理：腿没有驱动力，mj_step 会把摆好的步态压平（实测 500 步后
    # root 下沉 20 mm、膝角从 0.999 掉到 0.03），截图时只保持当前位姿。
    end = time.time() + args.hold
    while viewer.is_running() and time.time() < end:
        viewer.sync()
        time.sleep(0.05)
    viewer.close()
    return 0


def cmd_figures(args):
    """导出多机位场景图 + 四宫格（离屏渲染，需 OpenGL 上下文）。"""
    from humanoid_local_planner_bench.figures import render_scene_views, montage

    sim = CorridorSim()
    save = Path(args.save_dir)
    views, w, h = render_scene_views(sim, save)
    for path, title, _img in views:
        print("[图] {}  {}".format(path.resolve(), title))
    mpath, size = montage(views, save / "scene_montage.png")
    print("[图] {}  {}".format(mpath.resolve(), size))
    del sim
    return 0


def cmd_test(args):
    import subprocess
    here = Path(__file__).resolve().parent
    return subprocess.call([sys.executable, str(here / "test" / "test_standalone.py")])


def main():
    args = build_parser().parse_args()
    if args.test:
        return cmd_test(args)
    if args.shot:
        return cmd_shot(args)
    if args.figures:
        return cmd_figures(args)
    if args.gif:
        return cmd_gif(args)
    if args.demo:
        return cmd_demo(args)
    return cmd_headless(args)


if __name__ == "__main__":
    sys.exit(main())
