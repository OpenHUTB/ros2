# -*- coding: utf-8 -*-
"""MuJoCo 机器人仿真公共库：GL 后端选择、相机注入、仿真循环、视频输出。

用法：
    每个机器人脚本先调用 setup_gl()（必须在 import mujoco 之前），
    再用 load_with_camera() 加载带自定义相机的模型，
    用 run_sim() 跑物理仿真并把画面输出为 mp4。
"""
import os
import platform

import numpy as np


def setup_gl(force=None):
    """在 import mujoco 之前调用，选择渲染后端。

    - 已有 MUJOCO_GL 环境变量：尊重用户设置
    - Windows：wgl（MuJoCo 3.17 在 Windows 上支持 wgl/glfw）
    - macOS：cgl（系统原生 OpenGL 上下文，离屏可用）
    - Linux：默认 osmesa（软件渲染，虚拟机/SSH 无显示也能跑）；
      有独显可手动设 MUJOCO_GL=egl 提速
    """
    cur = os.environ.get("MUJOCO_GL", "").strip().lower()
    if force:
        os.environ["MUJOCO_GL"] = force
        return force
    if cur:
        return cur
    system = platform.system()
    if system == "Windows":
        backend = "wgl"
    elif system == "Darwin":  # macOS
        backend = "cgl"
    else:  # Linux 及其他 Unix
        backend = "osmesa"
    os.environ["MUJOCO_GL"] = backend
    return backend


def xyaxes_to_quat(xyaxes):
    """把 MJCF 的 xyaxes（相机 x/y 轴方向，6 个数）换算成四元数 [w, x, y, z]。"""
    x = np.asarray(xyaxes[:3], dtype=float)
    y = np.asarray(xyaxes[3:6], dtype=float)
    x /= np.linalg.norm(x)
    y -= x * np.dot(x, y)
    y /= np.linalg.norm(y)
    z = np.cross(x, y)
    r = np.column_stack([x, y, z])
    t = np.trace(r)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, qx, qy, qz = 0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        w, qx, qy, qz = (r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        w, qx, qy, qz = (r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s
    else:
        s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        w, qx, qy, qz = (r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s
    q = np.array([w, qx, qy, qz], dtype=float)
    return q / np.linalg.norm(q)


def load_with_camera(robot, entry, cam, width=960, height=544):
    """加载 mujoco_menagerie 模型，注入自定义相机并设置离屏帧缓冲。

    通过 MuJoCo 的 MjSpec 在内存中修改模型：不向模型目录写任何临时文件
    （menagerie 安装目录可能只读，旧实现会 PermissionError 且残留文件）。

    cam 为字典：
        name  相机名（默认 "view0"）
        pos   相机位置 [x, y, z]
        xyaxes 或 quat  相机姿态（二选一）
        fovy  视场角（可选）
        mode  "track" 时为跟踪模式，配合 target（body 名）
    """
    import mujoco
    import mujoco_menagerie as mj

    r = mj.get(robot)
    xml_path = r.path() / f"{entry}.xml"
    spec = mujoco.MjSpec.from_file(str(xml_path))

    c = spec.worldbody.add_camera()
    c.name = cam.get("name", "view0")
    c.pos = [float(v) for v in cam["pos"]]
    if "xyaxes" in cam:
        c.quat = xyaxes_to_quat(cam["xyaxes"]).tolist()
    elif "quat" in cam:
        c.quat = [float(v) for v in cam["quat"]]
    if cam.get("fovy") is not None:
        c.fovy = float(cam["fovy"])
    if cam.get("mode") == "track":
        c.mode = mujoco.mjtCamera.mjCAMERA_TRACKING
        c.targetbody = cam["target"]

    spec.visual.global_.offwidth = width
    spec.visual.global_.offheight = height
    return spec.compile()


def write_video(frames, path, fps=30):
    import imageio.v2 as iio

    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    arr = np.stack(frames)
    iio.mimsave(
        path, arr, fps=fps, codec="libx264", quality=7,
    )
    print(f"视频已保存: {path}  ({len(frames)} 帧, {fps} fps)")


def run_sim(model, controller, duration=6.0, fps=30, video_path=None,
            preview_dir=None, height=544, width=960):
    """跑仿真并渲染。preview_dir 给定时，额外保存首/中/末三张 PNG 用于检查取景。"""
    import mujoco
    from PIL import Image

    data = mujoco.MjData(model)
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    controller(data, 0.0)

    renderer = mujoco.Renderer(model, height, width)
    dt = model.opt.timestep
    sub = max(1, round(1.0 / fps / dt))
    nframes = int(duration * fps)

    frames = []
    t = 0.0
    for fi in range(nframes):
        for _ in range(sub):
            controller(data, t)
            mujoco.mj_step(model, data)
            t += dt
        renderer.update_scene(data, camera="view0")
        frames.append(renderer.render().copy())
        if preview_dir:
            if fi in {0, nframes // 2, nframes - 1}:
                p = os.path.join(preview_dir, f"preview_{fi:03d}.png")
                os.makedirs(preview_dir, exist_ok=True)
                Image.fromarray(frames[-1]).save(p)
                print("预览帧:", p)
    renderer.close()

    if video_path:
        write_video(frames, video_path, fps)
    return frames


def parse_args():
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--preview", action="store_true",
                    help="只跑 1.5 秒，输出 3 张 PNG 检查取景")
    ap.add_argument("--out", default=None, help="视频输出路径")
    # roslaunch 会追加 __name:=、__log:= 等私有参数，须用 parse_known_args 忽略
    args, _ = ap.parse_known_args()
    return args
