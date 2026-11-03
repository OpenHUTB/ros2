# -*- coding: utf-8 -*-
"""MuJoCo 机器人仿真公共库：GL 后端选择、相机注入、仿真循环、视频输出。

用法：
    每个机器人脚本先调用 setup_gl()（必须在 import mujoco 之前），
    再用 load_with_camera() 加载带自定义相机的模型，
    用 run_sim() 跑物理仿真并把画面输出为 mp4。
"""
import os
import re
import platform

import numpy as np


def setup_gl(force=None):
    """在 import mujoco 之前调用，选择渲染后端。

    - 已有 MUJOCO_GL 环境变量：尊重用户设置
    - Windows：wgl（MuJoCo 3.17 在 Windows 上支持 wgl/glfw）
    - Linux：默认 osmesa（软件渲染，虚拟机无独显也能跑，和之前 sim.py 一致）；
      有独显可手动设 MUJOCO_GL=egl 提速
    """
    cur = os.environ.get("MUJOCO_GL", "").strip().lower()
    if force:
        os.environ["MUJOCO_GL"] = force
        return force
    if cur:
        return cur
    backend = "wgl" if platform.system() == "Windows" else "osmesa"
    os.environ["MUJOCO_GL"] = backend
    return backend


def load_with_camera(robot, entry, cam_xml, width=960, height=544):
    """加载 mujoco_menagerie 模型，注入自定义相机并加大离屏帧缓冲。

    cam_xml 形如 '<camera name="view0" pos="..." xyaxes="..."/>'，
    会被插到 <worldbody> 内部。
    """
    import mujoco
    import mujoco_menagerie as mj

    r = mj.get(robot)
    xml_path = r.path() / f"{entry}.xml"
    xml = xml_path.read_text(encoding="utf-8")

    # 1) 把相机插进 <worldbody> 开头
    m_wb = re.search(r"<worldbody[^>]*>", xml)
    if not m_wb:
        raise RuntimeError(f"{robot}/{entry}: 找不到 <worldbody>")
    xml = xml[: m_wb.end()] + cam_xml + xml[m_wb.end():]

    # 2) 统一设置离屏帧缓冲分辨率（去掉旧的 <visual>，避免冲突）
    xml = re.sub(r"<visual>.*?</visual>", "", xml, flags=re.S)
    xml = xml.replace(
        "</mujoco>",
        f'<visual><global offwidth="{width}" offheight="{height}"/></visual></mujoco>',
    )

    tmp = xml_path.parent / f"{entry}_view.xml"
    tmp.write_text(xml, encoding="utf-8")
    return mujoco.MjModel.from_xml_path(str(tmp))


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
    return ap.parse_args()
