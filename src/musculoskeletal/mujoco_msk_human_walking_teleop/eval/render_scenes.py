#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成三个场景的说明截图（headless，英文标注保证任意环境可复现）。

用法（在包根目录执行）：
  python3 eval/render_scenes.py [输出目录，默认当前目录]

每张截图：站立 1 s 后的 MuJoCo 离屏渲染帧 + 标注。
标注只用英文与 ASCII（MuJoCo 默认字体不含中文），叠加元素：
  - 左上角：场景名与重力向量
  - 坡道：红色箭头标出沿坡向下的重力 x 分量
  - 负载：躯干上的灰色矩形示意 10 kg 背包
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "scripts"))
import numpy as np
from PIL import Image, ImageDraw

import mujoco

from msk_human_core import Composer, MskHumanSim

# 离屏 framebuffer 默认 640x480（模型未设 offwidth/offheight）
W, H = 640, 480


def grab(sim, comp):
    """站立 1 s 后返回当前帧 (H,W,3) RGB。"""
    comp.set_action("stand")
    sim.reset()
    for _ in range(500):
        sim.step(comp.update(sim, 0.002))
    arr = np.zeros((H, W, 3), dtype=np.uint8)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [sim.pelvis_x(), 0.0, 0.8]
    cam.distance, cam.azimuth, cam.elevation = 3.2, 90.0, -8.0
    renderer = sim._renderer
    renderer.update_scene(sim.data, camera=cam)
    arr[:] = renderer.render()
    return Image.fromarray(arr)


def label(img, lines):
    d = ImageDraw.Draw(img)
    y = 10
    for s in lines:
        d.text((12, y), s, fill=(255, 255, 255))
        y += 22
    return d


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    from mujoco import Renderer
    imgs = {}
    for scene, title, grav in (
            ("flat", "Scene 1: Flat ground", "g = (0, 0, -9.81)"),
            ("slope5", "Scene 2: Slope 5 deg downhill (equivalent gravity)",
             "g = (+0.85, 0, -9.77) m/s^2"),
            ("load", "Scene 3: Load +10 kg backpack on torso",
             "g = (0, 0, -9.81) m/s^2")):
        sim = MskHumanSim(scene)
        # mujoco.Renderer 签名为 (model, height, width)，数据在 update_scene 传入
        sim._renderer = Renderer(sim.model, height=H, width=W)
        img = grab(sim, Composer())
        d = label(img, [title, grav])
        if scene == "slope5":
            # 红色箭头：沿坡向下的重力 x 分量（助推 +x 行走，即下坡）
            x0, y0 = 180, 60
            d.line([(x0, y0), (x0 + 120, y0)], fill=(230, 60, 60), width=5)
            d.polygon([(x0 + 120, y0), (x0 + 100, y0 - 8),
                       (x0 + 100, y0 + 8)], fill=(230, 60, 60))
            d.text((x0 - 30, y0 + 12), "+g*sin(5deg) = +0.85 m/s^2",
                   fill=(230, 60, 60))
        if scene == "load":
            # 躯干上叠加灰色背包矩形（lookat z=0.8 使骨盆位于画面中心偏下）
            cx, cy = W // 2, H // 2 + 20
            d.rounded_rectangle([cx - 55, cy - 70, cx + 55, cy + 40],
                                radius=12, fill=(90, 90, 95),
                                outline=(30, 30, 30), width=2)
            d.text((cx - 46, cy - 20), "+10 kg", fill=(255, 255, 255))
        path = os.path.join(out_dir, "scene_%s.png" % scene)
        img.save(path)
        print("saved", path)
        sim._renderer.close()
        sim.close()


if __name__ == "__main__":
    main()
