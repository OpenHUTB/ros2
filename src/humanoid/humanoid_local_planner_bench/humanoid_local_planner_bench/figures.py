# -*- coding: utf-8 -*-
"""出图：多机位场景渲染 + 四宫格拼图。

全部来自 MuJoCo 离屏渲染（真实模型、真实位姿），不含示意图。
"""

import math

# (文件名, 相机 lookat, 距离, 方位角, 仰角, 机器人位姿, 步态相位, 标题)
SCENE_VIEWS = [
    # 近距俯视：主打"0.56 m 身宽卡在 0.90 m 门里"，全走廊布局放在单图里
    ("scene_overview", (4.0, 0.0, 0.55), 3.4, 90, -86, (4.0, 0.0, 0.0), math.pi * 0.35,
     "A. top view at the door: body 0.56 m in a 0.90 m gap"),
    ("scene_persp", (4.0, 0.0, 0.7), 9.2, 118, -20, (0.0, 0.0, 0.0), math.pi * 0.5,
     "B. perspective: humanoid at start"),
    ("scene_door", (4.6, 0.0, 0.9), 3.4, 133, -22, (4.0, 0.0, 0.9), math.pi * 0.4,
     "C. walking through the 0.90 m door"),
    ("scene_obstacle", (2.45, 0.66, 0.9), 2.5, 140, -20, (2.45, 0.66, 0.0), math.pi * 0.6,
     "D. passing obstacle: surface clearance 0.177 m"),
]


def render_scene_views(sim, out_dir, width=1280, height=960):
    """渲染四张场景图，返回 [(path, title), ...] 与图片对象列表。"""
    import mujoco
    from PIL import Image

    w = min(width, sim.model.vis.global_.offwidth)
    h = min(height, sim.model.vis.global_.offheight)
    renderer = mujoco.Renderer(sim.model, height=h, width=w)
    cam = mujoco.MjvCamera()
    results = []
    for name, lookat, dist, az, el, pose, phase, title in SCENE_VIEWS:
        sim.set_pose(*pose, phase=phase)
        mujoco.mjv_defaultFreeCamera(sim.model, cam)
        cam.lookat[:] = lookat
        cam.distance = dist
        cam.azimuth = az
        cam.elevation = el
        renderer.update_scene(sim.data, camera=cam)
        img = Image.fromarray(renderer.render().copy())
        path = out_dir / "{}.png".format(name)
        out_dir.mkdir(parents=True, exist_ok=True)
        img.save(path)
        results.append((path, title, img))
    del renderer
    return results, w, h


def montage(images, out_path, cell=(640, 480), title_h=34, cols=2):
    """把若干 (path, title, image) 拼成网格图。"""
    from PIL import Image, ImageDraw, ImageFont

    cw, ch = cell
    rows = (len(images) + cols - 1) // cols
    canvas = Image.new("RGB", (cw * cols + 4 * (cols + 1),
                                (ch + title_h) * rows + 4 * (rows + 1)), (18, 18, 20))
    draw = ImageDraw.Draw(canvas)
    font = None
    for cand in ("C:/Windows/Fonts/msyh.ttc",
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"):
        try:
            font = ImageFont.truetype(cand, 19)
            break
        except Exception:                                    # noqa: BLE001
            continue
    if font is None:
        font = ImageFont.load_default()

    for i, (_path, title, img) in enumerate(images):
        x = 4 + (i % cols) * (cw + 4)
        y = 4 + (i // cols) * (ch + title_h + 4)
        canvas.paste(img.resize((cw, ch), Image.LANCZOS), (x, y + title_h))
        draw.text((x + 6, y + 7), title, fill=(240, 240, 240), font=font)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(out_path)
    return out_path, canvas.size
