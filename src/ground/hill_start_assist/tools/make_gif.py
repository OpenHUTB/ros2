#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""渲染坡道起步辅助的对比动图：无辅助滚下 → 有辅助稳住。"""
import math
import sys
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG_ROOT / "src"))

import mujoco                                    # noqa: E402
from PIL import Image, ImageDraw, ImageFont      # noqa: E402

from hill_start_assist import config             # noqa: E402
from hill_start_assist.control import HillStartController   # noqa: E402
from hill_start_assist.spawn import place_on_slope, settle_with_brake  # noqa: E402

W, H, FPS = 384, 240, 8
WHEELS = ("wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint")


def load_font(size=12):
    for path in ("C:/Windows/Fonts/msyh.ttc", "C:/Windows/Fonts/simhei.ttf",
                 "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
                 "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"):
        try:
            return ImageFont.truetype(path, size)
        except Exception:
            continue
    return None


def main():
    m = mujoco.MjModel.from_xml_path(config.resolve_scene_path(None))
    d = mujoco.MjData(m)
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")
    qadr, vadr = [], []
    for n in WHEELS:
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        qadr.append(int(m.jnt_qposadr[j]))
        vadr.append(int(m.jnt_dofadr[j]))
    ctrl = HillStartController()
    slope = math.radians(config.SLOPE_DEG)
    r = mujoco.Renderer(m, H, W)
    step = int(round(1.0 / (m.opt.timestep * FPS)))
    frames, labels = [], []

    for assist, label in ((False, "A. 无辅助：松刹车后沿坡滚下"),
                          (True, "B. 有辅助：四轮轮角保持 + 坡度前馈")):
        # 出生/落稳与 check_hill.py 共用同一实现，避免"动图用旧场景"这种偏差
        place_on_slope(m, d)
        phi0 = settle_with_brake(m, d, ctrl, qadr, vadr)
        x0 = float(d.xpos[bid][0])
        for k in range(int(2.5 * FPS)):
            for _ in range(step):
                for i in range(4):
                    if assist:
                        tau = ctrl.torque(float(d.qpos[qadr[i]]) - phi0[i],
                                          float(d.qvel[vadr[i]]))
                        d.ctrl[i] = tau / 20.0
                    else:
                        d.ctrl[i] = 0.0
                mujoco.mj_step(m, d)
            r.update_scene(d, camera="side")
            frames.append(Image.fromarray(r.render()))
            labels.append((label, float(d.xpos[bid][0]) - x0))

    font = load_font(12)
    if font is None:
        labels = [("A. no assist: rolls down", dx) if "无辅助" in lb
                  else ("B. assist ON: holds", dx) for lb, dx in labels]
    out = []
    for img, (label, dx) in zip(frames, labels):
        dr = ImageDraw.Draw(img)
        dr.rectangle([0, 0, W, 32], fill=(0, 0, 0))
        dr.text((6, 3), label, fill=(255, 255, 255), font=font)
        dr.text((6, 17), ("沿坡位移 %+.3f m" if font else "drift %+.3f m") % dx,
                fill=(255, 200, 80), font=font)
        out.append(img)

    # 直接写到仓库的 docs/img/，与文档里的引用保持一致
    # （旧版写 docs_gif/，而文档引用 docs/img/，评审指出过）
    dst = Path(PKG_ROOT).parent.parent.parent / "docs" / "img" / "hill_start_drive.gif"
    dst.parent.mkdir(parents=True, exist_ok=True)
    out[0].save(dst, save_all=True, append_images=out[1:],
                duration=int(1000 / FPS), loop=0, optimize=True)
    print("[GIF] %s（%d 帧，%.1f MB）" % (dst, len(out), dst.stat().st_size / 1e6))


if __name__ == "__main__":
    main()
