"""录制柔顺性演示动图：同一外力下「低刚度让位大 / 高刚度让位小」。

场景（总 9 s）：

    0.0 - 1.5 s   站立
    1.5 - 3.5 s   左腕受 6 N 侧向力，K_lin_la = 30（低刚度）
    3.5 - 4.5 s   撤销外力，回弹
    4.5 - 5.5 s   切到 K_lin_la = 250（高刚度）
    5.5 - 7.5 s   同样 6 N 侧向力
    7.5 - 9.0 s   撤销外力

用法::

    python scripts/record_gif.py
    python scripts/record_gif.py --width 480 --height 360 --out demo.gif
"""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.abspath(os.path.join(HERE, '..'))
sys.path.insert(0, PKG_ROOT)

from humanoid_g1_compliance.policy import (                   # noqa: E402
    Command, LacPolicy, StartupFSM,
)
from humanoid_g1_compliance.sim_core import G1Sim             # noqa: E402

FONTS = ['C:/Windows/Fonts/msyh.ttc', 'C:/Windows/Fonts/simhei.ttf',
         '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
         '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc']

FORCE = 6.0
K_LOW, K_HIGH = 30.0, 250.0
HAND = 'left_wrist_roll_rubber_hand'

# (开始, 结束, 刚度, 是否施力, 阶段名)
# 时序与 scripts/bench_compliance.py 对齐（先站 3 s 再施力），
# 这样动图上的让位读数能和文档里的刚度扫描表对上。
SEGMENTS = [
    (0.0, 3.0, K_HIGH, False, '站立'),
    (3.0, 5.0, K_LOW, True, '低刚度 · 受力'),
    (5.0, 6.0, K_LOW, False, '低刚度 · 撤力'),
    (6.0, 7.0, K_HIGH, False, '切换刚度'),
    (7.0, 9.0, K_HIGH, True, '高刚度 · 受力'),
    (9.0, 10.0, K_HIGH, False, '高刚度 · 撤力'),
]
DURATION = SEGMENTS[-1][1]


def find_font():
    for p in FONTS:
        if os.path.exists(p):
            return p
    return None


def segment_at(t):
    for seg in SEGMENTS:
        if seg[0] <= t < seg[1]:
            return seg
    return SEGMENTS[-1]


def draw_overlay(img, font_path, title, sub, badge, accent, delta_mm, delta_max):
    """顶部：阶段 + 刚度；底部：手腕让位条形指示。"""
    from PIL import Image, ImageDraw, ImageFont

    pil = Image.fromarray(img)
    draw = ImageDraw.Draw(pil, 'RGBA')
    w, h = pil.size
    big = ImageFont.truetype(font_path, 20) if font_path else ImageFont.load_default()
    small = ImageFont.truetype(font_path, 14) if font_path else ImageFont.load_default()
    tiny = ImageFont.truetype(font_path, 12) if font_path else ImageFont.load_default()

    # ---- 顶部信息条 ----
    draw.rectangle([0, 0, w, 54], fill=(12, 14, 20, 210))
    draw.text((10, 5), title, font=big, fill=(255, 255, 255, 255))
    draw.text((10, 32), sub, font=small, fill=(150, 210, 255, 255))

    tw = draw.textlength(badge, font=small)
    draw.rounded_rectangle([w - tw - 24, 8, w - 8, 30], radius=6, fill=accent + (232,))
    draw.text((w - tw - 16, 11), badge, font=small, fill=(255, 255, 255, 255))

    # ---- 底部：让位量 + 条形指示 ----
    bar_top = h - 42
    draw.rectangle([0, bar_top, w, h], fill=(12, 14, 20, 205))
    draw.text((10, bar_top + 4), '手腕让位', font=tiny, fill=(170, 175, 190, 255))
    draw.text((10, bar_top + 20), f'{delta_mm:+.1f} mm', font=small,
              fill=(255, 236, 170, 255))

    bx0, bx1 = 96, w - 12
    by0, by1 = bar_top + 14, bar_top + 26
    draw.rectangle([bx0, by0, bx1, by1], fill=(38, 42, 54, 255))
    frac = min(abs(delta_mm) / delta_max, 1.0) if delta_max > 0 else 0.0
    fill_w = int((bx1 - bx0) * frac)
    if fill_w > 0:
        color = (232, 130, 26, 255) if frac > 0.55 else (70, 150, 220, 255)
        draw.rectangle([bx0, by0, bx0 + fill_w, by1], fill=color)
    # 0.5 处的参考刻度
    mid = (bx0 + bx1) // 2
    draw.line([mid, by0 - 2, mid, by1 + 2], fill=(120, 126, 140, 255), width=1)
    draw.text((bx0, bar_top + 1), '0', font=tiny, fill=(130, 135, 150, 255))
    end = f'{delta_max:.0f} mm'
    ew = draw.textlength(end, font=tiny)
    draw.text((bx1 - ew, bar_top + 1), end, font=tiny, fill=(130, 135, 150, 255))

    return pil


def main():
    ap = argparse.ArgumentParser(description='录制 LAC 柔顺性演示动图')
    ap.add_argument('--out', default=os.path.join(
        PKG_ROOT, '..', '..', '..', 'docs', 'img', 'humanoid',
        'humanoid_g1_compliance.gif'))
    ap.add_argument('--width', type=int, default=420)
    ap.add_argument('--height', type=int, default=320)
    ap.add_argument('--fps', type=int, default=15)
    ap.add_argument('--font', default=None)
    ap.add_argument('--device', default='cpu')
    args = ap.parse_args()

    import mujoco
    from PIL import Image

    sim = G1Sim()
    policy = LacPolicy(device=args.device, verbose=False)
    fsm = StartupFSM(direct=True)
    hand_id = sim.body_id(HAND)

    # 提亮：默认 headlight 偏暗
    try:
        sim.model.vis.headlight.ambient[:] = (0.55, 0.55, 0.60)
        sim.model.vis.headlight.diffuse[:] = (0.78, 0.78, 0.78)
    except AttributeError:
        pass

    renderer = mujoco.Renderer(sim.model, height=args.height, width=args.width)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance = 2.55
    cam.azimuth = 150
    cam.elevation = -7

    font_path = args.font or find_font()
    step_every = max(1, int(round(1.0 / (sim.timestep * 4) / args.fps)))  # 50 Hz -> fps
    substeps = 4
    dt = sim.timestep * substeps
    n = int(round(DURATION / dt))

    sim.reset()
    policy.reset()
    fsm.reset()
    # 让位基线取「施力前一瞬」，避开起步阶段策略收敛带来的整体后移
    base_hand = None
    base_t = SEGMENTS[1][0] - 0.1
    delta_max = 55.0

    frames = []
    for k in range(n):
        now = k * dt
        t0, t1, stiff, pushing, label = segment_at(now)

        cmd = Command()
        cmd.stiffness[0] = stiff
        cmd.clamp()

        st = sim.read()
        q_policy = policy.infer(st, cmd)
        q_cmd, kp, kd = fsm.step(now, st.q, q_policy)

        sim.clear_forces()
        if pushing:
            sim.apply_force(sim.body_id(HAND), (0.0, FORCE, 0.0))
        sim.apply_pd(q_cmd, kp, kd)
        sim.step(substeps)

        if k % step_every:
            continue

        hand = sim.body_pos(hand_id)
        if base_hand is None and now >= base_t:
            base_hand = hand[1]
        delta_mm = 0.0 if base_hand is None else (hand[1] - base_hand) * 1000.0
        pel = sim.pelvis_pos()
        cam.lookat[:] = [pel[0], pel[1], 0.55]
        renderer.update_scene(sim.data, camera=cam)
        frame = renderer.render()

        if pushing:
            title = f'{label}   {FORCE:.0f} N →'
            sub = f'K_lin_la = {stiff:.0f} N/m    让位 {delta_mm:+.1f} mm'
            accent = (232, 130, 26)
        else:
            title = label
            sub = f'K_lin_la = {stiff:.0f} N/m'
            accent = (70, 150, 220)

        pil = draw_overlay(frame, font_path, title, sub, f'K={stiff:.0f}',
                           accent, delta_mm, delta_max)
        frames.append(pil.convert('RGB').convert(
            'P', palette=Image.ADAPTIVE, colors=96))

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    frames[0].save(args.out, save_all=True, append_images=frames[1:],
                   duration=int(1000 / args.fps), loop=0, optimize=True)
    size = os.path.getsize(args.out)
    print(f'已保存 {args.out}  {len(frames)} 帧  {size / 1024:.0f} KB')


if __name__ == '__main__':
    main()
