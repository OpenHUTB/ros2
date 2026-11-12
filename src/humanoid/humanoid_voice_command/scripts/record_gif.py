"""录制「说口令 -> 识别 -> 机器人动作」的演示动图。

仿真执行层直接复用 humanoid_keyboard_control 包的 HumanoidSim（不重复实现），
本脚本只负责：按口令序列推进仿真、渲染画面、把识别结果叠到画面上、合成 gif。

用法::

    python scripts/record_gif.py                       # 输出 docs 用动图
    python scripts/record_gif.py --out demo.gif --width 400 --height 300
    python scripts/record_gif.py --fps 10 --seconds 2.0

依赖：mujoco、pillow、numpy，以及本机已构建/可导入的 humanoid_keyboard_control。
"""

import argparse
import math
import os
import sys


HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.abspath(os.path.join(HERE, '..'))
sys.path.insert(0, PKG_ROOT)

# 口令分镜：(朗读文本, 意图, vx, wz)
SEGMENTS = [
    ('前进', 'forward', 0.30, 0.0),
    ('向左转', 'turn_left', 0.0, 0.60),
    ('向右转', 'turn_right', 0.0, -0.60),
    ('停止', 'stop', 0.0, 0.0),
]

FONT_CANDIDATES = [
    'C:/Windows/Fonts/msyh.ttc',
    'C:/Windows/Fonts/simhei.ttf',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
    '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
]


def find_font():
    for path in FONT_CANDIDATES:
        if os.path.exists(path):
            return path
    return None


def load_sim_core():
    """导入 humanoid_keyboard_control.sim_core（已安装、share 目录或源码树）。"""
    try:
        from humanoid_keyboard_control.sim_core import HumanoidSim
        return HumanoidSim
    except ImportError:
        pass
    # 源码树：src/humanoid/humanoid_keyboard_control/humanoid_keyboard_control/sim_core.py
    src_pkg = os.path.abspath(os.path.join(
        PKG_ROOT, '..', 'humanoid_keyboard_control'))
    if os.path.isdir(os.path.join(src_pkg, 'humanoid_keyboard_control')):
        sys.path.insert(0, src_pkg)
        try:
            from humanoid_keyboard_control.sim_core import HumanoidSim
            return HumanoidSim
        except ImportError:
            sys.path.pop(0)
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('humanoid_keyboard_control')
        sys.path.insert(0, share)
        from humanoid_keyboard_control.sim_core import HumanoidSim
        return HumanoidSim
    except Exception as exc:                            # noqa: BLE001
        raise SystemExit(f'找不到 humanoid_keyboard_control 包：{exc}') from exc


def find_model():
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('humanoid_keyboard_control'),
                            'models', 'humanoid_simple.xml')
        if os.path.exists(path):
            return path
    except Exception:                                   # noqa: BLE001
        pass
    guess = os.path.abspath(os.path.join(
        PKG_ROOT, '..', 'humanoid_keyboard_control', 'models', 'humanoid_simple.xml'))
    if os.path.exists(guess):
        return guess
    raise SystemExit('找不到 humanoid_simple.xml，请先构建 humanoid_keyboard_control')


def draw_overlay(img, font_path, title, subtitle, badge, accent):
    """在渲染图上叠半透明信息条。"""
    from PIL import Image, ImageDraw, ImageFont

    pil = Image.fromarray(img)
    draw = ImageDraw.Draw(pil, 'RGBA')
    w, h = pil.size
    big = ImageFont.truetype(font_path, 21) if font_path else ImageFont.load_default()
    small = ImageFont.truetype(font_path, 14) if font_path else ImageFont.load_default()

    # 顶部：口令文本 + 意图
    draw.rectangle([0, 0, w, 58], fill=(10, 12, 18, 210))
    draw.text((12, 6), '语音口令: ' + title, font=big, fill=(255, 255, 255, 255))
    draw.text((12, 36), subtitle, font=small, fill=(140, 200, 255, 255))

    # 右上角：意图徽标
    tw = draw.textlength(badge, font=small)
    draw.rounded_rectangle([w - tw - 26, 10, w - 10, 32], radius=6,
                           fill=accent + (235,))
    draw.text((w - tw - 18, 13), badge, font=small, fill=(255, 255, 255, 255))

    # 底部：速度指令
    draw.rectangle([0, h - 28, w, h], fill=(10, 12, 18, 205))
    return pil


def main():
    ap = argparse.ArgumentParser(description='录制语音指令演示动图')
    ap.add_argument('--out', default=os.path.join(PKG_ROOT, '..', '..', '..',
                                                  'docs', 'img', 'humanoid',
                                                  'humanoid_voice_command.gif'))
    ap.add_argument('--width', type=int, default=480)
    ap.add_argument('--height', type=int, default=360)
    ap.add_argument('--fps', type=int, default=12)
    ap.add_argument('--seconds', type=float, default=2.0, help='每条口令的时长')
    ap.add_argument('--font', default='')
    args = ap.parse_args()

    import mujoco
    from PIL import ImageDraw, ImageFont

    HumanoidSim = load_sim_core()
    sim = HumanoidSim(model_path=find_model())
    font_path = args.font or find_font()

    # 提亮灯光：默认场景偏暗，动图看不清机体
    try:
        sim.model.vis.headlight.ambient[:] = (0.55, 0.55, 0.60)
        sim.model.vis.headlight.diffuse[:] = (0.75, 0.75, 0.75)
    except AttributeError:
        pass

    # 隐藏场景里的参照物（红柱/蓝方块），避免遮挡机体；它们只是静态装饰，不影响物理
    for name in ('p1', 'p2', 'b1', 'b2'):
        gid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if gid >= 0:
            sim.model.geom_rgba[gid] = (0.0, 0.0, 0.0, 0.0)

    renderer = mujoco.Renderer(sim.model, height=args.height, width=args.width)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance = 4.1
    cam.azimuth = 145
    cam.elevation = -6

    frames = []
    steps_per_frame = max(1, int(round((1.0 / args.fps) / sim.dt)))
    total_frames = int(args.fps * args.seconds)

    for idx, (text, intent, vx, wz) in enumerate(SEGMENTS):
        accent = {'forward': (40, 130, 220), 'turn_left': (230, 150, 30),
                  'turn_right': (150, 80, 210), 'stop': (200, 50, 60)}[intent]
        for f in range(total_frames):
            for _ in range(steps_per_frame):
                sim.step(v_lin=vx, w_yaw=wz)
            od = sim.read_odom()
            cam.lookat[:] = [od['x'], od['y'], 0.95]
            renderer.update_scene(sim.data, camera=cam)
            frame = renderer.render()

            # 前半段显示「识别中」，后半段显示识别结果（模拟真实时延）
            if f < 2:
                title, sub, badge = '...', '正在识别语音', 'ASR'
            else:
                title = text
                sub = f'识别文本: {text}  →  意图: {intent}'
                badge = intent
            pil = draw_overlay(frame, font_path, title, sub, badge, accent)

            draw = ImageDraw.Draw(pil)
            small = ImageFont.truetype(font_path, 12) if font_path else ImageFont.load_default()
            draw.text((10, args.height - 23),
                      f'vx={vx:+.2f} m/s   wz={wz:+.2f} rad/s', font=small,
                      fill=(140, 200, 255, 255))
            pos = f"x={od['x']:+.2f} m   yaw={math.degrees(od['yaw']):+.0f}°"
            pw = draw.textlength(pos, font=small)
            draw.text((args.width - pw - 10, args.height - 23), pos, font=small,
                      fill=(170, 230, 180, 255))
            frames.append(pil.convert('RGB'))

    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    frames[0].save(out, save_all=True, append_images=frames[1:],
                   duration=int(1000 / args.fps), loop=0, optimize=True)
    size = os.path.getsize(out)
    print('已生成 %s  %d 帧  %.2f MB' % (out, len(frames), size / 1024 / 1024))
    renderer.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
