#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
acc_sim.py —— ACC 自适应巡航 Pygame 仿真主程序

场景：本车直线行驶，前车 15 m/s 巡航 -> 8s 减速停车（拥堵）-> 停止 7 s
      -> 重新起步回巡航。ACC 控制器自动完成"减速跟车 -> 跟停 -> 起步再跟"。

用法：
    python3 acc_sim.py                # 交互窗口（空格=暂停/继续，R=重置）
    python3 acc_sim.py --record demo.gif --duration 34   # 无头录制动图
    python3 acc_sim.py --test         # 无界面自动验证（headless_test）
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from vehicle_model import VehicleModel, LeadVehicle
from acc_controller import ACCController

# ---- 场景与参数（与 headless_test 保持一致）----
V_CRUISE = 15.0      # 巡航速度 m/s
D0 = 3.0             # 停车安全间距 m
T_GAP = 1.0          # 跟车时距 s
DT = 0.05            # 仿真步长 s
T_STOP_BEGIN = 8.0   # 前车开始减速
T_STOP_END = 13.0    # 前车停稳
T_GO_BEGIN = 20.0    # 前车重新起步
SIM_TIME = 34.0      # 总时长 s


def build_world():
    """构建仿真世界（本车 + 前车 + 控制器）。"""
    host = VehicleModel(x=0.0, v=V_CRUISE, v_max=V_CRUISE,
                        a_acc=2.5, a_brk=4.0, dt=DT)
    lead = LeadVehicle(x=50.0, v=V_CRUISE, v_cruise=V_CRUISE,
                       a_acc=2.0, a_brk=3.0, dt=DT)
    lead.set_phase_times(T_STOP_BEGIN, T_STOP_END, T_GO_BEGIN)
    ctrl = ACCController(v_cruise=V_CRUISE, d0=D0, t_gap=T_GAP)
    return host, lead, ctrl


def acc_state(d, d_des, v_host, v_lead, v_cmd, margin):
    """返回当前 ACC 工作状态的中文标签。"""
    if v_cmd <= 0.05 and v_host < 0.5:
        return "STOP 跟停"
    if v_lead > 0.5 and d > d_des + margin - 1.0 and v_host > 13.0:
        return "CRUISE 巡航"
    if v_cmd > 0.05 and v_lead > 0.5:
        return "FOLLOW 跟车"
    return "FOLLOW 跟车"


# ---------- Pygame 交互窗口 ----------

def run_interactive():
    import pygame
    from pygame import font as pgfont

    host, lead, ctrl = build_world()

    W, H = 1000, 520
    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("ACC 自适应巡航仿真 (空格=暂停/继续, R=重置, Esc=退出)")
    clock = pygame.time.Clock()

    f = _load_font_pygame()
    paused = False
    t = 0.0

    def reset():
        return build_world(), 0.0

    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    running = False
                elif ev.key == pygame.K_SPACE:
                    paused = not paused
                elif ev.key == pygame.K_r:
                    host, lead, ctrl = build_world()
                    t = 0.0

        if not paused:
            lead_v = lead.step()
            d_center = lead.x - host.x
            v_cmd, d_des = ctrl.compute(d_center, lead_v, host.v)
            host.step(v_cmd)
            t = lead.t

        draw_pygame(screen, f, host, lead, d_des, t, paused)
        pygame.display.flip()
        clock.tick(int(1.0 / DT))
    pygame.quit()


def _load_font_pygame(size=24):
    from pygame import font as pgfont
    pgfont.init()
    candidates = [
        r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\msyhbd.ttc",
        r"C:\Windows\Fonts\simhei.ttf", r"C:\Windows\Fonts\simsun.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ]
    for path in candidates:
        if os.path.exists(path):
            return pgfont.Font(path, size)
    return pgfont.Font(None, size)


def draw_pygame(screen, f, host, lead, d_des, t, paused):
    import pygame
    W, H = screen.get_size()
    SCALE = 7.0          # px/m
    HOST_SCREEN_X = 320  # 本车固定屏幕 x
    CAR_HALF = 16        # 车半长 px（4.5 m / 2 * 7 ≈ 16）
    CAR_HALF_H = 20      # 车半高 px

    # 背景道路
    screen.fill((30, 34, 40))
    pygame.draw.rect(screen, (60, 66, 74), (0, 120, W, 300))          # 路面
    pygame.draw.rect(screen, (235, 235, 235), (0, 120, W, 4))         # 上缘
    pygame.draw.rect(screen, (235, 235, 235), (0, 416, W, 4))         # 下缘
    # 车道分隔虚线（世界坐标 -> 屏幕）
    lane_y = 270
    dash = 40
    gap = 30
    cam_x = host.x * SCALE
    start = int((0 - (320 - cam_x)) // (dash + gap)) - 1
    for i in range(start, start + 40):
        wx = i * (dash + gap)
        sx = 320 + (wx - host.x * SCALE)
        pygame.draw.rect(screen, (220, 220, 220), (sx, lane_y, dash, 5))

    # 前车（灰色）
    lead_sx = HOST_SCREEN_X + int((lead.x - host.x) * SCALE)
    pygame.draw.rect(screen, (150, 158, 166),
                     (lead_sx - CAR_HALF, 250 - CAR_HALF_H,
                      CAR_HALF * 2, CAR_HALF_H * 2), border_radius=8)
    pygame.draw.rect(screen, (110, 118, 126),
                     (lead_sx - CAR_HALF, 250 - CAR_HALF_H,
                      CAR_HALF * 2, CAR_HALF_H * 2), width=2, border_radius=8)
    # 本车（蓝色）
    pygame.draw.rect(screen, (52, 152, 219),
                     (HOST_SCREEN_X - CAR_HALF, 250 - CAR_HALF_H,
                      CAR_HALF * 2, CAR_HALF_H * 2), border_radius=8)
    pygame.draw.rect(screen, (41, 128, 185),
                     (HOST_SCREEN_X - CAR_HALF, 250 - CAR_HALF_H,
                      CAR_HALF * 2, CAR_HALF_H * 2), width=2, border_radius=8)

    # 车距标注（保险杠间距）
    d_center = lead.x - host.x
    d = d_center - ctrl.car_len
    if 0 < d < 120:
        lx, rx = HOST_SCREEN_X + CAR_HALF, lead_sx - CAR_HALF
        if lx < rx:
            pygame.draw.line(screen, (93, 173, 226), (lx, 235), (rx, 235), 2)
            mx = (lx + rx) / 2
            pygame.draw.rect(screen, (30, 34, 40), (mx - 52, 222, 104, 22))
            txt = f.render("%.1f m" % d, True, (255, 255, 255))
            screen.blit(txt, (mx - 26, 224))

    # HUD
    state = acc_state(d, d_des, host.v, lead.v, 0.0 if host.v < 0.05 else 1.0,
                      ctrl.margin)
    d_des_now = ctrl.desired_gap(host.v, lead.v)
    lines = [
        ("ACC 自适应巡航仿真", 30, (255, 255, 255)),
        ("本车速度: %5.1f km/h" % (host.v * 3.6), 70, (52, 152, 219)),
        ("前车速度: %5.1f km/h" % (lead.v * 3.6), 100, (150, 158, 166)),
        ("车距: %5.1f m   期望车距: %4.1f m" % (d, d_des_now), 130, (255, 255, 255)),
        ("状态: %s" % state, 160, (46, 204, 113)),
        ("时间: %4.1f / %.0f s" % (t, SIM_TIME), 190, (200, 200, 200)),
    ]
    if paused:
        lines.append(("【已暂停】 空格继续", 230, (241, 196, 15)))
    for text, y, color in lines:
        surf = f.render(text, True, color)
        screen.blit(surf, (24, y))

    # 时间进度条
    pygame.draw.rect(screen, (60, 66, 74), (24, H - 40, W - 48, 10))
    pygame.draw.rect(screen, (52, 152, 219),
                     (24, H - 40, int((W - 48) * min(t / SIM_TIME, 1.0)), 10))


# ---------- 无头录制（Pillow）----------

def run_record(out_path, duration):
    from PIL import Image, ImageDraw, ImageFont

    host, lead, ctrl = build_world()

    W, H = 1000, 520
    SCALE = 7.0
    HOST_SCREEN_X = 320
    CAR_HALF = 16        # 车半长 px（4.5 m / 2 * 7 ≈ 16）
    CAR_HALF_H = 20      # 车半高 px
    font = _load_font_pil(22)
    font_big = _load_font_pil(30)

    frames = []
    fps = 10
    n_frames = int(duration * fps)
    for i in range(n_frames):
        # 推进仿真（每帧 0.1 s = 2 个 0.05 步）
        for _ in range(int(0.1 / DT)):
            lead_v = lead.step()
            d_center = lead.x - host.x
            v_cmd, d_des = ctrl.compute(d_center, lead_v, host.v)
            host.step(v_cmd)

        img = Image.new("RGB", (W, H), (30, 34, 40))
        dr = ImageDraw.Draw(img)

        # 道路
        dr.rectangle([0, 120, W, 420], fill=(60, 66, 74))
        dr.rectangle([0, 120, W, 124], fill=(235, 235, 235))
        dr.rectangle([0, 416, W, 420], fill=(235, 235, 235))
        # 车道虚线
        dash, gap = 40, 30
        for k in range(-10, 50):
            wx = k * (dash + gap)
            sx = HOST_SCREEN_X + int((wx - host.x * SCALE))
            dr.rectangle([sx, 268, sx + dash, 273], fill=(220, 220, 220))

        # 前车（灰）
        lead_sx = HOST_SCREEN_X + int((lead.x - host.x) * SCALE)
        dr.rounded_rectangle([lead_sx - CAR_HALF, 250 - CAR_HALF_H,
                              lead_sx + CAR_HALF, 250 + CAR_HALF_H],
                             radius=8, fill=(150, 158, 166),
                             outline=(110, 118, 126), width=2)
        # 本车（蓝）
        dr.rounded_rectangle([HOST_SCREEN_X - CAR_HALF, 250 - CAR_HALF_H,
                              HOST_SCREEN_X + CAR_HALF, 250 + CAR_HALF_H],
                             radius=8, fill=(52, 152, 219),
                             outline=(41, 128, 185), width=2)

        # 车距标注（保险杠间距）
        d_center = lead.x - host.x
        d = d_center - ctrl.car_len
        if 0 < d < 120:
            lx, rx = HOST_SCREEN_X + CAR_HALF, lead_sx - CAR_HALF
            if lx < rx:
                dr.line([lx, 235, rx, 235], fill=(93, 173, 226), width=2)
                mx = (lx + rx) / 2
                dr.rectangle([mx - 52, 222, mx + 52, 244], fill=(30, 34, 40))
                dr.text((mx - 26, 224), "%.1f m" % d, fill=(255, 255, 255),
                        font=font)

        # HUD
        d_des_now = ctrl.desired_gap(host.v, lead.v)
        state = acc_state(d, d_des_now, host.v, lead.v,
                          0.0 if host.v < 0.05 else 1.0, ctrl.margin)
        dr.text((24, 30), "ACC 自适应巡航仿真", fill=(255, 255, 255), font=font_big)
        dr.text((24, 70), "本车速度: %5.1f km/h" % (host.v * 3.6),
                fill=(52, 152, 219), font=font)
        dr.text((24, 100), "前车速度: %5.1f km/h" % (lead.v * 3.6),
                fill=(150, 158, 166), font=font)
        dr.text((24, 130), "车距: %5.1f m   期望车距: %4.1f m"
                % (d, d_des_now), fill=(255, 255, 255), font=font)
        dr.text((24, 160), "状态: %s" % state, fill=(46, 204, 113), font=font)
        dr.text((24, 190), "时间: %4.1f / %.0f s" % (lead.t, SIM_TIME),
                fill=(200, 200, 200), font=font)

        # 进度条
        dr.rectangle([24, H - 40, W - 24, H - 30], fill=(60, 66, 74))
        dr.rectangle([24, H - 40,
                      24 + int((W - 48) * min(lead.t / SIM_TIME, 1.0)),
                      H - 30], fill=(52, 152, 219))

        frames.append(img)
        if (i + 1) % 25 == 0:
            print("  录制进度 %d/%d 帧 ..." % (i + 1, n_frames))

    frames[0].save(out_path, save_all=True, append_images=frames[1:],
                   duration=1000 // fps, loop=0)
    print("动图已保存: %s （%d 帧，%d s）" % (out_path, len(frames), duration))


def _load_font_pil(size):
    from PIL import ImageFont
    candidates = [
        r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf",
        r"C:\Windows\Fonts\simsun.ttc", r"C:\Windows\Fonts\arial.ttf",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ]
    for path in candidates:
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


# ---------- 入口 ----------

def main():
    ap = argparse.ArgumentParser(description="ACC 自适应巡航仿真")
    ap.add_argument("--record", metavar="GIF",
                    help="无头录制动图到指定文件（如 demo.gif）")
    ap.add_argument("--duration", type=float, default=SIM_TIME,
                    help="录制时长（秒），默认 %.0f" % SIM_TIME)
    ap.add_argument("--test", action="store_true", help="运行无界面自动验证")
    args = ap.parse_args()

    if args.test:
        import headless_test
        sys.exit(headless_test.main())

    # 无头模式需要 Pillow；交互模式需要 Pygame
    if args.record:
        try:
            from PIL import Image  # noqa: F401
        except ImportError:
            print("错误：录制模式需要 pillow，请先安装：pip3 install pillow")
            return 1
        print("开始录制 ACC 仿真动图（%d s）..." % args.duration)
        run_record(args.record, args.duration)
        return 0

    try:
        import pygame  # noqa: F401
    except ImportError:
        print("错误：交互模式需要 pygame，请先安装：pip3 install pygame")
        return 1
    print("ACC 自适应巡航仿真启动（空格=暂停/继续, R=重置, Esc=退出）")
    run_interactive()
    return 0


# 模块级引用（acc_state 使用）
ctrl = ACCController(v_cruise=V_CRUISE, d0=D0, t_gap=T_GAP)

if __name__ == "__main__":
    sys.exit(main())
