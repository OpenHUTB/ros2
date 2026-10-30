"""超车变道 Pygame 交互仿真 + GIF 录制。

用法：
    显示窗口交互运行：
        python3 overtake_sim.py
    无窗口录制演示动图（Pillow 兼容模式，无需 pygame）：
        python3 overtake_sim.py --record demo.gif --duration 26

兼容 Pillow >= 7.0（不使用 rounded_rectangle）。
"""

import argparse
import io

from vehicle_model import VehicleModel
from overtake_controller import OvertakeController

# ---------- 场景参数 ----------
DT = 0.02            # 仿真步长（s）
FPS = 30             # 交互显示帧率
GIF_FPS = 6          # 录制 gif 帧率
VIEW_X0 = -15.0      # 视口起点相对本车（m）
VIEW_W = 120.0       # 视口纵向宽度（m）
VIEW_Y0 = -5.5       # 视口横向起点（m）
VIEW_H = 11.0        # 视口横向高度（m）
W, H = 720, 480      # 画布尺寸
LANES = [-3.5, 0.0, 3.5]   # 三条车道中心线横向位置（m）


def draw_scene(draw, t, host, lead, ctrl):
    """用 ImageDraw 绘制一帧（Pillow 兼容写法）。"""
    sx = W / VIEW_W
    sy = H / VIEW_H

    def px(x):
        return int((x - (host.x + VIEW_X0)) * sx)

    def py(y):
        return int((y - VIEW_Y0) * sy)

    def draw_car(car, color):
        """按固定视觉尺寸绘制车辆矩形（长沿 x，宽沿 y）。"""
        car_w = 58          # 纵向长度（px）
        car_h = 30          # 横向宽度（px）
        cx = px(car.x)
        cy = py(car.y)
        x0, y0 = cx - car_w // 2, cy - car_h // 2
        x1, y1 = cx + car_w // 2, cy + car_h // 2
        draw.rectangle([x0, y0, x1, y1], fill=color, outline=(255, 255, 255), width=2)
        # 车头标记（右侧小三角）
        draw.polygon([(x1, cy - 6), (x1 + 9, cy), (x1, cy + 6)], fill=color)
        # 车尾标记（左侧小方块）
        draw.rectangle([x0 - 4, cy - 4, x0, cy + 4], fill=(255, 255, 255))

    # 背景（草地/路肩）
    draw.rectangle([0, 0, W, H], fill=(46, 54, 60))
    # 道路（y 坐标：画面顶部小、底部大）
    r_a = py(VIEW_Y0 + VIEW_H)     # +5.5 → 画面底部
    r_b = py(VIEW_Y0)              # -5.5 → 画面顶部
    road_top = min(r_a, r_b)
    road_bottom = max(r_a, r_b)
    draw.rectangle([0, road_top, W, road_bottom], fill=(62, 62, 68))
    # 车道中心线（白色虚线简化：实线）
    for lane_c in LANES:
        y = py(lane_c)
        draw.line([0, y, W, y], fill=(255, 255, 255), width=2)
    # 道路边线（黄色）
    draw.line([0, road_top, W, road_top], fill=(255, 220, 120), width=3)
    draw.line([0, road_bottom, W, road_bottom], fill=(255, 220, 120), width=3)

    # 距离刻度：每 20m（画在道路上缘内侧）
    base = int((host.x + VIEW_X0) // 20) * 20
    for xm in range(base, int(host.x + VIEW_X0 + VIEW_W) + 1, 20):
        x = px(xm)
        if 0 <= x <= W:
            draw.line([x, road_top, x, road_top + 10], fill=(255, 220, 120), width=2)
            draw.text((x - 8, road_top + 16), f"{xm}m", fill=(230, 230, 230))

    # 前车（红）与本车（蓝）
    draw_car(lead, (220, 60, 60))
    draw_car(host, (70, 140, 230))

    # HUD 面板
    draw.rectangle([6, 6, W - 6, 62], fill=(24, 24, 28))
    kmh = host.v * 3.6
    bgap = (lead.x - host.x) - ctrl.car_len
    state = ctrl.state
    left_sig = 'L' if state in ('SIGNAL', 'CHANGING') else ' '
    right_sig = 'R' if state == 'CHANGING_BACK' else ' '
    draw.text((16, 12), f"t={t:5.1f}s  v={kmh:5.1f}km/h  gap={bgap:5.1f}m",
              fill=(240, 240, 240))
    draw.text((16, 34), f"state={state}  lane={1 if host.y > 1.75 else 0}  "
                        f"sig=[{left_sig}|{right_sig}]",
              fill=(255, 255, 120))
    draw.text((W - 210, 12), f"lead v={lead.v*3.6:5.1f}km/h", fill=(255, 160, 160))
    draw.text((W - 210, 34), f"y={host.y:+5.2f}m", fill=(160, 220, 255))


def build_gif(duration):
    """逐帧渲染并存为 GIF（Pillow 兼容模式，无 pygame 依赖）。"""
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        print("需要 Pillow：pip3 install pillow")
        return None

    host = VehicleModel(x=0.0, y=0.0, v=15.0)
    lead = VehicleModel(x=80.0, y=0.0, v=8.0)
    ctrl = OvertakeController()

    frames = []
    t = 0.0
    n = int(duration / DT)
    frame_step = max(1, int(1.0 / (GIF_FPS * DT)))
    idx = 0
    while t < duration and idx < n:
        a, vy, state, info = ctrl.compute(host.x, host.y, host.v, lead.x, lead.v, DT)
        host.update(a, vy, DT)
        lead.update(0.0, 0.0, DT)
        if idx % frame_step == 0:
            img = Image.new('RGB', (W, H), (46, 54, 60))
            draw = ImageDraw.Draw(img)
            draw_scene(draw, t, host, lead, ctrl)
            frames.append(img)
        t += DT
        idx += 1

    if not frames:
        print("没有生成帧")
        return None
    buf = io.BytesIO()
    frames[0].save(buf, format='GIF', save_all=True, append_images=frames[1:],
                   duration=int(1000 / GIF_FPS), loop=0, optimize=True)
    return buf.getvalue()


def run_interactive():
    """pygame 窗口交互运行（需要 pygame）。"""
    try:
        import pygame
    except ImportError:
        print("需要 pygame：pip3 install pygame；或使用 --record 录制 gif")
        return

    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption('Overtake Simulation (超车变道)')
    clock = pygame.time.Clock()

    host = VehicleModel(x=0.0, y=0.0, v=15.0)
    lead = VehicleModel(x=80.0, y=0.0, v=8.0)
    ctrl = OvertakeController()
    t = 0.0
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

        a, vy, state, info = ctrl.compute(host.x, host.y, host.v, lead.x, lead.v, DT)
        host.update(a, vy, DT)
        lead.update(0.0, 0.0, DT)
        t += DT

        from PIL import Image, ImageDraw
        img = Image.new('RGB', (W, H), (46, 54, 60))
        draw = ImageDraw.Draw(img)
        draw_scene(draw, t, host, lead, ctrl)
        pygame_img = pygame.image.fromstring(img.tobytes(), img.size, 'RGB')
        screen.blit(pygame_img, (0, 0))
        pygame.display.flip()
        clock.tick(FPS)
    pygame.quit()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--record', help='输出 GIF 文件名（无窗口录制）')
    ap.add_argument('--duration', type=float, default=26.0, help='仿真时长（s）')
    args = ap.parse_args()

    if args.record:
        data = build_gif(args.duration)
        if data:
            with open(args.record, 'wb') as f:
                f.write(data)
            print(f"已生成 {args.record} ({len(data)/1024:.0f} KB, {args.duration:.0f}s)")
        return

    run_interactive()


if __name__ == '__main__':
    main()
