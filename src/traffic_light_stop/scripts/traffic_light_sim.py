"""红绿灯启停 Pygame 仿真。

车辆沿道路直线行驶，前方路口有红绿灯（含停止线）：
- 绿灯：巡航通过；
- 红灯/黄灯：自动减速并在停止线前停住；
- 变绿后自动起步通过路口。

用法（窗口演示）：
    python traffic_light_sim.py
    空格：开始/暂停    R：重置    ESC：退出

用法（无头录制演示动图）：
    python traffic_light_sim.py --record demo.gif --duration 25
"""
import argparse
import math
import os
import sys

import pygame

from vehicle_model import VehicleModel
from traffic_light import TrafficLight
from stop_controller import StopController

# ---------- 场景参数 ----------
V_CRUISE = 10.0        # 巡航速度 m/s（36 km/h）
STOP_X = 50.0          # 停止线位置 m
SAFETY = 1.0           # 停车时车头距停止线的安全余量 m
A_MAX = 3.0            # 最大减速度 m/s^2
A_ACC = 2.0            # 最大加速度 m/s^2
DT = 0.05              # 仿真步长 s
PHASES = [(4.0, 'green'), (1.0, 'yellow'), (9.0, 'red'), (12.0, 'green')]

# ---------- 渲染参数 ----------
W, H = 1200, 500
SCALE = 12.0           # px per meter
CAM_AHEAD = 18.0       # 相机领先车头的米数
ROAD_Y = 340           # 道路中心线屏幕 y

# 中文字体候选（Windows / Ubuntu 常见路径），找不到则退回默认字体
_FONT_CANDIDATES = [
    "C:/Windows/Fonts/msyh.ttc",
    "C:/Windows/Fonts/msyh.ttf",
    "C:/Windows/Fonts/simhei.ttf",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
]


def load_font(size):
    for path in _FONT_CANDIDATES:
        if os.path.exists(path):
            try:
                return pygame.font.Font(path, size)
            except Exception:
                continue
    return pygame.font.Font(None, size)


class Simulation:
    def __init__(self):
        self.reset()

    def reset(self):
        self.model = VehicleModel(x=0.0, v=0.0)
        self.light = TrafficLight(PHASES)
        self.controller = StopController(
            v_cruise=V_CRUISE, stop_x=STOP_X, safety=SAFETY,
            a_max=A_MAX, a_acc=A_ACC)
        self.t = 0.0
        self.running = False

    def step(self):
        color = self.light.update(DT)
        a = self.controller.compute(self.model.head(), self.model.v, color)
        self.model.step(a)
        self.t += DT

    # ---------- 绘制 ----------
    def render(self, surface):
        surface.fill((135, 190, 235))                     # 天空
        pygame.draw.rect(surface, (90, 150, 90), (0, 300, W, 200))   # 地面
        pygame.draw.rect(surface, (80, 80, 80), (0, 310, W, 70))     # 路面

        cam_x = self.model.x - CAM_AHEAD

        def sx(wx):
            return (wx - cam_x) * SCALE

        # 车道虚线
        dash = 0
        wx = math.floor(cam_x / 3.0) * 3.0
        while sx(wx) < W:
            if dash % 2 == 0:
                pygame.draw.line(surface, (220, 220, 220),
                                 (sx(wx), ROAD_Y), (sx(wx) + 1.5 * SCALE, ROAD_Y), 4)
            wx += 3.0
            dash += 1

        # 里程碑（每 10 m）
        wx = math.floor(cam_x / 10.0) * 10.0
        font = load_font(16)
        while sx(wx) < W:
            pygame.draw.line(surface, (50, 50, 50),
                             (sx(wx), 315), (sx(wx), 375), 2)
            label = font.render("%dm" % int(wx), True, (255, 255, 255))
            surface.blit(label, (sx(wx) - label.get_width() / 2, 385))
            wx += 10.0

        # 停止线
        sx_stop = sx(STOP_X)
        if 0 <= sx_stop <= W:
            pygame.draw.line(surface, (255, 255, 255),
                             (sx_stop, 310), (sx_stop, 380), 6)

        # 红绿灯（杆在停止线上方，灯头面向来车）
        self._draw_traffic_light(surface, sx_stop)

        # 车辆
        self._draw_car(surface, self.model.x, self.model.v, sx)

        # HUD
        self._draw_hud(surface)

    def _draw_traffic_light(self, surface, sx_stop):
        if not (0 <= sx_stop <= W):
            return
        pole_x = sx_stop - 8
        pygame.draw.rect(surface, (60, 60, 60), (pole_x, 240, 6, 70))
        box_x = pole_x - 16
        pygame.draw.rect(surface, (40, 40, 40), (box_x, 196, 40, 110), border_radius=6)
        color = self.light.color
        for i, (name, pos, on_color, off_color) in enumerate([
                ('red', (box_x + 13, 206), (255, 40, 40), (90, 30, 30)),
                ('yellow', (box_x + 13, 246), (255, 200, 30), (90, 80, 20)),
                ('green', (box_x + 13, 286), (40, 255, 70), (30, 90, 40))]):
            pygame.draw.circle(surface,
                               on_color if color == name else off_color,
                               (pos[0], pos[1]), 11)

    def _draw_car(self, surface, x, v, sx):
        car_w = 4.4 * SCALE
        car_h = 1.9 * SCALE
        cx = sx(x) + car_w / 2
        cy = ROAD_Y
        pygame.draw.rect(surface, (220, 60, 60),
                         (cx - car_w / 2, cy - car_h / 2, car_w, car_h),
                         border_radius=8)
        # 车窗
        pygame.draw.rect(surface, (150, 200, 240),
                         (cx - car_w / 4, cy - car_h * 0.28, car_w / 2, car_h * 0.35),
                         border_radius=4)
        # 车轮
        for wx_off in (-car_w / 3, car_w / 6):
            pygame.draw.rect(surface, (20, 20, 20),
                             (cx + wx_off - 8, cy - car_h / 2 - 4, 16, 8))
            pygame.draw.rect(surface, (20, 20, 20),
                             (cx + wx_off - 8, cy + car_h / 2 - 4, 16, 8))

    def _draw_hud(self, surface):
        color = self.light.color
        color_cn = {'red': '红灯', 'yellow': '黄灯', 'green': '绿灯'}[color]
        d = STOP_X - SAFETY - self.model.x
        if d < 0:
            d = 0.0
        state = self._state_name(color)
        font = load_font(20)
        lines = [
            "灯: %s (%s)" % (color_cn, color),
            "速度: %.1f m/s (%.1f km/h)" % (self.model.v, self.model.v * 3.6),
            "距停止线(车头): %.1f m" % max(0.0, STOP_X - self.model.head()),
            "状态: %s" % state,
            "时间: %.1f s" % self.t,
        ]
        y = 12
        for line in lines:
            surf = font.render(line, True, (255, 255, 255), (0, 0, 0))
            surface.blit(surf, (12, y))
            y += 26

    def _state_name(self, color):
        if color != 'green' and self.model.v < 0.2:
            return '已停车等灯'
        if color != 'green':
            return '减速中'
        if self.model.x < STOP_X and self.model.v < V_CRUISE - 0.5:
            return '起步加速'
        return '巡航行驶'


def run_window():
    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("Traffic Light Stop - 红绿灯启停仿真")
    clock = pygame.time.Clock()
    sim = Simulation()
    font = load_font(28)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    return
                if event.key == pygame.K_SPACE:
                    sim.running = not sim.running
                if event.key == pygame.K_r:
                    sim.reset()

        if sim.running:
            sim.step()

        sim.render(screen)
        hint = font.render("空格: 开始/暂停   R: 重置   ESC: 退出",
                           True, (255, 255, 255), (0, 0, 0))
        screen.blit(hint, (W - 430, H - 34))
        pygame.display.flip()
        clock.tick(int(1.0 / DT))


def record_gif(path, duration=25.0, fps=10):
    """无头渲染并合成演示动图。"""
    from PIL import Image
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    pygame.init()
    surface = pygame.Surface((W, H))
    sim = Simulation()
    sim.running = True

    frames = []
    frame_dt = 1.0 / fps
    n_steps = max(1, int(round(frame_dt / DT)))
    total = int(duration / frame_dt)
    sim2 = sim
    for i in range(total):
        for _ in range(n_steps):
            sim2.step()
        sim2.render(surface)
        raw = pygame.image.tostring(surface, "RGB")
        img = Image.frombytes("RGB", (W, H), raw)
        img = img.resize((W * 3 // 5, H * 3 // 5), Image.LANCZOS)
        frames.append(img)

    frames[0].save(path, save_all=True, append_images=frames[1:],
                   duration=int(frame_dt * 1000), loop=0, optimize=True)
    print("saved %d frames -> %s (%.1f KB)" %
          (len(frames), path, os.path.getsize(path) / 1024.0))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--record", metavar="GIF", help="无头录制演示动图到指定文件")
    parser.add_argument("--duration", type=float, default=25.0, help="录制时长(秒)")
    args = parser.parse_args()

    if args.record:
        record_gif(args.record, args.duration)
    else:
        run_window()


if __name__ == "__main__":
    main()
