"""Pygame 渲染：正弦赛道、小车循线、横向误差实时曲线。"""

import math
import os

import pygame

from . import config
from .car import Car
from .track import build_track


def _font(size):
    for p in ["C:/Windows/Fonts/simhei.ttf",
              "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc"]:
        try:
            if os.path.exists(p):
                return pygame.font.Font(p, size)
        except pygame.error:
            pass
    return pygame.font.Font(None, size)


def main():
    pygame.init()
    screen = pygame.display.set_mode((config.WORLD_W, config.WORLD_H))
    pygame.display.set_caption("line_following_pid 循线小车 PID 横向控制")
    clock = pygame.time.Clock()
    f18 = _font(18)
    track = build_track()
    car = Car(track)
    running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False

        screen.fill((25, 25, 25))
        # 赛道中心线
        for i in range(1, len(track)):
            pygame.draw.line(screen, (200, 200, 200), track[i - 1], track[i], 3)

        if not car.finished():
            car.step()

        # 轨迹
        for i in range(1, len(car.trail)):
            pygame.draw.line(screen, (0, 180, 220), car.trail[i - 1], car.trail[i], 2)
        # 车辆
        pygame.draw.circle(screen, (0, 220, 80),
                           (int(car.x), int(car.y)), config.ROBOT_R)
        hx = car.x + math.cos(car.heading) * 14
        hy = car.y + math.sin(car.heading) * 14
        pygame.draw.line(screen, (255, 210, 0), (car.x, car.y), (hx, hy), 2)

        # HUD
        pygame.draw.rect(screen, (25, 25, 25), (0, 0, 700, 72))
        screen.blit(f18.render(f"横向误差: {car.err:+.2f} px", True, (220, 220, 220)), (10, 8))
        screen.blit(f18.render(f"PID: Kp={config.KP} Ki={config.KI} Kd={config.KD}",
                               True, (180, 180, 180)), (10, 32))
        pygame.display.flip()
        clock.tick(config.FPS)
    pygame.quit()


if __name__ == "__main__":
    main()
