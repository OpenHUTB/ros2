"""Pygame 可视化入口：运行 Bug2 仿真并渲染。"""

import os
import sys
import pygame

from . import config
from .bug2 import Bug2, State


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
    pygame.display.set_caption("bug2_wall_following Bug2 反应式避障")
    clock = pygame.time.Clock()
    f20 = _font(20)
    sim = Bug2()
    running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
        sim.step()

        screen.fill((30, 30, 30))
        # Lm 连线
        pygame.draw.line(screen, (80, 80, 80), config.START, config.GOAL, 1)
        # 障碍
        cx, cy, r = config.OBSTACLE
        pygame.draw.circle(screen, (120, 120, 120), (int(cx), int(cy)), int(r))
        # 起点 / 目标
        pygame.draw.circle(screen, (0, 200, 0), (int(config.START[0]), int(config.START[1])), 8)
        pygame.draw.circle(screen, (220, 50, 50), (int(config.GOAL[0]), int(config.GOAL[1])), 8)
        # 轨迹
        for i in range(1, len(sim.trail)):
            pygame.draw.line(screen, (0, 200, 200),
                             sim.trail[i - 1], sim.trail[i], 2)
        # hit point
        if sim.hit_point:
            pygame.draw.circle(screen, (255, 200, 0),
                               (int(sim.hit_point[0]), int(sim.hit_point[1])), 5)
        # 机器人
        col = {"goal_seeking": (0, 255, 0),
               "wall_following": (255, 165, 0),
               "reached": (255, 255, 0)}[sim.state.value]
        pygame.draw.circle(screen, col,
                           (int(sim.pos[0]), int(sim.pos[1])), config.ROBOT_R)
        # HUD
        pygame.draw.rect(screen, (30, 30, 30), (0, 0, 500, 60))
        txt = f"状态: {sim.state.value} | 步数: {len(sim.trail)}"
        screen.blit(f20.render(txt, True, (220, 220, 220)), (10, 10))
        pygame.display.flip()
        clock.tick(config.FPS)
    pygame.quit()


if __name__ == "__main__":
    main()
