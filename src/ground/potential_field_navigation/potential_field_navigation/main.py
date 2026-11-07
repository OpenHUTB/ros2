"""pygame 渲染：障碍、目标、小车轨迹与势场导航状态。"""

import math
import sys

import pygame

from . import config


def draw(screen, robot):
    screen.fill((22, 24, 28))

    for cx, cy, r in robot.obstacles:
        pygame.draw.circle(screen, (58, 62, 72), (int(cx), int(cy)), int(r))
        pygame.draw.circle(screen, (96, 102, 116), (int(cx), int(cy)), int(r), 1)
        pygame.draw.circle(screen, (70, 120, 170),
                           (int(cx), int(cy)), int(r) + 1, 1)

    # 目标
    gx, gy = robot.goal
    pygame.draw.circle(screen, (90, 200, 120), (int(gx), int(gy)), 9, 2)

    # 轨迹
    if len(robot.trail) > 1:
        pts = [(int(p[0]), int(p[1])) for p in robot.trail]
        pygame.draw.lines(screen, (0, 170, 220), False, pts, 2)

    # 小车
    bx, by = robot.p
    hx = bx + 14 * math.cos(robot.heading)
    hy = by + 14 * math.sin(robot.heading)
    pygame.draw.circle(screen, (235, 140, 60), (int(bx), int(by)), 8)
    pygame.draw.line(screen, (255, 210, 120), (int(bx), int(by)), (int(hx), int(hy)), 3)

    font = pygame.font.SysFont("simhei", 19)
    line1 = "人工势场导航  步数 %d  状态 %s" % (robot.steps, robot.status)
    line2 = "距目标 %.1f px   最小障碍间隙 %.1f px" % (
        robot.distance(), robot.min_clearance)
    screen.blit(font.render(line1, True, (225, 225, 225)), (14, 10))
    screen.blit(font.render(line2, True, (172, 178, 188)), (14, 36))


def run(headless_frames=None):
    pygame.init()
    screen = pygame.display.set_mode((config.WORLD_W, config.WORLD_H))
    pygame.display.set_caption("人工势场法导航")
    clock = pygame.time.Clock()
    robot = _make_robot()
    frames = 0
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        if not robot.done:
            robot.step()
        draw(screen, robot)
        pygame.display.flip()
        clock.tick(config.FPS)
        frames += 1
        if headless_frames is not None and frames >= headless_frames:
            running = False
    pygame.quit()


def _make_robot():
    from .robot import Robot
    return Robot()


def main():
    run()


if __name__ == "__main__":
    main()
    sys.exit(0)
