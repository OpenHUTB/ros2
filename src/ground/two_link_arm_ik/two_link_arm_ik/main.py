"""pygame 渲染：基座、两段连杆、末端轨迹与目标点。"""

import math
import sys

import pygame

from . import config


def to_screen(p, base):
    """世界坐标（y 向上）转到屏幕坐标（y 向下）。"""
    return (base[0] + p[0], base[1] - p[1])


def draw(screen, arm, base):
    screen.fill((22, 24, 28))

    # 工作空间边界
    r_max = config.L1 + config.L2
    pygame.draw.circle(screen, (44, 48, 56), base, int(r_max), 1)

    # 目标点
    for i, t in enumerate(arm.targets):
        color = (120, 120, 120)
        if i == arm.index:
            color = (235, 90, 90)
        elif t in arm.reached:
            color = (90, 200, 120)
        pygame.draw.circle(screen, color, to_screen(t, base), 6, 2)

    # 末端轨迹
    if len(arm.trail) > 1:
        pts = [to_screen(p, base) for p in arm.trail]
        pygame.draw.lines(screen, (0, 170, 220), False, pts, 2)

    elbow, tip = arm.elbow, arm.tip
    pygame.draw.line(screen, (200, 205, 215), base,
                     to_screen(elbow, base), 6)
    pygame.draw.line(screen, (240, 240, 245), to_screen(elbow, base),
                     to_screen(tip, base), 5)
    pygame.draw.circle(screen, (90, 160, 240), base, 10)
    pygame.draw.circle(screen, (255, 210, 0), to_screen(elbow, base), 7)
    pygame.draw.circle(screen, (90, 220, 140), to_screen(tip, base), 7)

    font = pygame.font.SysFont("simhei", 20)
    title = "二连杆逆运动学  目标 %d/%d  状态 %s" % (
        min(arm.index + 1, len(arm.targets)), len(arm.targets), arm.last_status)
    screen.blit(font.render(title, True, (220, 220, 220)), (14, 10))
    info = "q1 %.3f rad   q2 %.3f rad   末端误差 %.1f px" % (
        arm.q1, arm.q2, arm.error())
    screen.blit(font.render(info, True, (170, 175, 185)), (14, 36))


def run(headless_frames=None):
    pygame.init()
    screen = pygame.display.set_mode((config.WORLD_W, config.WORLD_H))
    pygame.display.set_caption("二连杆机械臂正逆运动学")
    clock = pygame.time.Clock()
    base = (170, 420)
    arm = _make_arm()
    frames = 0
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        if arm.target is not None:
            arm.step()
        draw(screen, arm, base)
        pygame.display.flip()
        clock.tick(config.FPS)
        frames += 1
        if headless_frames is not None and frames >= headless_frames:
            running = False
    pygame.quit()


def _make_arm():
    from .arm import Arm
    return Arm()


def main():
    run()


if __name__ == "__main__":
    main()
    sys.exit(0)
