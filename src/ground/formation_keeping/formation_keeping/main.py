"""pygame 渲染：领航者圆周轨迹、三个成员与期望队形位置。"""

import argparse
import math
import os
import sys

import pygame

from . import config


def draw(screen, team, font):
    screen.fill((22, 24, 28))

    # 领航者参考轨迹圆
    pygame.draw.circle(screen, (44, 48, 56), (int(config.CENTER[0]),
                                              int(config.CENTER[1])),
                       int(config.RADIUS), 1)

    # 期望队形位置
    for d in team.desired():
        pygame.draw.circle(screen, (110, 110, 120), (int(d[0]), int(d[1])), 6, 1)

    # 轨迹
    if len(team.leader_trail) > 1:
        pygame.draw.lines(screen, (120, 150, 200), False,
                          [(int(p[0]), int(p[1])) for p in team.leader_trail], 1)
    colors = [(0, 170, 220), (90, 200, 140)]
    for trail, c in zip(team.trails, colors):
        if len(trail) > 1:
            pygame.draw.lines(screen, c, False,
                              [(int(p[0]), int(p[1])) for p in trail], 2)

    # 领航者
    lx, ly = team.leader_pos
    pygame.draw.circle(screen, (240, 200, 80), (int(lx), int(ly)), 9)
    pygame.draw.line(screen, (255, 230, 140), (int(lx), int(ly)),
                     (int(lx + 16 * math.cos(team.leader_heading)),
                      int(ly + 16 * math.sin(team.leader_heading))), 3)

    # 跟随者
    for p, c in zip(team.followers, colors):
        pygame.draw.circle(screen, c, (int(p[0]), int(p[1])), 7)

    line1 = "多车编队保持  步数 %d  队形误差 %.2f px" % (team.step_count, team.error())
    line2 = "领航者圆周半径 %.0f px   跟随者增益 %.2f   收敛阈值 %.0f px" % (
        config.RADIUS, config.GAIN, config.TOL)
    screen.blit(font.render(line1, True, (225, 225, 225)), (14, 10))
    screen.blit(font.render(line2, True, (172, 178, 188)), (14, 36))


def _make_font():
    """字体在主循环外创建一次，绘制时复用，不每帧重建。"""
    return pygame.font.SysFont(
        ["simhei", "Noto Sans CJK SC", "WenQuanYi Micro Hei", "sans-serif"], 19)


def run(headless_frames=None):
    if headless_frames is not None:
        # 指定帧数时走无头渲染，视频与音频都用 dummy 驱动，不依赖真实设备。
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    pygame.init()
    screen = pygame.display.set_mode((config.WORLD_W, config.WORLD_H))
    pygame.display.set_caption("多车编队保持")
    clock = pygame.time.Clock()
    team = _make_team()
    font = _make_font()
    frames = 0
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        team.step()
        draw(screen, team, font)
        pygame.display.flip()
        clock.tick(config.FPS)
        frames += 1
        if headless_frames is not None and frames >= headless_frames:
            running = False
    pygame.quit()


def _make_team():
    from .team import Team
    return Team()


def main(argv=None):
    parser = argparse.ArgumentParser(description="多车编队保持 2D 仿真（pygame）")
    parser.add_argument("--frames", type=int, default=None,
                        help="只跑指定帧数后退出，并以 SDL dummy 驱动无头渲染，"
                             "用于生成文档截图")
    args = parser.parse_args(argv)
    run(args.frames)


if __name__ == "__main__":
    main()
    sys.exit(0)
