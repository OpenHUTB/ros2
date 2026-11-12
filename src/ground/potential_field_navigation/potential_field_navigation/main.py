"""pygame 渲染与主循环：障碍、目标、小车轨迹与势场导航状态。

渲染支持三种用法：
- ``python main.py``：开窗口观察；
- ``python main.py --headless --frames 400``：离屏（SDL dummy）跑固定帧数；
- ``python main.py --shots``：离屏跑到结束，并把阶段截图写到仓库 ``docs/img/``。
"""

import argparse
import math
import os

from . import config

# 中文字体候选：Windows 优先 SimHei，Linux/macOS 回退到常见 CJK 字体，
# 避免只写 "simhei" 时在非 Windows 环境渲染成缺字方块。
FONT_CANDIDATES = [
    "simhei",
    "Noto Sans CJK SC",
    "Noto Sans SC",
    "WenQuanYi Micro Hei",
    "WenQuanYi Zen Hei",
    "Source Han Sans SC",
    "PingFang SC",
    "Heiti SC",
    "Microsoft YaHei",
    "sans-serif",
]

RECORD_STEPS = (40, 200)

_REPO_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
IMG_DIR = os.path.join(_REPO_ROOT, "docs", "img")

_SHOT_NAMES = {
    40: "potential_field_navigation_01_start.png",
    200: "potential_field_navigation_02_detour.png",
}


def load_font(size=19):
    """按候选列表找中文字体，找不到时退回 pygame 默认字体。"""
    import pygame

    return pygame.font.SysFont(FONT_CANDIDATES, size)


def draw(screen, robot, font):
    import pygame

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

    line1 = "人工势场导航  步数 %d  状态 %s" % (robot.steps, robot.status)
    line2 = "距目标 %.1f px   最小障碍间隙 %.1f px" % (
        robot.distance(), robot.min_clearance)
    screen.blit(font.render(line1, True, (225, 225, 225)), (14, 10))
    screen.blit(font.render(line2, True, (172, 178, 188)), (14, 36))


def run(headless=False, frames=None, shots=False, out_dir=None):
    """推进仿真并渲染；返回本次运行的指标。

    参数:
        headless: 为 True 时用 SDL dummy 驱动离屏渲染，不开窗口；
        frames: 最多推进的帧数，None 表示一直跑到抵达或超时；
        shots: 跑到结束并把阶段截图写入 ``out_dir``（默认仓库 docs/img/）；
        out_dir: 截图输出目录。
    """
    if headless or shots:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

    import pygame

    pygame.init()
    screen = pygame.display.set_mode((config.WORLD_W, config.WORLD_H))
    pygame.display.set_caption("人工势场法导航")
    font = load_font(19)
    clock = pygame.time.Clock()
    robot = _make_robot()

    shots_dir = out_dir or IMG_DIR
    saved = []

    frame = 0
    running = True
    while running and not robot.done:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        if not running:
            break
        robot.step()
        frame += 1
        draw(screen, robot, font)
        if shots and frame in _SHOT_NAMES:
            path = _save(screen, shots_dir, _SHOT_NAMES[frame])
            saved.append(path)
        pygame.display.flip()
        if frames is not None and frame >= frames:
            break
        clock.tick(config.FPS)

    if shots and robot.done:
        path = _save(screen, shots_dir, "potential_field_navigation_03_arrived.png")
        saved.append(path)

    pygame.quit()

    for path in saved:
        print("已保存截图:", path)

    return {
        "steps": robot.steps,
        "status": robot.status,
        "distance": round(robot.distance(), 2),
        "min_clearance": round(robot.min_clearance, 2),
        "shots": saved,
    }


def _save(surface, out_dir, filename):
    import pygame

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, filename)
    pygame.image.save(surface, path)
    return path


def _make_robot():
    from .robot import Robot
    return Robot()


def main(argv=None):
    parser = argparse.ArgumentParser(description="人工势场法导航 2D 仿真")
    parser.add_argument("--headless", action="store_true",
                        help="离屏运行（SDL_VIDEODRIVER=dummy），不打开窗口")
    parser.add_argument("--frames", type=int, default=None,
                        help="最多推进的帧数，默认跑到抵达或超时")
    parser.add_argument("--shots", action="store_true",
                        help="离屏跑到结束，并把三张阶段截图写入 docs/img/")
    args = parser.parse_args(argv)

    metrics = run(headless=args.headless or args.shots,
                  frames=args.frames,
                  shots=args.shots)
    print("仿真指标:", metrics)
    return metrics


if __name__ == "__main__":
    main()
