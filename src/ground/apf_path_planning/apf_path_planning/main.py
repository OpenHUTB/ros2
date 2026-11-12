# -*- coding: utf-8 -*-
"""人工势场法（APF）路径规划 pygame 可视化与演示 gif 录制。

支持两种模式：
  - 交互模式（默认）：实时迭代、显示当前合力箭头、键盘 ESC 退出
  - 录制模式（--record）：预计算完整路径，逐帧播放路径生长动画，保存 gif
"""

from __future__ import annotations
import argparse
import math
import os
import random
import sys

try:
    import pygame
except ImportError:  # pragma: no cover
    pygame = None

try:
    from .apf import (APFParams, Obstacle, plan, step, total_force,
                      distance, obstacle_distance, potential_at)
    from . import config as C
except ImportError:  # pragma: no cover - 根目录 main.py 直接执行
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from apf import (APFParams, Obstacle, plan, step, total_force,
                    distance, obstacle_distance, potential_at)
    import config as C


# ---------------------------------------------------------------------------
# 场景构建
# ---------------------------------------------------------------------------
def _font(size: int, bold: bool = False):
    """创建字体，SysFont 不可用时回退到 pygame 内置字体（Windows/Py3.12 兜底）。"""
    try:
        return pygame.font.SysFont("dejavusans,arial", size, bold=bold)
    except Exception:
        return pygame.font.Font(None, size + 4)


def build_scenario(args) -> tuple:
    start = tuple(args.start) if args.start else C.START_POSE
    goal = tuple(args.end) if args.end else C.GOAL_POSE
    obstacles = [Obstacle(x, y, r) for x, y, r in C.OBSTACLES]
    params = APFParams(
        k_att=C.APF_K_ATT, k_rep=C.APF_K_REP,
        d_thresh_att=C.APF_D_THRESH_ATT, d_thresh_rep=C.APF_D_THRESH_REP,
        step_size=C.APF_STEP_SIZE, max_iter=C.APF_MAX_ITER,
        goal_tol=C.APF_GOAL_TOL,
        escape_jitter=C.APF_ESCAPE_JITTER,
        escape_patience=C.APF_ESCAPE_PATIENCE,
    )
    if args.radius:
        params.d_thresh_rep = float(args.radius)
    return start, goal, obstacles, params


# ---------------------------------------------------------------------------
# 绘制
# ---------------------------------------------------------------------------
def draw_potential_field(surface, goal, obstacles, params, step=20):
    """势场热图背景：低势能暗蓝、高势能亮红。降采样以加速。"""
    w, h = surface.get_size()
    for y in range(0, h, step):
        for x in range(0, w, step):
            u = potential_at((float(x), float(y)), goal, obstacles, params)
            if math.isinf(u):
                ratio = 1.0
            else:
                ratio = min(1.0, u / 5000.0)
            r = int(C.POTENTIAL_LOW[0] + (C.POTENTIAL_HIGH[0] - C.POTENTIAL_LOW[0]) * ratio)
            g = int(C.POTENTIAL_LOW[1] + (C.POTENTIAL_HIGH[1] - C.POTENTIAL_LOW[1]) * ratio)
            b = int(C.POTENTIAL_LOW[2] + (C.POTENTIAL_HIGH[2] - C.POTENTIAL_LOW[2]) * ratio)
            pygame.draw.rect(surface, (r, g, b), (x, y, step, step))


def draw_obstacles(surface, obstacles):
    for obs in obstacles:
        pygame.draw.circle(surface, C.OBSTACLE, (int(obs.x), int(obs.y)), int(obs.radius))
        pygame.draw.circle(surface, C.OBSTACLE_EDGE, (int(obs.x), int(obs.y)), int(obs.radius), 2)


def draw_pose_marker(surface, pos, color, label=None, font=None):
    px, py = int(pos[0]), int(pos[1])
    pygame.draw.circle(surface, color, (px, py), 8)
    pygame.draw.circle(surface, (255, 255, 255), (px, py), 8, 2)
    if label and font:
        txt = font.render(label, True, color)
        surface.blit(txt, (px + 12, py - 8))


def draw_path(surface, path, progress):
    """绘制路径前 progress 个点（progress 可为浮点，末段渐显）。"""
    if len(path) < 2:
        return
    n = min(len(path), int(progress) + 1)
    if n < 2:
        return
    pts = [(int(p[0]), int(p[1])) for p in path[:n]]
    pygame.draw.lines(surface, C.PATH, False, pts, 2)
    # 当前末端高亮
    pygame.draw.circle(surface, (255, 255, 255), pts[-1], 5)


def draw_force_arrow(surface, pos, force, color=C.FORCE, scale=3.0):
    fx, fy = force
    mag = math.hypot(fx, fy)
    if mag < 1e-6:
        return
    ex = pos[0] + fx / mag * scale * 10
    ey = pos[1] + fy / mag * scale * 10
    pygame.draw.line(surface, color, (int(pos[0]), int(pos[1])), (int(ex), int(ey)), 2)
    # 箭头
    ang = math.atan2(ey - pos[1], ex - pos[0])
    for da in (2.6, -2.6):
        pygame.draw.line(surface, color, (int(ex), int(ey)),
                        (int(ex - 6 * math.cos(ang + da)),
                         int(ey - 6 * math.sin(ang + da))), 2)


def draw_hud(surface, font, status, n_steps, dist_goal, force_mag, params):
    lines = [
        f"APF Path Planning  |  status: {status}",
        f"steps: {n_steps}  |  dist_to_goal: {dist_goal:.1f} px",
        f"|force|: {force_mag:.2f}  |  k_att={params.k_att}  k_rep={params.k_rep}",
        f"d_thresh_att={params.d_thresh_att}  d_thresh_rep={params.d_thresh_rep}",
        "ESC: quit",
    ]
    y = 10
    for line in lines:
        txt = font.render(line, True, C.HUD)
        surface.blit(txt, (12, y))
        y += 18


# ---------------------------------------------------------------------------
# 录制 gif
# ---------------------------------------------------------------------------
def _to_image(surface):
    """把 pygame Surface 转成 PIL.Image。

    不用 pygame.surfarray：那个子模块依赖 numpy，会额外拉一个重依赖，
    而本模块的算法层本来就不需要 numpy。这里走 image.tostring + frombytes。
    """
    from PIL import Image
    raw = pygame.image.tostring(surface, "RGB")
    return Image.frombytes("RGB", surface.get_size(), raw)


def record_demo(output, start, goal, obstacles, params):
    """预计算路径，逐帧播放路径生长动画，保存 gif。"""
    pygame.init()
    surface = pygame.Surface((C.WIDTH, C.HEIGHT))
    font = _font(14)

    # 预计算完整路径
    path, status = plan(start, goal, obstacles, params)
    n_frames = C.RECORD_FRAMES

    frames = []
    for i in range(n_frames):
        progress = (i + 1) / n_frames * len(path)
        surface.fill(C.BG)
        draw_potential_field(surface, goal, obstacles, params, step=20)
        # 网格
        for gx in range(0, C.WIDTH, 40):
            pygame.draw.line(surface, C.GRID, (gx, 0), (gx, C.HEIGHT))
        for gy in range(0, C.HEIGHT, 40):
            pygame.draw.line(surface, C.GRID, (0, gy), (C.WIDTH, gy))
        draw_obstacles(surface, obstacles)
        draw_path(surface, path, progress)
        cur = path[min(len(path) - 1, int(progress))]
        f = total_force(cur, goal, obstacles, params)
        draw_force_arrow(surface, cur, f)
        draw_pose_marker(surface, start, C.START, "S", font)
        draw_pose_marker(surface, goal, C.GOAL, "G", font)
        draw_hud(surface, font, status, len(path), distance(cur, goal),
                 math.hypot(*f), params)
        frames.append(_to_image(surface))

    # 保存 gif
    imgs = frames
    imgs[0].save(output, save_all=True, append_images=imgs[1:],
                 duration=1000 // C.RECORD_FPS, loop=0, optimize=True)
    print(f"[APF] gif saved: {output} ({len(frames)} frames)")
    print(f"[APF] status: {status}, steps: {len(path)}, "
          f"final dist: {distance(path[-1], goal):.1f} px")


# ---------------------------------------------------------------------------
# 交互模式
# ---------------------------------------------------------------------------
def interactive(start, goal, obstacles, params):
    pygame.init()
    screen = pygame.display.set_mode((C.WIDTH, C.HEIGHT))
    pygame.display.set_caption("APF Path Planning")
    clock = pygame.time.Clock()
    font = _font(14)

    rng = random.Random(0)
    pos = (float(start[0]), float(start[1]))
    path = [pos]
    no_progress = 0
    last_dist = distance(pos, goal)
    status = "running"

    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                running = False

        if status == "running":
            if distance(pos, goal) <= params.goal_tol:
                status = "reached"
            else:
                f = total_force(pos, goal, obstacles, params)
                fmag = math.hypot(*f)
                if fmag < 1e-6:
                    no_progress += 1
                new_pos, _ = step_iter(pos, goal, obstacles, params, rng)
                new_dist = distance(new_pos, goal)
                if new_dist >= last_dist - 1e-3:
                    no_progress += 1
                else:
                    no_progress = 0
                if no_progress > params.escape_patience:
                    new_pos = (pos[0] + rng.uniform(-1, 1) * params.escape_jitter,
                               pos[1] + rng.uniform(-1, 1) * params.escape_jitter)
                    no_progress = 0
                last_dist = distance(new_pos, goal)
                path.append(new_pos)
                pos = new_pos
                if len(path) >= params.max_iter:
                    status = "timeout"

        f = total_force(pos, goal, obstacles, params)
        screen.fill(C.BG)
        draw_potential_field(screen, goal, obstacles, params, step=20)
        for gx in range(0, C.WIDTH, 40):
            pygame.draw.line(screen, C.GRID, (gx, 0), (gx, C.HEIGHT))
        for gy in range(0, C.HEIGHT, 40):
            pygame.draw.line(screen, C.GRID, (0, gy), (C.WIDTH, gy))
        draw_obstacles(screen, obstacles)
        pygame.draw.lines(screen, C.PATH, False,
                         [(int(p[0]), int(p[1])) for p in path], 2)
        draw_force_arrow(screen, pos, f)
        draw_pose_marker(screen, start, C.START, "S", font)
        draw_pose_marker(screen, goal, C.GOAL, "G", font)
        draw_hud(screen, font, status, len(path), distance(pos, goal),
                 math.hypot(*f), params)
        pygame.display.flip()
        clock.tick(C.FPS)

    pygame.quit()


def step_iter(pos, goal, obstacles, params, rng):
    """交互模式单步（含 escape 由调用方处理）。"""
    return step(pos, goal, obstacles, params)


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------
def main():
    if pygame is None:
        print("错误：未安装 pygame。请 pip install pygame", file=sys.stderr)
        sys.exit(1)

    p = argparse.ArgumentParser(description="人工势场法（APF）路径规划可视化")
    p.add_argument("--record", action="store_true", help="录制模式：离屏渲染并合成演示 gif")
    p.add_argument("--no-display", action="store_true", help="无显示设备环境（VM/CI）下离屏运行")
    p.add_argument("--output", default="apf_demo.gif", help="录制模式下输出 gif 路径")
    p.add_argument("--start", nargs=3, type=float, metavar=("X", "Y", "Z"),
                   help="起点 (x, y, yaw) —— Z 不使用，仅为与同构模块一致")
    p.add_argument("--end", nargs=3, type=float, metavar=("X", "Y", "Z"),
                   help="终点 (x, y, yaw)")
    p.add_argument("--radius", type=float, help="斥力影响阈值（覆盖默认）")
    p.add_argument("--mode", choices=("interactive", "record"), default=None,
                   help="运行模式；ROS 2 launch 用这个参数驱动")
    p.add_argument("--params", default="",
                   help="params.yaml 路径；留空则按 环境变量/安装空间/源码树 顺序自动查找")
    args = p.parse_args()

    used = C.load_params(args.params or None)
    if used:
        print(f"[APF] 参数文件: {used}", flush=True)
    else:
        print("[APF] 未找到 params.yaml，使用内置默认值", flush=True)

    record = args.record or args.no_display or args.mode == "record"
    if args.mode == "interactive":
        record = False

    if record:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

    start, goal, obstacles, params = build_scenario(args)

    if record:
        record_demo(args.output, start, goal, obstacles, params)
    else:
        interactive(start, goal, obstacles, params)


if __name__ == "__main__":
    main()