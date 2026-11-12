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
    from .apf import (APFParams, EscapeTracker, Obstacle, plan,
                      attractive_potential, repulsive_potential,
                      total_force, distance, obstacle_distance)
    from . import config as C
except ImportError:  # pragma: no cover - 根目录 main.py 直接执行
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from apf import (APFParams, EscapeTracker, Obstacle, plan,
                     attractive_potential, repulsive_potential,
                     total_force, distance, obstacle_distance)
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
def _repulsive_reference(params):
    """斥力势的对数色标参考值：d_thresh/10 处的斥力势。

    斥力在障碍物表面趋于无穷，直接用全图最大势归一化会只剩一小圈死红、
    失去梯度；取影响范围 1/10 处的势能作参考，再按 log1p 压缩，
    既能看清障碍物周围的斥力环，又不会被无穷远处的数值支配。
    """
    d = params.d_thresh_rep / 10.0
    return 0.5 * params.k_rep * (1.0 / d - 1.0 / params.d_thresh_rep) ** 2


def potential_shade(u_att, u_rep, att_ref, rep_ref):
    """把引力势与斥力势分别归一化后取较大者，返回 [0, 1] 的着色比例。

    引力势（远场 U ≈ 10⁴ 量级）比斥力势（障碍物边缘外 U ≈ 10² 量级）
    大一到两个数量级，两者相加后线性归一化必然让 95% 以上像素饱和成同一
    颜色。这里按各自量级归一化：引力开方、斥力对数，取较大值着色，
    于是「远离目标越来越红」与「障碍物周围的红环」能同时看见。
    """
    att = 0.0 if att_ref <= 0.0 else math.sqrt(max(u_att, 0.0) / att_ref)
    denom = math.log1p(rep_ref)
    if math.isinf(u_rep):
        rep = 1.0
    elif denom <= 1e-12:
        rep = 0.0
    else:
        rep = math.log1p(max(u_rep, 0.0)) / denom
    return min(1.0, max(0.0, max(att, rep)))


def draw_potential_field(surface, goal, obstacles, params, step=20):
    """势场热图背景：按归一化后的势值着色（暗蓝低、亮红高）。降采样以加速。

    引力/斥力分两个量级，见 ``potential_shade``；引力参考值取画布四角的
    最大势，保证不同画布尺寸与参数下颜色都能铺满整个色标。
    """
    w, h = surface.get_size()
    corners = ((0.0, 0.0), (float(w), 0.0), (0.0, float(h)), (float(w), float(h)))
    att_ref = max(attractive_potential(c, goal, params.k_att, params.d_thresh_att)
                  for c in corners)
    rep_ref = _repulsive_reference(params)
    low, high = C.POTENTIAL_LOW, C.POTENTIAL_HIGH
    for y in range(0, h, step):
        for x in range(0, w, step):
            pos = (float(x), float(y))
            ratio = potential_shade(
                attractive_potential(pos, goal, params.k_att, params.d_thresh_att),
                repulsive_potential(pos, obstacles, params.k_rep, params.d_thresh_rep),
                att_ref, rep_ref)
            color = (int(low[0] + (high[0] - low[0]) * ratio),
                     int(low[1] + (high[1] - low[1]) * ratio),
                     int(low[2] + (high[2] - low[2]) * ratio))
            pygame.draw.rect(surface, color, (x, y, step, step))


def make_background(goal, obstacles, params, step=6):
    """预渲染静态势场热图，供录制/交互逐帧复用。

    势场只与 goal/obstacles/params 有关，与机器人位置无关；每帧重算一遍
    既慢又没必要。预渲染一次后每帧只 blit，顺带可以把采样步长从 20 px
    降到 6 px，热图不再是明显的大色块。
    """
    bg = pygame.Surface((C.WIDTH, C.HEIGHT))
    draw_potential_field(bg, goal, obstacles, params, step=step)
    return bg


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
    """HUD 固定在右上角，并垫一层半透明底，避免文字压在路径/起点标记上看不清。"""
    lines = [
        f"APF Path Planning  |  status: {status}",
        f"steps: {n_steps}  |  dist_to_goal: {dist_goal:.1f} px",
        f"|force|: {force_mag:.2f}  |  k_att={params.k_att}  k_rep={params.k_rep}",
        f"d_thresh_att={params.d_thresh_att}  d_thresh_rep={params.d_thresh_rep}",
        "ESC: quit",
    ]
    rendered = [font.render(line, True, C.HUD) for line in lines]
    pad, line_h = 8, 18
    panel_w = max(t.get_width() for t in rendered) + 2 * pad
    panel_h = line_h * len(rendered) + 2 * pad - 4
    x0 = max(8, surface.get_width() - panel_w - 8)
    panel = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)
    panel.fill((12, 15, 20, 185))
    surface.blit(panel, (x0, 6))
    for i, txt in enumerate(rendered):
        surface.blit(txt, (x0 + pad, 6 + pad + i * line_h))


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
    background = make_background(goal, obstacles, params)

    frames = []
    for i in range(n_frames):
        progress = (i + 1) / n_frames * len(path)
        surface.blit(background, (0, 0))
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
    tracker = EscapeTracker(goal, obstacles, params, rng)
    tracker.reset(pos)
    status = "running"
    background = make_background(goal, obstacles, params)

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
            elif any(obstacle_distance(pos, obs) <= 0.0 for obs in obstacles):
                status = "trapped"
            else:
                # 与 plan() 共用同一套停滞检测/逃逸逻辑，避免两处行为不一致
                pos, _ = tracker.advance(pos)
                path.append(pos)
                if len(path) >= params.max_iter:
                    status = "timeout"

        f = total_force(pos, goal, obstacles, params)
        screen.blit(background, (0, 0))
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