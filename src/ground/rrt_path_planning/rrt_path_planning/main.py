# -*- coding: utf-8 -*-
"""RRT 路径规划 pygame 可视化与演示。

运行方式：
    python main.py              # 交互模式，实时显示树扩展
    python main.py --record     # 录制模式，离屏渲染帧序列并合成 gif
    python main.py --no-display # 无显示设备环境（VM/CI）下离屏运行

在无显示设备的虚拟机中，设置环境变量 SDL_VIDEODRIVER=dummy 即可，
本模块会自动检测并切到离屏渲染，仍可生成演示 gif。
"""
from __future__ import annotations
import os
import sys
import math
import argparse

import pygame

from . import config as C
from .map import ObstacleMap
from .rrt import RRT, Node


def build_map() -> ObstacleMap:
    return ObstacleMap(C.OBSTACLES, C.WIDTH, C.HEIGHT)


def build_planner(seed: int | None = None) -> RRT:
    return RRT(
        obs_map=build_map(),
        start=C.START,
        goal=C.GOAL,
        step_size=C.STEP_SIZE,
        goal_radius=C.GOAL_RADIUS,
        goal_sample_rate=C.GOAL_SAMPLE_RATE,
        seed=C.SEED if seed is None else seed,
    )


def draw_scene(surface, planner: RRT, path=None, info: dict | None = None):
    """渲染一帧：背景、障碍、树、起终点、路径、HUD。"""
    surface.fill(C.BG_COLOR)
    # 网格
    for gx in range(0, C.WIDTH, 30):
        pygame.draw.line(surface, C.GRID_COLOR, (gx, 0), (gx, C.HEIGHT))
    for gy in range(0, C.HEIGHT, 30):
        pygame.draw.line(surface, C.GRID_COLOR, (0, gy), (C.WIDTH, gy))
    # 障碍
    for (ox, oy, ow, oh) in planner.map.obstacles:
        pygame.draw.rect(surface, C.OBSTACLE_COLOR, (ox, oy, ow, oh))
        pygame.draw.rect(surface, C.OBSTACLE_EDGE, (ox, oy, ow, oh), 2)
    # 树（父子连线）
    for n in planner.tree:
        if n.parent is not None:
            pygame.draw.line(surface, C.TREE_COLOR,
                             (n.parent.x, n.parent.y), (n.x, n.y), 1)
    # 树节点（小点）
    for n in planner.tree:
        pygame.draw.circle(surface, C.TREE_COLOR, (int(n.x), int(n.y)), 2)
    # 路径
    if path and len(path) >= 2:
        for i in range(len(path) - 1):
            pygame.draw.line(surface, C.PATH_COLOR,
                             (path[i].x, path[i].y),
                             (path[i + 1].x, path[i + 1].y), 3)
        for n in path:
            pygame.draw.circle(surface, C.PATH_COLOR, (int(n.x), int(n.y)), 4)
    # 起终点
    pygame.draw.circle(surface, C.START_COLOR,
                       (int(planner.start.x), int(planner.start.y)), 8)
    pygame.draw.circle(surface, C.START_COLOR,
                       (int(planner.start.x), int(planner.start.y)), 8, 2)
    pygame.draw.circle(surface, C.GOAL_COLOR,
                       (int(planner.goal.x), int(planner.goal.y)), int(C.GOAL_RADIUS), 2)
    pygame.draw.circle(surface, C.GOAL_COLOR,
                       (int(planner.goal.x), int(planner.goal.y)), 6)
    # HUD
    info = info or {}
    lines = [
        f"iter:   {info.get('iter', 0)}",
        f"nodes:  {info.get('nodes', len(planner.tree))}",
        f"status: {info.get('status', 'running')}",
    ]
    if path:
        lines.append(f"path_len: {info.get('path_len', 0):.1f} px")
        lines.append(f"path_pts: {len(path)}")
    try:
        font = pygame.font.SysFont("dejavusans,arial", 16)
    except Exception:
        font = pygame.font.Font(None, 18)
    y = 8
    for ln in lines:
        surf = font.render(ln, True, C.TEXT_COLOR)
        surface.blit(surf, (10, y))
        y += 20


def run_planner_stepwise(planner: RRT):
    """生成器：逐步执行 RRT，每步 yield (iter_index, reached, path)。"""
    planner.tree = [planner.start]
    planner.node_reached_goal = None
    for i in range(C.MAX_ITER):
        sx, sy = planner.sample()
        near = planner.nearest(planner.tree, sx, sy)
        new_node = planner.steer(near, sx, sy)
        if planner.map.segment_collides(near.x, near.y, new_node.x, new_node.y):
            continue
        if planner.map.point_in_obstacle(new_node.x, new_node.y):
            continue
        planner.tree.append(new_node)
        if new_node.dist_to(planner.goal.x, planner.goal.y) <= C.GOAL_RADIUS:
            planner.node_reached_goal = new_node
            yield i, True, planner.retrace(new_node)
            return
        yield i, False, None
    yield C.MAX_ITER - 1, False, None


def record_gif(out_path: str, planner: RRT):
    """离屏渲染 RRT 过程并合成 gif。"""
    frames_dir = out_path + ".frames"
    os.makedirs(frames_dir, exist_ok=True)
    surface = pygame.Surface((C.WIDTH, C.HEIGHT))
    frame_idx = 0
    final_path = None
    final_iter = 0

    def save_frame(info):
        nonlocal frame_idx
        draw_scene(surface, planner, path=info.get("path"), info=info)
        pygame.image.save(surface, os.path.join(frames_dir, f"f_{frame_idx:05d}.png"))
        frame_idx += 1

    for it, reached, path in run_planner_stepwise(planner):
        if it % C.RECORD_EVERY == 0:
            save_frame({"iter": it, "nodes": len(planner.tree),
                        "status": "reached" if reached else "expanding"})
        if reached:
            final_path = path
            final_iter = it
            break

    status = "reached" if final_path else "failed"
    # 尾部帧：展示最终路径
    for _ in range(C.RECORD_TAIL):
        save_frame({"iter": final_iter, "nodes": len(planner.tree),
                    "status": status, "path": final_path,
                    "path_len": planner.path_length(final_path) if final_path else 0.0})

    # 用 PIL 合成 gif
    from PIL import Image
    files = sorted(f for f in os.listdir(frames_dir) if f.endswith(".png"))
    images = [Image.open(os.path.join(frames_dir, f)).convert("P") for f in files]
    if images:
        # 缩小尺寸 + 限制帧率以控制体积
        scale = 0.55
        if scale != 1.0:
            images = [im.resize((int(C.WIDTH * scale), int(C.HEIGHT * scale)),
                                Image.LANCZOS) for im in images]
        images[0].save(out_path, save_all=True, append_images=images[1:],
                       duration=60, loop=0, disposal=2, optimize=True)
    # 清理帧目录
    for f in files:
        os.remove(os.path.join(frames_dir, f))
    os.rmdir(frames_dir)
    return final_path, final_iter, len(planner.tree)


def main():
    parser = argparse.ArgumentParser(description="RRT 路径规划仿真")
    parser.add_argument("--record", action="store_true",
                        help="录制模式：离屏渲染并合成演示 gif")
    parser.add_argument("--no-display", action="store_true",
                        help="无显示设备环境（VM/CI）下离屏运行")
    parser.add_argument("--output", default="rrt_demo.gif",
                        help="录制模式下输出 gif 路径")
    parser.add_argument("--seed", type=int, default=None,
                        help="随机种子（默认用 config.py 里的 SEED）")
    parser.add_argument("--max-iter", type=int, default=None,
                        help="最大迭代次数（默认用 config.py 里的 MAX_ITER）")
    args = parser.parse_args()

    if args.seed is not None:
        C.SEED = args.seed
    if args.max_iter is not None:
        C.MAX_ITER = args.max_iter

    headless = args.no_display or args.record or "SDL_VIDEODRIVER" in os.environ
    pygame.init()
    if headless:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    flags = pygame.SCALED if not headless else 0
    try:
        screen = pygame.display.set_mode((C.WIDTH, C.HEIGHT), flags)
    except pygame.error:
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        screen = pygame.display.set_mode((C.WIDTH, C.HEIGHT))
    pygame.display.set_caption("RRT Path Planning")
    clock = pygame.time.Clock()
    planner = build_planner(args.seed)

    if args.record or headless:
        # 录制 / 无头模式
        out = os.path.abspath(args.output)
        path, it, nodes = record_gif(out, planner)
        if path:
            print(f"[RRT] 规划成功: 迭代 {it + 1}, 树节点 {nodes}, "
                  f"路径 {len(path)} 个点, 长度 {planner.path_length(path):.1f} px")
            print(f"[RRT] 演示 gif 已保存: {out}")
        else:
            print(f"[RRT] 规划失败: 达到最大迭代 {C.MAX_ITER}, 树节点 {nodes}")
            print(f"[RRT] 演示 gif（未找到路径）已保存: {out}")
        pygame.quit()
        return

    # 交互模式：实时显示
    gen = run_planner_stepwise(planner)
    final_path = None
    final_iter = 0
    done = False
    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit()
                return
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                pygame.quit()
                return
        if not done:
            try:
                it, reached, path = next(gen)
            except StopIteration:
                done = True
                it, reached, path = final_iter, bool(final_path), final_path
            if reached:
                final_path = path
                final_iter = it
                done = True
        info = {"iter": it if not done else final_iter,
                "nodes": len(planner.tree),
                "status": "reached" if final_path else ("failed" if done else "expanding"),
                "path": final_path,
                "path_len": planner.path_length(final_path) if final_path else 0.0}
        draw_scene(screen, planner, path=final_path, info=info)
        pygame.display.flip()
        clock.tick(C.FPS)


if __name__ == "__main__":
    main()
