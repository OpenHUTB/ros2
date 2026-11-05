"""键盘遥操作（课程任务 1）：W/S 前后速度，A/D 偏航，空格悬停，R 重开。

窗口同时显示三项内容，方便对照：

* 追尾视角（chase_cam）——飞机在场景中的位置；
* 前视深度相机——网络实际看到的输入；
* 网络输出的三个扇区通行度与当前指令。

没有显示器时可用 `--script` 走无窗口脚本化测试（见 main.py）。
"""

from __future__ import annotations

import numpy as np

from .env import D_MAX, DepthUavEnv
from .policy import PolicyConfig, act


class Command:
    """把按键状态翻译成 (vx, yaw_rate)。"""

    def __init__(self, v_step: float = 0.4, w_step: float = 0.8):
        self.vx = 0.0
        self.yaw_rate = 0.0
        self.v_step = v_step
        self.w_step = w_step

    def on_key(self, key: str) -> str | None:
        """返回 'reset' 表示需要重置环境，其余返回 None。"""
        if key == "w":
            self.vx = min(self.vx + self.v_step, 2.4)
        elif key == "s":
            self.vx = max(self.vx - self.v_step, -1.2)
        elif key == "a":
            self.yaw_rate = min(self.yaw_rate + self.w_step, 1.5)
        elif key == "d":
            self.yaw_rate = max(self.yaw_rate - self.w_step, -1.5)
        elif key == " ":
            self.vx, self.yaw_rate = 0.0, 0.0
        elif key == "r":
            self.vx, self.yaw_rate = 0.0, 0.0
            return "reset"
        return None


def run_headless(env: DepthUavEnv, net, keys: str, cfg: PolicyConfig | None = None) -> list:
    """按给定的按键序列驱动（用于自动测试，不需要窗口）。"""
    cfg = cfg or PolicyConfig()
    cmd = Command()
    trace = []
    for ch in keys:
        if cmd.on_key(ch) == "reset":
            env.reset()
        depth = env.depth()
        vx, yaw_rate, c = act(net, depth, cfg, yaw=env.state().yaw)
        # 键盘优先：有按键时用键盘指令，否则用网络建议（半自动模式）
        v_cmd = cmd.vx if abs(cmd.vx) > 1e-6 else vx
        w_cmd = cmd.yaw_rate
        state = env.step(v_cmd, w_cmd)
        trace.append((state.pos.copy(), float(v_cmd), float(w_cmd), c.copy()))
    return trace


def run_window(env: DepthUavEnv, net, cfg: PolicyConfig | None = None) -> int:
    """pygame 窗口遥操作，按 ESC 或关闭窗口退出。"""
    import pygame

    cfg = cfg or PolicyConfig()
    pygame.init()
    pygame.font.init()
    screen = pygame.display.set_mode((760, 470))
    pygame.display.set_caption("uav_mujoco_depth_avoidance — 键盘遥操作 (W/S 速度, A/D 偏航)")
    font = pygame.font.SysFont("consolas", 14)
    clock = pygame.time.Clock()
    cmd = Command()
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key in (pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_d,
                                   pygame.K_SPACE, pygame.K_r):
                    name = {pygame.K_w: "w", pygame.K_s: "s", pygame.K_a: "a",
                            pygame.K_d: "d", pygame.K_SPACE: " ", pygame.K_r: "r"}[event.key]
                    if cmd.on_key(name) == "reset":
                        env.reset()

        depth = env.depth()
        state = env.state()
        _, _, c = act(net, depth, cfg, yaw=state.yaw)
        env.step(cmd.vx, cmd.yaw_rate)

        chase = env.rgb("chase_cam")
        dep = (np.clip(depth / D_MAX, 0, 1) * 255).astype(np.uint8)
        screen.fill((18, 20, 26))
        chase_s = pygame.surfarray.make_surface(np.transpose(chase, (1, 0, 2)))
        dep_rgb = np.repeat(dep[:, :, None], 3, axis=2)
        dep_s = pygame.surfarray.make_surface(np.transpose(dep_rgb, (1, 0, 2)))
        screen.blit(pygame.transform.scale(chase_s, (320, 320)), (10, 40))
        screen.blit(pygame.transform.scale(dep_s, (320, 320)), (340, 40))
        screen.blit(font.render("chase_cam", True, (200, 210, 225)), (10, 16))
        screen.blit(font.render("depth_cam (网络输入)", True, (200, 210, 225)), (340, 16))

        lines = [
            f"位置 x={state.pos[0]:5.2f} y={state.pos[1]:5.2f} z={state.pos[2]:4.2f}    "
            f"偏航 {state.yaw:+.2f} rad   速度 {state.vel[0]:+.2f} m/s",
            f"指令 vx={cmd.vx:+.2f}  yaw_rate={cmd.yaw_rate:+.2f}    "
            f"预测通行度 左={c[0]:.2f} 中={c[1]:.2f} 右={c[2]:.2f}",
            "W/S 调速  A/D 偏航  空格 悬停  R 重开  ESC 退出",
        ]
        for i, text in enumerate(lines):
            screen.blit(font.render(text, True, (225, 228, 235)), (10, 372 + 20 * i))
        pygame.display.flip()
        clock.tick(20)

    pygame.quit()
    return 0
