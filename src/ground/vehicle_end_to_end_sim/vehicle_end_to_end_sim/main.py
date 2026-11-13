"""车载端到端仿真系统 —— Pygame 主程序。

启动流程：参数设置界面（调节目标车速）→ 主仿真循环。

主循环分层（端到端感知 → 决策 → 控制 → 物理）：
    感知  perception.compute_nearest_front_distance   雷达 360° 扫描 + 前方 40° 视野锥
    决策  decision.decide                             安全区 / 预警区 / 刹车区（AEB）
    控制  control.update_steering                     键盘转向 + 平滑回正
    物理  kinematics.Car.update                        自行车运动学模型

运行：python3 main.py（或 python3 -m vehicle_end_to_end_sim.main）
"""

import math
import sys

import pygame

from . import config
from .kinematics import Car
from .perception import compute_nearest_front_distance
from .decision import decide, DriveState
from .control import update_steering


def _load_font(size):
    """跨平台加载中文字体；找不到时回退到 pygame 默认字体。"""
    candidates = [
        "C:/Windows/Fonts/simhei.ttf",   # Windows 黑体
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",  # Linux 文泉驿
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "msyh.ttc",
        "simhei.ttf",
    ]
    for path in candidates:
        try:
            if pygame.font.match_font(path):
                return pygame.font.Font(path, size)
        except (pygame.error, OSError):
            continue
    # 回退：pygame 内置字体（可能无法渲染中文，但保证可运行）
    return pygame.font.Font(None, size)


def _run_setup_menu(cn_font, title_font):
    """① 启动设置界面：↑/↓ 调节目标车速，Enter 启动仿真。"""
    target_speed_kmh = config.DEFAULT_TARGET_SPEED_KMH
    in_setup_menu = True

    while in_setup_menu:
        screen = pygame.display.get_surface()
        screen.fill((20, 20, 20))
        title_cn = title_font.render("车辆仿真参数设置", True, (0, 255, 255))
        screen.blit(title_cn, (config.WIDTH // 2 - title_cn.get_width() // 2, 150))
        prompt_cn = cn_font.render(
            "请使用键盘 [ 上 或 下 ] 调节目标车速，按 [ 回车 ] 启动:",
            True, (200, 200, 200))
        screen.blit(prompt_cn, (config.WIDTH // 2 - prompt_cn.get_width() // 2, 250))
        speed_text = title_font.render(
            "{:.0f} km/h".format(target_speed_kmh), True, (0, 255, 0))
        screen.blit(speed_text, (config.WIDTH // 2 - speed_text.get_width() // 2, 300))
        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit(0)
            if event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                    in_setup_menu = False
                elif event.key == pygame.K_UP:
                    target_speed_kmh += config.TARGET_SPEED_STEP_KMH
                elif event.key == pygame.K_DOWN:
                    if target_speed_kmh > config.MIN_SPEED_KMH:
                        target_speed_kmh -= config.TARGET_SPEED_STEP_KMH

    return target_speed_kmh


def _draw_car(car):
    """绘制小车（带方向感与车轮）。"""
    car_surface = pygame.Surface((40, 24), pygame.SRCALPHA)
    for (cx, cy) in [(10, 2), (30, 2), (10, 22), (30, 22)]:
        pygame.draw.circle(car_surface, (20, 20, 20), (cx, cy), 4)   # 车轮
    pygame.draw.rect(car_surface, (0, 255, 0), (2, 4, 36, 16), border_radius=4)   # 车身
    pygame.draw.rect(car_surface, (100, 200, 255), (28, 6, 8, 12), border_radius=2)  # 车头

    rotated = pygame.transform.rotate(car_surface, -math.degrees(car.theta))
    rect = rotated.get_rect(center=(int(car.x), int(car.y)))
    pygame.display.get_surface().blit(rotated, rect.topleft)


def _draw_ui(cn_font, alert_font, dist_text, current_kmh, target_kmh,
             state, emergency_brake, min_dist):
    """绘制 HUD；用与背景同色底块强制抹除上一帧文字，防重影。"""
    screen = pygame.display.get_surface()
    bg_color = (30, 30, 30)

    # 防重影底块
    pygame.draw.rect(screen, bg_color, (0, 0, 600, 110))
    pygame.draw.rect(screen, bg_color, (config.WIDTH // 2 - 250, 110, 500, 40))

    speed_color = (255, 50, 50) if emergency_brake else (0, 255, 0)
    text1 = cn_font.render("正前方障碍物距离: {}".format(dist_text), True, (0, 255, 255))
    text2 = cn_font.render(
        "目标车速: {:.0f} km/h | 当前车速: {:.1f} km/h".format(target_kmh, current_kmh),
        True, speed_color)
    screen.blit(text1, (10, 10))
    screen.blit(text2, (10, 40))

    if state == DriveState.WARN:
        alert = alert_font.render("警告：请转弯！", True, (255, 165, 0))
        alert_rect = alert.get_rect(center=(config.WIDTH // 2, 130))
        screen.blit(alert, alert_rect)
    elif state == DriveState.BRAKE:
        alert = alert_font.render("减速刹车中...", True, (255, 0, 0))
        alert_rect = alert.get_rect(center=(config.WIDTH // 2, 130))
        screen.blit(alert, alert_rect)


def _run_simulation(cn_font, alert_font, target_speed_kmh):
    """② 主仿真循环。"""
    target_speed_pixel = config.kmh_to_pixel(target_speed_kmh)
    car = Car()
    obstacles = config.OBSTACLES
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # ---- 感知层：雷达 360° 扫描 + 前方 40° 视野锥，取最近障碍物距离 ----
        min_dist = compute_nearest_front_distance(car, obstacles)

        # ---- 决策层：三级状态机，就地更新车速 ----
        state, emergency_brake = decide(car, target_speed_pixel, min_dist)

        # ---- 控制层：键盘转向 + 平滑回正 ----
        keys = pygame.key.get_pressed()
        update_steering(keys[pygame.K_LEFT], keys[pygame.K_RIGHT], car)

        # ---- 物理层：自行车模型推进一帧 ----
        car.update()

        # ---- 渲染 ----
        screen = pygame.display.get_surface()
        screen.fill((30, 30, 30))

        for (ox, oy) in obstacles:
            pygame.draw.circle(screen, (100, 100, 100), (ox, oy), 20)

        _draw_car(car)

        if math.isinf(min_dist):
            dist_text = "前方安全"
        else:
            dist_text = "{:.1f} m".format(min_dist / config.PIXELS_PER_METER)

        current_kmh = config.pixel_to_kmh(car.v_pixel)
        _draw_ui(cn_font, alert_font, dist_text, current_kmh, target_speed_kmh,
                 state, emergency_brake, min_dist)

        pygame.display.flip()
        pygame.time.Clock().tick(config.FPS)


def main():
    """程序入口。"""
    pygame.init()
    pygame.display.set_mode((config.WIDTH, config.HEIGHT))
    pygame.display.set_caption("vehicle_end_to_end_sim 车载端到端仿真")

    cn_font = _load_font(20)
    alert_font = _load_font(30)
    title_font = _load_font(40)

    target_speed_kmh = _run_setup_menu(cn_font, title_font)
    _run_simulation(cn_font, alert_font, target_speed_kmh)

    pygame.quit()
