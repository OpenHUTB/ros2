import pygame
import math
import sys

pygame.init()
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Win11 + 逻辑分离版")
clock = pygame.time.Clock()

cn_font = pygame.font.Font("C:/Windows/Fonts/simhei.ttf", 20)
alert_font = pygame.font.Font("C:/Windows/Fonts/simhei.ttf", 30)
title_font = pygame.font.Font("C:/Windows/Fonts/simhei.ttf", 40)

PIXELS_PER_METER = 1.0 

class Car:
    def __init__(self, x, y, theta):
        self.x, self.y, self.theta = x, y, theta
        self.v_pixel = 0.0
        self.steer = 0.0
    
    def update(self):
        self.x += self.v_pixel * math.cos(self.theta)
        self.y += self.v_pixel * math.sin(self.theta)
        self.theta += (self.v_pixel / 7.0) * math.tan(self.steer * math.pi/4)

# ==========================================
# 1. 启动设置界面
# ==========================================
target_speed_kmh = 60.0 
in_setup_menu = True

while in_setup_menu:
    screen.fill((20, 20, 20))
    title_cn = title_font.render("🚗 车辆仿真参数设置", True, (0, 255, 255))
    screen.blit(title_cn, (WIDTH//2 - title_cn.get_width()//2, 150))
    prompt_cn = cn_font.render("请使用键盘 [ ↑ 或 ↓ ] 调节目标车速，按 [ 回车 ] 启动:", True, (200, 200, 200))
    screen.blit(prompt_cn, (WIDTH//2 - prompt_cn.get_width()//2, 250))
    speed_text = title_font.render(f"{target_speed_kmh:.0f} km/h", True, (0, 255, 0))
    screen.blit(speed_text, (WIDTH//2 - speed_text.get_width()//2, 300))
    pygame.display.flip()
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RETURN or event.key == pygame.K_KP_ENTER: 
                in_setup_menu = False
            elif event.key == pygame.K_UP:
                target_speed_kmh += 10
            elif event.key == pygame.K_DOWN:
                if target_speed_kmh > 10:
                    target_speed_kmh -= 10

# ==========================================
# 核心物理参数
# ==========================================
target_speed_ms = target_speed_kmh / 3.6
target_speed_pixel = (target_speed_ms / 60.0) * PIXELS_PER_METER

# 预警线和刹车线
WARN_DIST_PIXEL = 300.0
SAFE_BRAKE_DIST_PIXEL = 180.0

# ==========================================
# 2. 开始正式仿真
# ==========================================
car = Car(WIDTH//2, HEIGHT - 80, -math.pi/2)
obstacles = [(400, 200), (300, 100), (500, 150)]

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # 底层雷达
    min_dist_pixel = 99999.0
    for obs in obstacles:
        dx = obs[0] - car.x
        dy = obs[1] - car.y
        dist = math.hypot(dx, dy)
        angle_to_obs = math.atan2(dy, dx)
        angle_diff = (angle_to_obs - car.theta + math.pi) % (2 * math.pi) - math.pi
        
        if abs(angle_diff) < math.radians(40):
            if dist < min_dist_pixel:
                min_dist_pixel = dist

    # ==========================================
    # 【核心修复】严格分离预警与刹车
    # ==========================================
    if min_dist_pixel < SAFE_BRAKE_DIST_PIXEL:
        # 只有进入红线区，才开始减速
        car.v_pixel -= 0.03 
        if car.v_pixel < 0: 
            car.v_pixel = 0.0
        emergency_brake = True
    else:
        # 不管是否在预警区，只要没进红线，就一直保持目标速度开（或者加速）
        if car.v_pixel < target_speed_pixel:
            car.v_pixel += 0.02
        else:
            car.v_pixel = target_speed_pixel
        emergency_brake = False

    # 平滑转向与自动回正
    keys = pygame.key.get_pressed()
    steer_speed = 0.02  
    
    if keys[pygame.K_LEFT]:
        car.steer -= steer_speed
    elif keys[pygame.K_RIGHT]:
        car.steer += steer_speed
    else:
        if car.steer > 0:
            car.steer -= steer_speed
            if car.steer < 0: car.steer = 0.0 
        elif car.steer < 0:
            car.steer += steer_speed
            if car.steer > 0: car.steer = 0.0 
            
    car.steer = max(-1.0, min(1.0, car.steer))
    car.update()

    # UI 现实数据
    if min_dist_pixel < 99999.0:
        # 直接转换为现实世界的米/公里
        real_dist_m = min_dist_pixel / PIXELS_PER_METER
        dist_text = f"{real_dist_m:.1f} m"
    else:
        dist_text = "前方安全"
        
    current_kmh = car.v_pixel * 60.0 / PIXELS_PER_METER * 3.6

    # ==========================================
    # 3. 渲染画面
    # ==========================================
    screen.fill((30, 30, 30)) 
    
    for obs in obstacles:
        pygame.draw.circle(screen, (100, 100, 100), obs, 20)
    
    # 画小车
    car_surface = pygame.Surface((40, 24), pygame.SRCALPHA)
    pygame.draw.circle(car_surface, (20, 20, 20), (10, 2), 4)
    pygame.draw.circle(car_surface, (20, 20, 20), (30, 2), 4)
    pygame.draw.circle(car_surface, (20, 20, 20), (10, 22), 4)
    pygame.draw.circle(car_surface, (20, 20, 20), (30, 22), 4)
    pygame.draw.rect(car_surface, (0, 255, 0), (2, 4, 36, 16), border_radius=4)
    pygame.draw.rect(car_surface, (100, 200, 255), (28, 6, 8, 12), border_radius=2)
    
    rotated_car = pygame.transform.rotate(car_surface, -math.degrees(car.theta))
    rect = rotated_car.get_rect(center=(int(car.x), int(car.y)))
    screen.blit(rotated_car, rect.topleft)

    # 防重影底块
    pygame.draw.rect(screen, (30, 30, 30), (0, 0, 600, 110))
    pygame.draw.rect(screen, (30, 30, 30), (WIDTH//2 - 250, 110, 500, 40))

    # 渲染文字
    speed_color = (255, 50, 50) if emergency_brake else (0, 255, 0)
    
    text1 = cn_font.render(f"正前方障碍物距离: {dist_text}", True, (0, 255, 255))
    text2 = cn_font.render(f"目标车速: {target_speed_kmh:.0f} km/h | 当前车速: {current_kmh:.1f} km/h", True, speed_color)
    
    screen.blit(text1, (10, 10))
    screen.blit(text2, (10, 40))

    # 提示逻辑
    if SAFE_BRAKE_DIST_PIXEL <= min_dist_pixel < WARN_DIST_PIXEL:
        # 只有在这个区间才显示橙色警告，此时速度不变
        alert_text = alert_font.render("⚠ 警告：请转弯！", True, (255, 165, 0))
        alert_rect = alert_text.get_rect(center=(WIDTH//2, 130))
        screen.blit(alert_text, alert_rect)
    elif emergency_brake:
        # 进入红线区，显示红色刹车，此时才开始减速
        alert_text = alert_font.render("🛑 减速刹车中...", True, (255, 0, 0))
        alert_rect = alert_text.get_rect(center=(WIDTH//2, 130))
        screen.blit(alert_text, alert_rect)
    
    pygame.display.flip()
    clock.tick(60)

pygame.quit()