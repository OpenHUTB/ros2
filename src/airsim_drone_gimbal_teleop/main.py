import pygame
import airsim
import time

# 连接AirSim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# 起飞
client.takeoffAsync().join()

# 初始化pygame
pygame.init()
screen = pygame.display.set_mode((400, 300))
pygame.display.set_caption("Drone Keyboard Control")
clock = pygame.time.Clock()

# 速度参数
linear_speed = 2
yaw_speed = 30

print("=== 无人机控制（Pygame版，无需sudo）===")
print("W/S：前进/后退  A/D：左/右平移")
print("↑↓：上升/下降  ←→：左转/右转")
print("ESC：退出并降落")

running = True
while running:
    screen.fill((30,30,30))
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False

    keys = pygame.key.get_pressed()
    vx, vy, vz, yaw_rate = 0, 0, 0, 0

    if keys[pygame.K_w]:
        vx = linear_speed
    if keys[pygame.K_s]:
        vx = -linear_speed
    if keys[pygame.K_a]:
        vy = linear_speed
    if keys[pygame.K_d]:
        vy = -linear_speed
    if keys[pygame.K_UP]:
        vz = -linear_speed
    if keys[pygame.K_DOWN]:
        vz = linear_speed
    if keys[pygame.K_LEFT]:
        yaw_rate = yaw_speed
    if keys[pygame.K_RIGHT]:
        yaw_rate = -yaw_speed

    client.moveByVelocityBodyFrameAsync(vx, vy, vz, 0.1, yaw_rate=yaw_rate)
    pygame.display.flip()
    clock.tick(20)

# 退出流程
print("准备降落...")
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)
pygame.quit()
print("降落完成")
