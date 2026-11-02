import airsim
import pygame
import numpy as np
import time
import argparse

# 解析IP参数
parser = argparse.ArgumentParser()
parser.add_argument("--ip", default="127.0.0.1", help="AirSim主机IP")
args = parser.parse_args()

# 连接AirSim
client = airsim.MultirotorClient(ip=args.ip)
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# pygame初始化
pygame.init()
screen_w, screen_h = 640, 480
screen = pygame.display.set_mode((screen_w, screen_h))
pygame.display.set_caption("Drone Gimbal Teleop")
clock = pygame.time.Clock()

# 云台姿态（俯仰、偏航）
pitch = 0.0
yaw = 0.0
step = 2.0

running = True
while running:
    # 读取相机画面
    img_resp = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    ])[0]
    img_raw = np.frombuffer(img_resp.image_data_uint8, dtype=np.uint8)
    img_rgb = img_raw.reshape(img_resp.height, img_resp.width, 3)
    img_surf = pygame.surfarray.make_surface(np.rot90(img_rgb))
    screen.blit(img_surf, (0,0))

    # 按键检测
    keys = pygame.key.get_pressed()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False

    # 方向键控制云台
    if keys[pygame.K_UP]:
        pitch += step
    if keys[pygame.K_DOWN]:
        pitch -= step
    if keys[pygame.K_LEFT]:
        yaw += step
    if keys[pygame.K_RIGHT]:
        yaw -= step

    # 限制俯仰范围
    pitch = np.clip(pitch, -90, 90)
    # 设置相机云台姿态
    client.simSetCameraPose("0", airsim.Pose(
        airsim.Vector3r(0,0,0),
        airsim.to_quaternion(np.radians(pitch), 0, np.radians(yaw))
    ), external=False)

    # 背景填充（已经修复元组bug）
    screen.fill((25, 25, 30))
    screen.blit(img_surf, (0,0))
    pygame.display.flip()
    clock.tick(30)

# 退出流程：降落、释放控制权
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)
pygame.quit()
print("降落完成，API控制权已释放")
