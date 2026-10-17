#!/usr/bin/env python3
import airsim
import cv2
import numpy as np
import time
import os
import csv

DATA_DIR = os.path.expanduser("~/ros2/src/uav_vision_nn/data")
os.makedirs(DATA_DIR, exist_ok=True)

c = airsim.MultirotorClient('10.0.2.2')
c.confirmConnection()
c.enableApiControl(True)
c.armDisarm(True)

print("起飞...")
c.takeoffAsync().join()
time.sleep(1)

waypoints = [(5, 0, -3), (5, 5, -3), (0, 5, -3), (0, 0, -3)]

csv_path = os.path.join(DATA_DIR, "controls.csv")
idx = 0
with open(csv_path, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["image", "vx", "vy"])
    for target in waypoints:
        print(f"飞向航点 {target}")
        while True:
            state = c.getMultirotorState().kinematics_estimated.position
            dx = target[0] - state.x_val
            dy = target[1] - state.y_val
            dist = np.sqrt(dx*dx + dy*dy)
            if dist < 0.5:
                break
            speed = min(2.0, dist)
            vx = speed * dx / dist
            vy = speed * dy / dist
            c.moveByVelocityAsync(vx, vy, 0, 0.1)
            responses = c.simGetImages([airsim.ImageRequest('fpv', airsim.ImageType.Scene, False, False)])
            img = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
            img = img.reshape(responses[0].height, responses[0].width, 3)
            fname = f"frame_{idx:05d}.png"
            cv2.imwrite(os.path.join(DATA_DIR, fname), img)
            writer.writerow([fname, vx, vy])
            idx += 1
            time.sleep(0.2)

c.landAsync().join()
c.armDisarm(False)
c.enableApiControl(False)
print(f"完成！共收集 {idx} 帧数据到 {DATA_DIR}")
