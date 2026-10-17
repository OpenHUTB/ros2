#!/usr/bin/env python3
"""神经网络实时控制：前视图像 -> MLP -> 速度指令"""
import airsim
import cv2
import numpy as np
import torch
import torch.nn as nn
import time
import os

MODEL_PATH = os.path.expanduser("~/ros2/src/uav_vision_nn/model.pth")

class MLP(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(64*64, 128), nn.ReLU(),
            nn.Linear(128, 64), nn.ReLU(),
            nn.Linear(64, 2)
        )
    def forward(self, x):
        return self.net(x)

model = MLP()
model.load_state_dict(torch.load(MODEL_PATH, map_location="cpu"))
model.eval()

c = airsim.MultirotorClient('10.0.2.2')
c.confirmConnection()
c.enableApiControl(True)
c.armDisarm(True)

print("起飞...")
c.takeoffAsync().join()
time.sleep(1)

print("神经网络控制开始（100帧）...")
for i in range(100):
    responses = c.simGetImages([airsim.ImageRequest('fpv', airsim.ImageType.Scene, False, False)])
    img = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
    img = img.reshape(responses[0].height, responses[0].width, 3)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, (64, 64)).astype(np.float32) / 255.0
    x = torch.tensor(gray.flatten()).unsqueeze(0)
    with torch.no_grad():
        vx, vy = model(x)[0].numpy()
    print(f"帧 {i:3d}: vx={vx:.2f}, vy={vy:.2f}")
    c.moveByVelocityAsync(float(vx), float(vy), 0, 0.1)
    time.sleep(0.2)

c.landAsync().join()
c.armDisarm(False)
c.enableApiControl(False)
print("完成！")
