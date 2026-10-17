#!/usr/bin/env python3
"""Record neural-network-controlled flight trajectory and plot it"""
import airsim
import cv2
import numpy as np
import torch
import torch.nn as nn
import time
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

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

print("Taking off...")
c.takeoffAsync().join()
time.sleep(1)

track = []
print("NN controlling and recording trajectory...")
for i in range(80):
    responses = c.simGetImages([airsim.ImageRequest('fpv', airsim.ImageType.Scene, False, False)])
    img = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
    img = img.reshape(responses[0].height, responses[0].width, 3)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, (64, 64)).astype(np.float32) / 255.0
    x = torch.tensor(gray.flatten()).unsqueeze(0)
    with torch.no_grad():
        vx, vy = model(x)[0].numpy()
    c.moveByVelocityAsync(float(vx), float(vy), 0, 0.1)
    p = c.getMultirotorState().kinematics_estimated.position
    track.append((p.x_val, p.y_val))
    time.sleep(0.2)

c.landAsync().join()
c.armDisarm(False)
c.enableApiControl(False)

track = np.array(track)
plt.figure(figsize=(8, 6))
plt.plot(track[:, 0], track[:, 1], 'b-o', markersize=2, label='NN control trajectory')
plt.scatter(track[0, 0], track[0, 1], c='g', s=100, marker='o', label='Start')
plt.scatter(track[-1, 0], track[-1, 1], c='r', s=100, marker='x', label='End')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('AirSim NN Flight Trajectory')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.savefig(os.path.expanduser('~/ros2/src/uav_vision_nn/track.png'), dpi=120, bbox_inches='tight')
print(f"Trajectory saved: ~/ros2/src/uav_vision_nn/track.png, {len(track)} points")
