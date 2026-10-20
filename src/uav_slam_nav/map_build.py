#!/usr/bin/env python3
"""基于深度相机的占据栅格建图：前视深度图 -> 3D点云 -> 2D栅格地图"""
import airsim
import numpy as np
import cv2
import time
import os

W, H, FOV = 256, 144, 90
fx = (W/2) / np.tan(np.radians(FOV/2))
fy = (H/2) / np.tan(np.radians(FOV/2))
cx, cy = W/2 - 0.5, H/2 - 0.5

GRID_RES = 0.25
MAP_RANGE = 25.0
ALT = -5                 # 飞行高度 5m (NED: z 负=上)
MAX_DEPTH = 15.0

waypoints = [(0,0), (8,0), (8,8), (0,8), (-8,8), (-8,0), (-8,-8), (0,-8), (8,-8), (0,0)]

c = airsim.MultirotorClient('10.0.2.2')
c.confirmConnection()
c.enableApiControl(True)
c.armDisarm(True)
print("起飞...")
c.takeoffAsync().join()
time.sleep(1)

N = int(MAP_RANGE*2/GRID_RES)
grid = np.zeros((N, N), dtype=np.uint8)

def to_grid(x, y):
    return int((x + MAP_RANGE)/GRID_RES), int((y + MAP_RANGE)/GRID_RES)

def quat_to_rot(q):
    x, y, z, w = q.x_val, q.y_val, q.z_val, q.w_val
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]
    ])

print("扫描建图中...")
uu, vv = np.meshgrid(np.arange(W), np.arange(H))
for wp in waypoints:
    print(f"  -> 航点 {wp}")
    c.moveToPositionAsync(wp[0], wp[1], ALT, 4).join()
    time.sleep(0.3)
    for _ in range(2):
        resp = c.simGetImages([airsim.ImageRequest('fpv', airsim.ImageType.DepthPerspective, True)])
        D = np.array(resp[0].image_data_float, dtype=np.float32).reshape(H, W)
        state = c.getMultirotorState()
        pos = state.kinematics_estimated.position
        q = state.kinematics_estimated.orientation
        R = quat_to_rot(q)
        cam_z = D
        cam_x = (uu - cx) * D / fx
        cam_y = (vv - cy) * D / fy
        pvx = cam_z + 0.3
        pvy = cam_x
        pvz = cam_y - 0.2
        pts = np.stack([pvx.ravel(), pvy.ravel(), pvz.ravel()])
        wpts = R @ pts + np.array([[pos.x_val], [pos.y_val], [pos.z_val]])
        valid = (D.ravel() < MAX_DEPTH) & (D.ravel() > 0.3) & (wpts[2] < -0.3)
        gx_all = ((wpts[0][valid] + MAP_RANGE)/GRID_RES).astype(int)
        gy_all = ((wpts[1][valid] + MAP_RANGE)/GRID_RES).astype(int)
        m = (gx_all >= 0) & (gx_all < N) & (gy_all >= 0) & (gy_all < N)
        grid[gy_all[m], gx_all[m]] = 2
        time.sleep(0.3)

for wp in waypoints:
    gx, gy = to_grid(wp[0], wp[1])
    if 0 <= gx < N and 0 <= gy < N:
        grid[gy, gx] = 1

np.save(os.path.expanduser('~/ros2/src/uav_vision_nn/map.npy'), grid)
img = np.zeros((N, N, 3), dtype=np.uint8)
img[grid == 1] = [200, 200, 200]
img[grid == 2] = [0, 0, 255]
cv2.imwrite(os.path.expanduser('~/ros2/src/uav_vision_nn/map.png'), img)
print(f"地图已保存: map.npy + map.png ({N}x{N} 栅格)")

c.landAsync().join()
c.armDisarm(False)
c.enableApiControl(False)
print("完成！")
