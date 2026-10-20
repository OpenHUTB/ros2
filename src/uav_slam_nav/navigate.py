#!/usr/bin/env python3
"""自主导航：沿 A* 规划路径自动飞行（建图->规划->导航闭环）"""
import airsim
import numpy as np
import time
import os
import cv2

ALT = -5
path_w = np.load(os.path.expanduser('~/ros2/src/uav_vision_nn/path.npy'))

c = airsim.MultirotorClient('10.0.2.2')
c.confirmConnection()
c.enableApiControl(True)
c.armDisarm(True)
print("起飞...")
c.takeoffAsync().join()
time.sleep(1)

track = []
print("沿路径自主飞行...")
for i, (wx, wy) in enumerate(path_w):
    c.moveToPositionAsync(float(wx), float(wy), ALT, 3).join()
    p = c.getMultirotorState().kinematics_estimated.position
    track.append((p.x_val, p.y_val))
    print(f"  [{i}/{len(path_w)}] 到达 ({p.x_val:.1f}, {p.y_val:.1f})")

c.landAsync().join()
c.armDisarm(False)
c.enableApiControl(False)

np.save(os.path.expanduser('~/ros2/src/uav_vision_nn/nav_track.npy'), np.array(track))
print(f"导航完成！实际飞行 {len(track)} 个航点")
