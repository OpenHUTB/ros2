#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import os
import time

xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scene.xml")
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# 给小车初始速度，让它能从斜坡滑下来
data.qvel[0] = 0.5

stop_x = 0.5

print("=== 小车滑斜坡项目开始 ===")
print(f"目标停车位置：x={stop_x}m")

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        car_x = data.qpos[0]
        car_vx = data.qvel[0]

        if car_x > stop_x:
            # 到坡底，速度衰减刹车
            data.qvel[0] *= 0.9
            status = "坡底刹车中"
        else:
            status = "斜坡下滑中"

        print(f"[时间{data.time:.2f}s] {status} | x={car_x:.3f}m 速度x={car_vx:.3f}m/s")

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)
