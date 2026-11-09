#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import os
import time

xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scene.xml")
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

goals = [(1.5, 1.0), (-1.5, 1.0), (-1.5, -1.0)]
current_goal = 0

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        car_x = data.qpos[0]
        car_y = data.qpos[1]
        
        if current_goal < len(goals):
            target_x, target_y = goals[current_goal]
            data.qvel[0] = 0.05 * (target_x - car_x)
            data.qvel[1] = 0.05 * (target_y - car_y)
            
            if abs(target_x - car_x) < 0.1 and abs(target_y - car_y) < 0.1:
                print(f"到达目标点{current_goal+1}: x={target_x}, y={target_y}")
                current_goal += 1
        else:
            data.qvel[0] = 0
            data.qvel[1] = 0
            print("所有目标点已到达，停车")
            break
        
        print(f"小车位置: x={car_x:.2f}, y={car_y:.2f}")
        
        mujoco.mj_step(model, data)
        viewer.sync()
