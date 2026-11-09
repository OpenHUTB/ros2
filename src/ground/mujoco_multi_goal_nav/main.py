#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import os
import math

xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scene.xml")
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

goals = [(1.5, 1.0), (-1.5, 1.0), (-1.5, -1.0)]
current_goal = 0
last_print_time = 0

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        car_x = data.qpos[0]
        car_y = data.qpos[1]
        
        if current_goal < len(goals):
            target_x, target_y = goals[current_goal]
            dx = target_x - car_x
            dy = target_y - car_y
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist < 0.05:
                print(f"到达目标点{current_goal+1}: x={target_x}, y={target_y}")
                current_goal += 1
            else:
                vx = 0.3 * dx / dist
                vy = 0.3 * dy / dist
                data.qvel[0] = vx
                data.qvel[1] = vy
        else:
            data.qvel[0] = 0
            data.qvel[1] = 0
            print("所有目标点已到达，停车")
            break
        
        if data.time - last_print_time > 0.5:
            print(f"小车位置: x={car_x:.2f}, y={car_y:.2f}")
            last_print_time = data.time
        
        mujoco.mj_step(model, data)
        viewer.sync()
