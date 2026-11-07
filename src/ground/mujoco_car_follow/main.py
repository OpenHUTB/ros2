#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import numpy as np
import os
import time

xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scene.xml")
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

target_distance = 0.5
t = 0

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        t += 0.01
        
        data.ctrl[0] = 0.2
        data.ctrl[1] = 0.3 * np.cos(t * 2)
        
        lead_x = data.qpos[0]
        lead_y = data.qpos[1]
        follow_x = data.qpos[2]
        follow_y = data.qpos[3]
        
        dx = lead_x - follow_x
        dy = lead_y - follow_y
        
        if abs(dx) > target_distance:
            data.ctrl[2] = 0.3 * np.sign(dx)
        else:
            data.ctrl[2] = 0.0
            
        if abs(dy) > 0.1:
            data.ctrl[3] = 0.3 * np.sign(dy)
        else:
            data.ctrl[3] = 0.0
        
        print(f"引导车x={lead_x:.3f} y={lead_y:.3f} 跟随车x={follow_x:.3f} y={follow_y:.3f}")
        
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)
