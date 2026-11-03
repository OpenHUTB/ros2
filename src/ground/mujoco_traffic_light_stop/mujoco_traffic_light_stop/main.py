import mujoco
import mujoco.viewer
import numpy as np
import time
import os

xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scene.xml")
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

t_red = 5
t_yellow = 2
t_green = 5
cycle = t_red + t_yellow + t_green

id_red = model.geom("light_red").id
id_yellow = model.geom("light_yellow").id
id_green = model.geom("light_green").id

stop_line = 2.0
slow_dist = 1.0
f_normal = 0.6
f_slow = 0.2
f_brake = -0.3

t0 = time.time()
names = ["红灯", "黄灯", "绿灯"]

with mujoco.viewer.launch_passive(model, data) as v:
    while v.is_running():
        t = (time.time() - t0) % cycle
        if t < t_red:
            light = 0
        elif t < t_red + t_yellow:
            light = 1
        else:
            light = 2

        model.geom_rgba[id_red] = [1,0,0,1] if light==0 else [0.2,0,0,1]
        model.geom_rgba[id_yellow] = [1,1,0,1] if light==1 else [0.2,0.2,0,1]
        model.geom_rgba[id_green] = [0,1,0,1] if light==2 else [0,0.2,0,1]

        x = data.qpos[0]
        vx = data.qvel[0]
        f = f_normal
        d = stop_line - x

        if x >= stop_line:
            if light == 0:
                f = f_brake if vx > 0.001 else 0
            elif light == 1:
                f = f_slow
        elif d < slow_dist and light == 0:
            f = f_normal * d / slow_dist

        data.ctrl[0] = f
        mujoco.mj_step(model, data)
        v.sync()

        print(f"t={t:.2f} {names[light]} x={x:.3f} v={vx:.3f} f={f:.2f}")
        time.sleep(0.05)
