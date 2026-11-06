import mujoco
import mujoco.viewer
import numpy as np
import time

model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)

T_RED = 5.0
T_YELLOW = 2.0
T_GREEN = 5.0
CYCLE = T_RED + T_YELLOW + T_GREEN
TIME_OFFSET = 0
import os

xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scene.xml")
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

T_RED = 8.0
T_YELLOW = 2.0
T_GREEN = 5.0
CYCLE = T_RED + T_YELLOW + T_GREEN
start_t = time.time()

id_red = model.geom("light_red").id
id_yellow = model.geom("light_yellow").id
id_green = model.geom("light_green").id

stop_line = 2.0
normal_force = 0.6
slow_force = 0.2
brake_force = -0.3

start_t = time.time()
light_name = ["红灯", "黄灯", "绿灯"]

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        t_now = time.time() - start_t + TIME_OFFSET
        t_mod = t_now % CYCLE

brake_distance = 2.0
normal_force = 0.6
slow_force = 0.3
brake_force = -2.0

light_name = ["红", "黄", "绿"]

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        t_now = time.time() - start_t
        t_mod = t_now % CYCLE
        
        if t_mod < T_RED:
            light = 0
        elif t_mod < T_RED + T_YELLOW:
            light = 1
        else:
            light = 2

        model.geom_rgba[id_red]    = np.array([1.0,0,0,1]) if light==0 else np.array([0.25,0,0,1])
        model.geom_rgba[id_yellow] = np.array([1.0,1.0,0,1]) if light==1 else np.array([0.25,0.25,0,1])
        model.geom_rgba[id_green]  = np.array([0,1.0,0,1]) if light==2 else np.array([0,0.25,0,1])

        car_x = data.qpos[0]
        car_v = data.qvel[0]
        force = normal_force

        if car_x >= stop_line:
        
        model.geom_rgba[id_red]    = np.array([1.0,0,0,1]) if light==0 else np.array([0.25,0,0,1])
        model.geom_rgba[id_yellow] = np.array([1.0,1.0,0,1]) if light==1 else np.array([0.25,0.25,0,1])
        model.geom_rgba[id_green]  = np.array([0,1.0,0,1]) if light==2 else np.array([0,0.25,0,1])
        
        car_x = data.qpos[0]
        car_v = data.qvel[0]
        force = normal_force
        
        if car_x >= stop_line - brake_distance:
            if light == 0:
                if car_v > 0.001:
                    force = brake_force
                else:
                    force = 0.0
            elif light == 1:
                force = slow_force
            else:
                force = normal_force

        data.ctrl[0] = force
        mujoco.mj_step(model, data)
        viewer.sync()

        print(f"[时间{t_now:.2f}s] 信号灯：{light_name[light]} | 小车位置x={car_x:.3f} | 速度={car_v:.4f} | 推力={force:.3f}")
        time.sleep(0.05)
        
        print(f"[时间{t_now:.2f}s] 信号灯：{light_name[light]} | 小车位置x={car_x:.3f} | 速度={car_v:.4f} | 推力={force:.3f}")
        
        data.ctrl[0] = force
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.02)
