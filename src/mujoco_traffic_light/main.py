import mujoco
import mujoco.viewer
import numpy as np
import time

model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)

# 信号灯时序：红灯 5s，黄灯 2s，绿灯 5s
T_RED = 5.0
T_YELLOW = 2.0
T_GREEN = 5.0
CYCLE = T_RED + T_YELLOW + T_GREEN

id_red = model.geom("light_red").id
id_yellow = model.geom("light_yellow").id
id_green = model.geom("light_green").id

STOP_LINE = 0.5          # 停止线位置
NORMAL = 0.8             # 绿灯正常推力
SLOW = 0.2               # 黄灯慢速推力
BRAKE = -0.35            # 红灯反向刹车力

start_t = time.time()
name = ["红灯", "黄灯", "绿灯"]

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        t_now = time.time() - start_t
        t_mod = t_now % CYCLE

        # 判断当前信号灯状态
        if t_mod < T_RED:
            light = 0          # 红灯
        elif t_mod < T_RED + T_YELLOW:
            light = 1          # 黄灯
        else:
            light = 2          # 绿灯

        # 点亮当前灯，其余变暗
        model.geom_rgba[id_red] = np.array([1.0, 0, 0, 1]) if light == 0 else np.array([0.25, 0, 0, 1])
        model.geom_rgba[id_yellow] = np.array([1.0, 1.0, 0, 1]) if light == 1 else np.array([0.25, 0.25, 0, 1])
        model.geom_rgba[id_green] = np.array([0, 1.0, 0, 1]) if light == 2 else np.array([0, 0.25, 0, 1])

        car_x = data.qpos[0]
        car_v = data.qvel[0]
        force = NORMAL

        # 到达停止线后，严格遵守信号灯：红灯停、黄灯慢、绿灯行
        if car_x >= STOP_LINE:
            if light == 0:
                # 红灯：还在前进就反向刹车，速度归零后锁死不后退
                force = BRAKE if car_v > 0.001 else 0.0
            elif light == 1:
                force = SLOW
            else:
                force = NORMAL

        data.ctrl[0] = force
        mujoco.mj_step(model, data)
        viewer.sync()

        # 运行日志
        print(f"[{t_now:6.2f}s] 信号灯：{name[light]} | x={car_x:.3f} | v={car_v:.4f} | F={force:.3f}")
        time.sleep(0.02)
