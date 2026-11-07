from pathlib import Path
import mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt
import time
# 从独立控制器文件导入
from aeb_controller import AEBController

# ========== 动态路径，解决不同目录运行报错 ==========
SCRIPT_DIR = Path(__file__).resolve().parent
MODEL_PATH = SCRIPT_DIR.parent / "models" / "aeb_car.xml"

model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data = mujoco.MjData(model)

ttc_threshold = 5.0
brake_max = 6000.0
sim_time = 10.0
v_stop = 0.1

# 实例化控制器
controller = AEBController(ttc_threshold=ttc_threshold, gain=1.4, v_stop=v_stop)

t_rec, v_rec, ttc_rec, brake_rec = [], [], [], []
data.qvel[0] = 5.0

with mujoco.viewer.launch_passive(model, data) as viewer:
    # 摄像机设置不变
    viewer.cam.lookat[:] = [20, 0, 0]
    viewer.cam.distance = 55
    viewer.cam.elevation = -35
    viewer.cam.azimuth = 90
    wall_start = time.time()
    while viewer.is_running() and data.time < sim_time:
        # 实时慢放
        desired = data.time * 1.0
        current = time.time() - wall_start
        if desired > current:
            time.sleep(desired - current)
        mujoco.mj_step(model, data)
        x_ego = data.qpos[0]
        x_lead = data.qpos[11]
        v_ego = data.qvel[0]
        v_lead = data.qvel[10]
        dist = x_lead - x_ego
        rel_v = v_ego - v_lead

        if abs(v_ego) < v_stop:
            ttc = 99.0
        elif rel_v > 1e-3:
            ttc = dist / rel_v
        else:
            ttc = 99.0

        # 调用独立AEB控制器，传参对齐aeb_controller.py
        brake = controller.get_brake(ttc, v_ego)
        brake = np.clip(brake, 0, 1)
        force = brake * brake_max

        # 【最核心修复】使用 qfrc_applied 直接施加广义力到 X 轴
        data.qfrc_applied[0] = -force
        t_rec.append(data.time)
        v_rec.append(v_ego)
        ttc_rec.append(ttc)
        brake_rec.append(brake)
        viewer.sync()

# 绘图不变
plt.figure(figsize=(10,8))
plt.subplot(3,1,1)
plt.plot(t_rec, v_rec, label="Speed m/s", color='blue', linewidth=2)
plt.ylabel("Speed (m/s)")
plt.legend()
plt.grid(True)

plt.subplot(3,1,2)
plt.plot(t_rec, ttc_rec, label="TTC", color='orange', linewidth=2)
plt.axhline(y=ttc_threshold, color='r', linestyle='--', label='threshold')
plt.ylim(0, 12)
plt.ylabel("TTC (s)")
plt.legend()
plt.grid(True)

plt.subplot(3,1,3)
plt.plot(t_rec, brake_rec, label="Brake 0~1", color='red', linewidth=2)
plt.xlabel("Time (s)")
plt.ylabel("Brake")
plt.legend()
plt.grid(True)
plt.tight_layout()

# 新增：自动保存曲线图（评审要求）
out_img = SCRIPT_DIR.parent / "docs/img/result.png"
plt.savefig(out_img, dpi=150)
plt.show()
