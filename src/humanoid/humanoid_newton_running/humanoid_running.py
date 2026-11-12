import os
import mujoco
import mujoco.viewer
import numpy as np
import time

scene_xml = os.environ.get(
    "T1_SCENE_XML",
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "models", "booster_t1", "scene.xml"),
)
if not os.path.isfile(scene_xml):
    raise FileNotFoundError(
        f"未找到 T1 场景模型：{scene_xml}。请克隆 mujoco_menagerie 后设置环境变量 "
        "T1_SCENE_XML 指向 booster_t1/scene.xml。"
    )
model = mujoco.MjModel.from_xml_path(scene_xml)
data = mujoco.MjData(model)

total_mass = model.body_mass.sum()
g = 9.81

print("=" * 50)
print("牛顿第二定律 F=ma 理论计算")
print("=" * 50)
print(f"m = {total_mass:.3f} kg, G = {total_mass*g:.2f} N")
print(f"匀速: a=0, F=0 (平衡)")
print(f"加速: a=0.5 m/s², F=m*a={total_mass*0.5:.2f} N")
print()

p0 = np.zeros(23)
p0[11] = -0.15; p0[17] = -0.15
p0[3] = -1.5; p0[7] = 1.5

p_left = p0.copy()
p_left[0] = 0.3
p_left[11] = 0.9
p_left[14] = -0.8
p_left[12] = 0.15
p_left[10] = 0.25
p_left[15] = 0.2
p_left[2] = 0.8
p_left[6] = -0.8
p_left[8] = -0.5
p_left[4] = -0.5

p_right = p0.copy()
p_right[0] = -0.3
p_right[17] = 0.9
p_right[20] = -0.8
p_right[18] = -0.15
p_right[10] = -0.25
p_right[21] = 0.2
p_right[6] = 0.8
p_right[2] = -0.8
p_right[4] = -0.5
p_right[8] = -0.5

traj = []
z_base = data.qpos[2]

for cycle in range(4):
    frames = 40
    for f in range(frames):
        t = f / frames
        t = t*t*(3-2*t)
        if t < 0.5:
            blend = t * 2
            traj.append(p0*(1-blend) + p_left*blend)
        else:
            blend = (t-0.5) * 2
            traj.append(p_left*(1-blend) + p_right*blend)

speed = 0.5
for cycle in range(6):
    frames = int(60 / speed)
    for f in range(frames):
        t = f / frames
        t = t*t*(3-2*t)
        if t < 0.5:
            blend = t * 2
            traj.append(p0*(1-blend) + p_left*blend)
        else:
            blend = (t-0.5) * 2
            traj.append(p_left*(1-blend) + p_right*blend)
    speed *= 1.3

print(f"总帧数: {len(traj)}")
data.qpos[7:30] = p0
mujoco.mj_forward(model, data)

step = 0
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        data.qpos[7:30] = traj[step % len(traj)]
        # 前进位移每帧增量 0.0035 m（恒定；×1.3 只加快步频，位移不加速）
        data.qpos[0] = step * 0.0035
        data.qpos[2] = z_base + 0.04 * np.sin(step * 0.15)
        data.qvel[6:29] = 0
        mujoco.mj_forward(model, data)

        viewer.cam.lookat[0] = data.qpos[0]
        viewer.cam.distance = 3.0

        viewer.sync()
        time.sleep(1/30)
        step += 1
