import mujoco
import cv2
import numpy as np
import os

# 动态获取脚本所在路径，向上找一级到 models 文件夹
current_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(current_dir, '../models/quadruped.xml')
model = mujoco.MjModel.from_xml_path(model_path)

data = mujoco.MjData(model)

cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'side_camera')
renderer = mujoco.Renderer(model, height=480, width=640)

print("MuJoCo 四足机器人 trot 步态仿真")
print("Q = 退出")

# 步态参数
gait_period = 0.5  # 一个完整步态周期（秒），步频更快
hip_amp = 0.6       # 髋部摆动幅度（弧度），加大前后摆
knee_amp = 0.8     # 膝部弯曲幅度（弧度），抬腿更高
knee_offset = -0.25  # 膝部默认弯曲角度

time = 0.0

while True:
    # 计算步态相位（0 ~ 2π）
    phase = 2 * np.pi * (time / gait_period)

    # trot 对角步态：
    # 组1：FL（前左）+ RR（后右）
    # 组2：FR（前右）+ RL（后左）
    # 两组相位差 π（180度）

    # 组1相位
    p1 = phase
    # 组2相位（错开半个周期）
    p2 = phase + np.pi

    # 髋部：正弦波前后摆动
    fl_hip = hip_amp * np.sin(p1)
    rr_hip = hip_amp * np.sin(p1)
    fr_hip = hip_amp * np.sin(p2)
    rl_hip = hip_amp * np.sin(p2)

    # 膝部：摆动相时弯曲抬腿（sin > 0 时抬腿）
    # 用正弦的正半周作为抬腿信号
    fl_knee = knee_offset - knee_amp * max(0, np.sin(p1))
    rr_knee = knee_offset - knee_amp * max(0, np.sin(p1))
    fr_knee = knee_offset - knee_amp * max(0, np.sin(p2))
    rl_knee = knee_offset - knee_amp * max(0, np.sin(p2))

    # 写入控制量（ctrl 顺序与 actuator 定义一致）
    # 0: fl_hip, 1: fl_knee, 2: fr_hip, 3: fr_knee
    # 4: rl_hip, 5: rl_knee, 6: rr_hip, 7: rr_knee
    data.ctrl[0] = fl_hip
    data.ctrl[1] = fl_knee
    data.ctrl[2] = fr_hip
    data.ctrl[3] = fr_knee
    data.ctrl[4] = rl_hip
    data.ctrl[5] = rl_knee
    data.ctrl[6] = rr_hip
    data.ctrl[7] = rr_knee

    mujoco.mj_step(model, data)
    time += model.opt.timestep

    # 渲染
    renderer.update_scene(data, camera=cam_id)
    img = renderer.render()
    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    # 叠加显示当前状态
    trunk_pos = data.qpos[0:3]
    cv2.putText(img_bgr, f"t: {time:.1f}s  pos: ({trunk_pos[0]:.2f}, {trunk_pos[1]:.2f})",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow('quadruped_walk', img_bgr)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break

cv2.destroyAllWindows()
