import mujoco
import mujoco.viewer
import numpy as np
import time  # 导入time模块用于延时控制
from scipy.linalg import solve_continuous_are

# 加载模型
model = mujoco.MjModel.from_xml_path("balance.xml")
data = mujoco.MjData(model)

M = 5.0   # 小车质量 kg
m = 0.5   # 摆杆质量 kg
l = 0.15  # 转轴到摆杆质心距离
g = 9.8

# 倒立摆线性化状态空间 A,B
A = np.array([
    [0, 1, 0, 0],
    [0, 0, -(m*g)/M, 0],
    [0, 0, 0, 1],
    [0, 0, ((M+m)*g)/(M*l), 0]
])
B = np.array([
    [0],
    [1/M],
    [0],
    [-1/(M*l)]
])

# LQR权重矩阵
Q = np.diag([10, 1, 100, 1])
R = np.array([[0.1]])

# 求解黎卡提方程，得到反馈增益K
P = solve_continuous_are(A,B,Q,R)
K = np.linalg.inv(R) @ B.T @ P

step_count = 0
wait_steps = 50  # 等待50个仿真步，开头保持竖直

# 启动仿真窗口
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # 延迟施加扰动：前50步保持竖直
        if step_count == wait_steps:
            data.qpos[1] = 0.1
            data.qvel[:] = 0.0

        # 时序：读取状态 → 计算控制量 → 赋值 → 步进仿真
        x_state = np.array([data.qpos[0], data.qvel[0], data.qpos[1], data.qvel[1]])
        u = -K @ x_state
        data.ctrl[0] = u

        mujoco.mj_step(model, data)
        viewer.sync()
        step_count += 1

        # 新增：按仿真步长延时，保证实时匀速，防止仿真超速
        time.sleep(model.opt.timestep)
