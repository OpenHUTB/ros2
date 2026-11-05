import mujoco
import mujoco_viewer
import glfw
from pid_controller import PIDController

# 加载模型
model = mujoco.MjModel.from_xml_path("balance_car.xml")
data = mujoco.MjData(model)

# 启动可视化
viewer = mujoco_viewer.MujocoViewer(model, data)
viewer.cam.distance = 3.5
viewer.cam.elevation = -20

# PID参数已调好，符号已匹配模型方向
angle_pid = PIDController(kp=12.0, ki=0.0, kd=3.5, output_min=-2.0, output_max=2.0)
pos_pid = PIDController(kp=0.5, ki=0.0, kd=0.1, output_min=-0.15, output_max=0.15)

dt = model.opt.timestep

print("=== MuJoCo 两轮自平衡小车 PID 仿真 ===")
print("R键：重置仿真 | 方向键↑/↓：施加扰动 | ESC：退出")

def reset_sim():
    mujoco.mj_resetData(model, data)
    data.qpos[3] = 0.05 # 小角度初始偏移，保证可控
    angle_pid.reset()
    pos_pid.reset()

reset_sim()

# 在 while viewer.is_alive: 上面，定义目标位置
target_position = 0.0

while viewer.is_alive:
    window = viewer.window
    
    # --- 1. 按键控制区 ---
    # 必须鼠标点在仿真窗口上按键才有效
    if glfw.get_key(window, glfw.KEY_R) == glfw.PRESS:
        reset_sim()
        target_position = 0.0
        
    if glfw.get_key(window, glfw.KEY_UP) == glfw.PRESS:
        data.qvel[0] = 1.5  # 强行给它一个向前的速度
    if glfw.get_key(window, glfw.KEY_DOWN) == glfw.PRESS:
        data.qvel[0] = -1.5 # 强行给它一个向后的速度

    # --- 2. 读取状态 ---
    car_pos = data.qpos[0]
    pole_angle = data.qpos[3]

    # --- 3. 双环控制 ---
    pos_target = pos_pid.compute(target_position, car_pos, dt)
    torque = angle_pid.compute(pos_target, pole_angle, dt)

    # --- 4. 电机输出 (保留你之前的负号，千万别动) ---
    data.ctrl[0] = -torque
    data.ctrl[1] = -torque

    # --- 5. 步进仿真 ---
    mujoco.mj_step(model, data)
    viewer.render()

viewer.close()

