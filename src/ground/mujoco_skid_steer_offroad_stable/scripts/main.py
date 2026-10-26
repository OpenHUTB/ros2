import mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt
import time

model_path = "/home/jzy1/Desktop/mujoco_skid_steer_offroad_stable/models/skid_robot_offroad.xml"
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

sim_time = 35.0
target_x, target_y = 10.0, 0.0
wheel_radius = 0.14
max_wheel_rad = 8.0

stop_threshold = 0.35
slowdown_range = 2.0

# 姿态稳定P参数
Kp_roll = 2.2
Kp_pitch = 2.2

# IMU噪声
gyro_std = 0.018
accel_std = 0.09

# 日志数组
t_list = []
x_list, y_list = [], []
roll_list, pitch_list = [], []
lf_spd_list, rf_spd_list = [], []
lb_spd_list, rb_spd_list = [], []

dt = model.opt.timestep
realtime_factor = 1.0

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.distance = 16
    viewer.cam.elevation = -30
    viewer.cam.azimuth = -90
    viewer.cam.lookat[:] = [6, 0, 0]
    render_skip = 10
    render_cnt = 0
    t_start = time.time()

    # 生成随机起伏地形
    np.random.seed(42)
    hfield_data = model.hfield_data.reshape(128,128)
    noise = np.random.normal(0, 0.06, hfield_data.shape)
    model.hfield_data[:] = (hfield_data + noise).ravel()

    while viewer.is_running() and data.time < sim_time:
        mujoco.mj_step(model, data)

        # 车身位姿、四元数
        x = data.qpos[0]
        y = data.qpos[1]
        qw, qx, qy, qz = data.qpos[3:7]

        # 欧拉角
        roll  = np.arctan2(2*(qw*qx + qy*qz), 1-2*(qx**2+qy**2))
        pitch = np.arcsin(2*(qw*qy - qx*qz))
        yaw   = np.arctan2(2*(qw*qz + qx*qy), 1-2*(qy**2+qz**2))

        # =========【重点兼容修改：老MuJoCo用sensordata切片】=========
        gyro_raw = data.sensordata[0:3]
        accel_raw = data.sensordata[3:6]
        gyro_noisy = gyro_raw + np.random.normal(0, gyro_std, 3)
        accel_noisy = accel_raw + np.random.normal(0, accel_std, 3)

        roll_ctrl  = -Kp_roll * roll
        pitch_ctrl = -Kp_pitch * pitch

        dx = target_x - x
        dy = target_y - y
        dist_err = np.hypot(dx, dy)

        if dist_err < stop_threshold:
            lf, rf, lb, rb = 0.0, 0.0, 0.0, 0.0
        else:
            base_speed = np.clip(0.8 * (dist_err / slowdown_range), 0.15, 0.8)
            yaw_des = np.arctan2(dy, dx)
            yaw_err = np.arctan2(np.sin(yaw_des - yaw), np.cos(yaw_des - yaw))
            steer_gain = 1.2
            turn_eff = steer_gain * yaw_err

            # 四轮独立 + 姿态补偿
            lf = (base_speed - turn_eff) + pitch_ctrl*0.12
            rf = (base_speed + turn_eff) - pitch_ctrl*0.12
            lb = (base_speed - turn_eff) + roll_ctrl*0.12
            rb = (base_speed + turn_eff) - roll_ctrl*0.12

            lf = np.clip(lf/wheel_radius, -max_wheel_rad, max_wheel_rad)
            rf = np.clip(rf/wheel_radius, -max_wheel_rad, max_wheel_rad)
            lb = np.clip(lb/wheel_radius, -max_wheel_rad, max_wheel_rad)
            rb = np.clip(rb/wheel_radius, -max_wheel_rad, max_wheel_rad)

        data.ctrl[0] = lf
        data.ctrl[1] = rf
        data.ctrl[2] = lb
        data.ctrl[3] = rb

        render_cnt += 1
        if render_cnt >= render_skip:
            t_list.append(data.time)
            x_list.append(x)
            y_list.append(y)
            roll_list.append(np.rad2deg(roll))
            pitch_list.append(np.rad2deg(pitch))
            lf_spd_list.append(lf)
            rf_spd_list.append(rf)
            lb_spd_list.append(lb)
            rb_spd_list.append(rb)

            viewer.sync()
            render_cnt = 0
            expected = t_start + data.time / realtime_factor
            now = time.time()
            if now < expected:
                time.sleep(expected - now)

print("Simulation finished, generating full plot...")
plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
fig, axs = plt.subplots(4, 1, figsize=(12,12))

axs[0].plot(x_list, y_list, label="Robot Trajectory", linewidth=1.5)
axs[0].plot(target_x, target_y, "ro", markersize=10, label="Target Point")
axs[0].set_title("XY Trajectory (Off-road Cruise)")
axs[0].set_xlabel("X [m]")
axs[0].set_ylabel("Y [m]")
axs[0].axis("equal")
axs[0].grid(True)
axs[0].legend()

axs[1].plot(t_list, x_list, label="X Position")
axs[1].plot(t_list, y_list, label="Y Position", color="orange")
axs[1].set_title("Position Over Time")
axs[1].set_ylabel("Position [m]")
axs[1].grid(True)
axs[1].legend()

axs[2].plot(t_list, roll_list, label="Roll Angle", color="red")
axs[2].plot(t_list, pitch_list, label="Pitch Angle", color="green")
axs[2].set_title("Body Roll / Pitch Angle (Stabilization Effect)")
axs[2].set_ylabel("Angle [deg]")
axs[2].grid(True)
axs[2].legend()

axs[3].plot(t_list, lf_spd_list, label="LF Wheel")
axs[3].plot(t_list, rf_spd_list, label="RF Wheel")
axs[3].plot(t_list, lb_spd_list, label="LB Wheel")
axs[3].plot(t_list, rb_spd_list, label="RB Wheel")
axs[3].set_title("Four-Wheel Independent Rotation Speed")
axs[3].set_xlabel("Time [s]")
axs[3].set_ylabel("Angular Velocity [rad/s]")
axs[3].grid(True)
axs[3].legend()

plt.tight_layout()
plt.show()

