#!/usr/bin/env python3
"""MuJoCo 滑移转向越野小车定点巡航仿真（主程序）。

加载同包内的 ``models/skid_robot_offroad.xml``，在随机起伏地形上闭环驶向目标点，
靠近目标自动减速停车；全程记录轨迹、车身姿态与四轮转速，结束后输出曲线图
``traj_plot.png``（保存在功能包根目录）。

用法::

    python3 scripts/main.py                  # 带 viewer 实时显示
    python3 scripts/main.py --headless       # 无窗口运行（服务器 / 无显卡环境）
    python3 scripts/main.py --stabilize      # 启用俯仰差速补偿（实验性，默认关闭）
"""

import argparse
import os
import sys
import time

import numpy as np
import mujoco

# realpath 解析符号链接：devel 空间里 lib/<包名>/main.py 是指向源码的软链，
# 只有还原出真实脚本路径，dirname 两次才落在源码包根目录而不是 devel/lib。
_HERE = os.path.dirname(os.path.realpath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from stable_controller import AttitudeStableController  # noqa: E402

PKG_DIR = os.path.dirname(_HERE)


def _find_model():
    """定位 XML 模型，兼容源码目录、devel 空间与 install 空间三种摆放。"""
    name = "skid_robot_offroad.xml"
    candidates = [
        # 源码 / devel 空间：<包根>/models/
        os.path.join(PKG_DIR, "models", name),
        # install 空间：<prefix>/share/<包名>/models/
        os.path.join(os.path.dirname(PKG_DIR), "share",
                     "mujoco_skid_steer_offroad_stable", "models", name),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
    return candidates[0]


DEFAULT_MODEL = _find_model()
PLOT_PATH = os.path.join(PKG_DIR, "traj_plot.png")

WHEEL_RADIUS = 0.14      # m
MAX_WHEEL_RAD = 8.0      # rad/s，轮速限幅
TARGET_X, TARGET_Y = 10.0, 0.0
STOP_THRESHOLD = 0.35    # m，进入该半径即停车
SLOWDOWN_RANGE = 2.0     # m，该距离内开始线性减速
CRUISE_SPEED = 0.8       # m/s，巡航速度上限
CRAWL_SPEED = 0.08       # m/s，最低爬行速度（避免低速被地面摩擦卡死）
SIM_TIME = 35.0          # s，仿真超时上限
CONTROL_DT = 0.002       # s，控制周期（每个周期算一次轮速指令）
RECORD_DT = 0.01         # s，采样周期（绘图数据）
TERRAIN_BASE_HEIGHT = 0.05  # m，地形基准高度
TERRAIN_ROUGHNESS = 0.06    # m，地形起伏标准差
TERRAIN_SEED = 42

YAW_GAIN = 1.2           # 航向误差 -> 左右轮差速（m/s per rad）
TURN_LIMIT = 0.30        # m/s，转向差速限幅


def _blur(field, kernel):
    """可分离卷积模糊（仅用于生成更自然的连续地形，不依赖 scipy）。"""
    for axis in (0, 1):
        field = np.apply_along_axis(
            lambda row: np.convolve(row, kernel, mode="same"), axis, field)
    return field


def apply_terrain(model, base_height=TERRAIN_BASE_HEIGHT,
                  roughness=TERRAIN_ROUGHNESS, seed=TERRAIN_SEED):
    """在高度场上叠加平滑随机起伏，返回实际地形高程（m）。"""
    nrow = int(model.hfield_nrow[0])
    ncol = int(model.hfield_ncol[0])
    size_z = float(model.hfield_size[0][2])
    rng = np.random.default_rng(seed)
    noise = rng.normal(0.0, 1.0, (nrow, ncol))
    noise = _blur(noise, np.hanning(25) / np.hanning(25).sum())   # 约 6 m 起伏波长
    noise *= roughness / float(noise.std())      # 归一化到设定的起伏标准差
    elevation = np.clip(base_height + noise, 0.0, size_z)
    model.hfield_data[:] = (elevation / size_z).ravel()
    return elevation


def quat_to_rpy(qpos):
    """由四元数 (qw, qx, qy, qz) 计算 roll / pitch / yaw（rad）。"""
    qw, qx, qy, qz = qpos
    roll = np.arctan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    pitch = np.arcsin(np.clip(2.0 * (qw * qy - qx * qz), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return roll, pitch, yaw


def plan_speed(distance):
    """距离 -> 目标线速度：远处巡航，近处线性减速，到位后停车。"""
    if distance <= STOP_THRESHOLD:
        return 0.0
    return float(np.clip(CRUISE_SPEED * distance / SLOWDOWN_RANGE, CRAWL_SPEED, CRUISE_SPEED))


def wheel_targets(base_speed, yaw_err, u_pitch):
    """由目标速度、航向误差、俯仰补偿量算出四轮目标角速度（rad/s）。"""
    turn = float(np.clip(YAW_GAIN * yaw_err, -TURN_LIMIT, TURN_LIMIT))
    v_fl = base_speed - turn + u_pitch
    v_rf = base_speed + turn + u_pitch
    v_rl = base_speed - turn - u_pitch
    v_rr = base_speed + turn - u_pitch
    return [float(np.clip(v / WHEEL_RADIUS, -MAX_WHEEL_RAD, MAX_WHEEL_RAD))
            for v in (v_fl, v_rf, v_rl, v_rr)]


def set_fixed_camera(viewer, model):
    """把 viewer 切到 XML 里定义的固定全局相机。"""
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "global_view")
    if cam_id >= 0:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        viewer.cam.fixedcamid = cam_id


def run_simulation(model, data, use_stabilizer=False, viewer=None, realtime=True,
                   controller=None):
    """执行一次完整仿真，返回记录数据（dict of ndarray）。"""
    ctrl = controller if controller is not None else AttitudeStableController()
    mujoco.mj_forward(model, data)

    rec = {k: [] for k in ("t", "x", "y", "v", "roll", "pitch",
                           "lf", "rf", "lb", "rb")}
    next_ctrl = 0.0
    next_record = 0.0
    stop_time = None
    t_wall = time.time()
    step = 0

    while data.time < SIM_TIME:
        x, y = data.qpos[0], data.qpos[1]
        roll, pitch, yaw = quat_to_rpy(data.qpos[3:7])
        speed = float(np.hypot(data.qvel[0], data.qvel[1]))
        dist = float(np.hypot(TARGET_X - x, TARGET_Y - y))

        if data.time >= next_ctrl:
            next_ctrl += CONTROL_DT
            base = plan_speed(dist)
            if base > 0.0:
                heading = np.arctan2(TARGET_Y - y, TARGET_X - x)
                yaw_err = np.arctan2(np.sin(heading - yaw), np.cos(heading - yaw))
                u_pitch = ctrl.compute(pitch, base) if use_stabilizer else 0.0
                ctrl_wheels = wheel_targets(base, yaw_err, u_pitch)
            else:
                ctrl_wheels = [0.0, 0.0, 0.0, 0.0]
            data.ctrl[:] = ctrl_wheels

        mujoco.mj_step(model, data)

        if data.time >= next_record:
            next_record += RECORD_DT
            rec["t"].append(data.time)
            rec["x"].append(x)
            rec["y"].append(y)
            rec["v"].append(speed)
            rec["roll"].append(np.rad2deg(roll))
            rec["pitch"].append(np.rad2deg(pitch))
            rec["lf"].append(data.ctrl[0])
            rec["rf"].append(data.ctrl[1])
            rec["lb"].append(data.ctrl[2])
            rec["rb"].append(data.ctrl[3])

        if stop_time is None and dist <= STOP_THRESHOLD and speed < 0.02:
            stop_time = data.time
            break

        step += 1
        if viewer is not None:
            if step % 10 == 0:
                viewer.sync()
            if realtime:
                expected = t_wall + data.time
                lag = expected - time.time()
                if lag > 0:
                    time.sleep(lag)

    out = {k: np.asarray(v, dtype=float) for k, v in rec.items()}
    out["final_dist"] = float(np.hypot(TARGET_X - data.qpos[0], TARGET_Y - data.qpos[1]))
    out["stop_time"] = float(stop_time) if stop_time is not None else float("nan")
    out["stabilized"] = bool(use_stabilizer)
    return out


def summarize(out):
    """打印关键指标，便于无窗口运行核对结果。"""
    def rms(v):
        return float(np.sqrt(np.mean(np.square(v))))
    print("=" * 52)
    print("仿真结束：t = {:.2f} s".format(out["t"][-1]))
    print("终点位置：x = {:.3f} m, y = {:.3f} m".format(out["x"][-1], out["y"][-1]))
    print("距目标点：{:.3f} m（停车阈值 {:.2f} m）".format(out["final_dist"], STOP_THRESHOLD))
    print("到位停车时刻：{}".format("未触发" if np.isnan(out["stop_time"])
                                      else "{:.2f} s".format(out["stop_time"])))
    print("横滚 RMS = {:.2f}°  峰值 {:.2f}°".format(rms(out["roll"]), np.abs(out["roll"]).max()))
    print("俯仰 RMS = {:.2f}°  峰值 {:.2f}°".format(rms(out["pitch"]), np.abs(out["pitch"]).max()))
    print("姿态补偿：{}".format("开启（实验性）" if out["stabilized"] else "关闭（默认）"))
    print("=" * 52)


def plot_results(out, path=PLOT_PATH, show=True):
    """绘制轨迹、位置、姿态、轮速四联图，同时保存为 PNG。"""
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams["font.sans-serif"] = ["DejaVu Sans"]
    fig, axs = plt.subplots(4, 1, figsize=(11, 12), constrained_layout=True)

    axs[0].plot(out["x"], out["y"], linewidth=1.5, label="Robot Trajectory")
    axs[0].plot(TARGET_X, TARGET_Y, "ro", markersize=10, label="Target Point")
    axs[0].set_title("XY Trajectory (Off-road Cruise)")
    axs[0].set_xlabel("X [m]")
    axs[0].set_ylabel("Y [m]")
    axs[0].axis("equal")
    axs[0].grid(True)
    axs[0].legend()

    axs[1].plot(out["t"], out["x"], label="X Position")
    axs[1].plot(out["t"], out["y"], label="Y Position", color="orange")
    axs[1].set_title("Position Over Time")
    axs[1].set_xlabel("Time [s]")
    axs[1].set_ylabel("Position [m]")
    axs[1].grid(True)
    axs[1].legend()

    axs[2].plot(out["t"], out["roll"], label="Roll Angle", color="red")
    axs[2].plot(out["t"], out["pitch"], label="Pitch Angle", color="green")
    axs[2].set_title("Body Roll / Pitch Angle")
    axs[2].set_xlabel("Time [s]")
    axs[2].set_ylabel("Angle [deg]")
    axs[2].grid(True)
    axs[2].legend()

    axs[3].plot(out["t"], out["lf"], label="LF Wheel")
    axs[3].plot(out["t"], out["rf"], label="RF Wheel")
    axs[3].plot(out["t"], out["lb"], label="LB Wheel")
    axs[3].plot(out["t"], out["rb"], label="RB Wheel")
    axs[3].set_title("Four-Wheel Independent Rotation Speed")
    axs[3].set_xlabel("Time [s]")
    axs[3].set_ylabel("Angular Velocity [rad/s]")
    axs[3].grid(True)
    axs[3].legend()

    fig.savefig(path, dpi=150)
    print("曲线图已保存：{}".format(path))
    if show:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="MuJoCo 滑移转向越野小车定点巡航仿真")
    parser.add_argument("--model", default=DEFAULT_MODEL, help="MuJoCo XML 模型路径")
    parser.add_argument("--headless", action="store_true", help="不打开 viewer，直接跑完出图")
    parser.add_argument("--stabilize", action="store_true",
                        help="启用俯仰差速补偿（实验性功能，默认关闭）")
    parser.add_argument("--no-plot", action="store_true", help="不生成曲线图")
    parser.add_argument("--plot-path", default=PLOT_PATH, help="曲线图保存路径")
    args = parser.parse_args()

    if not os.path.isfile(args.model):
        raise SystemExit("找不到模型文件：{}".format(args.model))

    model = mujoco.MjModel.from_xml_path(os.path.abspath(args.model))
    data = mujoco.MjData(model)
    apply_terrain(model)
    print("模型加载完成：nq={}, nv={}, nu={}".format(model.nq, model.nv, model.nu))

    if args.headless:
        out = run_simulation(model, data, use_stabilizer=args.stabilize, realtime=False)
        summarize(out)
        if not args.no_plot:
            plot_results(out, path=args.plot_path, show=False)
    else:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            set_fixed_camera(viewer, model)
            out = run_simulation(model, data, use_stabilizer=args.stabilize,
                                 viewer=viewer)
        summarize(out)
        if not args.no_plot:
            plot_results(out, path=args.plot_path, show=True)


if __name__ == "__main__":
    main()
