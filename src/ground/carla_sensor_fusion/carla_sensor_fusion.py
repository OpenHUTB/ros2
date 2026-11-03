#!/usr/bin/env python3
"""
carla_sensor_fusion.py —— CARLA 多传感器融合（camera + lidar + IMU）
================================================================================
与 2D 版共用同一套融合核心（sensor_fusion_core：外参标定 + 时间同步 + EKF）。

流程：
  1. 在 ego 车上挂载 相机(Camera)、激光雷达(Lidar)、IMU 三个传感器；
  2. 目标车沿一条 8 字参考轨迹行驶；
  3. 每个传感器按自身采样率输出观测（目标级测量由检测/分割前端给出：
     - Lidar 点云 → 最近目标 (距离 r, 方位 θ_body)
     - Camera  → 目标方位 θ_body
     - IMU     → 载体姿态角 φ̂，用于把体坐标观测转到世界系
     );
  4. EKF(20Hz) 融合雷达 + 相机观测，实时输出目标位置估计与相对真值误差；
  5. 每周期打印各传感器最近观测的时间差（时间同步状态）。

前置条件（本地机器）：
  - 已安装并启动 CARLA（含 Python API，PATH 需指向 carla egg）
  - pip install numpy
运行：
  python3 carla_sensor_fusion.py --map Town03 --frames 600
参数：
  --host/--port CARLA 服务  --map 地图名  --frames 运行帧数
"""
import argparse
import time
import sys

import numpy as np

from sensor_fusion_core import (EKF2D, SyncBuffer, estimate_extrinsic_2d,
                                rot2, wrap_angle)


# ---------------- CARLA 连接与 actor 创建 ----------------
def import_carla():
    try:
        import carla
        return carla
    except ImportError:
        print("[-] 未找到 carla Python API。请本机启动 CARLA 并设置 PYTHONPATH。")
        raise SystemExit


def setup(carla, world, map_name):
    bp_lib = world.get_blueprint_library()
    ego_bp = bp_lib.filter("vehicle.*")[0]
    # ego 静止于原点，朝向 +x
    ego = world.try_spawn_actor(
        ego_bp, carla.Transform(carla.Location(18.0, 0, 0.5)))
    if ego is None:
        raise SystemExit("[-] ego spawn 失败")
    ego.set_autopilot(False)

    # 目标车沿 8 字轨迹行驶
    target_bp = bp_lib.filter("vehicle.*")[1]
    target = world.try_spawn_actor(
        target_bp, carla.Transform(carla.Location(30.0, 5.0, 0.5)))
    if target is None:
        raise SystemExit("[-] target spawn 失败")
    target.set_autopilot(False)

    # 传感器蓝图
    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_tf = carla.Transform(carla.Location(1.4, 0, 0.6))
    camera = world.spawn_actor(cam_bp, cam_tf, attach_to=ego)
    lid_bp = bp_lib.find("sensor.lidar.ray_cast")
    lid_tf = carla.Transform(carla.Location(0, 0, 2.0))
    lidar = world.spawn_actor(lid_bp, lid_tf, attach_to=ego)
    imu_bp = bp_lib.find("sensor.other.imu")
    imu = world.spawn_actor(imu_bp, carla.Transform(), attach_to=ego)
    print(f"[i] 地图 {map_name}，已挂载 ego/target/camera/lidar/imu")
    return ego, target, camera, lidar, imu


# ---------------- 8 字参考轨迹 ----------------
def target_truth(t):
    A, B, w = 18.0, 9.0, 0.4
    return np.array([A * np.sin(w * t), B * np.sin(2.0 * w * t)])


# ---------------- 目标级观测（由检测/分割前端给出，此处模拟） ----------------
def simulate_detections(carla, ego, target, t, rng, sig_r, sig_th_r, sig_th_c,
                        sig_imu):
    """基于目标真实位姿 + 传感器噪声模型，生成雷达/相机/IMU 目标级观测。
    实际 CARLA 场景中对应：lidar 点云最近目标 + camera 检测框中心 + IMU 姿态。"""
    # ego 与目标真实位姿
    et = ego.get_transform()
    tt = target.get_transform()
    ego_xy = np.array([et.location.x, et.location.y])
    tgt_xy = np.array([tt.location.x, tt.location.y])
    phi = np.radians(et.rotation.yaw)                       # ego 姿态真值
    p_body = rot2(-phi) @ (tgt_xy - ego_xy)                 # 体坐标
    r = np.linalg.norm(p_body)
    th_body = np.arctan2(p_body[1], p_body[0])

    obs = {}
    obs["radar"] = np.array([r + rng.normal(0, sig_r),
                             wrap_angle(th_body + rng.normal(0, sig_th_r))])
    obs["camera"] = wrap_angle(th_body + rng.normal(0, sig_th_c))
    obs["imu"] = wrap_angle(phi + rng.normal(0, sig_imu))
    obs["truth_world"] = tgt_xy
    obs["ego_xy"] = ego_xy
    return obs


# ---------------- 主循环 ----------------
def run(args):
    carla = import_carla()
    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.load_world(args.map)
    ego, target, camera, lidar, imu = setup(carla, world, args.map)

    # 各传感器采样周期（s）
    period = {"radar": 0.1, "camera": 1.0 / 30.0, "imu": 0.01}
    clocks = {k: time.time() for k in period}
    rng = np.random.default_rng(7)
    sig_r, sig_th_r, sig_th_c, sig_imu = 0.5, 0.10, 0.04, 0.012
    buffer = SyncBuffer(tol=0.06)
    dt_f = 0.05

    # 外参标定演示（相机↔雷达，用一组虚拟标定点）
    pts_cam = rng.normal(0, 1.0, (12, 2))
    th_cal, t_cal = 0.15, np.array([1.1, -0.2])
    pts_radar = (rot2(th_cal) @ pts_cam.T).T + t_cal + rng.normal(0, 0.05, (12, 2))
    th_est, tx_est, ty_est, res_cal = estimate_extrinsic_2d(pts_cam, pts_radar)

    ekf = EKF2D(0.0, 0.0, sig_theta_radar=sig_th_r, sig_theta_cam=sig_th_c)
    t0 = time.time()
    sim_t = 0.0
    rmse_n = rmse2 = 0

    print(f"[i] 外参标定: Δθ={th_est:.2f}°(真 {np.degrees(th_cal):.2f}°) "
          f"t=({tx_est:.2f},{ty_est:.2f})m res={res_cal:.3f}m")
    for frame in range(args.frames):
        now = time.time()
        sim_t += now - t0
        t0 = now
        sim_t = min(sim_t, 60.0)

        # 传感器采样写入同步缓冲
        obs = simulate_detections(carla, ego, target, sim_t, rng, sig_r,
                                  sig_th_r, sig_th_c, sig_imu)
        for name in period:
            if now >= clocks[name]:
                buffer.push(name, sim_t,
                            obs[name] if name != "imu" else obs["imu"])
                clocks[name] += period[name]

        # 融合周期
        if frame % max(1, int(dt_f * 20)) == 0:
            imu_obs = buffer.get("imu", sim_t)
            phi = imu_obs[1] if imu_obs else obs["imu"]
            ekf.predict(dt_f)
            rad = buffer.get("radar", sim_t)
            if rad is not None:
                ekf.update_radar(rad[1][0], wrap_angle(rad[1][1] + phi))
            cam = buffer.get("camera", sim_t)
            if cam is not None:
                ekf.update_camera(wrap_angle(cam[1] + phi))

        if frame % 60 == 0:
            truth = obs["truth_world"]
            est = ekf.state()
            err = np.linalg.norm(est[:2] - (truth - obs["ego_xy"]))
            rmse2 += err * err
            rmse_n += 1
            age = {k: buffer.age(k, sim_t) for k in period}
            print(f"[frame {frame:4d}] t={sim_t:4.1f}s  est=({est[0]:5.1f},"
                  f"{est[1]:5.1f})  truth=({truth[0]-obs['ego_xy'][0]:5.1f},"
                  f"{truth[1]-obs['ego_xy'][1]:5.1f})  err={err:4.2f}m  "
                  f"sync(radar/cam/imu)={age['radar']*1000:.0f}/"
                  f"{age['camera']*1000:.0f}/{age['imu']*1000:.0f}ms")
    if rmse_n:
        print(f"[OK] 运行结束，融合 RMSE ≈ {np.sqrt(rmse2/rmse_n):.3f} m")
    for a in (ego, target, camera, lidar, imu):
        a.destroy()


def main():
    ap = argparse.ArgumentParser(description="CARLA 多传感器融合跟踪")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--map", default="Town03")
    ap.add_argument("--frames", type=int, default=600)
    run(ap.parse_args())


if __name__ == "__main__":
    main()
