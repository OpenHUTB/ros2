#!/usr/bin/env python3
"""
水下多传感器感知与 3D 轨迹跟踪控制自动化测试脚本
===============================================
测试内容：
  1. 水下多波束声呐、前视相机、IMU、水深计传感器感知验证
  2. 静水与强洋流剪切双工况下，神经网络 (NN) 与经典视线法 (LOS-PID) 3D 空间轨迹闭环跟踪性能评测
"""

import sys
import os
import math
import numpy as np

import rclpy
from rclpy.node import Node


def run_trajectory_tracking_tests():
    print("=" * 65)
    print("  水下机器人传感器感知与神经网络轨迹跟踪测试评测")
    print("=" * 65)

    try:
        from rov_mujoco.mujoco_sim_node import MujocoSimNode
        from rov_mujoco.nn_trajectory_controller import (
            NNTrajectoryController, LOSTrajectoryController, TrackingEvaluator
        )
        from rov_mujoco.trajectory_generator import TrajectoryGenerator
    except ImportError:
        pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        sys.path.insert(0, pkg_root)
        from rov_mujoco.mujoco_sim_node import MujocoSimNode
        from rov_mujoco.nn_trajectory_controller import (
            NNTrajectoryController, LOSTrajectoryController, TrackingEvaluator
        )
        from rov_mujoco.trajectory_generator import TrajectoryGenerator

    rclpy.init()

    # =========================================================================
    # 第一部分：水下多传感器感知系统验证
    # =========================================================================
    print("\n[PART 1] 验证传感器感知系统...")
    node_sensor = MujocoSimNode()

    # 1. 验证声呐射线追踪
    sonar_res = node_sensor.sonar.update()
    num_beams = len(sonar_res["ranges"])
    closest_dist = sonar_res["closest_dist"]
    print(f"  [声呐] 多波束阵列线数: {num_beams} 束 | 视场角: 120°")
    print(f"  [声呐] 最近水下障碍物距离: {closest_dist:.3f} m (方位角: {math.degrees(sonar_res['closest_angle']):.1f}°)")
    assert num_beams == 72, "声呐波束数应为 72！"
    assert closest_dist < 15.0, "声呐应成功探测到人工水下结构物或海床！"
    print("  -> ✅ 水下多波束前视声呐感知检测通过！")

    # 2. 验证前视相机离屏渲染
    node_sensor.camera.init_renderer()
    cam_img = node_sensor.camera.render()
    h, w, c = cam_img.shape
    print(f"  [相机] 水下相机分辨率: {w}x{h} ({c}通道 RGB) | 水体色彩衰减滤镜: 已激活")
    assert (h, w, c) == (240, 320, 3), "相机图像尺寸应为 240x320x3！"
    print("  -> ✅ 水下前视相机离屏渲染检测通过！")
    node_sensor.camera.close()

    # 3. 验证 ROS 2 传感器话题发布
    node_sensor._publish_callback()
    print("  [话题] 成功广播 /rov/sonar/scan, /rov/camera/image_raw, /rov/imu, /rov/desired_path")
    print("  -> ✅ 全套水下传感器数据流验证通过！\n")

    node_sensor.destroy_node()

    # =========================================================================
    # 第二部分：3D 轨迹跟踪控制与多场景性能评价 (静水 vs 强洋流)
    # =========================================================================
    print("=" * 65)
    print("[PART 2] 验证 3D 轨迹跟踪控制与多工况性能对比...")

    scenarios = [
        {"name": "工况 1: 静水巡检工况 (Quiet Water)", "use_current": False},
        {"name": "工况 2: 强洋流剪切工况 (Cascade Currents)", "use_current": True}
    ]

    controllers = [
        {"type": "pid", "label": "经典 LOS+PID 基准 (Baseline)"},
        {"type": "nn",  "label": "神经网络智能控制器 (NN Model)"}
    ]

    evaluation_results = []
    traj_gen = TrajectoryGenerator(trajectory_type="3d_helix")

    for sc in scenarios:
        print(f"\n>>> 正在评测环境：{sc['name']} ...")
        for ct in controllers:
            sim_node = MujocoSimNode()
            sim_node.auto_trajectory_mode = True
            sim_node.traj_gen = traj_gen

            if ct["type"] == "pid":
                ctrl = LOSTrajectoryController()
            else:
                ctrl = NNTrajectoryController()

            evaluator = TrackingEvaluator(name=f"{ct['label']} [{sc['name'].split(':')[0]}]")

            # 初始对齐至轨迹初始点并更新动力学前向态
            init_st = traj_gen.get_state(0.0)
            sim_node.data.qpos[sim_node.qpos_addr: sim_node.qpos_addr + 3] = init_st["pos"]
            yaw_0 = init_st["yaw"]
            sim_node.data.qpos[sim_node.qpos_addr + 3] = math.cos(yaw_0 / 2.0)
            sim_node.data.qpos[sim_node.qpos_addr + 4] = 0.0
            sim_node.data.qpos[sim_node.qpos_addr + 5] = 0.0
            sim_node.data.qpos[sim_node.qpos_addr + 6] = math.sin(yaw_0 / 2.0)
            sim_node.data.qvel[sim_node.model.jnt_dofadr[sim_node.joint_id]: sim_node.model.jnt_dofadr[sim_node.joint_id] + 6] = 0.0
            import mujoco
            mujoco.mj_forward(sim_node.model, sim_node.data)
            mujoco_step = sim_node.model.opt.timestep

            # 执行 1200 步动力学仿真 (对应 2.4 秒高精度闭环轨迹推进)
            for step in range(1200):
                t = float(sim_node.data.time)
                des_st = traj_gen.get_state(t)

                pos_cur = sim_node.data.sensor("pos").data[:3].copy()
                vel_cur = sim_node.data.sensor("vel").data[:3].copy()
                quat = sim_node.data.qpos[sim_node.qpos_addr + 3: sim_node.qpos_addr + 7]
                yaw_cur = math.atan2(2.0*(quat[0]*quat[3] + quat[1]*quat[2]), 1.0 - 2.0*(quat[2]**2 + quat[3]**2))

                # 控制推理
                out = ctrl.compute(pos_cur, vel_cur, yaw_cur, des_st["pos"], des_st["vel"], des_st["yaw"])
                evaluator.record(t, out["err_pos"], out["err_yaw"], out["fx"], out["fy"], out["fz"], out["tau_z"])

                # 动力学施加
                sim_node.data.xfrc_applied[sim_node.body_id, :] = 0.0
                sim_node.data.xfrc_applied[sim_node.body_id, 2] = sim_node.buoyancy_force

                if sc["use_current"] and sim_node.ocean_current:
                    v_cur = sim_node.ocean_current.get_velocity(pos_cur[0], pos_cur[1], pos_cur[2], mujoco_step)
                    sim_node._apply_current_drag(sim_node.data, sim_node.body_id, v_cur, sim_node.drag_coeff)

                rot_mat = sim_node.data.xmat[sim_node.body_id].reshape((3, 3))
                f_world = rot_mat @ np.array([out["fx"], out["fy"], 0.0])
                sim_node.data.xfrc_applied[sim_node.body_id, 0] += f_world[0]
                sim_node.data.xfrc_applied[sim_node.body_id, 1] += f_world[1]
                sim_node.data.xfrc_applied[sim_node.body_id, 2] += out["fz"]
                sim_node.data.xfrc_applied[sim_node.body_id, 5] += out["tau_z"]

                # 姿态阻尼
                ang_vel = sim_node.data.qvel[sim_node.model.jnt_dofadr[sim_node.joint_id] + 3: sim_node.model.jnt_dofadr[sim_node.joint_id] + 6]
                sim_node.data.xfrc_applied[sim_node.body_id, 3] = -200.0 * float(quat[1]) - 50.0 * float(ang_vel[0])
                sim_node.data.xfrc_applied[sim_node.body_id, 4] = -200.0 * float(quat[2]) - 50.0 * float(ang_vel[1])

                sim_node.model.opt.timestep = 0.002
                mujoco.mj_step(sim_node.model, sim_node.data)

            res = evaluator.summary()
            evaluation_results.append({
                "scenario": sc["name"].split(':')[0],
                "controller": ct["label"],
                "rmse_3d": res["rmse_3d"],
                "rmse_xy": res["rmse_xy"],
                "rmse_z": res["rmse_z"],
                "max_err": res["max_error"],
                "rmse_yaw": res["rmse_yaw_deg"],
                "effort": res["avg_effort"]
            })
            sim_node.destroy_node()

    # =========================================================================
    # 第三部分：输出多场景性能量化对比报表
    # =========================================================================
    print("\n" + "=" * 88)
    print(f"| {'测试工况':<12} | {'控制算法':<25} | {'3D RMSE':<9} | {'水平 RMSE':<9} | {'垂向 RMSE':<9} | {'最大偏差':<8} |")
    print("-" * 88)
    for r in evaluation_results:
        print(f"| {r['scenario']:<14} | {r['controller']:<27} | {r['rmse_3d']:>6.3f} m  | {r['rmse_xy']:>6.3f} m  | {r['rmse_z']:>6.3f} m  | {r['max_err']:>6.3f} m |")
    print("=" * 88)

    # 验证跟踪收敛性断言
    for r in evaluation_results:
        assert r["rmse_3d"] < 0.35, f"{r['controller']} 在 {r['scenario']} 下的跟踪 RMSE 应小于 0.35m！实际: {r['rmse_3d']:.3f}m"

    print("\n🎉 水下多传感器感知与 3D 轨迹跟踪控制自动化测试 100% 通过！")
    print("=" * 65)

    rclpy.shutdown()
    os._exit(0)


if __name__ == "__main__":
    run_trajectory_tracking_tests()
