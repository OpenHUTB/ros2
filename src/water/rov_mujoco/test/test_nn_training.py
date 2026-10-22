#!/usr/bin/env python3
"""
任务 2 —— 神经网络训练有效性验证
==================================
验证“控制性能来自训练得到的权值”，而不是推理代码中的隐藏规则：

  1. rov_mujoco/nn_weights.npz 确实由 train/train_trajectory_policy.py 训练得到；
  2. 训练后的网络与 LOS 专家在随机状态上的动作高度一致 (行为克隆精度)；
  3. 零权值消融 (zero_weights=True) 输出恒为 0，闭环跟踪显著退化。

运行:
    ./main.sh --test-nn
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def load_modules():
    from rov_mujoco.mujoco_sim_node import MujocoSimNode
    from rov_mujoco.nn_trajectory_controller import (
        NNTrajectoryController, LOSTrajectoryController, TrackingEvaluator, STATE_SCALE,
    )
    from rov_mujoco.trajectory_generator import TrajectoryGenerator
    return MujocoSimNode, NNTrajectoryController, LOSTrajectoryController, \
        TrackingEvaluator, TrajectoryGenerator, STATE_SCALE


def test_expert_agreement(NNTrajectoryController, LOSTrajectoryController, STATE_SCALE):
    print("\n[1] 训练权值加载与专家一致性 ...")
    nn = NNTrajectoryController()
    los = LOSTrajectoryController()
    print(f"    权值来源: {nn.weights_source}")
    print(f"    weights_loaded={nn.weights_loaded}")
    assert nn.weights_loaded, "nn_weights.npz 未成功加载，仍在用兜底权值"
    assert nn.weights_source and nn.weights_source.endswith(".npz"), \
        f"应加载训练得到的 .npz，实际: {nn.weights_source}"

    rng = np.random.default_rng(7)
    n = 1000
    pos_err = np.column_stack([
        rng.uniform(-1.2, 1.2, n),
        rng.uniform(-1.2, 1.2, n),
        rng.uniform(-0.8, 0.8, n),
    ])
    vel_err = rng.uniform(-0.6, 0.6, (n, 3))
    yaw_err = rng.uniform(-math.pi, math.pi, n)
    vel_d = rng.uniform(-0.5, 0.5, (n, 2))

    out_nn, out_los = [], []
    # 以“期望位姿=原点”的方式复用两个控制器；速度按 err_vel=des_vel-cur_vel
    # 反解，保证采样点落在训练分布内（训练时 err_vel 与 vel_d 独立采样）。
    for i in range(n):
        cur_pos = -pos_err[i]
        des_pos = np.zeros(3)
        des_vel = np.array([vel_d[i, 0], vel_d[i, 1], 0.0])
        cur_vel = des_vel - vel_err[i]
        yaw = 0.0
        yaw_d = yaw_err[i]          # desired_yaw - current_yaw = yaw_err
        a = nn.compute(cur_pos, cur_vel, yaw, des_pos, des_vel, yaw_d)
        b = los.compute(cur_pos, cur_vel, yaw, des_pos, des_vel, yaw_d)
        out_nn.append([a["fx"], a["fy"], a["fz"], a["tau_z"]])
        out_los.append([b["fx"], b["fy"], b["fz"], b["tau_z"]])

    out_nn = np.array(out_nn)
    out_los = np.array(out_los)
    rmse = np.sqrt(np.mean((out_nn - out_los) ** 2, axis=0))
    corr = [float(np.corrcoef(out_nn[:, d], out_los[:, d])[0, 1]) for d in range(4)]
    print(f"    与 LOS 专家 RMSE: fx={rmse[0]:.2f} N, fy={rmse[1]:.2f} N, "
          f"fz={rmse[2]:.2f} N, tau={rmse[3]:.2f} N·m")
    print(f"    动作相关系数: {np.round(corr, 4)}")
    assert rmse[0] < 20.0 and rmse[1] < 20.0, "水平推力与专家偏差过大"
    assert rmse[2] < 25.0, "垂向推力与专家偏差过大"
    assert rmse[3] < 2.0, "偏航力矩与专家偏差过大"
    assert min(corr) > 0.99, "神经网络输出与专家动作相关性不足"

    # 权值不能是随机初始化：首层权值必须有训练后的结构
    w1_abs_mean = float(np.mean(np.abs(nn.W1)))
    print(f"    |W1| 均值 = {w1_abs_mean:.4f} (训练后应显著非零)")
    assert w1_abs_mean > 1e-3, "W1 近似全零，权值文件可能未训练"

    # 特征归一化系数必须与训练脚本一致
    assert np.allclose(STATE_SCALE[:3], 1.0 / 1.5), "STATE_SCALE 与训练脚本不一致"
    return nn


def run_closed_loop(MujocoSimNode, controller, steps=2500):
    """定点悬停消融：初始偏差 1.05 m，期望位姿固定在原点。

    定点工况没有轨迹前馈，网络输出是否真正产生纠偏推力会被直接放大；
    零权值网络输出恒为 0，ROV 应停在初始位置附近。
    """
    import mujoco

    sim_node = MujocoSimNode()
    sim_node.data.qpos[sim_node.qpos_addr:sim_node.qpos_addr + 3] = [0.9, 0.5, 0.3]
    sim_node.data.qpos[sim_node.qpos_addr + 3] = 1.0     # w = 1, yaw = 0
    sim_node.data.qpos[sim_node.qpos_addr + 4: sim_node.qpos_addr + 7] = 0.0
    sim_node.data.qvel[sim_node.model.jnt_dofadr[sim_node.joint_id]:
                       sim_node.model.jnt_dofadr[sim_node.joint_id] + 6] = 0.0
    mujoco.mj_forward(sim_node.model, sim_node.data)

    des_pos = np.zeros(3)
    des_vel = np.zeros(3)
    errors = []
    for _ in range(steps):
        pos_cur = sim_node.data.sensor("pos").data[:3].copy()
        vel_cur = sim_node.data.sensor("vel").data[:3].copy()
        quat = sim_node.data.qpos[sim_node.qpos_addr + 3: sim_node.qpos_addr + 7]
        yaw_cur = math.atan2(2.0 * (quat[0] * quat[3] + quat[1] * quat[2]),
                             1.0 - 2.0 * (quat[2] ** 2 + quat[3] ** 2))

        out = controller.compute(pos_cur, vel_cur, yaw_cur, des_pos, des_vel, 0.0)
        errors.append(float(np.linalg.norm(out["err_pos"])))

        sim_node.data.xfrc_applied[sim_node.body_id, :] = 0.0
        sim_node.data.xfrc_applied[sim_node.body_id, 2] = sim_node.buoyancy_force
        rot_mat = sim_node.data.xmat[sim_node.body_id].reshape((3, 3))
        f_world = rot_mat @ np.array([out["fx"], out["fy"], 0.0])
        sim_node.data.xfrc_applied[sim_node.body_id, 0] += f_world[0]
        sim_node.data.xfrc_applied[sim_node.body_id, 1] += f_world[1]
        sim_node.data.xfrc_applied[sim_node.body_id, 2] += out["fz"]
        sim_node.data.xfrc_applied[sim_node.body_id, 5] += out["tau_z"]

        ang_vel = sim_node.data.qvel[
            sim_node.model.jnt_dofadr[sim_node.joint_id] + 3:
            sim_node.model.jnt_dofadr[sim_node.joint_id] + 6]
        sim_node.data.xfrc_applied[sim_node.body_id, 3] = -200.0 * float(quat[1]) - 50.0 * float(ang_vel[0])
        sim_node.data.xfrc_applied[sim_node.body_id, 4] = -200.0 * float(quat[2]) - 50.0 * float(ang_vel[1])

        sim_node.model.opt.timestep = 0.002
        mujoco.mj_step(sim_node.model, sim_node.data)

    sim_node.destroy_node()
    return (float(np.sqrt(np.mean(np.square(errors)))),
            float(errors[-1]), float(np.max(errors)))


def main():
    import rclpy
    MujocoSimNode, NNTrajectoryController, LOSTrajectoryController, \
        TrackingEvaluator, TrajectoryGenerator, STATE_SCALE = load_modules()

    print("=" * 66)
    print("  任务 2: 神经网络训练有效性与零权值消融测试")
    print("=" * 66)

    rclpy.init()
    nn = test_expert_agreement(NNTrajectoryController, LOSTrajectoryController, STATE_SCALE)

    print("\n[2] 零权值消融 (zero_weights=True) ...")
    zero = NNTrajectoryController(zero_weights=True)
    assert zero.weights_source == "zero_ablation"
    act = zero.compute(np.zeros(3), np.zeros(3), 0.0,
                       np.ones(3), np.zeros(3), 0.0)["action"]
    print(f"    零权值网络输出: {np.round(act, 6)}")
    assert np.allclose(act, 0.0, atol=1e-8), "零权值网络输出应为 0"

    print("\n[3] 定点悬停闭环对比 (初始偏差 1.05 m, 2500 步 = 5 s) ...")
    rmse_nn, final_nn, max_nn = run_closed_loop(MujocoSimNode, nn)
    rmse_zero, final_zero, max_zero = run_closed_loop(MujocoSimNode, zero)
    print(f"    训练后网络 : RMSE={rmse_nn:.3f} m, 终点误差={final_nn:.3f} m, 最大偏差={max_nn:.3f} m")
    print(f"    零权值消融 : RMSE={rmse_zero:.3f} m, 终点误差={final_zero:.3f} m, 最大偏差={max_zero:.3f} m")
    print(f"    RMSE 退化倍数: {rmse_zero / max(rmse_nn, 1e-9):.1f}x")

    assert final_nn < 0.10, f"训练后网络未收敛到目标点: 终点误差 {final_nn:.3f} m"
    assert final_zero > 0.80, "零权值消融仍能收敛，闭环未真正使用网络输出"
    assert rmse_zero > 2.0 * rmse_nn, "零权值消融未显著退化，网络可能未被真正使用"

    print("\n✅ 迁移测试全部通过：性能来自训练权值，零权值网络会明显失效。")
    rclpy.shutdown()
    os._exit(0)


if __name__ == "__main__":
    main()
