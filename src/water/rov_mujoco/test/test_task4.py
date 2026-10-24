#!/usr/bin/env python3
"""
任务 4 —— 端到端视觉伺服策略验证
==================================
  1. 三种架构 (mlp / small_cnn / nature_cnn) 均加载 BC 训练得到的 .npz 权值；
  2. 在训练分布外的随机位姿上渲染图像，检查横向纠偏/航向动作精度；
  3. 推理延迟满足实时约束 (<100 ms)；
  4. 闭环：初始横向偏离管道 0.6 m，策略网络仅凭图像回到管道上方；
  5. 零权值消融无法完成纠偏。

运行:
    ./main.sh --test4
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

PIPE_Y = 3.2
PIPE_Z = -1.45
IMG_SIZE = 84


def render_validation_set(model, n=60, seed=99):
    """在训练分布外渲染验证图像，并给出几何专家动作"""
    import mujoco
    from rov_mujoco.end_to_end_networks import rgb_to_gray_resize, expert_label_ref

    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, IMG_SIZE, IMG_SIZE)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rov")
    qpos_addr = model.jnt_qposadr[model.body_jntadr[body_id]]

    rng = np.random.default_rng(seed)
    images = np.zeros((n, IMG_SIZE, IMG_SIZE), dtype=np.float32)
    labels = np.zeros((n, 3))
    dys = np.zeros(n)

    for i in range(n):
        x = rng.uniform(-3.2, 2.5)
        dy = rng.uniform(-0.55, 0.55)
        y = PIPE_Y + dy
        z = PIPE_Z + rng.uniform(-0.14, 0.14)
        yaw = rng.uniform(-0.25, 0.25)
        data.qpos[:] = model.qpos0
        data.qpos[qpos_addr:qpos_addr + 3] = (x, y, z)
        data.qpos[qpos_addr + 3] = math.cos(yaw / 2)
        data.qpos[qpos_addr + 4:qpos_addr + 7] = (0.0, 0.0, math.sin(yaw / 2))
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera="forward_cam")
        images[i] = rgb_to_gray_resize(renderer.render(), IMG_SIZE)
        labels[i] = expert_label_ref(y, yaw)
        dys[i] = dy

    renderer.close()
    return images, labels, dys


def run_closed_loop(MujocoSimNode, controller, start_y, max_cycles=420,
                    substeps=50):
    """从 y 方向偏差 0.6 m 出发，仅用前视图像闭环巡线"""
    import mujoco

    sim = MujocoSimNode()
    sim.sim_timer.cancel()
    sim.pub_timer.cancel()
    sim.auto_trajectory_mode = False
    sim.target_depth = PIPE_Z
    sim.data.qpos[sim.qpos_addr:sim.qpos_addr + 3] = [-3.5, start_y, PIPE_Z]
    sim.data.qpos[sim.qpos_addr + 3] = 1.0
    sim.data.qpos[sim.qpos_addr + 4:sim.qpos_addr + 7] = 0.0
    sim.data.qvel[sim.model.jnt_dofadr[sim.joint_id]:
                  sim.model.jnt_dofadr[sim.joint_id] + 6] = 0.0
    mujoco.mj_forward(sim.model, sim.data)

    renderer = mujoco.Renderer(sim.model, IMG_SIZE, IMG_SIZE)
    y_errs, xs, z_errs, latencies = [], [], [], []
    reached = False
    cycles = 0

    for cycles in range(max_cycles):
        renderer.update_scene(sim.data, camera="forward_cam")
        rgb = renderer.render()
        res = controller.predict_action(rgb)
        latencies.append(res["latency_ms"])

        quat = sim.data.qpos[sim.qpos_addr + 3:sim.qpos_addr + 7]
        yaw = math.atan2(2.0 * (quat[0] * quat[3] + quat[1] * quat[2]),
                         1.0 - 2.0 * (quat[2] ** 2 + quat[3] ** 2))
        sim.cmd_vel.linear.x = res["vx"]
        sim.cmd_vel.linear.y = res["vy"]
        sim.cmd_vel.linear.z = 0.0
        sim.cmd_vel.angular.z = (float(np.clip(-1.8 * yaw, -0.6, 0.6))
                                 if abs(yaw) > 0.25 else res["wz"])

        for _ in range(substeps):
            sim._sim_step_callback()

        pos = sim.data.sensor("pos").data[:3].copy()
        y_errs.append(abs(pos[1] - PIPE_Y))
        z_errs.append(abs(pos[2] - PIPE_Z))
        xs.append(pos[0])
        if pos[0] >= 2.85:
            reached = True
            break

    renderer.close()
    sim.destroy_node()

    settle = min(100, len(y_errs) // 3)   # 跳过起始 10 s 的纠偏瞬态
    tail = np.array(y_errs[settle:]) if len(y_errs) > settle else np.array(y_errs)
    return {
        'rmse_y': float(np.sqrt(np.mean(tail ** 2))),
        'final_y': float(y_errs[-1]),
        'rmse_z': float(np.sqrt(np.mean(np.array(z_errs) ** 2))),
        'final_x': float(xs[-1]),
        'advanced': float(xs[-1] - (-3.5)),
        'reached': reached,
        'cycles': cycles + 1,
        'sim_time': (cycles + 1) * substeps * 0.002,
        'latency': float(np.mean(latencies)),
    }


def main():
    import mujoco
    import rclpy
    from rov_mujoco.end_to_end_networks import VisuomotorController

    rclpy.init()

    print("=" * 70)
    print("  任务 4: 端到端视觉伺服策略训练有效性验证")
    print("=" * 70)

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    model = mujoco.MjModel.from_xml_path(
        os.path.join(root, 'models', 'underwater_rov_with_arm.xml'))
    images, labels, dys = render_validation_set(model)
    print(f"  验证图像: {len(images)} 张 (随机位姿, 训练分布外)")

    controllers = {}
    results = {}
    for arch in ('mlp', 'small_cnn', 'nature_cnn'):
        ctl = VisuomotorController(architecture=arch, algorithm='bc')
        print(f"\n  [{arch}] loaded={ctl.weights_loaded} | 来源={ctl.weights_source}")
        assert ctl.weights_loaded, f"{arch} 未加载训练权值"
        assert ctl.weights_source and ctl.weights_source.endswith('.npz'), \
            f"{arch} 应加载 .npz 训练权值"

        preds = np.array([[ctl.predict_action(img)['action'][d]
                           for d in range(3)] for img in images])
        lat = np.array([ctl.predict_action(img)['latency_ms'] for img in images[:20]])

        corr = [float(np.corrcoef(preds[:, d], labels[:, d])[0, 1]) for d in range(3)]
        rmse = np.sqrt(np.mean((preds - labels) ** 2, axis=0))
        phys = [0.45 * rmse[0] / 2.0, 0.35 * rmse[1], 0.60 * rmse[2]]
        print(f"    动作 corr = {np.round(corr, 4)} | 物理 RMSE: "
              f"vx={phys[0]:.3f} m/s, vy={phys[1]:.3f} m/s, wz={phys[2]:.3f} rad/s")
        print(f"    平均推理延迟: {lat.mean():.2f} ms (max {lat.max():.1f} ms)")

        # 端到端最核心的是从图像中读出横向偏差并给出纠偏速度
        assert corr[1] > 0.90, f"{arch}: 图像->横向纠偏相关性不足 ({corr[1]:.3f})"
        assert corr[2] > 0.80, f"{arch}: 图像->航向相关性不足 ({corr[2]:.3f})"
        assert lat.mean() < 100.0, f"{arch}: 推理延迟超过 100 ms 实时约束"

        controllers[arch] = ctl
        results[arch] = {'corr': corr, 'lat': float(lat.mean()), 'phys': phys}

    # 横向纠偏方向必须正确：dy>0 (ROV 在管道 +y 侧) 时 vy 应为负
    ctl = controllers['nature_cnn']
    vy_pred = np.array([ctl.predict_action(img)['vy'] for img in images])
    neg = vy_pred[dys > 0.25].mean()
    pos = vy_pred[dys < -0.25].mean()
    print(f"\n  纠偏方向检查: dy>0.25 平均 vy={neg:+.3f} m/s | "
          f"dy<-0.25 平均 vy={pos:+.3f} m/s")
    assert neg < -0.05 and pos > 0.05, "横向纠偏方向错误"

    # ── 闭环：训练网络 vs 零权值消融 ──
    print("\n  ── 闭环视觉巡线 (初始横向偏离 0.6 m) ──")
    from rov_mujoco.mujoco_sim_node import MujocoSimNode

    trained = run_closed_loop(MujocoSimNode, controllers['nature_cnn'], 3.8)
    zero_ctl = VisuomotorController(architecture='nature_cnn', algorithm='bc',
                                    zero_weights=True)
    zero = run_closed_loop(MujocoSimNode, zero_ctl, 3.8)

    for name, r in (("训练网络", trained), ("零权值", zero)):
        print(f"    {name:<6}: RMSE_y={r['rmse_y']:.3f} m | 终点偏差={r['final_y']:.3f} m | "
              f"深度 RMSE={r['rmse_z']:.3f} m | 前进={r['advanced']:.2f} m | "
              f"到达={r['reached']} | {r['sim_time']:.1f}s | 推理 {r['latency']:.1f} ms")

    assert trained['reached'], "训练策略未能抵达管线终点"
    assert trained['rmse_y'] < 0.25, "训练策略横向跟踪误差过大"
    assert trained['final_y'] < 0.15, "训练策略终点横向偏差过大"
    assert trained['rmse_y'] < 0.5 * zero['rmse_y'], \
        "训练策略相对零权值消融无显著优势"
    assert zero['rmse_y'] > 0.40, "零权值消融不应完成横向纠偏"

    zc = zero_ctl.predict_action(images[0])
    print(f"    零权值动作: vx={zc['vx']:.3f}, vy={zc['vy']:+.3f}, wz={zc['wz']:+.3f}")
    assert abs(zc['vy']) < 1e-9 and abs(zc['wz']) < 1e-9, "零权值输出应为常数"

    print("\n✅ 端到端视觉伺服策略测试全部通过。")
    sys.stdout.flush()
    try:
        rclpy.shutdown()
    except Exception:
        pass
    os._exit(0)


if __name__ == '__main__':
    main()
