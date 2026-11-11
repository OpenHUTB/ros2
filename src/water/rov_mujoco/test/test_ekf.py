#!/usr/bin/env python3
"""
水下多源融合导航 (EKF) 自动化评测
==================================
同步驱动 MuJoCo 仿真（不走 ROS 定时器，速度快、可重复），
在 50 Hz 下合成 IMU/DVL/深度/姿态量测，对比：
    1. 纯 IMU 航位推算 (Dead Reckoning)
    2. EKF 融合 (IMU + DVL + 深度 + 姿态)
并与仿真真值 (GT) 比较，输出 RMSE / 漂移 / NEES。

用法:
    python3 test/test_ekf.py --duration 60 --out /tmp/ekf_eval.json
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np


def _add_src_path():
    here = os.path.dirname(os.path.abspath(__file__))
    pkg_root = os.path.dirname(here)
    if pkg_root not in sys.path:
        sys.path.insert(0, pkg_root)


def quat_diff_omega(q_prev, q_now, dt):
    """由相邻姿态求机体角速度。"""
    q_prev = np.asarray(q_prev, dtype=float)
    q_now = np.asarray(q_now, dtype=float)
    q_inv = np.array([q_prev[0], -q_prev[1], -q_prev[2], -q_prev[3]])
    w, x, y, z = np.array([
        q_inv[0] * q_now[0] - q_inv[1] * q_now[1] - q_inv[2] * q_now[2] - q_inv[3] * q_now[3],
        q_inv[0] * q_now[1] + q_inv[1] * q_now[0] + q_inv[2] * q_now[3] - q_inv[3] * q_now[2],
        q_inv[0] * q_now[2] - q_inv[1] * q_now[3] + q_inv[2] * q_now[0] + q_inv[3] * q_now[1],
        q_inv[0] * q_now[3] + q_inv[1] * q_now[2] - q_inv[2] * q_now[1] + q_inv[3] * q_now[0],
    ])
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-9:
        return np.zeros(3)
    angle = 2.0 * math.atan2(n, w)
    return np.array([x, y, z]) / n * angle / dt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--warmup', type=float, default=6.0)
    ap.add_argument('--dropout', type=float, default=0.10)
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--current', type=float, default=1.0,
                    help='1 = 启用配置里的三层洋流；0 = 关闭洋流')
    ap.add_argument('--facc', type=float, default=0.60, help='滤波器过程噪声 sigma_acc')
    ap.add_argument('--fgyro', type=float, default=0.04, help='滤波器过程噪声 sigma_gyro')
    ap.add_argument('--fdvl', type=float, default=0.04, help='DVL 量测噪声')
    ap.add_argument('--fdepth', type=float, default=0.06, help='深度量测噪声')
    ap.add_argument('--fatt', type=float, default=0.02, help='姿态量测噪声')
    ap.add_argument('--out', default='/tmp/ekf_eval.json')
    args = ap.parse_args()

    _add_src_path()
    try:
        from rov_mujoco.mujoco_sim_node import MujocoSimNode
        from rov_mujoco.ekf_fusion import ROVEKF, SpaceState, SensorModel, quat_to_rot
    except ImportError:
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), 'rov_mujoco'))
        from mujoco_sim_node import MujocoSimNode
        from ekf_fusion import ROVEKF, SpaceState, SensorModel, quat_to_rot

    import rclpy

    print('=' * 72)
    print('  水下多源融合导航 (EKF) 评测：IMU + DVL + 深度计 + 姿态')
    print('=' * 72)

    rclpy.init()
    node = MujocoSimNode()
    # 自动沿三维螺旋线巡航（PID 基准控制器，便于复现）
    from rov_mujoco.nn_trajectory_controller import LOSTrajectoryController
    node.controller = LOSTrajectoryController()
    node.auto_trajectory_mode = True
    if args.current <= 0.0:
        node.ocean_current = None
        print('[scenario] 已关闭洋流扰动')

    dt_phys = node.model.opt.timestep
    sample_every = 10                    # 500 Hz / 10 = 50 Hz
    dt = dt_phys * sample_every

    sensor = SensorModel(seed=args.seed, dvl_dropout=args.dropout)

    p_gt = node.data.sensor('pos').data[:3].copy()
    v_gt = node.data.sensor('vel').data[:3].copy()
    q_gt = node.data.qpos[node.qpos_addr + 3:node.qpos_addr + 7].copy()
    q_prev = q_gt.copy()
    v_prev = v_gt.copy()

    ekf = None
    dr = None
    series = []
    nees_list = []
    t = 0.0
    steps = int(round(args.duration / dt))

    for k in range(steps):
        for _ in range(sample_every):
            node._sim_step_callback()
        t += dt

        p_gt = node.data.sensor('pos').data[:3].copy()
        v_gt = node.data.sensor('vel').data[:3].copy()
        q_gt = node.data.qpos[node.qpos_addr + 3:node.qpos_addr + 7].copy()
        R = quat_to_rot(q_gt)

        omega = quat_diff_omega(q_prev, q_gt, dt)
        a_world = (v_gt - v_prev) / dt
        q_prev, v_prev = q_gt.copy(), v_gt.copy()

        a_m, w_m = sensor.imu(R, a_world, omega, dt)
        dvl = sensor.dvl(R, v_gt)
        depth = sensor.depth(p_gt)
        att = sensor.attitude(q_gt)

        if ekf is None:
            if t < args.warmup:
                continue
            ekf = ROVEKF(p_gt, v_gt, q_gt)
            ekf.sigma_acc = args.facc
            ekf.sigma_gyro = args.fgyro
            dr = SpaceState(p_gt, v_gt, q_gt)
            print('[init] t=%.1fs 起点 p=[%.2f %.2f %.2f]' % (t, *p_gt))

        ekf.predict(a_m, w_m, dt)
        dr.predict(a_m, w_m, dt)
        if dvl is not None:
            ekf.update_dvl(dvl, sigma=args.fdvl)
        ekf.update_depth(depth, sigma=args.fdepth)
        ekf.update_attitude(att, sigma=args.fatt)

        err_ekf = ekf.p - p_gt
        err_dr = dr.p - p_gt
        try:
            nees = float(err_ekf @ np.linalg.solve(ekf.position_cov(), err_ekf))
        except np.linalg.LinAlgError:
            nees = float('nan')
        nees_list.append(nees)

        if k % 5 == 0:   # 10 Hz 记录
            series.append({
                't': round(t, 3),
                'gt': [round(float(x), 4) for x in p_gt],
                'dr': [round(float(x), 4) for x in dr.p],
                'ekf': [round(float(x), 4) for x in ekf.p],
                'err_ekf': [round(float(x), 4) for x in err_ekf],
                'err_dr': [round(float(x), 4) for x in err_dr],
                'nees': round(nees, 3),
                'dvl_used': dvl is not None,
            })

    node.destroy_node()
    rclpy.shutdown()

    e_ekf = np.array([s['err_ekf'] for s in series])
    e_dr = np.array([s['err_dr'] for s in series])
    path_len = float(np.sum(np.linalg.norm(
        np.diff(np.array([s['gt'] for s in series]), axis=0), axis=1)))

    def rms(e):
        return float(np.sqrt(np.mean(np.sum(e * e, axis=1))))

    def final(e):
        return float(np.linalg.norm(e[-1]))

    nees_arr = np.array([n for n in nees_list if not math.isnan(n)])
    metrics = {
        'duration_s': round(t, 2),
        'path_length_m': round(path_len, 2),
        'dropout': args.dropout,
        'seed': args.seed,
        'current_scale': args.current,
        'filter_noise': {'acc': args.facc, 'gyro': args.fgyro, 'dvl': args.fdvl,
                         'depth': args.fdepth, 'att': args.fatt},
        'rmse_3d': {'dr': round(rms(e_dr), 4), 'ekf': round(rms(e_ekf), 4)},
        'rmse_axis': {
            'dr': [round(float(np.sqrt(np.mean(e_dr[:, i] ** 2))), 4) for i in range(3)],
            'ekf': [round(float(np.sqrt(np.mean(e_ekf[:, i] ** 2))), 4) for i in range(3)],
        },
        'max_err': {'dr': round(float(np.max(np.linalg.norm(e_dr, axis=1))), 4),
                    'ekf': round(float(np.max(np.linalg.norm(e_ekf, axis=1))), 4)},
        'final_err': {'dr': round(final(e_dr), 4), 'ekf': round(final(e_ekf), 4)},
        'drift_pct': {'dr': round(100.0 * final(e_dr) / max(path_len, 1e-6), 2),
                      'ekf': round(100.0 * final(e_ekf) / max(path_len, 1e-6), 2)},
        'nees_mean': round(float(np.mean(nees_arr)), 3) if nees_arr.size else None,
        'nees_in_95': round(float(np.mean(nees_arr < 7.815)), 3) if nees_arr.size else None,
    }

    print('\n路径长度: %.2f m | 时长: %.1f s | DVL 丢包率: %.0f%% | 洋流: %s'
          % (path_len, t, args.dropout * 100, '开' if args.current > 0 else '关'))
    print('-' * 72)
    print('%-18s %12s %12s' % ('指标', '航位推算 DR', 'EKF 融合'))
    print('%-18s %12.4f %12.4f' % ('3D RMSE [m]', metrics['rmse_3d']['dr'], metrics['rmse_3d']['ekf']))
    for i, ax in enumerate('XYZ'):
        print('%-18s %12.4f %12.4f' % ('RMSE %s [m]' % ax,
                                       metrics['rmse_axis']['dr'][i], metrics['rmse_axis']['ekf'][i]))
    print('%-18s %12.4f %12.4f' % ('最大误差 [m]', metrics['max_err']['dr'], metrics['max_err']['ekf']))
    print('%-18s %12.4f %12.4f' % ('终点误差 [m]', metrics['final_err']['dr'], metrics['final_err']['ekf']))
    print('%-18s %12.2f %12.2f' % ('漂移率 [%]', metrics['drift_pct']['dr'], metrics['drift_pct']['ekf']))
    print('-' * 72)
    print('EKF NEES 均值 %.2f（3 自由度 95%% 上界 7.815），落在界内比例 %.1f%%'
          % (metrics['nees_mean'] or 0.0, 100.0 * (metrics['nees_in_95'] or 0.0)))
    print('提升倍数 (RMSE): %.2fx' % (metrics['rmse_3d']['dr'] / max(metrics['rmse_3d']['ekf'], 1e-9)))

    out = {'metrics': metrics, 'series': series}
    with open(args.out, 'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False)
    print('\n结果已写入', args.out)


if __name__ == '__main__':
    main()
