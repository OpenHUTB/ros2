# -*- coding: utf-8 -*-
"""main.py —— 一键跑完整个模块。

流程：
  1) 采集感知网络训练集并训练（图像 -> 目标像素热图 -> 质心）
  2) 用几何专家飞随机场景，采集示教数据（行为克隆）
  3) DAgger 迭代：让学习器自己飞，专家重新标注，聚合后重训
  4) 四个场景 × 三种控制器 × 多个随机种子闭环评估
  5) 输出 results/ 下的模型、CSV、曲线图与跟踪动图

用法：
  python3 main.py                      # 完整流程（CPU 约 3~5 分钟）
  python3 main.py --quick              # 缩小数据量快速冒烟
  python3 main.py --no-gif             # 跳过动图渲染
  ros2 run uav_camera_tracking main    # 安装后等价
"""
import argparse
import csv
import os
import sys
import time

import numpy as np

from . import params as P
from . import plotting
from .camera import Camera
from .controllers import (ControllerNet, E2EController, E2ENet,
                          ExpertController, TwoStageController)
from .detector import DetectorNet
from .scenes import build_scenes
from .sim import (collect_demo, dagger_collect, evaluate,
                  random_training_scenes, rollout)


def _pkg_root():
    """包源码根目录（uav_camera_tracking/ 的上一级），用于定位 results/ 与 models/。"""
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _default_out():
    """默认结果目录：包源码根目录下的 results/（不随当前工作目录漂移）。"""
    return os.path.join(_pkg_root(), 'results')


def _default_models():
    """默认权重目录：包源码根目录下的 models/，由 setup.py 的 data_files 安装。"""
    return os.path.join(_pkg_root(), 'models')


def _write_csv(path, rows, header):
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(header)
        for r in rows:
            w.writerow(r)


def _cat(A, B):
    """拼接样本矩阵；任一侧为空（如两阶段控制器不收集图像样本）时安全退化。"""
    if A is None or np.size(A) == 0:
        return B
    if B is None or np.size(B) == 0:
        return A
    return np.vstack([A, B])


def train_pipeline(args, cam):
    """训练三条链路，返回 (detector, control, e2e, histories)。"""
    # ---------- 1) 感知网络 ----------
    print('[1/5] 采集并训练感知网络（%d 张合成图像）...' % args.det_samples)
    t = time.time()
    detector = DetectorNet(seed=1)
    Xd, Yd = detector.build_dataset(n=args.det_samples, seed=7)
    h_det = detector.fit(Xd, Yd, epochs=args.det_epochs)
    detector.save(os.path.join(args.model_dir, 'detector.npz'))
    print('      完成 %.1fs，val loss %.4f' % (time.time() - t,
                                          h_det['val_loss'][-1]))

    # ---------- 2) 专家示教 ----------
    print('[2/5] 几何专家飞行采集示教数据（%d 个随机场景）...' % args.train_scenes)
    t = time.time()
    scenes_train = random_training_scenes(seed=3, count=args.train_scenes)
    Xf, Yf, Xi, Yi = collect_demo(scenes_train, ExpertController(), cam, seed=11)
    print('      完成 %.1fs，有效示教样本 %d 条' % (time.time() - t, len(Xf)))

    # ---------- 3) 行为克隆 + DAgger ----------
    print('[3/5] 行为克隆 + %d 轮 DAgger 迭代...' % args.dagger_rounds)
    t = time.time()
    control = ControllerNet(seed=2)
    h_ctrl = control.fit(Xf, Yf, epochs=args.ctrl_epochs)
    e2e = E2ENet(seed=3)
    h_e2e = e2e.fit(Xi, Yi, epochs=args.e2e_epochs)

    for r in range(args.dagger_rounds):
        # 两阶段控制器：用感知网络+控制网络闭环飞行，专家重新标注
        Xa, Ya, Xb, Yb = dagger_collect(
            scenes_train, TwoStageController(detector, control), cam,
            seed=100 + r)
        Xf = _cat(Xf, Xa)
        Yf = _cat(Yf, Ya)
        control = ControllerNet(seed=2)
        h_ctrl = control.fit(Xf, Yf, epochs=args.ctrl_epochs, lr=1e-3)
        # 端到端策略同样做 DAgger（第一轮即可，样本量有限时收益递减）
        if r == 0:
            Xi = _cat(Xi, Xb)
            Yi = _cat(Yi, Yb)
            e2e = E2ENet(seed=3)
            h_e2e = e2e.fit(Xi, Yi, epochs=args.e2e_epochs, lr=1e-3)
        print('      DAgger %d：示教样本 %d 条，控制网 val %.4f'
              % (r + 1, len(Xf), h_ctrl['val_loss'][-1]))

    control.save(os.path.join(args.model_dir, 'control.npz'))
    e2e.save(os.path.join(args.model_dir, 'e2e.npz'))
    print('      完成 %.1fs（控制网 val %.4f，端到端 val %.4f）'
          % (time.time() - t, h_ctrl['val_loss'][-1], h_e2e['val_loss'][-1]))
    return detector, control, e2e, {'detector': h_det, 'control': h_ctrl,
                                    'e2e': h_e2e}


def main(argv=None):
    ap = argparse.ArgumentParser(description='无人机机载相机视觉目标跟踪')
    ap.add_argument('--out', default=_default_out(), help='结果输出目录')
    ap.add_argument('--model-dir', default=_default_models(),
                    help='网络权重目录（默认包内 models/）')
    ap.add_argument('--seeds', type=int, default=3, help='每个场景重复回合数')
    ap.add_argument('--det-samples', type=int, default=3000,
                    help='感知网络训练图像数')
    ap.add_argument('--det-epochs', type=int, default=45)
    ap.add_argument('--ctrl-epochs', type=int, default=120)
    ap.add_argument('--e2e-epochs', type=int, default=80)
    ap.add_argument('--train-scenes', type=int, default=8, help='示教场景数')
    ap.add_argument('--dagger-rounds', type=int, default=2)
    ap.add_argument('--quick', action='store_true', help='缩小数据量快速验证')
    ap.add_argument('--no-gif', action='store_true', help='跳过动图渲染')
    args = ap.parse_args(argv)

    if args.quick:
        args.det_samples, args.det_epochs = 600, 15
        args.ctrl_epochs, args.e2e_epochs = 40, 30
        args.train_scenes, args.dagger_rounds, args.seeds = 3, 1, 1

    os.makedirs(args.out, exist_ok=True)
    os.makedirs(args.model_dir, exist_ok=True)
    t0 = time.time()
    cam = Camera()

    detector, control, e2e, histories = train_pipeline(args, cam)

    # ---------- 4) 多场景评估 ----------
    print('[4/5] 四场景闭环评估（每场景 %d 回合）...' % args.seeds)
    controllers = [
        ExpertController(),
        TwoStageController(detector, control),
        E2EController(e2e, detector.features_e2e),
    ]
    scenes = build_scenes()
    rows, summary = [], {}
    for scene in scenes:
        summary[scene.key] = {}
        for ctrl in controllers:
            res = [evaluate(rollout(scene, ctrl, cam,
                                    np.random.default_rng(1000 + 17 * s)))
                   for s in range(args.seeds)]
            agg = {
                'success_rate': float(np.mean([r['success'] for r in res])),
                'err_rms_mean': float(np.mean([r['err_rms'] for r in res])),
                'lost_frac_mean': float(np.mean([r['lost_frac'] for r in res])),
                'speed_mean': float(np.mean([r['mean_speed'] for r in res])),
            }
            summary[scene.key][ctrl.label] = agg
            rows.append([scene.key, scene.name, ctrl.name, args.seeds,
                         '%.2f' % agg['success_rate'],
                         '%.4f' % agg['err_rms_mean'],
                         '%.4f' % agg['lost_frac_mean'],
                         '%.3f' % agg['speed_mean']])
            print('      %-10s %-22s 成功率 %.2f  误差RMS %.4f  丢失 %.3f'
                  % (scene.key, ctrl.name, agg['success_rate'],
                     agg['err_rms_mean'], agg['lost_frac_mean']))

    _write_csv(os.path.join(args.out, 'compare_summary.csv'), rows,
               ['scene', 'scene_name', 'controller', 'episodes',
                'success_rate', 'err_rms', 'lost_frac', 'mean_speed'])

    # ---------- 5) 图表 ----------
    print('[5/5] 输出训练曲线、场景对比图与跟踪动图...')
    plotting.plot_training(histories, os.path.join(args.out,
                                                   'training_curves.png'))
    plotting.plot_comparison(summary, os.path.join(args.out, 'compare.png'))
    if not args.no_gif:
        scene = [s for s in scenes if s.key == 'moving'][0]
        log = rollout(scene, TwoStageController(detector, control), cam,
                      np.random.default_rng(2024), record_every=3)
        plotting.make_gif(scene, log, os.path.join(args.out, 'tracking.gif'),
                          title='two-stage tracking (moving target)')
        scene2 = [s for s in scenes if s.key == 'occlusion'][0]
        log2 = rollout(scene2, ExpertController(), cam,
                       np.random.default_rng(2025), record_every=3)
        plotting.make_gif(scene2, log2,
                          os.path.join(args.out, 'occlusion.gif'),
                          title='re-acquisition after occlusion')

    print('\n全部完成，用时 %.1f s，结果目录：%s' % (time.time() - t0, args.out))
    print('\n%-12s %-22s %8s %9s %8s' % ('场景', '控制器', '成功率', '误差RMS', '丢失率'))
    for r in rows:
        print('%-12s %-22s %8s %9s %8s' % (r[0], r[2], r[4], r[5], r[6]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
