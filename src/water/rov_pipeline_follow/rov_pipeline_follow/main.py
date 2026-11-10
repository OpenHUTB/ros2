"""离线流水线：采集 → 行为克隆 → DAgger → 感知网络 → 多场景评测 → 出图。

五个阶段可以分开跑，也可以一条命令跑完：

    python -m rov_pipeline_follow.main                 # 全流程
    python -m rov_pipeline_follow.main --quick         # 少种子少轮次
    python -m rov_pipeline_follow.main --stage eval    # 只做评测

训练数据只来自清水场景（straight / curve），浑水（turbid）与海流
（current）只做测试 —— 检验感知与定位的泛化，而不是背布局。
"""

import argparse
import csv
import json
import os
import time

import numpy as np

from . import params as P
from . import plotting
from .cnn import Adam, CNN
from .model_io import default_model_dir, load_net, save_net
from .policies import E2EPolicy, PERCEP_IN_CH, make_policy
from .sim import (collect_demo, evaluate, rollout, stack_dataset,
                  stack_percep)
from .world import make_scene

TRAIN_SCENES = ('straight', 'curve')
# pipe_cnn（用 CNN 回归 (dx, dtheta) 替代几何检测）也能跑，但闭环里
# 次次撞：回归量在 1 rad 级航向偏差下不可靠，|desired| 判据被逐帧抖动
# 触发成"转一下、停一下"的振荡。它作为负面结果留在 make_policy 里，
# 不进主对比（见 docs/water/rov_pipeline_follow.md 的负面结果一节）。
POLICY_KINDS = ('oracle', 'pipe_slam', 'e2e', 'e2e_nogoal')


def _log(msg):
    print('[%s] %s' % (time.strftime('%H:%M:%S'), msg), flush=True)


def _paths(args):
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = args.out or os.path.join(root, 'results')
    mdir = args.model_dir or default_model_dir()
    os.makedirs(out, exist_ok=True)
    os.makedirs(mdir, exist_ok=True)
    return out, mdir


# ------------------------------------------------------------------ 训练
def fit(net, X, Y, epochs, lr=8e-4, batch=32, seed=0, log=None, tag='',
        steer_col=1):
    rng = np.random.default_rng(seed)
    opt = Adam(net.params(), lr=lr)
    log = log if log is not None else {'loss': [], 'epoch': [], 'val': []}
    n = len(X)
    for ep in range(epochs):
        idx = rng.permutation(n)
        tot, cnt = 0.0, 0
        for i in range(0, n, batch):
            b = idx[i:i + batch]
            # 转向维权重：|y1| 越大权重越高，对抗回归均值化
            sw = 1.0 + 2.0 * np.abs(Y[b, steer_col])
            tot += net.train_step(X[b], Y[b], opt, lw=2.0, sw=sw)
            cnt += 1
        log['loss'].append(tot / max(1, cnt))
        log['epoch'].append(len(log['loss']))
        _log('  %s epoch %2d/%d  loss=%.5f' % (tag, ep + 1, epochs, log['loss'][-1]))
    return log


def holdout_loss(net, X, Y, batch=64):
    if len(X) == 0:
        return float('nan')
    tot = 0.0
    for i in range(0, len(X), batch):
        out = net.forward(X[i:i + batch], train=False)
        tot += float(np.mean((out - Y[i:i + batch]) ** 2))
    return tot / max(1, (len(X) + batch - 1) // batch)


# ------------------------------------------------------------------ 阶段
def stage_data(args, out, mdir):
    scenes = TRAIN_SCENES if not args.quick else ('straight',)
    seeds = (0, 1, 2) if not args.quick else (0,)
    _log('采集专家数据：scenes=%s seeds=%s' % (scenes, seeds))
    eps = collect_demo(scenes, seeds)
    X, Y = stack_dataset(eps)
    Xp, Yp = stack_percep(eps)
    n_val = len(eps[-1]['frames']) if len(eps) > 1 else 0
    _log('  e2e %d 个监督样本 / percep %d 个（%d 条轨迹，最后一条留作 hold-out）'
         % (len(X), len(Xp), len(eps)))
    np.savez_compressed(os.path.join(out, 'bc_dataset.npz'), X=X, Y=Y,
                        Xp=Xp, Yp=Yp, n_val=np.array(n_val))
    return X, Y, Xp, Yp, eps


def stage_train(args, out, mdir, X, Y, Xp, Yp, eps, n_val=None):
    if n_val is None:
        n_val = len(eps[-1]['frames']) if (eps is not None and len(eps) > 1) else 0
    n_pval = min(n_val, len(Xp))
    split = lambda A: (A[:-n_val], A[-n_val:]) if n_val else (A, A[:0])
    Xtr, Xv = split(X); Ytr, Yv = split(Y)
    if n_pval:
        Xptr, Xpv = Xp[:-n_pval], Xp[-n_pval:]
        Yptr, Ypv = Yp[:-n_pval], Yp[-n_pval:]
    else:
        Xptr, Xpv = Xp, Xp[:0]
        Yptr, Ypv = Yp, Yp[:0]

    log = {'loss': [], 'epoch': [], 'val': [], 'stage': []}

    # ---------------- 感知网络：RGB-D → (dx_err, dtheta) 回归
    percep = CNN(E2EPolicy.IN_H, E2EPolicy.IN_W,
                 np.random.default_rng(args.seed + 100), in_ch=PERCEP_IN_CH)
    epochs_p = 6 if args.quick else 25
    _log('感知网络训练：%d 样本，%d epoch' % (len(Xptr), epochs_p))
    t0 = time.time()
    fit(percep, Xptr, Yptr, epochs_p, seed=args.seed + 5, log=log, tag='percep',
        steer_col=0)
    log['stage'].append([len(log['loss']), 'percep'])
    _log('  感知网络 %.0fs  hold-out MSE=%.5f'
         % (time.time() - t0, holdout_loss(percep, Xpv, Ypv)))
    save_net(percep, os.path.join(mdir, 'percep.npz'))

    # ---------------- 端到端策略：BC
    net = CNN(E2EPolicy.IN_H, E2EPolicy.IN_W, np.random.default_rng(args.seed))
    epochs = 8 if args.quick else 40
    _log('行为克隆：%d 训练 / %d 验证 样本，%d epoch' % (len(Xtr), len(Xv), epochs))
    t0 = time.time()
    fit(net, Xtr, Ytr, epochs, seed=args.seed, log=log, tag='BC')
    if len(Xv):
        log['val'] = [holdout_loss(net, Xv, Yv)] * len(log['loss'])
    log['stage'].append([len(log['loss']), 'BC'])
    _log('  训练 %.0fs  hold-out MSE=%.5f' % (time.time() - t0, holdout_loss(net, Xv, Yv)))

    # ---------------- DAgger
    # 种子覆盖评测用的全部种子：学生必须见过专家在自己失败状态下的纠偏
    rounds = 2 if args.quick else 3
    for r in range(rounds):
        beta = 0.7 * (1.0 - r / max(1, rounds))
        eps_d = []
        for sc in (TRAIN_SCENES if not args.quick else ('straight',)):
            for sd in ((0, 1, 2) if not args.quick else (0,)):
                oracle = make_policy('oracle', make_scene(sc, sd))
                pol = E2EPolicy(net)
                rres = rollout(sc, pol, seed=sd, record=True,
                               dagger_oracle=oracle, beta=beta)
                if len(rres['frames']) > 5:
                    eps_d.append({'frames': rres['frames'],
                                  'depths': rres['depths'],
                                  'odoms': rres['odoms'],
                                  'goals': rres['goals'],
                                  'actions': rres['labels'],
                                  'percep_labels': rres['percep_labels']})
        if not eps_d:
            continue
        Xd, Yd = stack_dataset(eps_d)
        Xd_p, Yd_p = stack_percep(eps_d)
        Xtr = np.concatenate([Xtr, Xd], axis=0)
        Ytr = np.concatenate([Ytr, Yd], axis=0)
        Xptr = np.concatenate([Xptr, Xd_p], axis=0)
        Yptr = np.concatenate([Yptr, Yd_p], axis=0)
        _log('DAgger 第 %d 轮：beta=%.2f 新增 %d 样本，累计 %d' %
             (r + 1, beta, len(Xd), len(Xtr)))
        fit(net, Xtr, Ytr, 4 if args.quick else 10, seed=args.seed + r + 1,
            log=log, tag='DAgger%d' % (r + 1))
        log['stage'].append([len(log['loss']), 'DAgger%d' % (r + 1)])

    # DAgger 数据同步微调感知网络（学生视角的帧更接近部署分布）
    fit(percep, Xptr, Yptr, 4 if args.quick else 8, seed=args.seed + 9,
        log=log, tag='percep-DAgger', steer_col=0)
    log['stage'].append([len(log['loss']), 'percep-DAgger'])
    save_net(percep, os.path.join(mdir, 'percep.npz'))

    save_net(net, os.path.join(mdir, 'e2e.npz'))
    with open(os.path.join(out, 'train_log.json'), 'w', encoding='utf-8') as f:
        json.dump(log, f)
    _log('权重已保存到 %s 和 %s' % (os.path.join(mdir, 'e2e.npz'),
                                    os.path.join(mdir, 'percep.npz')))
    return net, percep, log


def stage_eval(args, out, mdir, net, percep):
    scenes = P.SCENES if not args.quick else ('straight', 'turbid')
    seeds = (0, 1, 2) if not args.quick else (0,)
    rows = []
    for kind in POLICY_KINDS:
        if kind in ('e2e', 'e2e_nogoal') and net is None:
            continue
        if kind == 'pipe_cnn' and percep is None:
            continue
        t0 = time.time()
        rs = evaluate(kind, scenes=scenes, seeds=seeds, net=net,
                      percep_net=percep)
        ok = sum(r['success'] for r in rs)
        _log('%-10s 成功率 %d/%d  (%.0fs)' % (kind, ok, len(rs), time.time() - t0))
        rows.extend(rs)

    path = os.path.join(out, 'compare_summary.csv')
    fields = ['policy', 'scene', 'seed', 'success', 'collided', 'reached_b',
              'time_s', 'path_len', 'final_dist', 'lat_rmse', 'ate_rmse',
              'loc_valid', 'loop_err', 'map_iou', 'pipe_cov', 'coverage',
              'n_landmarks', 'mean_speed']
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, '') for k in fields})
    _log('指标写入 %s' % path)
    return rows


def stage_figures(args, out, rows, net, percep, log):

    _log('出图：对比柱状图 / 训练曲线 / 轨迹 / GIF')
    if rows:
        plotting.plot_compare(rows, os.path.join(out, 'compare.png'))
    if log and log.get('loss'):
        plotting.plot_training(log, os.path.join(out, 'training_curves.png'))

    # 轨迹对比（straight / seed 0）
    world = make_scene('straight', 0)
    trajs = []
    for kind, color in (('oracle', '#2f6fd0'), ('pipe_slam', '#d4692f'),
                        ('e2e', '#2f9e6b')):
        if kind == 'e2e' and net is None:
            continue
        pol = make_policy(kind, make_scene('straight', 0), net=net,
                          percep_net=percep)
        r = rollout('straight', pol, seed=0, record=True)
        trajs.append((plotting.POLICY_STYLE[kind][1], color, r['traj']))
    plotting.plot_traj(world, trajs, os.path.join(out, 'trajectories.png'),
                       title='Round-trip inspection trajectories (straight, seed 0)')

    if args.no_gif:
        return
    # GIF：清水用经典流水线，浑水用 CNN 感知 —— 一张看建图，一张看
    # 学习式感知在几何检测失效时的表现
    jobs = [('curve', 'pipe_slam', 6), ('turbid', 'pipe_slam', 5)]
    for scene, kind, stride in jobs:
        _log('  生成 %s（%s）的 GIF ...' % (scene, kind))
        frames, traj = [], []

        def cb(world_, pose, rgb, policy, t, _traj=traj, _frames=frames,
               _st=stride, _scene=scene, _kind=kind):
            _traj.append(pose.copy())
            if len(_frames) >= 90 or (len(_traj) % _st):
                return
            g = getattr(policy, 'grid', None)
            note = '%s / %s  t=%.1fs' % (_scene, _kind, t)
            if g is not None:
                note += '  (EKF-SLAM map)'
            _frames.append(plotting.render_panel(rgb, None, g, _traj, world_,
                                                 world_.goal, pose, note=note))

        pol = make_policy(kind, make_scene(scene, 0), percep_net=percep)
        rollout(scene, pol, seed=0, on_frame=cb)
        path = os.path.join(out, '%s.gif' % scene)
        plotting.make_gif(frames, path, fps=10)
        if os.path.exists(path):
            _log('  %s  %.1f MB' % (path, os.path.getsize(path) / 1e6))


# ------------------------------------------------------------------ 入口
def main(argv=None):
    ap = argparse.ArgumentParser(
        description='水下 ROV 管道巡检：视觉感知 + EKF-SLAM + 端到端学习（离线流水线）')
    ap.add_argument('--stage', default='all',
                    choices=['all', 'data', 'train', 'eval', 'figures'])
    ap.add_argument('--out', default=None, help='结果目录，默认包内 results/')
    ap.add_argument('--model-dir', default=None, help='权重目录，默认 ament share/models')
    ap.add_argument('--quick', action='store_true', help='快速模式：少种子、少轮次')
    ap.add_argument('--no-gif', action='store_true')
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args(argv)

    out, mdir = _paths(args)
    _log('结果目录 %s' % out)

    X = Y = Xp = Yp = eps = None
    n_val = None
    log = None
    net = percep = None

    if args.stage in ('all', 'data'):
        X, Y, Xp, Yp, eps = stage_data(args, out, mdir)
    if args.stage in ('all', 'train'):
        if X is None:
            d = np.load(os.path.join(out, 'bc_dataset.npz'))
            X, Y = d['X'], d['Y']
            Xp, Yp = (d['Xp'], d['Yp']) if 'Xp' in d.files else (None, None)
            n_val = int(d['n_val']) if 'n_val' in d.files else None
            eps = None
        net, percep, log = stage_train(args, out, mdir, X, Y, Xp, Yp, eps, n_val)
    if args.stage in ('all', 'eval', 'figures'):
        mp = os.path.join(mdir, 'e2e.npz')
        pp = os.path.join(mdir, 'percep.npz')
        if net is None and os.path.exists(mp):
            try:
                net = load_net(mp)
                _log('已加载端到端权重 %s' % mp)
            except Exception as e:
                _log('加载 e2e 权重失败（%s），端到端一路跳过' % e)
        if percep is None and os.path.exists(pp):
            try:
                percep = load_net(pp)
                _log('已加载感知网络权重 %s' % pp)
            except Exception as e:
                _log('加载 percep 权重失败（%s），pipe_cnn 跳过' % e)
    rows = []
    if args.stage in ('all', 'eval'):
        rows = stage_eval(args, out, mdir, net, percep)
    if args.stage in ('all', 'figures'):
        lp = os.path.join(out, 'train_log.json')
        if log is None and os.path.exists(lp):
            with open(lp, encoding='utf-8') as f:
                log = json.load(f)
        if not rows:
            try:
                with open(os.path.join(out, 'compare_summary.csv'),
                          encoding='utf-8') as f:
                    rows = [dict(r) for r in csv.DictReader(f)]
                    for r in rows:
                        r['success'] = r['success'] in ('True', '1', True)
                        for k in ('time_s', 'path_len', 'final_dist', 'lat_rmse',
                                  'ate_rmse', 'loc_valid', 'loop_err', 'map_iou',
                                  'pipe_cov', 'coverage', 'n_landmarks',
                                  'mean_speed'):
                            try:
                                r[k] = float(r[k])
                            except (TypeError, ValueError):
                                r[k] = float('nan')
            except OSError:
                rows = []
        stage_figures(args, out, rows, net, percep, log)

    _log('完成。')


if __name__ == '__main__':
    main()
