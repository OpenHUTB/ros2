"""离线流水线：采集 → 行为克隆 → DAgger → 多场景评测 → 出图。

五个阶段可以分开跑，也可以一条命令跑完：

    python -m ugv_visual_slam_nav.main                 # 全流程
    python -m ugv_visual_slam_nav.main --quick         # 少种子少轮次
    python -m ugv_visual_slam_nav.main --stage eval    # 只做评测
"""

import argparse
import json
import os
import time

import numpy as np

from . import params as P
from . import plotting
from .cnn import Adam, CNN
from .model_io import default_model_dir, load_net, save_net
from .policies import E2EPolicy, make_policy
from .sim import collect_demo, evaluate, rollout, stack_dataset
from .world import make_scene

TRAIN_SCENES = ('corridor', 'cluttered')


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
def fit(net, X, Y, epochs, lr=8e-4, batch=32, seed=0, log=None, tag=''):
    rng = np.random.default_rng(seed)
    opt = Adam(net.params(), lr=lr)
    log = log if log is not None else {'loss': [], 'epoch': [], 'val': []}
    n = len(X)
    for ep in range(epochs):
        idx = rng.permutation(n)
        tot, cnt = 0.0, 0
        for i in range(0, n, batch):
            b = idx[i:i + batch]
            # 转向帧权重：|w| 越大权重越高，对抗回归均值化
            sw = 1.0 + 2.0 * np.abs(Y[b, 1])
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
    scenes = TRAIN_SCENES if not args.quick else ('corridor',)
    seeds = (0, 1, 2) if not args.quick else (0,)
    _log('采集专家数据：scenes=%s seeds=%s' % (scenes, seeds))
    eps = collect_demo(scenes, seeds)
    X, Y = stack_dataset(eps)
    n_val = len(eps[-1]['frames']) if len(eps) > 1 else 0
    _log('  得到 %d 个监督样本（%d 条轨迹，最后 %d 个样本留作 hold-out）'
         % (len(X), len(eps), n_val))
    np.savez_compressed(os.path.join(out, 'bc_dataset.npz'), X=X, Y=Y,
                        n_val=np.array(n_val))
    return X, Y, eps


def stage_train(args, out, mdir, X, Y, eps, n_val=None):
    # 留出最后一条轨迹做 hold-out；缓存数据集里也记了 n_val，重跑 train 时仍能留出
    if n_val is None:
        n_val = len(eps[-1]['frames']) if (eps is not None and len(eps) > 1) else 0
    Xtr, Ytr = X[:-n_val] if n_val else X, Y[:-n_val] if n_val else Y
    Xv, Yv = (X[-n_val:], Y[-n_val:]) if n_val else (X[:0], Y[:0])

    net = CNN(E2EPolicy.IN_H, E2EPolicy.IN_W, np.random.default_rng(args.seed))
    log = {'loss': [], 'epoch': [], 'val': [], 'stage': []}
    epochs = 8 if args.quick else 40
    _log('行为克隆：%d 训练 / %d 验证 样本，%d epoch' % (len(Xtr), len(Xv), epochs))
    t0 = time.time()
    fit(net, Xtr, Ytr, epochs, seed=args.seed, log=log, tag='BC')
    if len(Xv):
        log['val'] = [holdout_loss(net, Xv, Yv)] * len(log['loss'])
    log['stage'].append([len(log['loss']), 'BC'])
    _log('  训练 %.0fs  hold-out MSE=%.5f' % (time.time() - t0, holdout_loss(net, Xv, Yv)))

    # ---------------- DAgger
    # DAgger 的种子要覆盖评测用的全部种子：否则学生只在 0/1 号场景布局的
    # 失败状态上见过专家纠偏，2 号布局上照样会一头撞上去。
    rounds = 2 if args.quick else 3
    for r in range(rounds):
        beta = 0.7 * (1.0 - r / max(1, rounds))
        eps_d = []
        for sc in (TRAIN_SCENES if not args.quick else ('corridor',)):
            for sd in ((0, 1, 2) if not args.quick else (0,)):
                oracle = make_policy('oracle', make_scene(sc, sd))
                pol = E2EPolicy(net)
                rres = rollout(sc, pol, seed=sd, record=True,
                               dagger_oracle=oracle, beta=beta)
                if len(rres['frames']) > 5:
                    eps_d.append({'frames': rres['frames'], 'depths': rres['depths'],
                                  'odoms': rres['odoms'], 'goal': rres['goal'],
                                  'actions': rres['labels']})
        if not eps_d:
            continue
        Xd, Yd = stack_dataset(eps_d)
        Xa = np.concatenate([Xtr, Xd], axis=0)
        Ya = np.concatenate([Ytr, Yd], axis=0)
        Xtr, Ytr = Xa, Ya
        _log('DAgger 第 %d 轮：beta=%.2f 新增 %d 样本，累计 %d' %
             (r + 1, beta, len(Xd), len(Xtr)))
        fit(net, Xtr, Ytr, 4 if args.quick else 10, seed=args.seed + r + 1,
            log=log, tag='DAgger%d' % (r + 1))
        log['stage'].append([len(log['loss']), 'DAgger%d' % (r + 1)])

    save_net(net, os.path.join(mdir, 'e2e.npz'))
    with open(os.path.join(out, 'train_log.json'), 'w', encoding='utf-8') as f:
        json.dump(log, f)
    _log('权重已保存到 %s' % os.path.join(mdir, 'e2e.npz'))
    return net, log


def stage_eval(args, out, mdir, net):
    scenes = P.SCENES if not args.quick else ('corridor', 'cluttered')
    seeds = (0, 1, 2) if not args.quick else (0,)
    rows = []
    for kind in ('oracle', 'slam_nav', 'e2e', 'e2e_nogoal'):
        if kind == 'e2e' and net is None:
            continue
        t0 = time.time()
        rs = evaluate(kind, scenes=scenes, seeds=seeds, net=net)
        ok = sum(r['success'] for r in rs)
        _log('%-9s 成功率 %d/%d  (%.0fs)' % (kind, ok, len(rs), time.time() - t0))
        rows.extend(rs)

    import csv
    path = os.path.join(out, 'compare_summary.csv')
    fields = ['policy', 'scene', 'seed', 'success', 'collided', 'time_s',
              'path_len', 'final_dist', 'ate_rmse', 'loc_valid', 'map_iou',
              'coverage', 'mean_speed']
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, '') for k in fields})
    _log('指标写入 %s' % path)
    return rows


def stage_figures(args, out, rows, net, log):

    _log('出图：对比柱状图 / 训练曲线 / 轨迹 / GIF')
    if rows:
        plotting.plot_compare(rows, os.path.join(out, 'compare.png'))
    if log and log.get('loss'):
        plotting.plot_training(log, os.path.join(out, 'training_curves.png'))

    # 轨迹对比（corridor / seed 0）
    world = make_scene('corridor', 0)
    trajs = []
    for kind, color in (('oracle', '#2f6fd0'), ('slam_nav', '#d4692f'), ('e2e', '#2f9e6b')):
        if kind == 'e2e' and net is None:
            continue
        pol = make_policy(kind, make_scene('corridor', 0), net=net)
        r = rollout('corridor', pol, seed=0, record=True)
        trajs.append((plotting.POLICY_STYLE[kind][1], color, r['traj']))
    plotting.plot_traj(world, trajs, os.path.join(out, 'trajectories.png'),
                       title='Trajectories on corridor scene (seed 0)')

    if args.no_gif:
        return
    for scene, stride in (('corridor', 6), ('dynamic', 5)):
        _log('  生成 %s 的 GIF ...' % scene)
        frames, traj = [], []

        def cb(world_, pose, rgb, policy, t, _traj=traj, _frames=frames, _st=stride):
            _traj.append(pose.copy())
            if len(_frames) >= 80 or (_traj.__len__() % _st):
                return
            g = getattr(policy, 'grid', None)
            note = 'EKF-SLAM map (grey) + path' if g is not None else ''
            _frames.append(plotting.render_panel(rgb, None, g, _traj, world_,
                                                 world_.goal, pose,
                                                 note='%s  t=%.1fs' % (scene, t)))

        pol = make_policy('slam_nav', make_scene(scene, 0))
        rollout(scene, pol, seed=0, on_frame=cb)
        path = os.path.join(out, '%s.gif' % scene)
        plotting.make_gif(frames, path, fps=10)
        if os.path.exists(path):
            _log('  %s  %.1f MB' % (path, os.path.getsize(path) / 1e6))


# ------------------------------------------------------------------ 入口
def main(argv=None):
    ap = argparse.ArgumentParser(description='无人车视觉 SLAM 建图与自主导航（离线流水线）')
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

    X = Y = eps = None
    n_val = None
    log = None
    net = None

    if args.stage in ('all', 'data'):
        X, Y, eps = stage_data(args, out, mdir)
    if args.stage in ('all', 'train'):
        if X is None:
            d = np.load(os.path.join(out, 'bc_dataset.npz'))
            X, Y = d['X'], d['Y']
            n_val = int(d['n_val']) if 'n_val' in d.files else None
            eps = None
        net, log = stage_train(args, out, mdir, X, Y, eps, n_val)
    if args.stage in ('all', 'eval', 'figures'):
        mp = os.path.join(mdir, 'e2e.npz')
        if net is None and os.path.exists(mp):
            try:
                net = load_net(mp)
                _log('已加载权重 %s' % mp)
            except Exception as e:      # 权重不兼容时不要让整条流水线挂掉
                _log('加载权重失败（%s），端到端一路跳过' % e)
                net = None
    rows = []
    if args.stage in ('all', 'eval'):
        rows = stage_eval(args, out, mdir, net)
    if args.stage in ('all', 'figures'):
        lp = os.path.join(out, 'train_log.json')
        if log is None and os.path.exists(lp):
            with open(lp, encoding='utf-8') as f:
                log = json.load(f)
        if not rows:
            try:
                import csv
                with open(os.path.join(out, 'compare_summary.csv'), encoding='utf-8') as f:
                    rows = [dict(r) for r in csv.DictReader(f)]
                    for r in rows:
                        r['success'] = r['success'] in ('True', '1', True)
                        for k in ('time_s', 'path_len', 'final_dist', 'ate_rmse',
                                  'loc_valid', 'map_iou', 'coverage', 'mean_speed'):
                            try:
                                r[k] = float(r[k])
                            except (TypeError, ValueError):
                                r[k] = float('nan')
            except OSError:
                rows = []
        stage_figures(args, out, rows, net, log)

    _log('完成。')


if __name__ == '__main__':
    main()
