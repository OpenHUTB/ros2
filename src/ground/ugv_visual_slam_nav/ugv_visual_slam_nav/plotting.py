"""出图：多场景对比柱状图、训练曲线、轨迹对比图、导航过程 GIF。

统一用 Agg 后端，中文不参与绘图文字（标签全英文），避免缺字体导致豆腐块。
"""

import io
import math
import os

import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Rectangle

from . import params as P

POLICY_STYLE = {
    'oracle': ('#2f6fd0', 'Oracle'),
    'slam_nav': ('#d4692f', 'EKF-SLAM + A*'),
    'e2e': ('#2f9e6b', 'End-to-end CNN'),
    'e2e_nogoal': ('#8e6fb0', 'E2E, goal channel zeroed'),
}
SCENE_LABEL = {'corridor': 'corridor', 'cluttered': 'cluttered',
               'dynamic': 'dynamic', 'night': 'night'}


def _agg(rows, key):
    d = {}
    for r in rows:
        d.setdefault((r['policy'], r['scene']), []).append(r.get(key, float('nan')))
    return {k: float(np.nanmean(v)) for k, v in d.items()}


def plot_compare(rows, out_path, policies=None):
    """四联图：成功率 / 定位误差 / 建图 IoU / 到达时间。"""
    policies = policies or ['oracle', 'slam_nav', 'e2e', 'e2e_nogoal']
    policies = [p for p in policies
                if p in {r['policy'] for r in rows} or p in ('oracle', 'slam_nav')]
    scenes = [s for s in P.SCENES]
    succ = _agg(rows, 'success')
    ate = _agg(rows, 'ate_rmse')
    iou = _agg(rows, 'map_iou')
    tim = _agg(rows, 'time_s')

    fig, axes = plt.subplots(2, 2, figsize=(11.5, 7.2))
    width = 0.8 / max(1, len(policies))
    xs = np.arange(len(scenes))

    def grouped(ax, table, ylabel, title):
        for i, pol in enumerate(policies):
            vals = [table.get((pol, s), np.nan) for s in scenes]
            ax.bar(xs + (i - (len(policies) - 1) / 2) * width, vals, width,
                   label=POLICY_STYLE.get(pol, (None, pol))[1],
                   color=POLICY_STYLE.get(pol, ('#888', pol))[0])
        ax.set_xticks(xs)
        ax.set_xticklabels([SCENE_LABEL.get(s, s) for s in scenes])
        ax.set_ylabel(ylabel)
        ax.set_title(title, fontsize=11)
        ax.grid(axis='y', alpha=0.3, linestyle=':')
        ax.set_axisbelow(True)

    grouped(axes[0, 0], succ, 'success rate', '(a) Navigation success (3 seeds / scene)')
    axes[0, 0].set_ylim(0, 1.05)

    # ATE 只有带 SLAM 的策略才有
    for i, pol in enumerate(['slam_nav']):
        vals = [ate.get((pol, s), np.nan) for s in scenes]
        axes[0, 1].bar(xs, vals, 0.55, color=POLICY_STYLE[pol][0],
                       label='EKF-SLAM')
    axes[0, 1].set_xticks(xs)
    axes[0, 1].set_xticklabels([SCENE_LABEL.get(s, s) for s in scenes])
    axes[0, 1].set_ylabel('ATE RMSE [m]')
    axes[0, 1].set_title('(b) SLAM localization error', fontsize=11)
    axes[0, 1].grid(axis='y', alpha=0.3, linestyle=':')
    axes[0, 1].set_axisbelow(True)

    vals = [iou.get(('slam_nav', s), np.nan) for s in scenes]
    axes[1, 0].bar(xs, vals, 0.55, color=POLICY_STYLE['slam_nav'][0])
    axes[1, 0].set_xticks(xs)
    axes[1, 0].set_xticklabels([SCENE_LABEL.get(s, s) for s in scenes])
    axes[1, 0].set_ylabel('occupancy IoU')
    axes[1, 0].set_ylim(0, 1.0)
    axes[1, 0].set_title('(c) Map quality in explored area', fontsize=11)
    axes[1, 0].grid(axis='y', alpha=0.3, linestyle=':')
    axes[1, 0].set_axisbelow(True)

    grouped(axes[1, 1], tim, 'time to goal [s]',
            '(d) Completion time (timeout = 150 s)')

    h, l = axes[0, 0].get_legend_handles_labels()
    fig.legend(h, l, loc='lower center', ncol=3, frameon=False, fontsize=10)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    return out_path


def plot_training(hist, out_path):
    """hist: {'loss': [...], 'epoch': [...], 'val': [...]}"""
    fig, ax = plt.subplots(figsize=(7.2, 4.0))
    ep = hist.get('epoch', list(range(1, len(hist['loss']) + 1)))
    ax.plot(ep, hist['loss'], color='#2f6fd0', lw=1.6, label='train loss (MSE)')
    if hist.get('val'):
        # hold-out 只在 BC 阶段记录（DAgger 各轮的留出集重叠，没有可比性），
        # 长度短于全程时只画前缀
        ax.plot(ep[:len(hist['val'])], hist['val'], color='#d4692f',
                lw=1.4, ls='--', label='hold-out loss (BC only)')
    if hist.get('stage'):
        for x, name in hist['stage']:
            ax.axvline(x, color='#999', lw=0.8, ls=':')
            ax.text(x, ax.get_ylim()[1] * 0.92, name, fontsize=8, color='#666',
                    rotation=90, va='top', ha='right')
    ax.set_xlabel('training step (10 batches)')
    ax.set_ylabel('MSE')
    ax.set_title('End-to-end CNN behaviour cloning + DAgger', fontsize=11)
    ax.grid(alpha=0.3, linestyle=':')
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    return out_path


def _draw_world(ax, world, truth=True):
    for x0, y0, x1, y1 in world.rects:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0,
                               facecolor='#b9bec7' if truth else 'none',
                               edgecolor='none' if truth else '#b9bec7',
                               lw=0.8, ls='--', zorder=1))
    for cx, cy, cid in world.landmarks:
        ax.add_patch(Circle((cx, cy), P.LM_RADIUS, facecolor='#c0392b',
                            edgecolor='none', zorder=3))
    ax.plot([world.start[0]], [world.start[1]], 'o', ms=8, mfc='#2f9e6b',
            mec='k', mew=0.6, zorder=5)
    ax.plot([world.goal[0]], [world.goal[1]], '*', ms=16, mfc='#f1c40f',
            mec='k', mew=0.6, zorder=5)
    ax.set_xlim(0, P.WORLD_W)
    ax.set_ylim(0, P.WORLD_H)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')


def plot_traj(world, trajs, out_path, title=''):
    """trajs: [(name, color, (T,3) array)]"""
    fig, ax = plt.subplots(figsize=(9.0, 6.6))
    _draw_world(ax, world)
    for name, color, tr in trajs:
        if len(tr) == 0:
            continue
        ax.plot(tr[:, 0], tr[:, 1], color=color, lw=1.8, label=name, zorder=4)
        ax.plot(tr[-1, 0], tr[-1, 1], 'o', ms=5, color=color, zorder=4)
    ax.set_title(title or 'Trajectories (top view)', fontsize=11)
    ax.legend(frameon=False, loc='upper left', fontsize=9)
    ax.grid(alpha=0.25, linestyle=':')
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    return out_path


def make_gif(frames, out_path, fps=10, size=None):
    """frames: list of RGB uint8 arrays；用 PIL 合成 GIF。"""
    from PIL import Image

    if not frames:
        return None
    imgs = []
    for f in frames:
        im = Image.fromarray(f)
        if size is not None:
            im = im.resize(size, Image.BILINEAR)
        imgs.append(im.convert('P', palette=Image.ADAPTIVE, colors=128))
    imgs[0].save(out_path, save_all=True, append_images=imgs[1:],
                 duration=int(1000 / fps), loop=0, optimize=True)
    return out_path


def render_panel(rgb, depth, grid, traj, world, goal, pose, note=''):
    """把一帧相机图 + 俯视地图拼成一张 RGB 面板（给 gif 用）。"""
    fig = plt.figure(figsize=(8.6, 3.4), dpi=100)
    ax1 = fig.add_axes([0.02, 0.06, 0.30, 0.86])
    ax2 = fig.add_axes([0.36, 0.06, 0.62, 0.86])

    ax1.imshow(rgb)
    ax1.set_xticks([])
    ax1.set_yticks([])
    ax1.set_title('front camera 160x120', fontsize=9)
    for s in ax1.spines.values():
        s.set_color('#888')

    if grid is not None:
        prob = grid.prob()
        ax2.imshow(np.flipud(prob), extent=(0, P.WORLD_W, 0, P.WORLD_H),
                   cmap='gray_r', vmin=0, vmax=1, alpha=0.85,
                   interpolation='nearest', zorder=1)
    _draw_world(ax2, world, truth=False)
    if len(traj) > 1:
        tr = np.asarray(traj)
        ax2.plot(tr[:, 0], tr[:, 1], color='#2f6fd0', lw=1.6, zorder=6)
    ax2.plot([pose[0]], [pose[1]], 'o', ms=6, mfc='#e74c3c', mec='k',
             mew=0.5, zorder=7)
    ax2.plot([goal[0]], [goal[1]], '*', ms=14, mfc='#f1c40f', mec='k', zorder=7)
    ax2.set_title(note, fontsize=9)
    ax2.set_xlim(0, P.WORLD_W)
    ax2.set_ylim(0, P.WORLD_H)
    ax2.set_aspect('equal')
    ax2.tick_params(labelsize=7)

    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=100)
    plt.close(fig)
    buf.seek(0)
    from PIL import Image
    im = Image.open(buf).convert('RGB')
    return np.asarray(im)
