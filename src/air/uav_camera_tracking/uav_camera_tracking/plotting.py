# -*- coding: utf-8 -*-
"""plotting.py —— 训练曲线、场景对比图与跟踪动图（GIF）。

约定：PNG 控制在 3 MB 内、GIF 控制在 10 MB 内（仓库要求），
所以动图只画三块面板并降采样帧率。
"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt          # noqa: E402
import numpy as np                        # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402
from matplotlib.ticker import MaxNLocator                 # noqa: E402

from . import params as P                 # noqa: E402

plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


def plot_training(histories, path):
    """三张子图：感知网络、控制网络、端到端网络的训练/验证损失。"""
    titles = [('detector', 'Perception net (image -> pixel)'),
              ('control', 'Control net (error -> velocity)'),
              ('e2e', 'End-to-end net (image -> velocity)')]
    fig, axes = plt.subplots(1, 3, figsize=(10, 2.8))
    for ax, (key, title) in zip(axes, titles):
        h = histories.get(key)
        if not h:
            ax.axis('off')
            continue
        ax.plot(h['loss'], lw=1.4, label='train')
        ax.plot(h['val_loss'], lw=1.2, ls='--', label='val')
        ax.set_title(title, fontsize=9)
        ax.set_xlabel('epoch', fontsize=8)
        ax.set_ylabel('loss', fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def plot_comparison(summary, path):
    """按场景分组的成功率柱状图 + 像素误差 RMS 折线。"""
    scenes = list(summary.keys())
    ctrls = list(next(iter(summary.values())).keys())
    x = np.arange(len(scenes))
    w = 0.8 / max(1, len(ctrls))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 3.2))
    colors = ['#4c72b0', '#dd8452', '#55a868']
    for i, c in enumerate(ctrls):
        succ = [summary[s][c]['success_rate'] * 100 for s in scenes]
        rms = [summary[s][c]['err_rms_mean'] for s in scenes]
        ax1.bar(x + (i - (len(ctrls) - 1) / 2) * w, succ, w,
                label=c, color=colors[i % len(colors)])
        ax2.plot(x, rms, 'o-', label=c, color=colors[i % len(colors)])
    ax1.set_xticks(x)
    ax1.set_xticklabels(scenes, fontsize=8)
    ax1.set_ylabel('success rate (%)', fontsize=9)
    ax1.set_ylim(0, 108)
    ax1.set_title('Success rate by scene', fontsize=9)
    ax1.grid(axis='y', alpha=0.3)
    ax1.legend(fontsize=7)
    ax2.set_xticks(x)
    ax2.set_xticklabels(scenes, fontsize=8)
    ax2.set_ylabel('pixel error RMS (norm.)', fontsize=9)
    ax2.set_title('Tracking error by scene', fontsize=9)
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def make_gif(scene, log, path, title='', fps=10):
    """三面板动图：三维航迹 / 机载相机画面 / 像素误差曲线。"""
    frames = log['frames']
    if not frames:
        return None
    n = len(frames)
    stride = max(1, int(round(1.0 / (fps * P.DT * 3))))
    idx = list(range(0, n, stride))
    t_all = log['t']
    err_all = log['err']

    fig = plt.figure(figsize=(9.2, 3.0), dpi=80)
    gs = fig.add_gridspec(1, 3, width_ratios=[1.15, 1.0, 1.2], wspace=0.28)
    ax3d = fig.add_subplot(gs[0, 0], projection='3d')
    axcam = fig.add_subplot(gs[0, 1])
    axerr = fig.add_subplot(gs[0, 2])

    p = log['p']
    tp = log['target']
    ax3d.plot(p[:, 0], p[:, 1], p[:, 2], lw=0.8, color='#4c72b0', alpha=0.6)
    ax3d.plot(tp[:, 0], tp[:, 1], np.zeros(len(tp)), lw=1.0,
              color='#c44e52', alpha=0.7, label='target')
    for cx, cy, hx, hy, hz in scene.walls:
        ax3d.plot([cx, cx], [cy, cy], [0.0, 2 * hz], color='0.4', lw=2.4)
    uav_dot, = ax3d.plot([], [], [], 'o', color='#4c72b0', ms=5)
    tgt_dot, = ax3d.plot([], [], [], 's', color='#c44e52', ms=5)
    ax3d.set_xlabel('x (m)', fontsize=7)
    ax3d.set_ylabel('y (m)', fontsize=7)
    ax3d.set_zlabel('')          # 竖直轴即高度 z(m)，标题里已说明，避免小图被裁切
    ax3d.tick_params(labelsize=6)
    # 坐标轴刻度限制数量，否则小图上 x 轴数字会挤成一团
    for axis in (ax3d.xaxis, ax3d.yaxis, ax3d.zaxis):
        axis.set_major_locator(MaxNLocator(4))
    ax3d.set_zlim(0.0, max(10.0, float(p[:, 2].max()) + 2.0))
    ax3d.set_box_aspect((1.0, 1.0, 0.55))
    ax3d.set_title(title or getattr(scene, 'key', ''), fontsize=8)
    ax3d.view_init(elev=22, azim=-58)

    im = axcam.imshow(frames[0])
    axcam.axis('off')
    axcam.set_title('onboard camera', fontsize=8)

    axerr.plot(t_all, err_all, lw=0.8, color='0.6')
    line_err, = axerr.plot([], [], lw=1.4, color='#dd8452')
    axerr.axhline(P.ERR_OK, ls='--', lw=0.8, color='#c44e52')
    axerr.set_xlim(0, t_all[-1])
    axerr.set_ylim(0, max(0.35, float(err_all.max()) * 1.1))
    axerr.set_xlabel('t (s)', fontsize=7)
    axerr.set_ylabel('pixel error (norm.)', fontsize=7)
    axerr.tick_params(labelsize=6)
    axerr.grid(alpha=0.3)
    txt = axerr.text(0.02, 0.92, '', transform=axerr.transAxes, fontsize=7)

    def update(k):
        im.set_data(frames[k])
        kk = int(round(k * stride * P.DT / P.DT))
        kk = min(kk, len(p) - 1)
        uav_dot.set_data([p[kk, 0]], [p[kk, 1]])
        uav_dot.set_3d_properties([p[kk, 2]])
        tgt_dot.set_data([tp[kk, 0]], [tp[kk, 1]])
        tgt_dot.set_3d_properties([0.0])
        line_err.set_data(t_all[:kk + 1], err_all[:kk + 1])
        txt.set_text('t=%.1fs  err=%.3f' % (t_all[kk], err_all[kk]))
        return im, uav_dot, tgt_dot, line_err, txt

    anim = FuncAnimation(fig, update, frames=idx, interval=100, blit=False)
    anim.save(path, writer=PillowWriter(fps=fps))
    plt.close(fig)
    return path
