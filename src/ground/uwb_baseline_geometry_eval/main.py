"""
UWB 多基站测距定位布站几何精度对比评测（纯 Python 数值仿真）

依赖: numpy, matplotlib, Pillow
使用: python main.py
说明: 在 100m×100m 区域内布置四种典型基站构型（等边三角形、正方形、
      一字形、L 形），对每种构型在区域网格上计算几何精度因子 GDOP，
      并通过蒙特卡洛测距仿真（测距叠加高斯噪声、最小二乘三边定位）统计
      定位 RMSE、95% 误差半径与最大误差，量化布站几何对室内/场地测距
      定位精度的影响。所有结果写入本脚本同级的 results/ 目录，
      与运行时的工作目录无关。
"""
import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use('Agg')  # 无图形界面环境下后台出图
import matplotlib.pyplot as plt
from PIL import Image

# 结果目录固定在脚本同级，避免从其他目录运行时把结果写到别处
RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')

# ---------- 仿真参数 ----------
AREA = 100.0              # 区域边长 (m)
GRID_N = 41               # GDOP 网格单边点数
SIGMA_RANGE = 0.3         # 测距噪声标准差 (m)
MC_POINTS = 200           # 蒙特卡洛目标点数
GN_ITERS = 6              # 三边定位高斯-牛顿迭代次数
GN_DAMPING = 0.6          # 迭代步长阻尼，抑制病态几何下跳飞
SEED = 20261111           # 随机种子，保证噪声序列可复现

# 四种典型布站构型：站名 -> 基站坐标列表 (m)
BASELINES = {
    'triangle': [(50.0, 10.0), (10.0, 90.0), (90.0, 90.0)],
    'square':   [(15.0, 15.0), (85.0, 15.0), (85.0, 85.0), (15.0, 85.0)],
    'line':     [(15.0, 50.0), (38.0, 50.0), (62.0, 50.0), (85.0, 50.0)],
    'l_shape':  [(15.0, 15.0), (85.0, 15.0), (15.0, 85.0)],
}

CONFIG_LABEL = {
    'triangle': 'Triangle (3)',
    'square': 'Square (4)',
    'line': 'Line (4, degenerate)',
    'l_shape': 'L-shape (3)',
}


# ---------- 几何精度因子 ----------
def gdop_at(point, baselines):
    """计算某点的 GDOP：sqrt(trace((J^T J)^-1))。

    J 每行是测距方向单位向量 (p-b_i)/d_i；几何退化（共线/共点）时
    J^T J 接近奇异，此时返回 nan 由调用方剔除。
    """
    baselines = np.asarray(baselines, dtype=float)
    J = []
    for b in baselines:
        d = np.linalg.norm(point - b)
        if d < 1e-6:
            return float('nan')
        J.append((point - b) / d)
    J = np.asarray(J)
    try:
        cov = np.linalg.inv(J.T @ J)
    except np.linalg.LinAlgError:
        return float('nan')
    return math.sqrt(float(np.trace(cov)))


def gdop_grid(baselines):
    """在区域网格上扫描 GDOP，返回坐标网格与 GDOP 矩阵。"""
    xs = np.linspace(0.0, AREA, GRID_N)
    ys = np.linspace(0.0, AREA, GRID_N)
    grid = np.full((GRID_N, GRID_N), np.nan)
    for i, y in enumerate(ys):
        for j, x in enumerate(xs):
            grid[i, j] = gdop_at(np.array([x, y]), baselines)
    return xs, ys, grid


# ---------- 最小二乘三边定位 ----------
def trilaterate(baselines, ranges, iters=GN_ITERS, damping=GN_DAMPING):
    """带阻尼的高斯-牛顿迭代求解二维位置。

    初值取基站质心；每步用当前位置线性化测距方程，最小二乘解位置修正量。
    """
    baselines = np.asarray(baselines, dtype=float)
    p = baselines.mean(axis=0)
    for _ in range(iters):
        H, h = [], []
        for b, r in zip(baselines, ranges):
            d = np.linalg.norm(p - b)
            H.append((p - b) / max(d, 1e-6))
            h.append(r - d)
        H = np.asarray(H)
        h = np.asarray(h)
        try:
            dp = np.linalg.lstsq(H, h, rcond=None)[0]
        except np.linalg.LinAlgError:
            break
        p = p + damping * dp
    return p


def monte_carlo(baselines, rng, n_points=MC_POINTS):
    """随机目标点叠加测距噪声后定位，返回各点定位误差 (m)。"""
    baselines = np.asarray(baselines, dtype=float)
    errors = []
    for _ in range(n_points):
        p_true = rng.uniform(10.0, AREA - 10.0, size=2)
        true_dist = np.linalg.norm(baselines - p_true, axis=1)
        ranges = true_dist + rng.normal(0.0, SIGMA_RANGE, size=len(baselines))
        p_est = trilaterate(baselines, ranges)
        errors.append(float(np.linalg.norm(p_est - p_true)))
    return np.asarray(errors)


# ---------- 指标汇总 ----------
def evaluate(rng):
    """对每种构型计算网格 GDOP 统计量与蒙特卡洛定位误差指标。"""
    summary = []
    grids = {}
    mc_errors = {}
    for name, baselines in BASELINES.items():
        xs, ys, grid = gdop_grid(baselines)
        valid = grid[~np.isnan(grid)]
        errors = monte_carlo(baselines, rng)
        rmse = float(np.sqrt(np.mean(errors ** 2)))
        p95 = float(np.percentile(errors, 95))
        summary.append({
            'configuration': name,
            'stations': len(baselines),
            'mean_gdop': round(float(np.mean(valid)), 3),
            'max_gdop': round(float(np.max(valid)), 3),
            'rmse_m': round(rmse, 3),
            'p95_error_m': round(p95, 3),
            'max_error_m': round(float(np.max(errors)), 3),
        })
        grids[name] = (xs, ys, grid)
        mc_errors[name] = errors
    return summary, grids, mc_errors


# ---------- 绘图 ----------
def plot_layout(grids, out_path):
    """四种构型基站布局俯视图。"""
    fig, axes = plt.subplots(2, 2, figsize=(10, 9))
    for ax, (name, baselines) in zip(axes.flat, BASELINES.items()):
        bs = np.asarray(baselines, dtype=float)
        ax.scatter(bs[:, 0], bs[:, 1], c='black', marker='^', s=90, label='base station')
        for k, b in enumerate(bs):
            ax.annotate(f'B{k+1}', (b[0] + 2, b[1] + 2), fontsize=9)
        ax.set_xlim(0, AREA)
        ax.set_ylim(0, AREA)
        ax.set_title(CONFIG_LABEL[name])
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.grid(alpha=0.3)
        ax.set_aspect('equal')
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_gdop_heatmap(grids, out_path):
    """2×2 GDOP 热力图，统一色标便于横向比较。"""
    vmax = np.nanmax([g[2] for g in grids.values()])
    fig, axes = plt.subplots(2, 2, figsize=(10, 9))
    for ax, (name, (xs, ys, grid)) in zip(axes.flat, grids.items()):
        mesh = ax.pcolormesh(xs, ys, grid, cmap='viridis', vmin=0, vmax=vmax, shading='auto')
        bs = np.asarray(BASELINES[name], dtype=float)
        ax.scatter(bs[:, 0], bs[:, 1], c='red', marker='^', s=70)
        ax.set_title(CONFIG_LABEL[name])
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_aspect('equal')
        fig.colorbar(mesh, ax=ax, label='GDOP')
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_error_scatter(mc_errors, rng, out_path):
    """蒙特卡洛估计落点散点：真值与估计位置对比。"""
    fig, axes = plt.subplots(2, 2, figsize=(10, 9))
    for ax, name in zip(axes.flat, BASELINES):
        bs = np.asarray(BASELINES[name], dtype=float)
        # 复现一组真值点用于画散点（与指标统计同一随机序列）
        truths = rng.uniform(10.0, AREA - 10.0, size=(MC_POINTS, 2))
        ests = []
        for p_true in truths:
            true_dist = np.linalg.norm(bs - p_true, axis=1)
            ranges = true_dist + rng.normal(0.0, SIGMA_RANGE, size=len(bs))
            ests.append(trilaterate(bs, ranges))
        ests = np.asarray(ests)
        ax.scatter(truths[:, 0], truths[:, 1], c='gray', s=12, label='true')
        ax.scatter(ests[:, 0], ests[:, 1], c='tab:orange', s=12, alpha=0.7, label='estimated')
        ax.scatter(bs[:, 0], bs[:, 1], c='black', marker='^', s=70)
        ax.set_xlim(0, AREA)
        ax.set_ylim(0, AREA)
        ax.set_title(CONFIG_LABEL[name])
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)
        ax.set_aspect('equal')
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_metrics_bar(summary, out_path):
    """RMSE、95% 误差、平均 GDOP 分组柱状图。"""
    names = list(BASELINES.keys())
    labels = [CONFIG_LABEL[n] for n in names]
    rmse = [next(r['rmse_m'] for r in summary if r['configuration'] == n) for n in names]
    p95 = [next(r['p95_error_m'] for r in summary if r['configuration'] == n) for n in names]
    gdop = [next(r['mean_gdop'] for r in summary if r['configuration'] == n) for n in names]

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))
    x = np.arange(len(names))
    # RMSE 与 p95 跨度大（退化构型可达数十米），用对数纵轴保证小误差柱也可读
    axes[0].bar(x, rmse, color='tab:blue')
    axes[0].set_yscale('log')
    axes[0].set_title('Positioning RMSE (m, log)')
    axes[1].bar(x, p95, color='tab:green')
    axes[1].set_yscale('log')
    axes[1].set_title('95% error radius (m, log)')
    axes[2].bar(x, gdop, color='tab:purple')
    axes[2].set_title('Mean GDOP')
    for ax, vals in zip(axes, [rmse, p95, gdop]):
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=20, fontsize=8)
        ax.grid(alpha=0.3, axis='y')
        for xi, v in zip(x, vals):
            ax.text(xi, v, f'{v:g}', ha='center', va='bottom', fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def render_demo_frame(trajectory, estimates_by_name, idx, out_size=(800, 450)):
    """渲染一帧演示画面：目标真值轨迹与两种构型估计位置。"""
    fig, (ax, ax_err) = plt.subplots(1, 2, figsize=(11, 4.5),
                                     gridspec_kw={'width_ratios': [1.2, 1]})
    # 左：俯视图
    bs_square = np.asarray(BASELINES['square'], dtype=float)
    bs_line = np.asarray(BASELINES['line'], dtype=float)
    ax.scatter(bs_square[:, 0], bs_square[:, 1], c='tab:blue', marker='^', s=70)
    ax.scatter(bs_line[:, 0], bs_line[:, 1], c='tab:red', marker='x', s=70)
    cur = min(idx, len(trajectory))
    ax.plot(trajectory[:cur, 0], trajectory[:cur, 1], 'k--', lw=1.2, label='true path')
    for name, color in [('square', 'tab:blue'), ('line', 'tab:red')]:
        est = estimates_by_name[name]
        ax.plot(est[:cur, 0], est[:cur, 1], lw=1.2, color=color, alpha=0.8,
                label=f'{name} estimate')
    ax.set_xlim(0, AREA)
    ax.set_ylim(0, AREA)
    ax.set_title('UWB positioning demo')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    ax.set_aspect('equal')
    # 右：误差随时间曲线
    for name, color in [('square', 'tab:blue'), ('line', 'tab:red')]:
        err = estimates_by_name[name + '_err']
        ax_err.plot(range(cur), err[:cur], color=color, lw=1.2, label=name)
    ax_err.set_xlabel('frame')
    ax_err.set_ylabel('error (m)')
    ax_err.set_title('Positioning error')
    ax_err.legend(fontsize=8)
    ax_err.grid(alpha=0.3)
    fig.tight_layout()
    fig.canvas.draw()
    buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
    img = Image.fromarray(buf.reshape(fig.canvas.get_width_height()[::-1] + (4,)))
    plt.close(fig)
    return img.convert('RGB').resize(out_size)


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)
    rng = np.random.default_rng(SEED)

    summary, grids, mc_errors = evaluate(rng)

    # ---------- 写指标 CSV ----------
    csv_path = os.path.join(RESULTS_DIR, 'metrics.csv')
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(
            f, fieldnames=['configuration', 'stations', 'mean_gdop', 'max_gdop',
                            'rmse_m', 'p95_error_m', 'max_error_m'])
        writer.writeheader()
        writer.writerows(summary)

    # ---------- 静态图 ----------
    plot_layout(grids, os.path.join(RESULTS_DIR, 'uwb_baseline_layout.png'))
    plot_gdop_heatmap(grids, os.path.join(RESULTS_DIR, 'uwb_baseline_gdop_heatmap.png'))
    plot_error_scatter(mc_errors, np.random.default_rng(SEED + 1),
                        os.path.join(RESULTS_DIR, 'uwb_baseline_error_scatter.png'))
    plot_metrics_bar(summary, os.path.join(RESULTS_DIR, 'uwb_baseline_metrics.png'))

    # ---------- 演示动图：目标沿椭圆轨迹移动，对比 square 与 line 构型 ----------
    t = np.linspace(0, 2 * math.pi, 40)
    trajectory = np.column_stack([50.0 + 30.0 * np.cos(t), 50.0 + 25.0 * np.sin(t)])
    estimates_by_name = {}
    for name in ['square', 'line']:
        bs = np.asarray(BASELINES[name], dtype=float)
        ests, errs = [], []
        for p_true in trajectory:
            true_dist = np.linalg.norm(bs - p_true, axis=1)
            ranges = true_dist + rng.normal(0.0, SIGMA_RANGE, size=len(bs))
            p_est = trilaterate(bs, ranges)
            ests.append(p_est)
            errs.append(float(np.linalg.norm(p_est - p_true)))
        estimates_by_name[name] = np.asarray(ests)
        estimates_by_name[name + '_err'] = np.asarray(errs)

    frames = [render_demo_frame(trajectory, estimates_by_name, idx)
              for idx in range(2, len(trajectory) + 1, 2)]
    if frames:
        duration = max(80, int(1000 / len(frames) * 10))
        frames[0].save(os.path.join(RESULTS_DIR, 'uwb_baseline_demo.gif'),
                       save_all=True, append_images=frames[1:],
                       duration=duration, loop=0, optimize=True)

    # ---------- 控制台输出 ----------
    for row in summary:
        print(f"{row['configuration']:9s} stations={row['stations']} "
              f"mean_gdop={row['mean_gdop']:.3f} max_gdop={row['max_gdop']:.3f} "
              f"rmse={row['rmse_m']:.3f} p95={row['p95_error_m']:.3f} "
              f"max={row['max_error_m']:.3f}")
    print(f'实验完成，结果保存在 {RESULTS_DIR} 目录')


if __name__ == '__main__':
    main()
