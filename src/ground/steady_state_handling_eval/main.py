"""
车辆稳态操稳特性对比仿真（纯 Python 数值仿真）

依赖: numpy, matplotlib, Pillow
使用: python main.py
说明: 基于线性自行车模型，在固定质量、轴距与轮胎侧偏刚度下，改变质心
      前后位置（前重 / 中性 / 后重），计算稳态转向梯度 K，判定每种布置属于
      不足转向（understeer）、中性转向（neutral）还是过度转向（oversteer），
      并绘制横摆角速度增益、侧向加速度增益随车速的变化曲线，以及前后轴侧偏角
      随侧向加速度的变化曲线。所有结果写入脚本同级的 results/ 目录。
"""
import os
import csv

import numpy as np
import matplotlib
matplotlib.use('Agg')  # 无图形界面环境下后台出图
import matplotlib.pyplot as plt
from PIL import Image

RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')

# ---------- 车辆参数 ----------
MASS = 1500.0          # 整车质量 (kg)
WHEELBASE = 2.7        # 轴距 L = lf + lr (m)
CF = 80000.0           # 前轴综合侧偏刚度 (N/rad)
CR = 80000.0           # 后轴综合侧偏刚度 (N/rad)
G = 9.81               # 重力加速度 (m/s^2)
V_MAX = 40.0           # 仿真车速上限 (m/s)
AY_MAX = 8.0           # 侧偏角曲线侧向加速度上限 (m/s^2)
NEUTRAL_EPS = 1e-4     # 中性转向判定阈值

# 三种质心布置：名称 -> (质心到前轴距离 lf, 质心到后轴距离 lr)
LAYOUTS = {
    'front_weighted': (1.05, 1.65),   # 质心靠前（前重）
    'neutral':        (1.35, 1.35),   # 质心居中
    'rear_weighted':  (1.65, 1.05),   # 质心靠后（后重）
}

LABEL = {
    'front_weighted': 'Front-weighted',
    'neutral':        'Neutral',
    'rear_weighted':  'Rear-weighted',
}

COLOR = {
    'front_weighted': 'tab:blue',
    'neutral':        'tab:green',
    'rear_weighted':  'tab:red',
}


def analyze(lf, lr):
    """计算一种布置的轴荷、转向梯度、操稳判定与临界车速。"""
    wf = MASS * lr / WHEELBASE          # 前轴静载荷 (N)
    wr = MASS * lf / WHEELBASE          # 后轴静载荷 (N)
    k = wf / CF - wr / CR               # 稳态转向梯度 (s^2/m)
    if k > NEUTRAL_EPS:
        kind = 'understeer'
    elif k < -NEUTRAL_EPS:
        kind = 'oversteer'
    else:
        kind = 'neutral'
    # 过度转向在临界车速处分母趋于零，横摆增益发散
    if k < -NEUTRAL_EPS:
        v_crit = float(np.sqrt(WHEELBASE / abs(k)))
    else:
        v_crit = float('inf')
    return {'wf': wf, 'wr': wr, 'k': k, 'kind': kind, 'v_crit': v_crit}


def yaw_gain(v, k):
    """稳态横摆角速度增益 r/δ = v / (L + K·v²)。"""
    denom = WHEELBASE + k * v * v
    return np.where(denom > 1e-6, v / denom, np.nan)


def lateral_gain(v, k):
    """稳态侧向加速度增益 ay/δ = v² / (L + K·v²)。"""
    denom = WHEELBASE + k * v * v
    return np.where(denom > 1e-6, v * v / denom, np.nan)


def summarize():
    """汇总三种布置的指标。"""
    rows, info = [], {}
    for name, (lf, lr) in LAYOUTS.items():
        a = analyze(lf, lr)
        v20 = 20.0
        rows.append({
            'layout': name,
            'lf_m': round(lf, 3),
            'lr_m': round(lr, 3),
            'front_load_N': round(a['wf'], 1),
            'rear_load_N': round(a['wr'], 1),
            'steer_gradient_K': round(a['k'], 6),
            'handling': a['kind'],
            'critical_speed_ms': (round(a['v_crit'], 2)
                                  if np.isfinite(a['v_crit']) else 'inf'),
            'yaw_gain_at_20ms': round(float(yaw_gain(v20, a['k'])), 4),
        })
        info[name] = a
    return rows, info


# ---------- 绘图 ----------
def plot_gradient(rows, out_path):
    """转向梯度 K 柱状图，正负着色并标注操稳判定。"""
    names = list(LAYOUTS.keys())
    ks = [next(r['steer_gradient_K'] for r in rows if r['layout'] == n) for n in names]
    kinds = [next(r['handling'] for r in rows if r['layout'] == n) for n in names]
    colors = ['tab:blue' if k > 0 else ('tab:red' if k < 0 else 'tab:green') for k in ks]

    fig, ax = plt.subplots(figsize=(9, 4.6))
    x = np.arange(len(names))
    bars = ax.bar(x, ks, color=colors, width=0.55)
    ax.axhline(0, color='black', lw=1)
    ax.set_xticks(x)
    ax.set_xticklabels([LABEL[n] for n in names])
    ax.set_ylabel('Steer gradient K (s²/m)')
    ax.set_title('Steady-state understeer gradient by CG position')
    ax.grid(alpha=0.3, axis='y')
    for bar, k, kind in zip(bars, ks, kinds):
        ax.text(bar.get_x() + bar.get_width() / 2, k, f'K={k:.4f}\n{kind}',
                ha='center', va='bottom' if k >= 0 else 'top', fontsize=9)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_gains(info, out_path):
    """横摆增益与侧向加速度增益随车速曲线（双子图）。"""
    v = np.linspace(0.1, V_MAX, 400)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.6))
    for name, a in info.items():
        yg = yaw_gain(v, a['k'])
        lg = lateral_gain(v, a['k'])
        # 过度转向在临界车速附近增益发散，提前截断失稳段，避免纵轴被拉爆
        if a['kind'] == 'oversteer':
            safe = v <= 0.985 * a['v_crit']
            yg[~safe] = np.nan
            lg[~safe] = np.nan
            ax1.axvline(a['v_crit'], color=COLOR[name], ls='--', lw=1, alpha=0.7)
            ax2.axvline(a['v_crit'], color=COLOR[name], ls='--', lw=1, alpha=0.7)
            ax1.text(a['v_crit'], 15.0, f'v_crit={a["v_crit"]:.1f}',
                     color=COLOR[name], fontsize=8, rotation=90, va='top')
        ax1.plot(v, yg, color=COLOR[name], lw=1.6, label=LABEL[name])
        ax2.plot(v, lg, color=COLOR[name], lw=1.6, label=LABEL[name])
    ax1.set_xlabel('v (m/s)'); ax1.set_ylabel('Yaw gain r/δ (1/s)')
    ax1.set_title('Yaw-rate gain vs speed'); ax1.legend(fontsize=8); ax1.grid(alpha=0.3)
    ax1.set_ylim(0, 16)
    ax2.set_xlabel('v (m/s)'); ax2.set_ylabel('Lateral accel. gain ay/δ (m/s²/rad)')
    ax2.set_title('Lateral acceleration gain vs speed'); ax2.legend(fontsize=8); ax2.grid(alpha=0.3)
    ax2.set_ylim(0, 600)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_slip_angles(info, out_path):
    """前后轴侧偏角随侧向加速度曲线。"""
    ay = np.linspace(0.0, AY_MAX, 100)
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.2), sharey=True)
    for ax, (name, a) in zip(axes, info.items()):
        alpha_f = (a['wf'] / CF) * (ay / G)
        alpha_r = (a['wr'] / CR) * (ay / G)
        ax.plot(ay, np.degrees(alpha_f), color='tab:blue', lw=1.6, label='front slip')
        ax.plot(ay, np.degrees(alpha_r), color='tab:orange', lw=1.6, label='rear slip')
        ax.set_title(LABEL[name])
        ax.set_xlabel('ay (m/s²)'); ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    axes[0].set_ylabel('Slip angle (deg)')
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def render_demo_frame(info, v_cur, out_size=(800, 420)):
    """渲染一帧演示：横摆增益曲线 + 当前车速标记。"""
    v = np.linspace(0.1, V_MAX, 300)
    fig, ax = plt.subplots(figsize=(10, 4.6))
    for name, a in info.items():
        yg = yaw_gain(v, a['k'])
        if a['kind'] == 'oversteer':
            yg[v > 0.985 * a['v_crit']] = np.nan
        ax.plot(v, yg, color=COLOR[name], lw=1.4, label=LABEL[name])
    ax.axvline(v_cur, color='black', ls=':', lw=1.2)
    ax.set_xlim(0, V_MAX); ax.set_ylim(0, 16)
    ax.set_xlabel('v (m/s)'); ax.set_ylabel('Yaw gain r/δ (1/s)')
    ax.set_title(f'Steady-state handling @ v={v_cur:.1f} m/s')
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.canvas.draw()
    buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
    img = Image.fromarray(buf.reshape(fig.canvas.get_width_height()[::-1] + (4,)))
    plt.close(fig)
    return img.convert('RGB').resize(out_size)


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)
    rows, info = summarize()

    csv_path = os.path.join(RESULTS_DIR, 'metrics.csv')
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(
            f, fieldnames=['layout', 'lf_m', 'lr_m', 'front_load_N', 'rear_load_N',
                           'steer_gradient_K', 'handling', 'critical_speed_ms',
                           'yaw_gain_at_20ms'])
        writer.writeheader()
        writer.writerows(rows)

    plot_gradient(rows, os.path.join(RESULTS_DIR, 'handling_gradient.png'))
    plot_gains(info, os.path.join(RESULTS_DIR, 'handling_gains.png'))
    plot_slip_angles(info, os.path.join(RESULTS_DIR, 'handling_slip_angle.png'))

    speeds = np.linspace(2.0, V_MAX, 18)
    frames = [render_demo_frame(info, float(v)) for v in speeds]
    if frames:
        duration = max(80, int(1000 / len(frames) * 10))
        frames[0].save(os.path.join(RESULTS_DIR, 'handling_demo.gif'),
                       save_all=True, append_images=frames[1:],
                       duration=duration, loop=0, optimize=True)

    for r in rows:
        print(f"{r['layout']:16s} K={r['steer_gradient_K']:.5f} "
              f"handling={r['handling']} v_crit={r['critical_speed_ms']}")
    print(f'实验完成，结果保存在 {RESULTS_DIR} 目录')


if __name__ == '__main__':
    main()
