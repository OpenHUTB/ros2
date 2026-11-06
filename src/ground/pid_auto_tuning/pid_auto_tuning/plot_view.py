"""可视化：左侧航向角阶跃响应，右侧航向误差曲线。

运行后弹出窗口，可直接截图：
    ros2 run pid_auto_tuning main_plot
使用 matplotlib 默认后端；如需 Tk 弹窗请安装 python3-tk。
"""

import math

import matplotlib.pyplot as plt

from .autotune import select_best_gains, simulate_closed_loop
from .relay_tune import run_relay_feedback

MANUAL_SETS = [
    ('保守试凑', (0.5, 0.4, 0.0)),
    ('常规试凑', (1.5, 0.8, 0.0)),
    ('积极试凑', (3.0, 1.0, 0.0)),
]
MANUAL_COLORS = ['tab:orange', 'tab:brown', 'tab:red']
SETPOINT = math.pi / 2.0


def main():
    plt.rcParams['font.sans-serif'] = ['Noto Sans CJK JP', 'WenQuanYi Micro Hei',
                                       'Droid Sans Fallback', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    plt.rcParams['mathtext.fontset'] = 'dejavusans'

    relay = run_relay_feedback(setpoint=SETPOINT)
    auto_gains, auto_metrics, _ = select_best_gains(relay['ku'], relay['tu'],
                                                    setpoint=SETPOINT)
    manual_metrics = [(name, simulate_closed_loop(gains, setpoint=SETPOINT))
                      for name, gains in MANUAL_SETS]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.4))
    manager = getattr(fig.canvas, 'manager', None)
    if manager is not None:
        try:
            manager.set_window_title('航向 PID 参数自整定')
        except Exception:
            pass

    for (name, metrics), color in zip(manual_metrics, MANUAL_COLORS):
        ax1.plot(metrics['t'], [math.degrees(v) for v in metrics['y']],
                 color=color, label=f"{name}  超调 {metrics['overshoot_pct']:.1f}%")
    ax1.plot(auto_metrics['t'], [math.degrees(v) for v in auto_metrics['y']],
             color='tab:green', linewidth=2.0,
             label=f"自动整定  超调 {auto_metrics['overshoot_pct']:.1f}%")
    ax1.axhline(math.degrees(SETPOINT), color='tab:gray', linestyle='--', linewidth=1,
                label='目标 90°')
    ax1.set_xlabel('时间 (s)')
    ax1.set_ylabel('航向角 (deg)')
    ax1.set_title('航向角阶跃响应')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='lower right', fontsize=9)

    for (name, metrics), color in zip(manual_metrics, MANUAL_COLORS):
        ax2.plot(metrics['t'], [math.degrees(SETPOINT - v) for v in metrics['y']],
                 color=color, label=name)
    ax2.plot(auto_metrics['t'], [math.degrees(SETPOINT - v) for v in auto_metrics['y']],
             color='tab:green', linewidth=2.0, label='自动整定')
    ax2.axhline(0.0, color='gray', linewidth=0.8)
    ax2.set_xlabel('时间 (s)')
    ax2.set_ylabel('航向误差 (deg)')
    ax2.set_title(f'误差收敛  Kp={auto_gains[0]:.2f} Ki={auto_gains[1]:.2f} Kd={auto_gains[2]:.2f}')
    ax2.grid(True, alpha=0.3, which='both')
    ax2.legend(loc='upper right', fontsize=9)

    fig.tight_layout()
    try:
        fig.canvas.manager.toolbar.destroy()
    except Exception:
        pass
    plt.show()


if __name__ == '__main__':
    main()
