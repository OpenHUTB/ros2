"""一键运行整个模块：继电器整定 -> 参数筛选 -> 与手工试凑对比。

用法（不需要 ROS 环境）：
    python3 -m pid_auto_tuning.main
或安装后：
    ros2 run pid_auto_tuning main
结果写入当前目录下 resource/step_response.csv，并打印指标表。
"""

import csv
import math
import os

from .autotune import select_best_gains, simulate_closed_loop
from .relay_tune import run_relay_feedback

# 手工试凑的三组参数：都只用 P、I，没有微分项，覆盖保守/常规/积极三种水平
MANUAL_SETS = [
    ('保守试凑', (0.5, 0.4, 0.0)),
    ('常规试凑', (1.5, 0.8, 0.0)),
    ('积极试凑', (3.0, 1.0, 0.0)),
]
SETPOINT = math.pi / 2.0


def _fmt(metrics):
    rise = f"{metrics['rise_time']:5.2f}s" if metrics['rise_time'] is not None else '     —'
    settle = f"{metrics['settle_time']:5.2f}s" if metrics['settle_time'] is not None else '  未收敛'
    error = f"{metrics['steady_state_error']:6.3f}rad" if metrics['steady_state_error'] is not None else '      —'
    return (f"{metrics['overshoot_pct']:6.1f}%  "
            f"{rise}  "
            f"{settle}  "
            f"{error}  "
            f"{metrics['iae']:6.2f}")


def main():
    print('被控对象: 一阶惯性 + 纯延迟 (gain=1.0, tau=0.40s, delay=0.10s)')
    print(f'目标航向: {math.degrees(SETPOINT):.1f} deg')

    relay = run_relay_feedback(setpoint=SETPOINT)
    print(f'\n继电器反馈实验: Ku={relay["ku"]:.3f}, Tu={relay["tu"]:.3f}s, '
          f'振荡幅值={relay["amplitude"]:.3f}rad')

    auto_gains, auto_metrics, info = select_best_gains(relay['ku'], relay['tu'],
                                                       setpoint=SETPOINT)
    kp_s, ti_s, td_s = info['scales']
    boundary = '（落在网格边界，建议扩大搜索范围复核）' if info['hit_boundary'] else '（网格内部最优点）'
    print(f'参数筛选: 来源={info["source"]}, '
          f'比例=(kp:{kp_s}, ti:{ti_s}, td:{td_s}){boundary}')

    rows = []
    for name, gains in MANUAL_SETS:
        rows.append((name, gains, simulate_closed_loop(gains, setpoint=SETPOINT)))

    header = f"{'方案':<14}{'超调':>7}{'上升':>8}{'调节':>9}{'稳态误差':>12}{'IAE':>8}"
    print('\n' + header)
    print('-' * 58)
    for name, _, metrics in rows:
        print(f"{name:<14}{_fmt(metrics)}")
    print(f"{'自动整定':<14}{_fmt(auto_metrics)}")

    print(f"\n自动整定参数: Kp={auto_gains[0]:.3f}, Ki={auto_gains[1]:.3f}, Kd={auto_gains[2]:.4f}")
    for name, gains, _ in rows:
        print(f"{name}参数: Kp={gains[0]:.3f}, Ki={gains[1]:.3f}, Kd={gains[2]:.4f}")

    path = _write_csv(rows, auto_metrics)
    print(f'\n阶跃响应数据已写入 {path}')


def _write_csv(rows, auto):
    path = os.path.abspath(os.path.join('resource', 'step_response.csv'))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        header = ['t_s'] + [f"{name}_theta_rad" for name, _, _ in rows] + ['auto_theta_rad']
        writer.writerow(header)
        n = min([len(auto['t'])] + [len(m['t']) for _, _, m in rows])
        for i in range(n):
            writer.writerow([f"{auto['t'][i]:.3f}"]
                            + [f"{m['y'][i]:.4f}" for _, _, m in rows]
                            + [f"{auto['y'][i]:.4f}"])
    return path


if __name__ == '__main__':
    main()
