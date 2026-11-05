"""一键运行整个模块：继电器整定 -> 参数筛选 -> 与手工试凑对比。

用法（不需要 ROS 环境）：
    python3 -m pid_auto_tuning.main
或安装后：
    ros2 run pid_auto_tuning main
结果写入 resource/step_response.csv，并打印指标表。
"""

import csv
import math
import os

from .autotune import select_best_gains, simulate_closed_loop
from .relay_tune import run_relay_feedback

# 手工试凑的一组参数：只加了 P 和 I，没有微分项，模拟常见的手工调参结果
MANUAL_GAINS = (0.5, 0.4, 0.0)
SETPOINT = math.pi / 2.0


def _fmt(metrics):
    return (f"{metrics['overshoot_pct']:6.1f}%  "
            f"{metrics['rise_time']:5.2f}s  "
            f"{metrics['settle_time']:5.2f}s  "
            f"{metrics['steady_state_error']:6.3f}rad  "
            f"{metrics['iae']:6.2f}")


def main():
    print('被控对象: 一阶惯性 + 纯延迟 (gain=1.0, tau=0.40s, delay=0.10s)')
    print(f'目标航向: {math.degrees(SETPOINT):.1f} deg')

    relay = run_relay_feedback(setpoint=SETPOINT)
    print(f'\n继电器反馈实验: Ku={relay["ku"]:.3f}, Tu={relay["tu"]:.3f}s, '
          f'振荡幅值={relay["amplitude"]:.3f}rad')

    auto_gains, auto_metrics = select_best_gains(relay['ku'], relay['tu'], setpoint=SETPOINT)
    manual_metrics = simulate_closed_loop(MANUAL_GAINS, setpoint=SETPOINT)

    header = f"{'方案':<14}{'超调':>7}{'上升':>8}{'调节':>8}{'稳态误差':>12}{'IAE':>8}"
    print('\n' + header)
    print('-' * 58)
    print(f"{'手工试凑':<14}{_fmt(manual_metrics)}")
    print(f"{'自动整定':<14}{_fmt(auto_metrics)}")
    print(f"\n自动整定参数: Kp={auto_gains[0]:.3f}, Ki={auto_gains[1]:.3f}, Kd={auto_gains[2]:.4f}")
    print(f"手工试凑参数: Kp={MANUAL_GAINS[0]:.3f}, Ki={MANUAL_GAINS[1]:.3f}, Kd={MANUAL_GAINS[2]:.4f}")

    _write_csv(manual_metrics, auto_metrics)
    print('\n阶跃响应数据已写入 resource/step_response.csv')


def _write_csv(manual, auto):
    path = os.path.join(os.getcwd(), 'resource', 'step_response.csv')
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['t_s', 'manual_theta_rad', 'auto_theta_rad'])
        n = min(len(manual['t']), len(auto['t']))
        for i in range(n):
            writer.writerow([f"{manual['t'][i]:.3f}",
                             f"{manual['y'][i]:.4f}",
                             f"{auto['y'][i]:.4f}"])


if __name__ == '__main__':
    main()
