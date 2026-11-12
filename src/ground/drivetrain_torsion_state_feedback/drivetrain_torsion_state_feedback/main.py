"""离线入口：扭振阻尼比与状态反馈极点配置的对比。

不需要 ROS 2，只用 Python 标准库::

    cd src/ground/drivetrain_torsion_state_feedback
    python3 -m drivetrain_torsion_state_feedback.main
"""

import argparse
import csv
import os
from pathlib import Path

from .drivetrain import TwoInertiaDrivetrain

TARGET_ZETA = 0.7
DEFAULT_DURATION = 0.30


def settle_time(samples, threshold=0.02):
    """|theta| 最后一次超过 ``threshold * theta0`` 的时刻（之后不再越界）。"""
    initial = abs(samples[0][1])
    if initial == 0.0:
        return 0.0
    band = threshold * initial
    last = None
    for t, theta, _omega, _u in samples:
        if abs(theta) > band:
            last = t
    if last is None:
        return 0.0
    if last >= samples[-1][0] - 1e-12:
        return None
    step = samples[1][0] - samples[0][0]
    return last + step


def peak(samples):
    return max(abs(theta) for _t, theta, _omega, _u in samples)


def build_parser():
    parser = argparse.ArgumentParser(
        prog='drivetrain_torsion_state_feedback.main',
        description='双惯量传动扭振的状态反馈极点配置演示')
    parser.add_argument('--jm', type=float, default=0.05, help='电机侧转动惯量 kg·m²')
    parser.add_argument('--jw', type=float, default=0.6, help='车轮侧转动惯量 kg·m²')
    parser.add_argument('--k', type=float, default=1200.0, help='轴扭转刚度 Nm/rad')
    parser.add_argument('--c', type=float, default=2.0, help='轴阻尼 Nm·s/rad')
    parser.add_argument('--zeta', type=float, default=TARGET_ZETA,
                        help='闭环目标阻尼比')
    parser.add_argument('--duration', type=float, default=DEFAULT_DURATION,
                        help='仿真时长（秒）')
    parser.add_argument('--output-dir', default=None,
                        help='CSV 输出目录，默认 ~/.ros/drivetrain_torsion_state_feedback')
    return parser


def default_output_dir():
    return (Path(os.environ.get('ROS_HOME', Path.home() / '.ros'))
            / 'drivetrain_torsion_state_feedback')


def main(argv=None):
    args = build_parser().parse_args(argv)
    output_dir = (Path(args.output_dir) if args.output_dir
                  else default_output_dir())

    plant = TwoInertiaDrivetrain(jm=args.jm, jw=args.jw, k=args.k, c=args.c)
    wn = plant.natural_frequency
    print(f'双惯量传动：Jm={plant.jm} kg·m²，Jw={plant.jw} kg·m²，'
          f'k={plant.k} Nm/rad，c={plant.c} N·m·s/rad')
    print(f'扭振固有频率 {wn:.1f} rad/s = {wn / 6.283185:.2f} Hz，'
          f'开环阻尼比 {plant.damping_ratio:.3f}')
    print(f'开环极点 {plant.poles()[0]:.3f} / {plant.poles()[1]:.3f}')

    print('\n能控性')
    ctrb = plant.controllability_matrix()
    print(f'  能控性矩阵 [B AB] = [[{ctrb[0][0]:.4f}, {ctrb[0][1]:.4f}], '
          f'[{ctrb[1][0]:.4f}, {ctrb[1][1]:.4f}]]')
    print(f'  det = {plant.controllability_determinant():.4f}  →  '
          f'{"可控" if plant.is_controllable() else "不可控"}')

    poles = plant.desired_poles(wn, args.zeta)
    gain = plant.ackermann_gain(poles)
    _a_cl, closed_poles, zeta_cl, wn_cl = plant.closed_loop(gain)
    print(f'\n期望极点 {poles[0]:.3f} / {poles[1]:.3f}（zeta={args.zeta:.2f}）')
    print(f'Ackermann 反馈增益 K = [{gain[0][0]:.4f}, {gain[0][1]:.4f}]')
    print(f'闭环极点 {closed_poles[0]:.3f} / {closed_poles[1]:.3f}  '
          f'→ zeta={zeta_cl:.3f}，wn={wn_cl:.1f} rad/s')

    open_samples = plant.simulate(gain=None, duration=args.duration)
    closed_samples = plant.simulate(gain=gain, duration=args.duration)

    print('\n自由响应对比（初始扭角 0.01 rad，u=0 vs u=-Kx）')
    header = (f"{'方案':<10}{'峰值扭角(rad)':>16}{'调节时间(s)':>14}"
              f"{'阻尼比':>10}{'残余扭角(rad)':>16}")
    print(header)
    print('-' * len(header))
    for name, samples, zeta in (('开环', open_samples, plant.damping_ratio),
                                ('状态反馈', closed_samples, zeta_cl)):
        settle = settle_time(samples)
        residual = abs(samples[-1][1])
        settle_text = '未收敛' if settle is None else f'{settle:.4f}'
        print(f'{name:<10}{peak(samples):>16.6f}{settle_text:>14}'
              f'{zeta:>10.3f}{residual:>16.6f}')

    print(f'\n调节时间判据：|theta| 持续小于初始值的 2%；'
          f'仿真 {args.duration:.2f} s')
    print(f'要达到临界阻尼（zeta=1）需要轴阻尼 c={plant.critical_damping():.3f} '
          f'N·m·s/rad，是当前 {plant.c:.1f} 的 {plant.critical_damping() / plant.c:.1f} 倍；'
          '状态反馈不需要改机械结构就能达到目标阻尼。')

    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / 'torsion_response.csv'
    with open(csv_path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(['t_s', 'theta_open_rad', 'omega_open',
                         'u_open', 'theta_closed_rad', 'omega_closed', 'u_closed'])
        length = min(len(open_samples), len(closed_samples))
        for index in range(length):
            t, theta_o, omega_o, u_o = open_samples[index]
            _t2, theta_c, omega_c, u_c = closed_samples[index]
            writer.writerow([f'{t:.5f}', f'{theta_o:.8f}', f'{omega_o:.6f}',
                             f'{u_o:.6f}', f'{theta_c:.8f}', f'{omega_c:.6f}',
                             f'{u_c:.6f}'])
    print(f'\n时序数据已写入 {csv_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
