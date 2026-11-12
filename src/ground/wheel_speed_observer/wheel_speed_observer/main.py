"""离线入口：龙伯格观测器估计传动轴扭角，并给出「带宽—噪声」权衡。

不需要 ROS 2，只用 Python 标准库::

    cd src/ground/wheel_speed_observer
    python3 -m wheel_speed_observer.main
"""

import argparse
import csv
import math
import os
from pathlib import Path

from .observer import DrivetrainObserver, error_settle_time

BANDWIDTH_FACTORS = (0.5, 1.0, 2.0, 3.0)
NOISE_STD = 0.05          # 车轮转速测量噪声（rad/s）
DURATION = 0.5


def rms(values):
    if not values:
        return 0.0
    return math.sqrt(sum(value * value for value in values) / len(values))


def build_parser():
    parser = argparse.ArgumentParser(
        prog='wheel_speed_observer.main',
        description='轮速传感器 + 龙伯格观测器估计传动轴扭角')
    parser.add_argument('--jw', type=float, default=0.6, help='车轮侧转动惯量 kg·m²')
    parser.add_argument('--k', type=float, default=1200.0, help='轴扭转刚度 Nm/rad')
    parser.add_argument('--c', type=float, default=2.0, help='轴阻尼 N·m·s/rad')
    parser.add_argument('--zeta', type=float, default=0.7, help='观测器目标阻尼比')
    parser.add_argument('--noise', type=float, default=NOISE_STD,
                        help='车轮转速测量噪声标准差（rad/s）')
    parser.add_argument('--duration', type=float, default=DURATION, help='仿真时长（秒）')
    parser.add_argument('--output-dir', default=None,
                        help='CSV 输出目录，默认 ~/.ros/wheel_speed_observer')
    return parser


def default_output_dir():
    return (Path(os.environ.get('ROS_HOME', Path.home() / '.ros'))
            / 'wheel_speed_observer')


def main(argv=None):
    args = build_parser().parse_args(argv)
    output_dir = Path(args.output_dir) if args.output_dir else default_output_dir()

    plant = DrivetrainObserver(jw=args.jw, k=args.k, c=args.c)
    print(f'模型：Jw={plant.jw} kg·m²，k={plant.k} Nm/rad，c={plant.c} N·m·s/rad')
    print('状态 x = [轴扭转角 theta, 车轮转速 omega_w]，输入 u = 电机转速 omega_m，'
          '量测 y = omega_w（ABS 齿圈）')

    print('\n能观性')
    matrix = plant.observability_matrix()
    print(f'  能观性矩阵 = [[{matrix[0][0]:.4f}, {matrix[0][1]:.4f}], '
          f'[{matrix[1][0]:.4f}, {matrix[1][1]:.4f}]]')
    print(f'  det = {plant.observability_determinant():.4f}  →  '
          f'{"可观测" if plant.is_observable() else "不可观测"}')

    base_wn = math.sqrt(plant.k / plant.jw)
    print(f'  开环极点 {plant.open_loop_poles()[0]:.3f} / '
          f'{plant.open_loop_poles()[1]:.3f}（扭振固有频率 {base_wn:.1f} rad/s）')

    print('\n观测器设计与「带宽—噪声」权衡（初始误差 0.02 rad / 0.5 rad/s）')
    header = (f"{'带宽倍数':>9}{'wn(rad/s)':>12}{'L=[l1,l2]':>24}"
              f"{'极点':>22}{'收敛时间(s)':>13}{'噪声增益':>10}{'扭角RMS':>10}")
    print(header)
    print('-' * len(header.encode('gbk')))

    rows = []
    for factor in BANDWIDTH_FACTORS:
        wn = factor * base_wn
        gain, _poles = plant.design(wn=wn, zeta=args.zeta)
        observer_poles = plant.observer_poles(gain)

        clean = plant.simulate(gain, duration=args.duration, noise_std=0.0)
        settle = error_settle_time(clean)
        noisy = plant.simulate(gain, duration=args.duration,
                               noise_std=args.noise)
        steady = [sample[3] for sample in noisy if sample[0] >= 0.6 * args.duration]
        steady_rms = rms(steady)

        settle_text = '未收敛' if settle is None else f'{settle:.3f}'
        gain_norm = math.hypot(gain[0][0], gain[1][0])
        print(f'{factor:>9.1f}{wn:>12.1f}'
              f"{f'[{gain[0][0]:.3f},{gain[1][0]:.3f}]':>24}"
              f"{f'{observer_poles[0].real:.1f}±{abs(observer_poles[0].imag):.1f}j':>22}"
              f'{settle_text:>13}{gain_norm:>10.2f}{steady_rms:>10.5f}')
        rows.append({
            'bandwidth_factor': factor,
            'wn': wn,
            'l1': gain[0][0],
            'l2': gain[1][0],
            'pole_real': observer_poles[0].real,
            'pole_imag': abs(observer_poles[0].imag),
            'settle_time': settle,
            'gain_norm': math.hypot(gain[0][0], gain[1][0]),
            'noise_rms': steady_rms,
        })

    print(f'\n测量噪声标准差 {args.noise:.3f} rad/s（车轮转速）。')
    print('结论：带宽越高收敛越快，噪声增益 ||L|| 也随之单调增大；'
          '而扭角估计的噪声主要经 l1 进入，l1 接近 0 的区间反而最干净。'
          '工程上取「收敛时间够用、噪声增益可接受」的最低带宽。')

    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / 'observer_tradeoff.csv'
    with open(csv_path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    detail = plant.simulate(plant.design(wn=base_wn)[0], duration=args.duration)
    series_path = output_dir / 'observer_timeseries.csv'
    with open(series_path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(['t_s', 'theta_true', 'theta_hat', 'error', 'omega_w'])
        for t, true, hat, error, omega in detail:
            writer.writerow([f'{t:.4f}', f'{true:.6f}', f'{hat:.6f}',
                             f'{error:.6f}', f'{omega:.6f}'])

    print(f'\n权衡结果已写入 {csv_path}')
    print(f'基准带宽下的时序已写入 {series_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
