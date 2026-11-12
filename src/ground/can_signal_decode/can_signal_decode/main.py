"""离线入口：生成一段带故障注入的 CAN 报文流，解码并统计校验结果。

不需要 ROS 2 环境，只依赖 Python 标准库::

    cd src/ground/can_signal_decode
    python3 -m can_signal_decode.main
"""

import argparse
import csv
import math
import os
import random
from pathlib import Path

from .checksum import CHECK_BYTES, CHECK_VALUE, crc8_sae_j1850
from .database import FrameCodec, default_frames

DEFAULT_SEED = 20261111


def inject_crc_error(payload: bytes, rng: random.Random) -> bytes:
    """随机翻转 1 bit，模拟总线干扰导致的 CRC 失败。"""
    damaged = bytearray(payload)
    bit = rng.randrange(len(damaged) * 8)
    damaged[bit // 8] ^= 1 << (bit % 8)
    return bytes(damaged)


def ground_truth_vcu(t: float) -> dict:
    if t < 8.0:
        gear = 3.0
    elif t < 14.0:
        gear = 4.0
    else:
        gear = 5.0
    return {
        'VehicleSpeed': 60.0 + 25.0 * math.sin(0.35 * t),
        'MotorTorque': 120.0 * math.sin(0.35 * t + 0.4),
        'Gear': gear,
        'BrakePressure': max(0.0, 18.0 * math.sin(0.9 * t)),
    }


def ground_truth_steering(t: float) -> dict:
    return {'SteeringAngle': 210.0 * math.sin(0.25 * t)}


def simulate(duration=20.0, rate_hz=50.0, seed=DEFAULT_SEED,
             crc_error_rate=0.030, counter_skip_rate=0.020,
             range_error_rate=0.020):
    """生成 ``[(t, codec, payload, truth, injected)]`` 序列。

    两条报文各自维护滚动计数器（真实整车上每条报文独立计数）。
    """
    rng = random.Random(seed)
    frames = default_frames()
    vcu = FrameCodec(frames[0], counter_name='AliveCounter')
    steering = FrameCodec(frames[1], counter_name='AliveCounter')

    steps = int(duration * rate_hz)
    stream = []
    vcu_counter = 0
    steer_counter = 0

    for step in range(steps):
        t = step / rate_hz

        truth = ground_truth_vcu(t)
        injected = []
        if rng.random() < range_error_rate:
            truth = dict(truth)
            truth['VehicleSpeed'] = 300.0        # 超出 0..250 km/h 的标定量程
            injected.append('range')
        payload = vcu.build(truth, counter=vcu_counter)
        if rng.random() < crc_error_rate:
            payload = inject_crc_error(payload, rng)
            injected.append('crc')
        stream.append((t, vcu, payload, truth, tuple(injected)))

        if step % 3 == 0:                        # 0x101 周期 60 ms
            truth_s = ground_truth_steering(t)
            payload = steering.build(truth_s, counter=steer_counter)
            if rng.random() < crc_error_rate:
                payload = inject_crc_error(payload, rng)
                injected.append('crc')
            stream.append((t, steering, payload, truth_s, tuple(injected)))
            steer_counter = (steer_counter + 1) % 16
            if rng.random() < counter_skip_rate:
                steer_counter = (steer_counter + 1) % 16   # 模拟丢帧

        vcu_counter = (vcu_counter + 1) % 16
        if rng.random() < counter_skip_rate:
            vcu_counter = (vcu_counter + 1) % 16

    return stream


class Stats:
    """只统计通过校验的帧；另记录被丢弃的非法帧数量。"""

    def __init__(self):
        self.frames = {}
        self.signals = {}
        self.dropped = 0
        self.corrupted_accepted = 0

    def add(self, codec, decoded):
        frame = self.frames.setdefault(
            codec.name, {'count': 0, 'crc_fail': 0, 'counter_jump': 0,
                         'range_violation': 0, 'valid': 0})
        frame['count'] += 1
        frame['crc_fail'] += 0 if decoded['crc_ok'] else 1
        frame['counter_jump'] += 0 if decoded['counter_ok'] else 1
        frame['range_violation'] += 1 if decoded['range_violations'] else 0
        frame['valid'] += 1 if decoded['valid'] else 0

        if not decoded['valid']:
            self.dropped += 1
            return
        for signal in codec.spec.signals:
            name = signal.name
            value = decoded[name]
            item = self.signals.setdefault(
                name, {'unit': signal.unit, 'count': 0, 'min': value,
                       'max': value, 'sum': 0.0})
            item['count'] += 1
            item['min'] = min(item['min'], value)
            item['max'] = max(item['max'], value)
            item['sum'] += value


def run(stream):
    stats = Stats()
    previous = {}
    pending = {}          # 每条报文自上一帧 CRC 通过以来，丢掉了几个坏帧
    rows = []
    for t, codec, payload, truth, injected in stream:
        expected_step = 1 + pending.get(codec.name, 0)
        decoded = codec.decode(payload, previous=previous.get(codec.name),
                               expected_step=expected_step)
        # 只有 CRC 通过的帧才作为下一帧的计数器基准：坏帧的计数器位本身不可信。
        # 夹在中间的坏帧记入 pending，让下一帧的预期步进相应放宽，
        # 避免「一次误码罚两帧」。
        if decoded['crc_ok']:
            previous[codec.name] = payload
            pending[codec.name] = 0
        else:
            pending[codec.name] = pending.get(codec.name, 0) + 1
        stats.add(codec, decoded)
        rows.append((t, codec.name, decoded, truth, injected))
    return stats, rows


def half_step(codec):
    """返回该报文所有信号里最大的半个量化步长，用于往返精度判据。"""
    return max(signal.scale / 2.0 for signal in codec.spec.signals)


def write_csv(path, rows):
    with open(path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(['t_s', 'frame', 'valid', 'crc_ok', 'counter_ok',
                         'range_violations', 'VehicleSpeed', 'MotorTorque',
                         'SteeringAngle'])
        for t, name, decoded, _truth, _injected in rows:
            writer.writerow([
                f'{t:.3f}', name, int(decoded['valid']), int(decoded['crc_ok']),
                int(decoded['counter_ok']), '|'.join(decoded['range_violations']),
                f"{decoded.get('VehicleSpeed', 0.0):.2f}",
                f"{decoded.get('MotorTorque', 0.0):.2f}",
                f"{decoded.get('SteeringAngle', 0.0):.2f}",
            ])


def build_parser():
    parser = argparse.ArgumentParser(
        prog='can_signal_decode.main',
        description='CAN 报文信号解码与 CRC/计数器校验的离线演示')
    parser.add_argument('--duration', type=float, default=20.0, help='仿真时长（秒）')
    parser.add_argument('--rate', type=float, default=50.0, help='基频（Hz）')
    parser.add_argument('--seed', type=int, default=DEFAULT_SEED, help='故障注入随机种子')
    parser.add_argument('--crc-error-rate', type=float, default=0.030,
                        help='CRC 错误注入比例')
    parser.add_argument('--output-dir', default=None,
                        help='CSV 输出目录，默认 ~/.ros/can_signal_decode')
    return parser


def default_output_dir():
    return Path(os.environ.get('ROS_HOME', Path.home() / '.ros')) / 'can_signal_decode'


def main(argv=None):
    args = build_parser().parse_args(argv)
    output_dir = Path(args.output_dir) if args.output_dir else default_output_dir()

    print(f'CRC8/SAE-J1850 自检: crc8({CHECK_BYTES!r}) = 0x'
          f'{crc8_sae_j1850(CHECK_BYTES):02X}（期望 0x{CHECK_VALUE:02X}）')
    print(f'仿真: {args.duration:.0f} s @ {args.rate:.0f} Hz，'
          f'CRC 错误率 {args.crc_error_rate:.1%}，种子 {args.seed}')

    stream = simulate(duration=args.duration, rate_hz=args.rate, seed=args.seed,
                      crc_error_rate=args.crc_error_rate)
    stats, rows = run(stream)

    print('\n编解码往返精度（误差应不超过半个量化步长）')
    print('-' * 68)
    for spec in default_frames():
        codec = FrameCodec(spec, counter_name='AliveCounter')
        truth = (ground_truth_vcu(1.234) if codec.frame_id == 0x100
                 else ground_truth_steering(1.234))
        decoded = codec.decode(codec.build(truth, counter=3))
        worst = max(abs(decoded[name] - value) for name, value in truth.items())
        limit = half_step(codec)
        flag = 'OK' if worst <= limit + 1e-9 else 'FAIL'
        print(f'  0x{codec.frame_id:03X} {codec.name:<16} '
              f'最大误差 {worst:.4f} <= 半量化步长 {limit:.4f}  {flag}')

    print('\n报文校验统计（分母为该报文总帧数）')
    header = (f"{'报文':<20}{'帧数':>7}{'CRC失败':>9}{'计数跳变':>10}"
              f"{'越界帧':>8}{'通过率':>9}")
    print(header)
    print('-' * len(header))
    for name, frame in stats.frames.items():
        rate = 100.0 * frame['valid'] / frame['count'] if frame['count'] else 0.0
        print(f"{name:<20}{frame['count']:>7}{frame['crc_fail']:>9}"
              f"{frame['counter_jump']:>10}{frame['range_violation']:>8}"
              f"{rate:>8.1f}%")

    print(f'\n被丢弃的帧: {stats.dropped} / {len(rows)} '
          f'（{100.0 * stats.dropped / len(rows):.1f}%）')

    print('\n通过校验后的信号统计')
    print(f"{'信号':<18}{'单位':<8}{'最小':>10}{'最大':>10}{'均值':>10}")
    print('-' * 56)
    for name, item in stats.signals.items():
        mean = item['sum'] / item['count'] if item['count'] else 0.0
        print(f"{name:<18}{item['unit']:<8}{item['min']:>10.2f}"
              f"{item['max']:>10.2f}{mean:>10.2f}")

    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / 'decode_result.csv'
    write_csv(csv_path, rows)
    print(f'\n解码明细已写入 {csv_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
