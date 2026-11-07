# -*- coding: utf-8 -*-
"""命令行与报告逻辑（放在包内，供 ``main.py`` 与 ROS 2 节点共用）。

* :func:`build_parser`   —— 统一参数定义（默认值只在这里维护一份）；
* :func:`run_one_vehicle`—— 对单个车型执行选定的测试；
* :func:`print_table`    —— 打印多车型对比表；
* :func:`save_csv`       —— 结果落盘（便于独立核验）；
* :func:`save_plots`     —— 有 matplotlib 时画速度曲线。
"""

import argparse
import csv
import os
import sys
import time

from .carla_utils import (DEFAULT_DT, DEFAULT_HOST, DEFAULT_PORT, DEFAULT_TIMEOUT,
                          DEFAULT_TOWN, CarlaSession, reset_vehicle)
from .runner import measure_acceleration, measure_braking, measure_turn

# 参与对比的车型：车身尺寸/动力差别明显，便于看出区分度
DEFAULT_VEHICLES = [
    'vehicle.tesla.model3',         # 中型轿车
    'vehicle.bmw.grandtourer',      # 大型 SUV
    'vehicle.mini.cooper_s',        # 小型车
]
MODES = ('accel', 'brake', 'turn', 'all')


def kmh(value):
    """km/h -> m/s。"""
    return float(value) / 3.6


def build_parser():
    parser = argparse.ArgumentParser(
        description='CARLA 地面载具动力性测试（加速 / 制动 / 转向，多车型对比）')
    parser.add_argument('--mode', choices=MODES, default='all')
    parser.add_argument('--host', default=DEFAULT_HOST, help='Windows 宿主机 IP')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT)
    parser.add_argument('--town', default=DEFAULT_TOWN)
    parser.add_argument('--dt', type=float, default=DEFAULT_DT)
    parser.add_argument('--vehicle', default=DEFAULT_VEHICLES[0],
                        help='单个车型 blueprint，如 vehicle.tesla.model3')
    parser.add_argument('--vehicles', default=None, help='写 all 则循环内置多车型对比')
    parser.add_argument('--v-target-kmh', type=float, default=60.0)
    parser.add_argument('--steer', type=float, default=0.05,
                        help='阶跃转向的归一化转向指令（-1~1，默认 0.05≈3.5° 前轮转角）')
    parser.add_argument('--turn-speed', type=float, default=7.0,
                        help='转向测试的目标车速（m/s）')
    parser.add_argument('--turn-time', type=float, default=1.5,
                        help='转向段的持续时间（秒，太长会冲出路沿）')
    parser.add_argument('--max-time', type=float, default=40.0, help='单项测试超时（秒）')
    parser.add_argument('--timeout', type=float, default=DEFAULT_TIMEOUT,
                        help='CARLA 客户端 RPC 超时（秒，默认 30；切地图慢时可调大）')
    parser.add_argument('--retries', type=int, default=2,
                        help='样本出现碰撞/掉速时的自动重跑次数（0 表示不重跑，只标注）')
    parser.add_argument('--log-every', type=float, default=1.0)
    parser.add_argument('--out-dir', default='data')
    return parser


def sample_ok(result):
    """样本是否合格：跑完 + 无碰撞 + 速度序列没有非物理跳变。"""
    return (bool(result.get('ok')) and not result.get('collided')
            and not result.get('anomalies'))


def brake_sample_ok(result):
    """制动样本额外要求减速度在物理上说得通（>1.5 g 必然是撞了或被顶了）。"""
    if not sample_ok(result):
        return False
    return (result.get('decel') or 0.0) <= 15.0


def run_with_retries(session, vehicle, start_transform, monitor, retries, measure,
                     accept, label):
    """复位 → 测量 → 判定；样本异常（碰撞/掉速）就复位重跑，最多 ``retries+1`` 次。

    这样"撞了又爬起来"的那种 6.30 s 不会被当成加速成绩发出去。重跑仍异常时
    保留最后一次结果，由调用方在表格里打 ``*``，并在 CSV 里记 ``*_valid = False``。
    """
    attempts = max(1, int(retries) + 1)
    result = None
    for index in range(attempts):
        reset_vehicle(session, vehicle, start_transform)
        if monitor is not None:
            monitor.reset()
        result = measure()
        result['collided'] = bool(monitor is not None and monitor.count > 0)
        result['attempts'] = index + 1
        if result['collided']:
            print('[警告] %s：检测到碰撞事件 %d 次，样本不可信'
                  % (label, monitor.count))
        if result.get('ok') and accept(result):
            if index:
                print('[重跑] %s第 %d 次才拿到合格样本' % (label, index + 1))
            return result
        if index + 1 < attempts:
            reasons = []
            if not result.get('ok'):
                reasons.append('测试未完成')
            if result['collided']:
                reasons.append('碰撞')
            if result.get('anomalies'):
                reasons.append('速度跳变')
            print('[重跑] %s样本异常（%s），复位后重试（第 %d/%d 次）'
                  % (label, '、'.join(reasons) or '结果不合格', index + 2, attempts))
    print('[警告] %s 重跑 %d 次仍有异常，结果以 * 标注（仅供参考）' % (label, attempts))
    return result


def run_one_vehicle(session, blueprint, args):
    """对单个车型执行选定的测试，返回结果字典（含逐帧样本，供画图）。

    三项测试**各自从同一个起点重新开始**（``reset_vehicle``），
    否则加速测试已经把车开出去几十米、甚至撞过东西，
    后面的制动与转向测试就在被污染的状态下开始（实测会得出"减速度 1.8g"这种假数据）。
    """
    print('\n' + '=' * 76)
    print('车型：%s' % blueprint)
    print('=' * 76)
    vehicle = session.spawn_vehicle(blueprint)
    start_transform = vehicle.get_transform()
    params = session.vehicle_params(vehicle)
    print('[参数] 轴距 %.2f m（来源：%s），最大前轮转角 %.1f°，质量 %.0f kg'
          % (params['wheelbase'], params.get('wheelbase_source', '未知'),
             params['max_steer_deg'], params['mass']))
    print('[参数] 原始轮位 x = %s（cm，用于核对坐标含义）' % params['raw_wheel_x'])
    carla_map = session.world.get_map()
    result = {'vehicle': blueprint, 'wheelbase': params['wheelbase'],
              'max_steer_deg': params['max_steer_deg'], 'mass': params['mass']}

    monitor = session.watch_collisions(vehicle)
    v_target = kmh(args.v_target_kmh)
    try:
        if args.mode in ('accel', 'all'):
            acc = run_with_retries(
                session, vehicle, start_transform, monitor, args.retries,
                lambda: measure_acceleration(session, vehicle, carla_map,
                                             v_target=v_target, max_time=args.max_time,
                                             log_every=args.log_every),
                sample_ok, '加速')
            result['accel_time'] = acc['time']
            result['accel_mean'] = acc['accel']
            result['accel_samples'] = acc['samples']
            result['accel_attempts'] = acc.get('attempts', 1)
            result['accel_valid'] = sample_ok(acc)
        if args.mode in ('brake', 'all'):
            brk = run_with_retries(
                session, vehicle, start_transform, monitor, args.retries,
                lambda: measure_braking(session, vehicle, carla_map, v_start=v_target,
                                        log_every=args.log_every),
                brake_sample_ok, '制动')
            result['brake_distance'] = brk['distance']
            result['brake_decel'] = brk['decel']
            result['brake_samples'] = brk['samples']
            result['brake_attempts'] = brk.get('attempts', 1)
            result['brake_valid'] = brake_sample_ok(brk)
        if args.mode in ('turn', 'all'):
            turn = run_with_retries(
                session, vehicle, start_transform, monitor, args.retries,
                lambda: measure_turn(session, vehicle, carla_map, steer_cmd=args.steer,
                                     v_target=args.turn_speed, step_time=args.turn_time,
                                     log_every=args.log_every),
                sample_ok, '转向')
            result['turn_radius'] = turn.get('radius')
            result['turn_radius_yaw'] = turn.get('radius_yaw')
            result['turn_radius_fit'] = turn.get('radius_fit')
            result['turn_radius_theory'] = turn.get('theory')
            result['turn_error_pct'] = turn.get('error_pct')
            result['turn_points'] = turn.get('points', [])
            result['turn_attempts'] = turn.get('attempts', 1)
            result['turn_valid'] = sample_ok(turn)
    finally:
        monitor.destroy()
        session.destroy_actor(vehicle)
    return result


def _fmt(value, fmt, valid=None):
    """格式化数值；``valid is False`` 时加 ``*``（该样本重跑后仍有碰撞/掉速）。"""
    if not isinstance(value, (int, float)):
        return '—'
    text = fmt % value
    return text if valid is not False else text + '*'


def print_table(results, args):
    print('\n' + '=' * 76)
    print('动力性测试对比表（%s，目标车速 %.0f km/h）'
          % (time.strftime('%Y-%m-%d %H:%M:%S'), args.v_target_kmh))
    print('=' * 76)
    print('%-28s %9s %9s %9s %9s %9s'
          % ('车型', '0→v(s)', 'a(m/s²)', '制动(m)', '减速度', '转弯半径(m)'))
    print('-' * 76)
    for r in results:
        print('%-28s %9s %9s %9s %9s %9s'
              % (r['vehicle'].replace('vehicle.', ''),
                 _fmt(r.get('accel_time'), '%.2f', r.get('accel_valid')),
                 _fmt(r.get('accel_mean'), '%.2f', r.get('accel_valid')),
                 _fmt(r.get('brake_distance'), '%.2f', r.get('brake_valid')),
                 _fmt(r.get('brake_decel'), '%.2f', r.get('brake_valid')),
                 _fmt(r.get('turn_radius'), '%.2f', r.get('turn_valid'))))
    print('=' * 76)
    if any(r.get(k) is False for r in results
           for k in ('accel_valid', 'brake_valid', 'turn_valid')):
        print('* 带 * 的数值：重跑后仍检测到碰撞或速度跳变，仅供对照，不建议直接引用。')


def save_csv(results, path):
    """把对比结果写成 CSV（便于独立核验/画图）。"""
    columns = ['vehicle', 'wheelbase', 'max_steer_deg', 'mass', 'accel_time',
               'accel_mean', 'accel_attempts', 'accel_valid',
               'brake_distance', 'brake_decel', 'brake_attempts', 'brake_valid',
               'turn_radius', 'turn_radius_yaw', 'turn_radius_fit',
               'turn_radius_theory', 'turn_error_pct', 'turn_attempts', 'turn_valid']
    with open(path, 'w', newline='', encoding='utf-8') as fp:
        writer = csv.DictWriter(fp, fieldnames=columns, extrasaction='ignore')
        writer.writeheader()
        for r in results:
            writer.writerow(r)
    print('[保存] 对比结果已写入 %s' % path)


def save_plots(results, out_dir):
    """有 matplotlib 就画曲线；没有则跳过（不影响主流程）。"""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('[提示] 未安装 matplotlib，跳过曲线图（pip install matplotlib 可生成）')
        return
    os.makedirs(out_dir, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    for r in results:
        name = r['vehicle'].replace('vehicle.', '')
        if r.get('accel_samples'):
            axes[0].plot([s[0] for s in r['accel_samples']],
                         [s[1] * 3.6 for s in r['accel_samples']], label=name)
        if r.get('brake_samples'):
            axes[1].plot([s[0] for s in r['brake_samples']],
                         [s[1] * 3.6 for s in r['brake_samples']], label=name)
    axes[0].set_title('acceleration: speed vs time')
    axes[0].set_xlabel('t / s')
    axes[0].set_ylabel('v / km/h')
    axes[1].set_title('braking: speed vs time')
    axes[1].set_xlabel('t / s')
    axes[1].set_ylabel('v / km/h')
    for ax in axes:
        ax.grid(True, alpha=0.3)
        if ax.get_legend_handles_labels()[0]:      # 空子图不调用 legend，避免告警
            ax.legend()
    fig.tight_layout()
    path = os.path.join(out_dir, 'dynamics_curves.png')
    fig.savefig(path, dpi=120)
    print('[保存] 曲线图已保存：%s' % path)


def run(args):
    """按参数执行全部测试，返回结果列表。"""
    session = CarlaSession(args.host, args.port, timeout=args.timeout)
    results = []
    try:
        session.ensure_town(args.town)
        session.enable_sync(args.dt)
        vehicles = DEFAULT_VEHICLES if args.vehicles == 'all' else [args.vehicle]
        for blueprint in vehicles:
            results.append(run_one_vehicle(session, blueprint, args))
    finally:
        session.close()
    if results:
        print_table(results, args)
        os.makedirs(args.out_dir, exist_ok=True)
        save_csv(results, os.path.join(args.out_dir, 'dynamics_results.csv'))
        save_plots(results, args.out_dir)
    return results


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        run(args)
    except KeyboardInterrupt:
        print('\n[中断] 用户中止')
    except RuntimeError as exc:
        # 未安装 carla 包、连不上服务端、出生点被占用等，都给出可读提示而不是抛栈
        print('[错误] %s' % exc)
        return 1
    return 0


if __name__ == '__main__':                                  # pragma: no cover
    sys.exit(main())
