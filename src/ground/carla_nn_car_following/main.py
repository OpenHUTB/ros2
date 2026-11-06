#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""carla_nn_car_following 统一入口（仓库约定：入口为 main.*）。

用法示例
--------
::

    # 只检查连接是否可用
    python3 main.py --mode check  --host 192.168.148.1

    # 用经典 ACC 跑一遍工况，终端打印车距变化（先不用 ROS，便于调试）
    python3 main.py --mode scene  --host 192.168.148.1 --scenario decelerate

    # 采集训练数据（四种工况各跑一次）
    python3 main.py --mode collect --host 192.168.148.1 --scenario all

    # 训练感知网络与控制网络（不需要 CARLA）
    python3 main.py --mode train

    # 用神经网络控制器在线跑跟车
    python3 main.py --mode run --host 192.168.148.1 --scenario decelerate --controller nn

    # 多场景性能评测（默认 4 种前车工况；追加天气评测见 --help）
    python3 main.py --mode eval --host 192.168.148.1 --scenario all
"""

import argparse
import math
import os
import sys
import time
from collections import deque

try:
    import carla
except ImportError:      # 允许在没有 CARLA 的机器上运行 --mode train
    carla = None

from carla_nn_car_following import scenario as scenario_mod
from carla_nn_car_following.carla_client import CarlaSession, distance_between, speed_of
from carla_nn_car_following.reference_acc import ReferenceACC

# ----------------------------------------------------------------------------
# 默认参数（与 config/params.yaml 一一对应）
# ----------------------------------------------------------------------------
DEFAULTS = {
    'host': '192.168.148.1',
    'port': 2000,
    'town': 'Town03',
    'weather': 'ClearNoon',
    'dt': 0.05,
    'duration': 30.0,
    'lead_distance': 25.0,
    'spawn_index': -1,
    'initial_speed': 'auto',
    'v_set': 'auto',
    'v_set_margin': 2.0,
    'spectator_follow': True,
    'realtime': True,
    'controller': 'classic',        # classic = 经典 ACC；nn = 神经网络控制器
    'models_dir': 'models',
    'data_dir': 'data',
    'train_epochs': 300,
    'train_lr': 0.01,
    'train_batch': 64,
    'd0': 5.0,
    'headway': 1.2,
    'kp': 0.35,
    'kv': 1.5,
    'k_throttle': 0.25,
    'k_brake': 0.35,
    'k_feedforward': 0.03,
    'radar_fov': 30.0,
    'radar_range': 100.0,
    'radar_vertical_fov': 4.0,
    'radar_velocity_tolerance': 8.0,
    'radar_azimuth_tolerance_deg': 8.0,
}

# 数据表每一列的含义（保存 npz 与打印表头共用，避免两处不一致）
COLUMNS = [
    't',                  # 0  仿真时间 (s)
    'radar_d',            # 1  雷达最短回波距离 (m) —— 感知网络输入
    'radar_azimuth',      # 2  对应方位角 (rad)
    'radar_closing',      # 3  接近速度 (m/s)，差分得到
    'radar_mean_depth',   # 4  平均回波距离 (m)
    'radar_n_points',     # 5  点云数量
    'gap_true',           # 6  真实车距 (m) —— 感知网络回归标签
    'v_rel_true',         # 7  真实相对速度 (m/s) = v_lead - v_ego
    'v_ego',              # 8  自车速度 (m/s)
    'label_throttle',     # 9  经典 ACC 给出的油门 —— 控制网络标签
    'label_brake',        # 10 经典 ACC 给出的刹车 —— 控制网络标签
    'follow_state',       # 11 跟车状态分类标签（0 远 / 1 近 / 2 危险）
]


# ----------------------------------------------------------------------------
# 参数
# ----------------------------------------------------------------------------
def load_params(path):
    """读取 YAML 参数文件并覆盖默认值；任何问题都退回默认值，不阻塞运行。"""
    cfg = dict(DEFAULTS)
    if not path:
        return cfg
    if not os.path.isfile(path):
        print('[警告] 参数文件不存在，使用内置默认值：%s' % path)
        return cfg
    try:
        import yaml
        with open(path, 'r', encoding='utf-8') as fp:
            user = yaml.safe_load(fp) or {}
    except ImportError:
        print('[警告] 未安装 PyYAML，使用内置默认值（sudo apt install -y python3-yaml）')
        return cfg
    except Exception as exc:
        print('[警告] 参数文件解析失败（%s），使用内置默认值' % exc)
        return cfg

    for key, value in user.items():
        if key in cfg:
            cfg[key] = value
        else:
            print('[警告] 忽略未知参数：%s' % key)
    return cfg


def build_acc(cfg, v_set=None):
    return ReferenceACC(d0=cfg['d0'], headway=cfg['headway'], kp=cfg['kp'], kv=cfg['kv'],
                        k_throttle=cfg['k_throttle'], k_brake=cfg['k_brake'],
                        k_feedforward=cfg.get('k_feedforward', 0.03), v_set=v_set)


def fmt_loc(actor):
    loc = actor.get_transform().location
    return '(%.1f, %.1f, %.1f)' % (loc.x, loc.y, loc.z)


def set_spectator_follow(world, vehicle, distance=9.0, height=4.5, pitch=-15.0):
    """让 CARLA 窗口的观察者跟随自车（第三人称跟随镜头）。

    为什么需要它：CARLA 服务端的窗口显示的是观察者（spectator）视角，默认是固定不动的。
    不设置时，服务端窗口只会停在一个静止的城市画面上，**录制演示动图时看不到被控车辆**。
    """
    transform = vehicle.get_transform()
    forward = transform.get_forward_vector()
    location = carla.Location(x=transform.location.x - forward.x * distance,
                              y=transform.location.y - forward.y * distance,
                              z=transform.location.z + height)
    world.get_spectator().set_transform(
        carla.Transform(location, carla.Rotation(pitch=pitch, yaw=transform.rotation.yaw)))


# ----------------------------------------------------------------------------
# 一次完整仿真
# ----------------------------------------------------------------------------
def run_episode(cfg, host, port, scenario_name, duration, verbose=True):
    """跑一次「自车 + 前车 + 控制器」闭环，返回 (逐帧数据表, 性能指标)。"""
    session = CarlaSession(host, port)
    rows = []
    try:
        if cfg.get('town') and cfg['town'] not in session.current_map_name():
            print('[世界] 加载地图 %s（首次较慢）...' % cfg['town'])
            session.load_town(cfg['town'])
        print('[世界] 地图：%s' % session.current_map_name())
        if cfg.get('weather'):
            session.set_weather(cfg['weather'])
            print('[世界] 天气：%s' % cfg['weather'])

        session.enable_sync(cfg['dt'])
        print('[世界] 同步模式已开启，步长 %.3f s（%.0f Hz）' % (cfg['dt'], 1.0 / cfg['dt']))

        ego = session.spawn_ego(spawn_index=cfg.get('spawn_index', -1))
        print('[自车] 已生成 @ %s' % fmt_loc(ego))

        lead = session.spawn_in_lane(ego, cfg['lead_distance'], role_name='lead_vehicle')
        if lead is None:
            raise RuntimeError('无法在前方 %.1f m 处生成前车（出生点可能被占用）' % cfg['lead_distance'])
        gap_initial = distance_between(ego, lead)
        print('[前车] 已生成 @ %s，初始车距 %.1f m' % (fmt_loc(lead), gap_initial))
        if abs(gap_initial - cfg['lead_distance']) > 8.0:
            print('[警告] 初始车距 %.1f m 与设定值 %.1f m 相差较大，可能是出生点位置异常，'
                  '建议先用 diagnose_drive.py 换一个出生点' % (gap_initial, cfg['lead_distance']))

        radar = session.attach_radar(
            ego,
            fov=cfg['radar_fov'],
            radar_range=cfg['radar_range'],
            azimuth_tolerance_deg=cfg['radar_azimuth_tolerance_deg'],
            vertical_fov=cfg.get('radar_vertical_fov', 4.0),
            velocity_tolerance=cfg.get('radar_velocity_tolerance', 8.0),
            tick=cfg['dt'],
        )
        # 等雷达出第一帧有效数据再进入控制循环。
        # 否则第一帧的 min_depth 还是空，features() 会返回量程上限（100 m），
        # 实测这一帧会让感知网络给出错误估计并触发一次急刹，同时把数据统计的标准差抬高约 4 m。
        for _ in range(40):
            if radar.min_depth is not None:
                break
            session.tick()
        if radar.min_depth is None:
            print('[警告] 雷达在 40 帧内没有测到目标，请检查前车位置与雷达参数')

        scenario = scenario_mod.build(scenario_name)

        # 设定车速：默认取工况巡航速度 + 余量。前车实际巡航速度会略高于其目标速度
        #（P 控制器的稳态偏差），若没有余量，自车会被限速卡住、车距被缓慢拉开。
        # 这一项同时也防止"跟丢前车后无限加速"。
        v_set = cfg.get('v_set', 'auto')
        if isinstance(v_set, str) and v_set.strip().lower() == 'auto':
            margin = float(cfg.get('v_set_margin', 2.0))
            v_set = (float(getattr(scenario, 'v0', 0.0)) + margin) or None
        elif v_set is not None:
            v_set = float(v_set) or None
        acc = build_acc(cfg, v_set)

        # 控制器选择：classic = 经典 ACC（用真值车距）；nn = 神经网络（用雷达数据估计）
        controller_name = str(cfg.get('controller', 'classic')).lower()
        nn_controller = None
        if controller_name == 'nn':
            from carla_nn_car_following.nn_controller import NNController
            nn_controller = NNController.load(cfg.get('models_dir', 'models'))
            acc = None
            print('[工况] %s —— %s' % (scenario.name, scenario.label))
            print('[控制器] 神经网络（感知网络估计车距 → 控制网络输出油门/刹车）')
        else:
            print('[工况] %s —— %s（经典 ACC，设定车速 %s）'
                  % (scenario.name, scenario.label,
                     ('%.1f m/s' % v_set) if v_set else '无'))

        # 让两车以巡航速度开场（而不是从静止起步）：一开场就是稳态跟车，
        # 更贴近真实交通流，也避免"自车从 0 起步猛冲"造成的剧烈振荡。
        cruise = cfg.get('initial_speed', 'auto')
        if isinstance(cruise, str) and cruise.strip().lower() == 'auto':
            cruise = float(getattr(scenario, 'v0', 0.0))
        cruise = float(cruise)
        if cruise > 1.0:
            for actor in (ego, lead):
                forward = actor.get_transform().get_forward_vector()
                actor.set_target_velocity(carla.Vector3D(x=forward.x * cruise,
                                                         y=forward.y * cruise,
                                                         z=0.0))
            session.settle(2)
            print('[起始] 自车与前车初始速度设为 %.1f m/s（%.0f km/h），进入稳态跟车'
                  % (cruise, cruise * 3.6))

        dt = float(cfg['dt'])
        steps = int(round(duration / dt))
        extras = []
        previous_gap = {}
        gap_warned = False
        target_id = None
        carla_map = session.world.get_map()      # 车道保持（横向控制）需要地图接口

        # 前车轨迹（面包屑）：自车沿前车 1.2 秒前走过的位置行驶，
        # 这样前车在路口拐弯时自车会跟着拐，不会"跟丢"到另一条路上。
        trail_len = max(1, int(round(1.2 / dt)))
        lead_trail = deque(maxlen=trail_len)
        next_tick = time.time()

        for index in range(steps):
            t = index * dt
            session.tick()                      # 同步推进一步，传感器回调在此时触发

            if scenario.wants_cut_in(t) and not extras:
                extra = session.spawn_in_lane(ego, scenario.cut_in_distance,
                                              blueprint_id='vehicle.audi.a2',
                                              role_name='cut_in_vehicle', free_radius=6.0)
                if extra is not None:
                    extras.append(extra)
                    scenario.mark_spawned()
                    print('[加塞] t=%.1fs：前方 %.1f m 处出现加塞车辆'
                          % (t, scenario.cut_in_distance))
            scenario.update(t, dt, lead, extras, carla_map)

            features = radar.features(dt)
            v_ego = speed_of(ego)

            front_vehicles = [lead] + extras
            nearest = min(front_vehicles, key=lambda actor: distance_between(ego, actor))
            gap_true = distance_between(ego, nearest)

            # 目标切换（例如加塞车辆出现）时：重新初始化雷达跟踪，面包屑也改跟新目标
            if nearest.id != target_id:
                radar.reset_tracking()
                target_id = nearest.id
            lead_trail.append(nearest.get_location())   # 记录"当前跟随目标"的轨迹

            previous = previous_gap.get(nearest.id)
            # 车距的导数就是相对速度 v_rel = v_lead - v_ego：车距在减小 -> 负值
            v_rel_true = 0.0 if previous is None else (gap_true - previous) / dt
            # 物理上两车相对速度不会超过约 ±35 m/s（126 km/h）；
            # 超出的一定是"车距真值在单帧内发生跳变"（例如刚生成的 actor 位置尚未刷新）造成的伪值
            v_rel_true = max(-35.0, min(35.0, v_rel_true))
            previous_gap[nearest.id] = gap_true

            if gap_true > 100.0 and not gap_warned:
                print('[警告] t=%.1fs 车距已超过 100 m：前车可能已驶出本车道'
                      '（此时自车只能按设定车速巡航）' % t)
                gap_warned = True

            gap_hat = v_rel_hat = None
            if nn_controller is not None:
                # 神经网络控制器：只用雷达特征，先估计车距/相对速度，再输出油门刹车
                throttle, brake, gap_hat, v_rel_hat = nn_controller.labels(features, v_ego)
            else:
                throttle, brake = acc.labels(gap_true, v_rel_true, v_ego)

            # 横向：优先沿"前车 1.2 秒前走过的位置"行驶（面包屑跟踪），
            # 轨迹还没攒够时退回车道中心线纯追踪。
            if len(lead_trail) >= trail_len:
                steer = scenario_mod.steer_to_point(ego, lead_trail[0], gain=1.0)
            else:
                steer = scenario_mod.lane_steer(ego, carla_map)
            ego.apply_control(carla.VehicleControl(
                throttle=throttle, brake=brake, steer=steer))

            if cfg.get('spectator_follow', True):
                set_spectator_follow(session.world, ego)     # 第三人称跟随镜头（录制动图用）

            rows.append((t, features['d_radar'], features['azimuth'], features['closing_rate'],
                         features['mean_depth'], features['n_points'], gap_true, v_rel_true,
                         v_ego, throttle, brake, float(follow_state_of(gap_true, v_ego, cfg))))

            if verbose and index % max(1, int(round(0.5 / dt))) == 0:
                estimate = ''
                if gap_hat is not None:
                    estimate = '  估计=%6.2fm(误差%+5.2f)' % (gap_hat, gap_hat - gap_true)
                print('  t=%5.1fs  雷达=%6.2fm  真值=%6.2fm  v_rel=%+5.2f m/s  '
                      '自车=%5.2f m/s  前车=%5.2f m/s  油门=%.2f  刹车=%.2f%s'
                      % (t, features['d_radar'], gap_true, v_rel_true, v_ego,
                         speed_of(nearest), throttle, brake, estimate))

            # 实时节奏控制：每步睡够 dt 秒，否则仿真会以"能跑多快就跑多快"的速度前进
            # （实测比真实时间快约 3 倍，录出来的动图像快进）。
            # 用 --fast 可以关闭（采集数据时略快一点，但不影响数据本身）。
            if cfg.get('realtime', True):
                next_tick += dt
                delay = next_tick - time.time()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.time()      # 已经落后了，重新对齐，不做追赶

        ego.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
        return rows, summarize(rows, cfg)

    finally:
        session.close()
        print('[清理] 已销毁传感器与车辆，世界设置已恢复')


def summarize(rows, cfg):
    """计算性能指标（与控制器无关，全部由真值直接计算）。"""
    if not rows:
        return {}
    d0 = float(cfg['d0'])
    headway = float(cfg['headway'])
    errors = [abs(row[6] - (d0 + headway * row[8])) for row in rows]
    ttc_list = []
    for row in rows:
        closing = -row[7]                      # v_rel = v_lead - v_ego，取负得到接近速度
        if closing > 1e-6:
            ttc_list.append(row[6] / closing)
    return {
        'steps': len(rows),
        'mean_abs_gap_error': sum(errors) / len(errors),
        'min_gap': min(row[6] for row in rows),
        'min_ttc': min(ttc_list) if ttc_list else float('inf'),
        'mean_v_ego': sum(row[8] for row in rows) / len(rows),
    }


def follow_state_of(gap, v_ego, cfg):
    """跟车状态分类（0 远 / 1 近 / 2 危险），仅用于数据标注与文档说明。"""
    error = gap - (float(cfg['d0']) + float(cfg['headway']) * v_ego)
    if error < -2.0:
        return 2
    if error < 3.0:
        return 1
    return 0


def save_dataset(path, rows, scenario_name):
    import numpy as np
    array = np.array(rows, dtype=np.float64)
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    np.savez(
        path,
        scenario=scenario_name,
        columns=np.array(COLUMNS),
        t=array[:, 0],
        radar_d=array[:, 1],
        radar_azimuth=array[:, 2],
        radar_closing=array[:, 3],
        radar_mean_depth=array[:, 4],
        radar_n_points=array[:, 5],
        gap_true=array[:, 6],
        v_rel_true=array[:, 7],
        v_ego=array[:, 8],
        label_throttle=array[:, 9],
        label_brake=array[:, 10],
        follow_state=array[:, 11].astype(int),
    )


# ----------------------------------------------------------------------------
# 各模式
# ----------------------------------------------------------------------------
def mode_check(cfg, host, port):
    session = CarlaSession(host, port)
    try:
        print('[通过] 已连接，服务端版本 %s' % session.client.get_server_version())
        print('[通过] 地图：%s' % session.current_map_name())
        if cfg.get('town') and cfg['town'] not in session.current_map_name():
            session.load_town(cfg['town'])
            print('[通过] 已切换到地图：%s' % session.current_map_name())
        session.enable_sync(cfg['dt'])
        ego = session.spawn_ego()
        print('[通过] 自车已生成 @ %s' % fmt_loc(ego))
        radar = session.attach_radar(ego, fov=cfg['radar_fov'], radar_range=cfg['radar_range'],
                                     azimuth_tolerance_deg=cfg['radar_azimuth_tolerance_deg'],
                                     tick=cfg['dt'])
        for _ in range(50):
            session.tick()
        print('[通过] 雷达回调帧数：%d' % radar.frame)
        print('[通过] 正前方最近回波：%s'
              % ('%.2f m' % radar.min_depth if radar.min_depth is not None else '无目标'))
        if radar.frame < 40:
            print('[失败] 雷达帧数过少，请检查同步模式与 sensor_tick 设置')
            return 1
        return 0
    finally:
        session.close()


def print_summary(summary):
    print('[结果] 平均车距误差 %.2f m | 最小车距 %.2f m | 最小 TTC %s s | 平均车速 %.2f m/s'
          % (summary.get('mean_abs_gap_error', 0.0), summary.get('min_gap', 0.0),
             ('%.2f' % summary['min_ttc']) if summary.get('min_ttc', float('inf')) != float('inf')
             else 'inf',
             summary.get('mean_v_ego', 0.0)))


def mode_scene(cfg, host, port, scenarios, duration):
    """用当前控制器跑一遍工况（终端打印逐帧数据）。"""
    for name in scenarios:
        print('\n' + '=' * 72)
        print('工况：%s' % name)
        print('=' * 72)
        _, summary = run_episode(cfg, host, port, name, duration)
        print_summary(summary)
    return 0


def mode_collect(cfg, host, port, scenarios, duration, out_dir):
    """采集训练数据（用经典 ACC 作为标签来源，因此始终使用经典控制器）。"""
    collect_cfg = dict(cfg)
    collect_cfg['controller'] = 'classic'
    for name in scenarios:
        print('\n' + '=' * 72)
        print('采集工况：%s' % name)
        print('=' * 72)
        rows, summary = run_episode(collect_cfg, host, port, name, duration)
        path = os.path.join(out_dir, 'dataset_%s.npz' % name)
        save_dataset(path, rows, name)
        print('[保存] %s（%d 帧）' % (path, len(rows)))
        print_summary(summary)
    return 0


def mode_train(cfg):
    """训练感知网络与控制网络（不需要 CARLA）。"""
    from carla_nn_car_following.train import train_models

    print('=' * 72)
    print('训练感知网络与控制网络（纯 NumPy，不需要 CARLA）')
    print('=' * 72)
    train_models(data_dir=cfg['data_dir'], models_dir=cfg['models_dir'],
                 epochs=int(cfg['train_epochs']), lr=float(cfg['train_lr']),
                 batch_size=int(cfg['train_batch']))
    print('\n[完成] 模型已保存到 %s/（perception.npz、control.npz）' % cfg['models_dir'])
    print('       下一步：python3 main.py --mode run --controller nn --host <宿主机IP>')
    return 0


WEATHER_PRESETS = ('ClearNoon', 'HardRainNoon', 'ClearNight')


def mode_eval(cfg, host, port, scenarios, duration, out_dir):
    """多场景性能评测：默认遍历 4 种前车工况，结果写入 CSV 便于画图。"""
    weathers = cfg.get('weather_all') or (cfg.get('weather'),)
    results = []
    for weather in weathers:
        for name in scenarios:
            print('\n' + '=' * 72)
            print('评测：天气 %s / 工况 %s / 控制器 %s'
                  % (weather, name, cfg.get('controller')))
            print('=' * 72)
            local_cfg = dict(cfg)
            local_cfg['weather'] = weather
            _, summary = run_episode(local_cfg, host, port, name, duration)
            print_summary(summary)
            results.append((weather, name, summary))

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'eval_%s.csv' % cfg.get('controller', 'classic'))
    with open(path, 'w', encoding='utf-8') as fp:
        fp.write('weather,scenario,mean_abs_gap_error,min_gap,min_ttc,mean_v_ego\n')
        for weather, name, summary in results:
            fp.write('%s,%s,%.4f,%.4f,%.4f,%.4f\n'
                     % (weather, name, summary['mean_abs_gap_error'], summary['min_gap'],
                        summary['min_ttc'] if summary['min_ttc'] != float('inf') else -1.0,
                        summary['mean_v_ego']))
    print('\n[汇总] 共 %d 组评测，结果已写入 %s（可直接画图或粘进文档）' % (len(results), path))
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='基于神经网络的自适应跟车（ACC）仿真 —— 统一入口')
    parser.add_argument('--mode', default='scene',
                        choices=['check', 'scene', 'collect', 'train', 'run', 'eval'],
                        help='check 连接自检 / scene 跑工况 / collect 采集数据 / '
                             'train 训练网络（不需要 CARLA）/ run 在线运行 / eval 多场景评测')
    parser.add_argument('--params', default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                         'config', 'params.yaml'))
    parser.add_argument('--host', default=None, help='Windows 宿主机 IP（ipconfig 查看）')
    parser.add_argument('--port', type=int, default=None)
    parser.add_argument('--town', default=None, help='如 Town03')
    parser.add_argument('--weather', default=None, help='如 ClearNoon / HardRainNoon / ClearNight')
    parser.add_argument('--weather-all', action='store_true',
                        help='eval 模式下遍历预设的三种天气')
    parser.add_argument('--scenario', default=None,
                        help='constant / decelerate / emergency / cutin / all')
    parser.add_argument('--duration', type=float, default=None, help='单次仿真时长（秒）')
    parser.add_argument('--spawn-index', type=int, default=None,
                        help='指定自车出生点序号；默认 -1 表示自动预检挑选（推荐）')
    parser.add_argument('--no-follow', action='store_true',
                        help='关闭第三人称跟随镜头（默认开启；CARLA 窗口会跟随自车，便于录制动图）')
    parser.add_argument('--fast', action='store_true',
                        help='不做实时节奏控制（默认按真实时间 20 Hz 推进，录动图时画面速度正常）')
    parser.add_argument('--controller', default=None, choices=['classic', 'nn'],
                        help='classic = 经典 ACC；nn = 神经网络控制器')
    parser.add_argument('--models-dir', default=None, help='模型目录（默认 models）')
    parser.add_argument('--data-dir', default=None, help='数据目录（默认 data）')
    parser.add_argument('--out', default=None, help='同 --data-dir（采集/评测的输出目录）')
    parser.add_argument('--epochs', type=int, default=None, help='训练轮数')
    parser.add_argument('--lr', type=float, default=None, help='学习率')
    args = parser.parse_args(argv)

    cfg = load_params(args.params)
    for key, value in (('host', args.host), ('port', args.port),
                       ('town', args.town), ('weather', args.weather),
                       ('duration', args.duration), ('spawn_index', args.spawn_index),
                       ('controller', args.controller), ('models_dir', args.models_dir),
                       ('data_dir', args.data_dir if args.data_dir else args.out),
                       ('train_epochs', args.epochs), ('train_lr', args.lr)):
        if value is not None:
            cfg[key] = value
    if args.weather_all:
        cfg['weather_all'] = WEATHER_PRESETS
    if args.no_follow:
        cfg['spectator_follow'] = False
    if args.fast:
        cfg['realtime'] = False

    # train 模式完全不需要 CARLA，放在最前面，方便在没有仿真器的机器上复现
    if args.mode == 'train':
        try:
            return mode_train(cfg)
        except Exception as exc:
            print('\n[异常] %s: %s' % (type(exc).__name__, exc))
            return 1

    if carla is None:
        print('[错误] 未找到 carla 包。请先执行：')
        print('       python3 -m pip install --user carla==0.9.16')
        return 2

    scenario_name = args.scenario or 'decelerate'
    scenarios = scenario_mod.all_names() if scenario_name == 'all' else [scenario_name]
    for name in scenarios:
        if name not in scenario_mod.SCENARIOS:
            print('[错误] 未知工况：%s（可选：%s）' % (name, ', '.join(scenario_mod.all_names())))
            return 2

    print('目标服务端：%s:%d' % (cfg['host'], cfg['port']))
    try:
        if args.mode == 'check':
            return mode_check(cfg, cfg['host'], cfg['port'])
        if args.mode == 'eval':
            return mode_eval(cfg, cfg['host'], cfg['port'], scenarios,
                             cfg['duration'], cfg['data_dir'])
        if args.mode == 'collect':
            return mode_collect(cfg, cfg['host'], cfg['port'], scenarios,
                                cfg['duration'], cfg['data_dir'])
        return mode_scene(cfg, cfg['host'], cfg['port'], scenarios, cfg['duration'])
    except KeyboardInterrupt:
        print('\n[中断] 用户手动终止')
        return 130
    except Exception as exc:
        print('\n[异常] %s: %s' % (type(exc).__name__, exc))
        print('        连接类问题请先确认：宿主机 CarlaUE4.exe 已启动、IP 正确、防火墙已放行。')
        return 1


if __name__ == '__main__':
    sys.exit(main())
