# -*- coding: utf-8 -*-
"""命令行入口与演示流程。

演示流程（``--mode demo``，默认）按 **白天 → 黄昏 → 夜间 → 暴雨** 四个阶段切换天气，
每个阶段内依次执行"巡航 → 点刹 → 打右转向 → 回正"，用来完整展示四类灯光：

===========================  ==========================================
阶段/动作                    应当点亮的灯
===========================  ==========================================
白天 · 巡航                  无
黄昏 · 巡航                 近光灯
夜间 · 巡航                 近光灯
暴雨 · 巡航                 近光灯 + 雾灯
任意阶段 · 点刹              刹车灯
任意阶段 · 打方向/回正        对应侧转向灯（回正后自动熄灭）
===========================  ==========================================
"""

import argparse
import time

from .carla_utils import (DEFAULT_DT, DEFAULT_HOST, DEFAULT_PORT, DEFAULT_TOWN,
                          WEATHER_PRESETS, LightSession, carla, lane_steer,
                          set_spectator_follow)
from .light_logic import (BlinkerStateMachine, compose_state, decide_lights,
                          state_to_names, sun_phase)

PHASES = ('day', 'dusk', 'night', 'rain')
PHASE_SECONDS = 6.0          # 每个阶段总时长
BRAKE_WINDOW = (2.0, 2.5)    # 阶段内的点刹时间窗（秒）
STEER_WINDOW = (4.0, 5.0)    # 阶段内的打方向时间窗（秒）
BLINKER_STEER = 0.25         # 演示用的方向盘量（超过 0.15 会点亮转向灯）
CRUISE_THROTTLE = 0.35       # 演示巡航油门


def _phase_control(phase_time):
    """根据阶段内的时间给出 (throttle, brake, steer) 与动作说明。"""
    if BRAKE_WINDOW[0] <= phase_time < BRAKE_WINDOW[1]:
        return 0.0, 0.6, 0.0, '点刹'
    if STEER_WINDOW[0] <= phase_time < STEER_WINDOW[1]:
        return CRUISE_THROTTLE, 0.0, BLINKER_STEER, '打右转向'
    if phase_time >= STEER_WINDOW[1]:
        return CRUISE_THROTTLE, 0.0, 0.0, '回正'
    return CRUISE_THROTTLE, 0.0, 0.0, '巡航'


def _fmt_names(names):
    return '、'.join(names) if names else '无'


def run_demo(session, args, on_state=None):
    """执行四个天气阶段的演示，返回每个阶段的状态记录。

    ``on_state`` 是可选回调（ROS 节点用来把状态发布成话题），签名 ``(phase, action, state)``。
    """
    vehicle = session.spawn_vehicle(args.vehicle, args.spawn_index)
    carla_map = session.world.get_map()
    blinker = BlinkerStateMachine()
    records = []

    for index, key in enumerate(PHASES, 1):
        preset = WEATHER_PRESETS[key]
        sun, precipitation = session.set_weather(preset)
        print('\n' + '-' * 70)
        print('阶段 %d/%d：%s（太阳高度角 %.1f°，降水 %.0f，%s）'
              % (index, len(PHASES), preset['label'], sun, precipitation,
                 sun_phase(sun)))
        print('-' * 70)

        phase_time = 0.0
        last_action = None
        decision = decide_lights(sun, precipitation, 0.0, 0.0)
        while phase_time < args.phase_seconds:
            throttle, brake, steer_cmd, action = _phase_control(phase_time)
            # 巡航段用车道保持，演示转向段用固定方向盘
            steer = lane_steer(vehicle, carla_map) if steer_cmd == 0.0 else steer_cmd
            vehicle.apply_control(carla.VehicleControl(throttle=throttle, brake=brake,
                                                       steer=steer))
            session.tick()
            # 仿真步进 + 真实时间等待：同步模式下仿真是"跑到哪算哪"，
            # 不加这一句整段演示几秒就结束，肉眼根本看不清灯光切换。
            if args.realtime:
                time.sleep(session.dt)
            if args.follow:
                set_spectator_follow(session.world, vehicle)
            phase_time += session.dt

            decision = decide_lights(sun, precipitation, steer_cmd, brake)
            blinker_state, _ = blinker.update(steer_cmd)
            state = compose_state(decision, blinker_state)
            session.apply_lights(vehicle, state)

            if action != last_action:
                names = state_to_names(state)
                print('  t=%4.1fs %-6s → 灯光：%s%s'
                      % (phase_time, action, _fmt_names(names),
                         '   （%s）' % decision['reason'] if names else ''))
                last_action = action
            if on_state is not None:
                on_state(preset['label'], action, state)

        records.append({'phase': preset['label'], 'sun': sun,
                        'precipitation': precipitation,
                        'lights': state_to_names(compose_state(decision, blinker_state))})

    print('\n' + '=' * 70)
    print('演示汇总')
    print('=' * 70)
    print('%-8s %10s %10s   %s' % ('阶段', '太阳(°)', '降水', '灯光'))
    print('-' * 70)
    for record in records:
        print('%-8s %10.1f %10.0f   %s'
              % (record['phase'], record['sun'], record['precipitation'],
                 _fmt_names(record['lights'])))
    print('=' * 70)
    return records


def parse_positive_float(text, name='phase_seconds'):
    """共享的正数校验：``main.py`` 与 ROS 节点都用它，保证两条入口行为一致。

    没有这道校验时，阶段时长传 0 或负数会直接跳过整个循环——程序不报错、
    也没有任何输出，看起来像卡死。
    """
    try:
        value = float(text)
    except (TypeError, ValueError):
        raise ValueError('%s 必须是数值，收到 %r' % (name, text))
    if value <= 0.0:
        raise ValueError('%s 必须大于 0，收到 %r' % (name, text))
    return value


def _positive_float(text):
    """argparse 专用的包装：把 :func:`parse_positive_float` 的报错转成命令行提示。"""
    try:
        return parse_positive_float(text, '--phase-seconds')
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc))


def build_parser():
    parser = argparse.ArgumentParser(description='CARLA 地面载具灯光自动控制（演示）')
    parser.add_argument('--host', default=DEFAULT_HOST, help='Windows 宿主机 IP')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT)
    parser.add_argument('--town', default=DEFAULT_TOWN)
    parser.add_argument('--dt', type=float, default=DEFAULT_DT)
    parser.add_argument('--vehicle', default='vehicle.tesla.model3')
    parser.add_argument('--spawn-index', type=int, default=-1)
    parser.add_argument('--phase-seconds', type=_positive_float, default=PHASE_SECONDS,
                        help='每个天气阶段的时长（秒，必须大于 0）')
    parser.add_argument('--fast', action='store_true',
                        help='关闭实时节奏（默认按仿真时间 1:1 播放，方便观察灯光）')
    parser.add_argument('--no-follow', action='store_true',
                        help='关闭第三人称跟随镜头（默认跟随车辆）')
    return parser


def finalize_args(args):
    """把 ``--fast`` / ``--no-follow`` 两个开关折算成 ``realtime`` / ``follow``。

    ``main.py`` 与 ROS 2 节点都走 :func:`carla_auto_light.app.run_demo`，两者共用同一套
    参数定义，但开关的默认值必须在这里统一折算——否则节点那条路径上 ``args.realtime``
    根本不存在，`ros2 launch` 会直接抛 ``AttributeError``。
    """
    args.realtime = not getattr(args, 'fast', False)
    args.follow = not getattr(args, 'no_follow', False)
    return args


def main(argv=None):
    args = finalize_args(build_parser().parse_args(argv))
    session = None
    try:
        session = LightSession(args.host, args.port)
        session.ensure_town(args.town)
        session.enable_sync(args.dt)
        run_demo(session, args)
    except KeyboardInterrupt:
        print('\n[中断] 用户中止')
    except RuntimeError as exc:
        # 未安装 carla 包、连不上服务端、出生点被占用等，给出可读提示而不是抛栈
        print('[错误] %s' % exc)
        return 1
    finally:
        if session is not None:
            session.close()
    return 0
