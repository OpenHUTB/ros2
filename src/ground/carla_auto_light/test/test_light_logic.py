#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``carla_auto_light.light_logic`` 的离线单元测试（不需要 CARLA）。

运行::

    python3 test/test_light_logic.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from carla_auto_light.light_logic import (      # noqa: E402
    BlinkerStateMachine, compose_state, decide_lights, state_to_names, sun_phase)

PASSED = []
FAILED = []


def check(name, condition, detail=''):
    if condition:
        PASSED.append(name)
        print('  [OK]   %s %s' % (name, detail))
    else:
        FAILED.append(name)
        print('  [FAIL] %s %s' % (name, detail))


def main():
    print('=' * 72)
    print('车辆灯光决策逻辑单元测试（不需要 CARLA）')
    print('=' * 72)

    print('\n[1] 太阳高度角 → 时段')
    check('太阳 60° → 白天', sun_phase(60.0) == '白天')
    check('太阳 3° → 黄昏/黎明', sun_phase(3.0) == '黄昏/黎明')
    check('太阳 -10° → 夜间', sun_phase(-10.0) == '夜间')

    print('\n[2] 自动大灯与雾灯')
    day = decide_lights(sun_altitude=60.0, precipitation=0.0, steer=0.0, brake=0.0)
    check('白天无降水：全部熄灭', not any(day[k] for k in
          ('low_beam', 'fog', 'left_blinker', 'right_blinker', 'brake_light')))
    dusk = decide_lights(sun_altitude=3.0, precipitation=0.0, steer=0.0, brake=0.0)
    check('黄昏：近光灯亮', dusk['low_beam'] and not dusk['fog'])
    night = decide_lights(sun_altitude=-10.0, precipitation=0.0, steer=0.0, brake=0.0)
    check('夜间：近光灯亮', night['low_beam'] and not night['fog'])
    rain = decide_lights(sun_altitude=60.0, precipitation=50.0, steer=0.0, brake=0.0)
    check('白天暴雨：近光灯 + 雾灯都亮', rain['low_beam'] and rain['fog'])
    light_rain = decide_lights(sun_altitude=60.0, precipitation=10.0, steer=0.0, brake=0.0)
    check('小雨（10）：不开雾灯', not light_rain['fog'])
    check('决策能给出原因', '太阳高度角' in dusk['reason'] and '降水' in rain['reason'])

    print('\n[3] 转向灯（瞬时判断）')
    right = decide_lights(sun_altitude=60.0, precipitation=0.0, steer=0.30, brake=0.0)
    check('方向盘 +0.30：右转向灯', right['right_blinker'] and not right['left_blinker'])
    left = decide_lights(sun_altitude=60.0, precipitation=0.0, steer=-0.30, brake=0.0)
    check('方向盘 -0.30：左转向灯', left['left_blinker'] and not left['right_blinker'])
    middle = decide_lights(sun_altitude=60.0, precipitation=0.0, steer=0.10, brake=0.0)
    check('方向盘 0.10（未到阈值）：不打灯',
          not middle['left_blinker'] and not middle['right_blinker'])

    print('\n[4] 刹车灯')
    braking = decide_lights(sun_altitude=60.0, precipitation=0.0, steer=0.0, brake=0.30)
    check('刹车 0.30：刹车灯亮', braking['brake_light'])
    coasting = decide_lights(sun_altitude=60.0, precipitation=0.0, steer=0.0, brake=0.0)
    check('不刹车：刹车灯不亮', not coasting['brake_light'])

    print('\n[5] 转向灯状态机（回正自动熄灭）')
    sm = BlinkerStateMachine()
    state, changed = sm.update(0.30)
    check('第一次打方向：点亮右灯', state == 'right' and changed)
    state, changed = sm.update(0.10)
    check('方向盘回到 0.10（仍在熄灯阈值外）：保持点亮', state == 'right' and not changed)
    state, changed = sm.update(0.05)
    check('方向盘回到 0.05（阈值边界，含边界）：自动熄灭', state is None and changed)
    state, changed = sm.update(0.0)
    check('继续保持熄灭且无状态变化', state is None and not changed)
    state, _ = sm.update(-0.30)
    check('反向打方向：切到左灯', state == 'left')
    state, _ = sm.update(0.30)
    check('再切回右侧', state == 'right')
    sm.reset()
    check('reset 后为熄灭', sm.active is None and sm.latched is False)

    print('\n[6] 合成最终状态（转向灯状态机优先）')
    decision = decide_lights(sun_altitude=-10.0, precipitation=0.0, steer=0.0, brake=0.2)
    merged = compose_state(decision, 'left')
    check('合成后左灯亮', merged['left_blinker'] and not merged['right_blinker'])
    check('合成的状态仍保留近光灯与刹车灯',
          merged['low_beam'] and merged['brake_light'])
    merged_none = compose_state(decision, None)
    check('转向灯状态机为 None 时不影响其它灯',
          merged_none['low_beam'] and not merged_none['left_blinker'])

    print('\n[7] 日志名称转换')
    check('夜间 + 刹车灯的中文名',
          state_to_names({'low_beam': True, 'brake_light': True}) == ['近光灯', '刹车灯'])
    check('全灭时返回空列表', state_to_names({}) == [])

    print('\n[8] 两条入口共用参数折算（ros2 launch 那条路径曾经漏了 realtime/follow）')
    from carla_auto_light.app import build_parser, finalize_args
    defaults = finalize_args(build_parser().parse_args([]))
    check('默认：realtime 与 follow 都为真', defaults.realtime and defaults.follow)
    fast = finalize_args(build_parser().parse_args(['--fast', '--no-follow']))
    check('--fast / --no-follow：两项都关', (not fast.realtime) and (not fast.follow))
    check('折算后仍保留原有参数', fast.vehicle == 'vehicle.tesla.model3' and fast.phase_seconds > 0)

    print('\n[9] 边界值（阈值本身即触发，全部含边界）')
    check('太阳正好 5°：开近光灯', decide_lights(5.0, 0.0, 0.0, 0.0)['low_beam'])
    check('太阳 5.1°：不开近光灯', not decide_lights(5.1, 0.0, 0.0, 0.0)['low_beam'])
    check('降水正好 30：开雾灯', decide_lights(60.0, 30.0, 0.0, 0.0)['fog'])
    check('降水 29.9：不开雾灯', not decide_lights(60.0, 29.9, 0.0, 0.0)['fog'])
    check('方向盘正好 0.15：右转向灯', decide_lights(60.0, 0.0, 0.15, 0.0)['right_blinker'])
    check('方向盘 -0.15：左转向灯', decide_lights(60.0, 0.0, -0.15, 0.0)['left_blinker'])
    check('刹车正好 0.05：刹车灯', decide_lights(60.0, 0.0, 0.0, 0.05)['brake_light'])
    check('刹车 0.049：不亮', not decide_lights(60.0, 0.0, 0.0, 0.049)['brake_light'])
    sm2 = BlinkerStateMachine()
    sm2.update(0.15)
    check('状态机：方向盘 0.15 即点亮', sm2.active == 'right')

    print('\n[10] 输入健壮性（非数值统一抛 ValueError）')
    for bad in ('abc', None):
        try:
            decide_lights(bad, 0.0, 0.0, 0.0)
        except ValueError:
            check('decide_lights 对 %r 抛 ValueError' % (bad,), True)
        except Exception as exc:                                    # noqa: BLE001
            check('decide_lights 对 %r 抛 ValueError' % (bad,), False, '抛了 %r' % exc)
        else:
            check('decide_lights 对 %r 抛 ValueError' % (bad,), False, '没有抛异常')
    check('数字字符串可以正常使用', decide_lights('60', '0', '0', '0')['low_beam'] is False)

    print('\n[11] 命令行参数校验')
    try:
        build_parser().parse_args(['--phase-seconds', '-1'])
    except SystemExit:
        check('--phase-seconds 为负时 argparse 报错退出', True)
    else:
        check('--phase-seconds 为负时 argparse 报错退出', False, '没有报错')
    check('正常值可通过',
          build_parser().parse_args(['--phase-seconds', '4']).phase_seconds == 4.0)

    print('\n[12] 两条入口共用同一个正数校验（main.py 与 ros2 launch）')
    from carla_auto_light.app import parse_positive_float
    check('"4" → 4.0', parse_positive_float('4') == 4.0)
    for bad in ('0', '-1', 'abc', None):
        try:
            parse_positive_float(bad)
        except ValueError as exc:
            check('parse_positive_float(%r) 抛 ValueError' % (bad,), True, '（%s）' % exc)
        else:
            check('parse_positive_float(%r) 抛 ValueError' % (bad,), False, '没有抛异常')

    print('\n[13] setup.py 打包时要把脚本入口一起装进 share')
    import os as _os
    setup_py = _os.path.join(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))),
                             'setup.py')
    with open(setup_py, encoding='utf-8') as fp:
        setup_src = fp.read()
    for name in ('main.py', 'main.sh', 'main.bat', 'package.xml', 'requirements.txt'):
        check('setup.py 的 data_files 含 %s' % name, "'%s'" % name in setup_src)

    print('\n' + '=' * 72)
    print('[结果] %d 项全部通过' % len(PASSED) if not FAILED
          else '[结果] 通过 %d 项，失败 %d 项：%s' % (len(PASSED), len(FAILED), FAILED))
    print('=' * 72)
    return 1 if FAILED else 0


if __name__ == '__main__':
    sys.exit(main())
