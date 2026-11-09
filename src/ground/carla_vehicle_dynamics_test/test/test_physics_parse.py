#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``parse_physics`` 的离线单元测试（用假对象模拟 CARLA 的物理参数结构）。

为什么单独测这个：这两个字段的含义/名字在不同 CARLA 版本里都变过，
如果只在连上 CARLA 时才发现，就会变成"跑一半报错"或"算出 0.11 m 的轴距"。
这里用假对象把两种真实遇到的形态都覆盖：

* 轮位是**车身系偏移**（cm）—— 早期版本；
* 轮位是**世界坐标**（cm）—— 实测 CARLA 0.9.16 行为，车头转过角度后
  "前轴 x 减后轴 x"会完全失真（实测得出 0.11 m），必须投影到车头方向。

运行::

    python3 test/test_physics_parse.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from carla_vehicle_dynamics_test.carla_utils import parse_physics      # noqa: E402

PASSED = []
FAILED = []


class Wheel:
    """模拟 carla.WheelPhysicsControl（位置单位 cm）。"""

    def __init__(self, x, y=0.0, max_steer_angle=None, max_steer=None):
        self.position = type('P', (), {'x': x, 'y': y, 'z': 0.0})()
        if max_steer_angle is not None:
            self.max_steer_angle = max_steer_angle
        if max_steer is not None:
            self.max_steer = max_steer


class Physics:
    def __init__(self, wheels, mass=1500.0):
        self.wheels = wheels
        self.mass = mass


class Transform:
    def __init__(self, x, y, yaw):
        self.location = type('L', (), {'x': x, 'y': y, 'z': 0.0})()
        self.rotation = type('R', (), {'yaw': yaw})()


def check(name, condition, detail=''):
    if condition:
        PASSED.append(name)
        print('  [OK]   %s %s' % (name, detail))
    else:
        FAILED.append(name)
        print('  [FAIL] %s %s' % (name, detail))


def close(a, b, tol=1e-6):
    return abs(float(a) - float(b)) <= tol


def main():
    print('=' * 72)
    print('车辆物理参数解析测试（假对象，不需要 CARLA）')
    print('=' * 72)

    print('\n[1] 轮位 = 车身系偏移（cm），字段名 max_steer_angle')
    wheels = [Wheel(140.0, max_steer_angle=70.0), Wheel(140.0, max_steer_angle=70.0),
              Wheel(-140.0, max_steer_angle=70.0), Wheel(-140.0, max_steer_angle=70.0)]
    params = parse_physics(Physics(wheels, mass=1700.0))
    check('轴距 = 2.80 m', close(params['wheelbase'], 2.80),
          '(来源：%s)' % params['wheelbase_source'])
    check('最大前轮转角 = 70°', close(params['max_steer_deg'], 70.0))
    check('换算成弧度', close(params['max_steer_rad'], math.radians(70.0)))
    check('质量透传', close(params['mass'], 1700.0))

    print('\n[2] 轮位 = 世界坐标（cm）且车头转过 90°（实测 CARLA 0.9.16 的坑）')
    # 车在 (1000, 2000) cm，车头朝 +y（yaw=90）：前轴 y=+140，后轴 y=-140
    world_wheels = [Wheel(1000.0, 2140.0, max_steer_angle=70.0),
                    Wheel(1000.0, 2140.0, max_steer_angle=70.0),
                    Wheel(1000.0, 1860.0, max_steer_angle=70.0),
                    Wheel(1000.0, 1860.0, max_steer_angle=70.0)]
    tf = Transform(10.0, 20.0, 90.0)                     # 世界坐标单位 m
    params_w = parse_physics(Physics(world_wheels), tf)
    check('轴距仍然算对 = 2.80 m', close(params_w['wheelbase'], 2.80),
          '(来源：%s)' % params_w['wheelbase_source'])
    check('识别为"世界坐标投影"而不是车身系',
          params_w['wheelbase_source'] == '世界坐标投影')

    print('\n[3] 轮位 = 世界坐标且 yaw = 0（此时两种解释巧合相等，任一都可）')
    world0 = [Wheel(1140.0, 0.0, max_steer_angle=70.0), Wheel(1140.0, 0.0, max_steer_angle=70.0),
              Wheel(860.0, 0.0, max_steer_angle=70.0), Wheel(860.0, 0.0, max_steer_angle=70.0)]
    params_0 = parse_physics(Physics(world0), Transform(10.0, 0.0, 0.0))
    check('轴距 = 2.80 m', close(params_0['wheelbase'], 2.80),
          '(来源：%s)' % params_0['wheelbase_source'])

    print('\n[3b] 轮位 = 车身系偏移 + 车辆在离原点很远的地方（判别器必须选对）')
    body = [Wheel(123.5), Wheel(123.5), Wheel(-123.5), Wheel(-123.5)]
    params_b = parse_physics(Physics(body), Transform(350.0, -1200.0, 55.0))
    check('轴距 = 2.47 m 而不是被投影算错', close(params_b['wheelbase'], 2.47),
          '(来源：%s)' % params_b['wheelbase_source'])
    check('识别为"车身系偏差"', params_b['wheelbase_source'] == '车身系偏差')

    print('\n[4] 兼容早期字段名 max_steer')
    old = [Wheel(130.0, max_steer=60.0), Wheel(130.0, max_steer=60.0),
           Wheel(-130.0, max_steer=60.0), Wheel(-130.0, max_steer=60.0)]
    params_old = parse_physics(Physics(old))
    check('轴距 = 2.60 m', close(params_old['wheelbase'], 2.60))
    check('转角 = 60°', close(params_old['max_steer_deg'], 60.0))

    print('\n[5] 异常/缺省情况必须兜底而不是崩')
    check('没有轮子时轴距用兜底值 2.8 m',
          close(parse_physics(Physics([], mass=0.0))['wheelbase'], 2.8))
    check('没有轮子时转角用兜底值 70°',
          close(parse_physics(Physics([]))['max_steer_deg'], 70.0))
    check('物理参数缺 mass 属性时不报错',
          close(parse_physics(Physics(wheels))['mass'], 1500.0))
    check('轮子缺转角字段时用兜底 70°',
          close(parse_physics(Physics([Wheel(140.0), Wheel(140.0),
                                       Wheel(-140.0), Wheel(-140.0)]))['max_steer_deg'], 70.0))
    absurd = [Wheel(100.0), Wheel(100.0), Wheel(99.0), Wheel(99.0)]     # 轴距 0.01 m，不合理
    check('轮位给出荒谬轴距时回退到兜底值',
          close(parse_physics(Physics(absurd))['wheelbase'], 2.8),
          '(来源：%s)' % parse_physics(Physics(absurd))['wheelbase_source'])

    print('\n[6] 原始轮位必须原样带出（用于核对坐标含义）')
    check('raw_wheel_x 长度 = 4', len(params['raw_wheel_x']) == 4)
    check('raw_wheel_x 数值正确',
          params['raw_wheel_x'] == [140.0, 140.0, -140.0, -140.0])

    print('\n' + '=' * 72)
    print('[结果] %d 项全部通过' % len(PASSED) if not FAILED
          else '[结果] 通过 %d 项，失败 %d 项：%s' % (len(PASSED), len(FAILED), FAILED))
    print('=' * 72)
    return 1 if FAILED else 0


if __name__ == '__main__':
    sys.exit(main())
