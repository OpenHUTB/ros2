#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``control`` 的离线单元测试（不需要 CARLA）。

    python3 test/test_control.py
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from carla_lidar_pointcloud_segmentation import control as C   # noqa: E402
from carla_lidar_pointcloud_segmentation import labels as L    # noqa: E402

_passed = 0
_failed = []


def check(desc, ok, detail=''):
    global _passed
    if ok:
        _passed += 1
        print('[通过] %s' % desc)
    else:
        _failed.append(desc)
        print('[失败] %s%s' % (desc, ('  ' + detail) if detail else ''))


def expect_error(desc, func, exc=ValueError):
    try:
        func()
    except exc:
        check(desc, True)
    except Exception as other:      # noqa: BLE001
        check(desc, False, '抛了 %s' % type(other).__name__)
    else:
        check(desc, False, '没有抛异常')


def main():
    print('=' * 72)
    print('control.py 离线单元测试')
    print('=' * 72)

    # ---------------- 1. 走廊 ----------------
    print('\n[1] 车身走廊')
    pts = np.array([[5.0, 0.0, 0.0], [5.0, 1.5, 0.0], [5.0, 0.5, 0.0],
                    [60.0, 0.0, 0.0], [0.5, 0.0, 0.0]])
    mask = C.corridor_mask(pts, half_width=1.2, x_min=1.0, x_max=50.0)
    check('走廊内只留下 2 个点', int(mask.sum()) == 2, str(int(mask.sum())))
    check('横向超出的点被排除', not mask[1])
    check('纵向超出的点被排除', not mask[3])
    check('太近的点被排除', not mask[4])
    expect_error('点云缺列时报错', lambda: C.corridor_mask(np.zeros((3, 1))))
    expect_error('走廊半宽非正时报错', lambda: C.corridor_mask(pts, half_width=0.0))
    expect_error('纵向区间非法时报错', lambda: C.corridor_mask(pts, x_min=10.0, x_max=5.0))

    # ---------------- 2. 前方余量 ----------------
    print('\n[2] 前方余量')
    empty = C.free_distance(np.zeros((0, 3)), np.zeros(0, dtype=int), x_max=50.0)
    check('没有点时返回最大量程', abs(empty - 50.0) < 1e-9, '%.1f' % empty)

    scene = np.array([[10.0, 0.0, 0.0],    # 正前方车辆
                      [20.0, 0.0, 0.0],    # 更远的车
                      [8.0, 5.0, 0.0],     # 旁边的东西，不在走廊
                      [6.0, 0.0, -1.7]])   # 路面
    cls = np.array([L.VEHICLE, L.VEHICLE, L.OBSTACLE, L.DRIVABLE])
    free = C.free_distance(scene, cls, half_width=1.2, x_min=1.0, x_max=50.0)
    check('返回最近的阻挡物距离', abs(free - 10.0) < 1e-9, '%.2f' % free)

    only_road = C.free_distance(np.array([[5.0, 0.0, -1.7]]), np.array([L.DRIVABLE]))
    check('路面不算阻挡物', abs(only_road - 50.0) < 1e-9, '%.1f' % only_road)

    pedestrian = C.free_distance(np.array([[12.0, 0.5, 0.0]]), np.array([L.PEDESTRIAN]))
    check('行人算阻挡物', abs(pedestrian - 12.0) < 1e-9, '%.2f' % pedestrian)
    expect_error('点与类别数量不一致时报错',
                 lambda: C.free_distance(np.zeros((2, 3)), np.zeros(1, dtype=int)))

    # ---------------- 3. 速度系数 ----------------
    print('\n[3] 速度系数')
    check('很远时全速', C.speed_factor(30.0) == 1.0)
    check('太近时刹停', C.speed_factor(3.0) == 0.0)
    check('中间线性过渡', abs(C.speed_factor(8.0, comfort=12.0, stop=4.0) - 0.5) < 1e-9,
          '%.3f' % C.speed_factor(8.0))
    check('系数始终在 [0,1]', all(0.0 <= C.speed_factor(d) <= 1.0
                                  for d in (0.0, 3.9, 8.0, 11.9, 50.0)))
    expect_error('comfort 不大于 stop 时报错', lambda: C.speed_factor(5.0, comfort=4.0, stop=4.0))

    # ---------------- 4. 纯跟踪 ----------------
    print('\n[4] 纯跟踪')
    path = C.straight_path(length=100.0, step=5.0)
    check('参考轨迹点数正确', len(path) == 21, str(len(path)))
    steer = C.pure_pursuit((0.0, 0.0, 0.0), path, wheelbase=2.9, lookahead=6.0)
    check('正对直线时转角约为 0', abs(steer) < 1e-6, '%.6f' % steer)

    offset = [(x, 3.0) for x, _ in path]
    steer_left = C.pure_pursuit((0.0, 0.0, 0.0), offset, lookahead=6.0)
    check('目标在左侧时左转（正转角）', steer_left > 0.0, '%.4f' % steer_left)
    steer_right = C.pure_pursuit((0.0, 0.0, 0.0), [(x, -3.0) for x, _ in path], lookahead=6.0)
    check('目标在右侧时右转（负转角）', steer_right < 0.0, '%.4f' % steer_right)
    check('转角不超过上限', abs(C.pure_pursuit((0.0, 0.0, 0.0), [(5.0, 50.0)],
                                              max_steer=0.6)) <= 0.6 + 1e-9)
    behind = C.pure_pursuit((0.0, 0.0, 0.0), [(-10.0, 0.0), (-5.0, 0.0)])
    check('参考点全在身后时转角为 0', behind == 0.0, '%.4f' % behind)
    expect_error('轴距非正时报错', lambda: C.pure_pursuit((0.0, 0.0, 0.0), path, wheelbase=0.0))
    expect_error('参考轨迹为空时报错', lambda: C.pure_pursuit((0.0, 0.0, 0.0), []))

    # ---------------- 5. 归一化与自检 ----------------
    print('\n[5] 归一化与自检')
    check('转角转成归一化指令', abs(C.normalized_steer(0.3, 0.6) - 0.5) < 1e-9)
    check('归一化指令被截断', C.normalized_steer(5.0, 0.6) == 1.0)
    expect_error('最大转角非正时报错', lambda: C.normalized_steer(0.1, 0.0))

    classes = np.array([L.DRIVABLE, L.DRIVABLE, L.VEHICLE, L.DRIVABLE])
    check('可行驶比例为 3/4', abs(C.drivable_ratio(classes) - 0.75) < 1e-9,
          '%.3f' % C.drivable_ratio(classes))
    check('空点云返回 0', C.drivable_ratio(np.zeros(0, dtype=int)) == 0.0)
    pts2 = np.array([[10.0, 0.0, 0.0], [10.0, 9.0, 0.0]])
    cls2 = np.array([L.DRIVABLE, L.VEHICLE])
    check('按走廊统计更好', abs(C.drivable_ratio(cls2, half_width=4.0, points=pts2) - 1.0) < 1e-9,
          '%.3f' % C.drivable_ratio(cls2, half_width=4.0, points=pts2))

    print('\n' + '=' * 72)
    if _failed:
        print('[结果] %d 项通过，%d 项失败' % (_passed, len(_failed)))
        for name in _failed:
            print('  失败：%s' % name)
        return 1
    print('[结果] %d 项全部通过' % _passed)
    print('=' * 72)
    return 0


if __name__ == '__main__':
    sys.exit(main())