#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``labels`` 的离线单元测试（不需要 CARLA）。

    python3 test/test_labels.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from carla_lidar_pointcloud_segmentation import labels as L  # noqa: E402

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


def main():
    print('=' * 72)
    print('labels.py 离线单元测试')
    print('=' * 72)

    # ---------------- 1. 标签映射 ----------------
    print('\n[1] 标签映射')
    check('路面 Roads(7) -> 可行驶', L.tag_to_class(7) == L.DRIVABLE)
    check('车道线 RoadLines(6) -> 可行驶', L.tag_to_class(6) == L.DRIVABLE)
    check('车辆 Vehicles(11) -> 车辆', L.tag_to_class(11) == L.VEHICLE)
    check('行人 Pedestrians(4) -> 行人', L.tag_to_class(4) == L.PEDESTRIAN)
    check('建筑 Buildings(1) -> 障碍物', L.tag_to_class(1) == L.OBSTACLE)
    check('植被 Vegetation(10) -> 障碍物', L.tag_to_class(10) == L.OBSTACLE)
    check('未知标签返回 -1', L.tag_to_class(99) == L.UNKNOWN)
    check('标签名表覆盖 0~15', sorted(L.SEMANTIC_TAG_NAMES) == list(range(16)),
          str(sorted(L.SEMANTIC_TAG_NAMES)))
    check('可行驶标签不重复计数', len(set(L.TAG_TO_CLASS)) == len(L.TAG_TO_CLASS))
    check('四类名称与类别数一致', len(L.CLASS_NAMES) == L.N_CLASSES)

    # ---------------- 2. 批量转换与过滤 ----------------
    print('\n[2] 批量转换')
    tags = [7, 11, 4, 99, 1]
    grouped = L.group_labels(tags)
    check('批量转换长度不变', len(grouped) == len(tags))
    check('批量转换保留未知为 -1', grouped[3] == -1, str(grouped))
    keep, valid = L.filter_known(grouped)
    check('filter_known 丢掉未知点', keep == [0, 1, 2, 4], str(keep))
    check('filter_known 返回有效标签', valid == [L.DRIVABLE, L.VEHICLE,
                                                 L.PEDESTRIAN, L.OBSTACLE], str(valid))
    keep2, valid2 = L.filter_known([-1, -1, -1])
    check('全部未知时返回空', keep2 == [] and valid2 == [], '%s %s' % (keep2, valid2))

    # ---------------- 3. 类别权重 ----------------
    print('\n[3] 类别权重')
    many = [L.DRIVABLE] * 900 + [L.VEHICLE] * 90 + [L.OBSTACLE] * 10
    weights = L.class_weights(many)
    check('稀有类权重更大', weights[L.OBSTACLE] > weights[L.VEHICLE] > weights[L.DRIVABLE],
          str(weights))
    check('权重均值为 1', abs(sum(w for w in weights if w > 0) / 3 - 1.0) < 1e-6,
          '%.4f' % (sum(weights) / 3))
    check('无样本类别权重为 0', L.class_weights([L.DRIVABLE] * 10)[L.PEDESTRIAN] == 0.0)
    check('空输入返回全 0', L.class_weights([]) == [0.0] * L.N_CLASSES)

    # ---------------- 4. 类别分布 ----------------
    print('\n[4] 类别分布')
    dist = L.class_distribution(many)
    check('分布比例之和约为 1', abs(sum(dist.values()) - 1.0) < 1e-6, '%.6f' % sum(dist.values()))
    check('可行驶占九成', abs(dist['drivable'] - 0.9) < 1e-6, '%.4f' % dist['drivable'])
    empty = L.class_distribution([])
    check('空输入返回全 0', all(v == 0.0 for v in empty.values()))

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