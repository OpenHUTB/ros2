#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``carla_common`` 点云解析的离线单元测试（不需要 CARLA）。

这里用假的测量对象模拟两种 CARLA 客户端：

* PyPI 重打包版：点对象没有 ``x``，只有二进制缓冲区；
* 官方版：既有缓冲区，也有 ``x`` / ``object_tag`` 属性。

解析函数必须先吃缓冲区，取不到才回退到属性，两条路都要能过测试。

    python3 test/test_buffers.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from carla_lidar_pointcloud_segmentation import carla_common as cc   # noqa: E402

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


def expect_error(desc, func, exc=AttributeError):
    try:
        func()
    except exc:
        check(desc, True)
    except Exception as other:      # noqa: BLE001
        check(desc, False, '抛了 %s' % type(other).__name__)
    else:
        check(desc, False, '没有抛异常')


class FakeMeasurement:
    """假测量对象：有 raw_data 和长度，可选带点属性。"""

    def __init__(self, raw, items=None):
        self.raw_data = raw
        self._items = items or []
        self._n = len(items) if items else len(raw) // (4 * 6 if len(raw) % 24 == 0 else 16)

    def __len__(self):
        return self._n

    def __iter__(self):
        return iter(self._items)


class AttrPoint:
    """官方版的点：直接带 x / y / z。"""

    def __init__(self, x, y, z, tag=0):
        self.x, self.y, self.z, self.object_tag = x, y, z, tag


class NestedPoint:
    """另一种点：坐标在 point 里。"""

    def __init__(self, x, y, z, tag=0):
        self.point = AttrPoint(x, y, z)
        self.object_tag = tag


class EmptyPoint:
    """既没有 x 也没有 point.x 的点。"""

    object_tag = 3


def raw_buffer(points):
    """构造普通点云缓冲区（每点 4 个 float32）。"""
    return np.asarray(points, dtype=np.float32).tobytes()


def semantic_buffer(points, tags):
    """构造语义点云缓冲区（每点 6 个 32 位，最后一个是 uint32 标签）。"""
    arr = np.zeros((len(points), 6), dtype=np.float32)
    arr[:, :3] = np.asarray(points, dtype=np.float32)
    arr.view(np.uint32)[:, 5] = np.asarray(tags, dtype=np.uint32)
    return arr.tobytes()


def main():
    print('=' * 72)
    print('carla_common 点云解析测试')
    print('=' * 72)

    # ---------------- 1. 普通点云 ----------------
    print('\n[1] 普通点云')
    points = [[1.0, 2.0, 3.0, 0.9], [4.0, 5.0, 6.0, 0.8]]
    data = type('M', (), {'raw_data': raw_buffer(points), '__len__': lambda self: 2})()
    cloud = cc.points_from_raw(data)
    check('形状为 (N, 3)', cloud.shape == (2, 3), str(cloud.shape))
    check('坐标取前三列', np.allclose(cloud[0], [1.0, 2.0, 3.0]), str(cloud[0]))
    check('忽略 cos_inc_angle', cloud.shape[1] == 3)

    fallback = FakeMeasurement(b'', [AttrPoint(1, 2, 3), AttrPoint(4, 5, 6)])
    cloud2 = cc.points_from_raw(fallback)
    check('没有缓冲区时按属性兜底', np.allclose(cloud2, [[1, 2, 3], [4, 5, 6]]))
    nested = FakeMeasurement(b'', [NestedPoint(7, 8, 9)])
    check('兼容 point.x 写法', np.allclose(cc.points_from_raw(nested), [[7, 8, 9]]))
    expect_error('两种写法都没有时报错',
                 lambda: cc.points_from_raw(FakeMeasurement(b'', [EmptyPoint()])))

    empty = type('M', (), {'raw_data': b'', '__len__': lambda self: 0})()
    check('空点云返回 (0, 3)', cc.points_from_raw(empty).shape == (0, 3))

    # ---------------- 2. 语义点云 ----------------
    print('\n[2] 语义点云')
    tags = [7, 11]
    sem = type('M', (), {
        'raw_data': semantic_buffer([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], tags),
        '__len__': lambda self: 2})()
    coords, decoded = cc.points_from_semantic(sem)
    check('坐标形状为 (N, 3)', coords.shape == (2, 3), str(coords.shape))
    check('标签形状为 (N,)', decoded.shape == (2,), str(decoded.shape))
    check('标签按 uint32 解码', list(decoded) == tags, str(decoded.tolist()))
    check('标签是整数类型', decoded.dtype.kind in 'iu', str(decoded.dtype))
    check('坐标正确', np.allclose(coords[1], [4.0, 5.0, 6.0]), str(coords[1]))

    sem_fallback = FakeMeasurement(b'', [AttrPoint(1, 2, 3, tag=6), AttrPoint(4, 5, 6, tag=4)])
    coords2, tags2 = cc.points_from_semantic(sem_fallback)
    check('没有缓冲区时按属性兜底', list(tags2) == [6, 4], str(tags2.tolist()))
    sem_nested = FakeMeasurement(b'', [NestedPoint(1, 2, 3, tag=1)])
    _, tags3 = cc.points_from_semantic(sem_nested)
    check('兜底也支持 point.x 写法', list(tags3) == [1])

    sem_idx = type('M', (), {
        'raw_data': semantic_buffer([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], tags),
        '__len__': lambda self: 2})()
    _, tags_i, indices = cc.semantic_arrays(sem_idx)
    check('semantic_arrays 同时给出标签', list(tags_i) == tags, str(tags_i.tolist()))
    check('semantic_arrays 给出 actor 编号', indices.shape == (2,), str(indices.shape))

    empty_sem = type('M', (), {'raw_data': b'', '__len__': lambda self: 0})()
    coords_e, tags_e = cc.points_from_semantic(empty_sem)
    check('空语义点云返回两个空数组',
          coords_e.shape == (0, 3) and tags_e.size == 0)

    # ---------------- 3. 配对对齐 ----------------
    print('\n[3] 双雷达配对')
    same = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    pts, matched_tags, mask = cc.align_pair(same, same.copy(), np.array([7, 11], dtype=np.int32))
    check('坐标完全一致时直接对齐', mask.all() and list(matched_tags) == [7, 11],
          str(matched_tags.tolist()))

    raw_shifted = np.array([[1.02, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    _, tags4, mask4 = cc.align_pair(raw_shifted, same, np.array([7, 11], dtype=np.int32))
    check('偏移 2 cm 仍能配上', mask4.all(), str(mask4.tolist()))
    check('配上的标签取最近点', list(tags4) == [7, 11], str(tags4.tolist()))

    far = np.array([[50.0, 50.0, 50.0]], dtype=np.float32)
    _, _, mask5 = cc.align_pair(far, same, np.array([7, 11], dtype=np.int32))
    check('离得太远的不配对', not mask5.any(), str(mask5.tolist()))

    empty_match = cc.align_pair(np.zeros((0, 3), np.float32), same,
                                np.array([7, 11], dtype=np.int32))
    check('空输入返回空结果', empty_match[0].shape == (0, 3))

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