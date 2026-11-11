#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``features`` 的离线单元测试（不需要 CARLA）。

    python3 test/test_features.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from carla_lidar_pointcloud_segmentation import features as F  # noqa: E402

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
    print('features.py 离线单元测试')
    print('=' * 72)

    # ---------------- 1. 坐标与角度 ----------------
    print('\n[1] 逐点特征')
    pts = np.array([
        [3.0, 4.0, 0.0],      # r=5，方位角 atan2(4,3)
        [0.0, 2.0, 0.0],      # 正左方，方位角 pi/2
        [1.0, 0.0, 1.0],      # 正前方上 45 度
        [-2.0, 0.0, 0.0],     # 正后方
    ])
    feats = F.point_features(pts, ground_z=0.0, voxel=0.5)
    check('特征矩阵形状为 (N, 9)', feats.shape == (4, 9), str(feats.shape))
    check('x 原样保留', np.allclose(feats[:, 0], pts[:, 0]))
    check('y 原样保留', np.allclose(feats[:, 1], pts[:, 1]))
    check('水平距离 r = sqrt(x^2+y^2)', abs(feats[0, 3] - 5.0) < 1e-9, '%.6f' % feats[0, 3])
    check('正左方 sin_azimuth = 1', abs(feats[1, 4] - 1.0) < 1e-9, '%.6f' % feats[1, 4])
    check('正左方 cos_azimuth = 0', abs(feats[1, 5]) < 1e-9, '%.6f' % feats[1, 5])
    check('45 度俯仰角 ≈ 0.7854', abs(feats[2, 6] - np.pi / 4) < 1e-9, '%.6f' % feats[2, 6])
    check('原点附近不产生 nan', np.all(np.isfinite(F.point_features(
        np.array([[0.0, 0.0, 0.0]]), ground_z=0.0))))

    # ---------------- 2. 离地高度与路面估计 ----------------
    print('\n[2] 路面高度')
    ground = np.array([[float(i), 1.0, -2.0] for i in range(200)])
    obstacle = np.array([[5.0, 1.0, 0.5], [6.0, 1.0, 0.8]])
    mixed = np.vstack([ground, obstacle])
    z_est = F.estimate_ground_z(mixed)
    check('低分位能估出路面高度', abs(z_est + 2.0) < 1e-6, '%.4f' % z_est)
    check('点太少时退回 0', F.estimate_ground_z(ground[:3]) == 0.0)
    feats2 = F.point_features(mixed, ground_z=z_est)
    check('离地高度扣掉了路面', abs(feats2[-1, 7] - 2.8) < 1e-6, '%.4f' % feats2[-1, 7])
    check('路面点离地高度约为 0', abs(feats2[0, 7]) < 1e-6, '%.4f' % feats2[0, 7])

    # ---------------- 3. 邻域密度 ----------------
    print('\n[3] 体素密度')
    cluster = np.array([[0.0, 0.0, 0.0], [0.1, 0.1, 0.1], [0.2, 0.0, 0.1]])
    far = np.array([[20.0, 20.0, 20.0]])
    both = np.vstack([cluster, far])
    dens = F.neighbor_counts(both, voxel=0.5)
    check('同体素三点密度为 3', abs(dens[0] - 3.0) < 1e-9, '%.1f' % dens[0])
    check('孤立点密度为 1', abs(dens[-1] - 1.0) < 1e-9, '%.1f' % dens[-1])
    feats3 = F.point_features(both, ground_z=0.0, voxel=0.5)
    check('log_density = log(1+n)', abs(feats3[0, 8] - np.log(4.0)) < 1e-9)
    keys = F.voxel_keys(np.array([[0.6, -0.1, 1.2]]), voxel=0.5)
    check('体素键向下取整正确', list(keys[0]) == [1, -1, 2], str(keys[0].tolist()))

    # ---------------- 4. 下采样 ----------------
    print('\n[4] 体素下采样')
    pts4 = np.array([[0.0, 0.0, 0.0], [0.05, 0.0, 0.0], [5.0, 0.0, 0.0]])
    labs4 = np.array([1, 2, 0])
    kept_p, kept_l = F.voxel_downsample(pts4, labs4, voxel=0.5)
    check('同体素只保留一个点', kept_p.shape[0] == 2, str(kept_p.shape[0]))
    check('标签跟着点一起保留', list(kept_l) == [1, 0], str(kept_l.tolist()))
    check('下采样后点数不超过原点数', kept_p.shape[0] <= pts4.shape[0])

    # ---------------- 5. 标准化与统计 ----------------
    print('\n[5] 标准化')
    rng = np.random.default_rng(1)
    sample = rng.normal(size=(500, 9)) * 3.0 + 5.0
    mean, std = F.standardize_fit(sample)
    z = F.standardize_apply(sample, mean, std)
    check('标准化后均值约为 0', float(np.abs(z.mean(axis=0)).max()) < 1e-9)
    check('标准化后方差约为 1', float(np.abs(z.std(axis=0) - 1.0).max()) < 1e-9)
    const = np.ones((10, 9))
    mean_c, std_c = F.standardize_fit(const)
    check('常数维度的标准差按 1 处理', all(abs(s - 1.0) < 1e-9 for s in std_c))
    check('summarize 覆盖全部特征名', len(F.summarize(sample)) == len(F.FEATURE_NAMES) * 3 + 1)

    # ---------------- 6. 异常输入 ----------------
    print('\n[6] 输入校验')
    expect_error('点云形状错误时报错', lambda: F.point_features(np.zeros((5, 2))))
    expect_error('体素边长非正时报错', lambda: F.voxel_keys(pts, voxel=0.0))
    expect_error('标准化统计量维度不符时报错',
                 lambda: F.standardize_apply(sample, np.zeros(3), np.ones(3)))
    empty = F.point_features(np.zeros((0, 3)))
    check('空点云返回 (0, 9)', empty.shape == (0, 9), str(empty.shape))

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