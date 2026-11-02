#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""数据集诊断：逐工况检查"雷达特征"与"真实车距"的一致性。

不需要 CARLA，直接读取 ``data/dataset_*.npz``。

用法::

    cd ~/ros2_ws/src/carla_nn_car_following
    python3 analyze_dataset.py                 # 分析 data/
    python3 analyze_dataset.py data --detail   # 额外打印若干帧的原始数据

判据说明（重要）
----------------
**主要判据是"偏差"的统计量**，而不是相关系数：

   偏差 = 真实车距 - 雷达最短回波距离

雷达测到的是前车车尾，因此偏差应恒等于"半个车身长度"（约 2~4 m），
其**标准差**反映雷达测量的稳定性：

* 偏差标准差 ≤ 1.0 m 且"偏离超过 5 m 的帧"≤ 5%  → ✅ 合格，可以直接训练
* 偏差标准差 1.0~3.0 m                            → ⚠️ 一般，可训练但精度会受影响
* 偏差标准差 > 3.0 m                              → ❌ 不合格，需检查雷达安装与过滤参数

**相关系数仅作辅助参考**：在"匀速"这类真实车距几乎不变的工况里，
信号本身没有变化，相关系数会被噪声支配，低值是正常现象，不能据此判断雷达好坏。
"""

import glob
import os
import sys

import numpy as np


def analyze(path, detail=False):
    with np.load(path, allow_pickle=False) as data:
        gap = np.array(data['gap_true'])
        radar_d = np.array(data['radar_d'])
        radar_az = np.array(data['radar_azimuth'])
        radar_closing = np.array(data['radar_closing'])
        n_points = np.array(data['radar_n_points'])
        v_rel = np.array(data['v_rel_true'])
        v_ego = np.array(data['v_ego'])

    name = os.path.basename(path).replace('dataset_', '').replace('.npz', '')
    diff = gap - radar_d
    no_target = float(np.mean(radar_d >= 99.0) * 100.0)
    far_off = float(np.mean(np.abs(diff) > 5.0) * 100.0)
    # 相关系数只在车距有明显变化时才有参考价值
    if gap.std() > 1.0:
        correlation = float(np.corrcoef(radar_d, gap)[0, 1])
    else:
        correlation = float('nan')

    if abs(diff.std()) <= 1.0 and far_off <= 5.0:
        verdict = '[合格]'
    elif abs(diff.std()) <= 3.0 and far_off <= 15.0:
        verdict = '[一般]'
    else:
        verdict = '[不合格]'

    print('%-11s 帧数 %4d | 偏差均值 %+6.2f m | 偏差标准差 %5.2f m | 偏离>5m %5.1f%% | '
          '无目标 %5.1f%% | 相关系数 %s | %s'
          % (name, len(gap), diff.mean(), diff.std(), far_off, no_target,
             ('%+6.3f' % correlation) if correlation == correlation else '   —— ',
             verdict))
    print('            真实车距 %6.1f~%6.1f（均值 %5.1f）| 雷达 %6.1f~%6.1f（均值 %5.1f）| '
          '相对速度 %+6.1f~%+6.1f | 点云数 %3.0f~%3.0f'
          % (gap.min(), gap.max(), gap.mean(), radar_d.min(), radar_d.max(), radar_d.mean(),
             v_rel.min(), v_rel.max(), n_points.min(), n_points.max()))

    worst = np.argsort(-np.abs(diff))[:5]
    print('            偏差最大的 5 帧：' + ', '.join(
        't=%.1fs(雷达%.1f/真值%.1f)' % (i * 0.05, radar_d[i], gap[i]) for i in worst))

    if detail:
        print('            前 8 帧明细：')
        print('               t      雷达    方位角   接近速度   点云   真值   相对速度  自车')
        for i in range(min(8, len(gap))):
            print('             %5.1f  %7.2f  %+7.3f  %+8.2f  %4.0f  %6.2f  %+8.2f  %6.2f'
                  % (i * 0.05, radar_d[i], radar_az[i], radar_closing[i],
                     n_points[i], gap[i], v_rel[i], v_ego[i]))
    print()
    return name, diff.mean(), diff.std(), far_off, no_target, correlation, verdict


def main():
    data_dir = 'data'
    detail = False
    for arg in sys.argv[1:]:
        if arg == '--detail':
            detail = True
        else:
            data_dir = arg

    paths = sorted(glob.glob(os.path.join(data_dir, 'dataset_*.npz')))
    if not paths:
        print('[错误] 在 %s 下没有找到 dataset_*.npz，请先运行：'
              'python3 main.py --mode collect --scenario all' % data_dir)
        return 2

    print('=' * 132)
    print('数据集诊断：%d 个文件，目录 %s' % (len(paths), data_dir))
    print('主要判据：偏差 = 真值 - 雷达，应为半个车身长度（2~4 m），标准差越小越好')
    print('=' * 132)

    results = [analyze(path, detail) for path in paths]

    print('=' * 132)
    print('结论：')
    ok = [r for r in results if r[6] == '[合格]']
    print('  合格工况：%d / %d' % (len(ok), len(results)))
    for name, mean, std, far_off, no_target, _correlation, verdict in results:
        if verdict != '[合格]':
            reasons = []
            if std > 3.0:
                reasons.append('偏差标准差 %.2f m 过大' % std)
            if far_off > 15.0:
                reasons.append('%.1f%% 的帧偏差超过 5 m' % far_off)
            if no_target > 5.0:
                reasons.append('%.1f%% 的帧雷达完全没测到目标' % no_target)
            print('  %s %s：%s' % (verdict, name, '；'.join(reasons) or '偏差偏大'))
    if len(ok) == len(results):
        print('  全部合格，可以执行：python3 main.py --mode train')
    return 0


if __name__ == '__main__':
    sys.exit(main())
