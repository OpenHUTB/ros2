"""评价指标：视觉估计的横向偏移与地图真值之间的误差。

全部是纯函数，输入数组输出字典，方便离线测试与文档引用。
"""
import math

import numpy as np


def mae(prediction, target):
    return float(np.mean(np.abs(np.asarray(prediction) - np.asarray(target))))


def rmse(prediction, target):
    diff = np.asarray(prediction) - np.asarray(target)
    return float(np.sqrt(np.mean(diff ** 2)))


def offset_error(prediction, target):
    """左右中点给出的横向偏移误差，和 LDW 的判定量直接对应。"""
    predicted_offset = -(prediction[:, 0] + prediction[:, 1]) / 2.0
    true_offset = -(target[:, 0] + target[:, 1]) / 2.0
    return predicted_offset, true_offset


def evaluate(prediction, target, speeds=None):
    """一次给出全部指标。prediction/target 都是 (N, 2)。"""
    prediction = np.asarray(prediction, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    predicted_offset, true_offset = offset_error(prediction, target)
    error = np.abs(predicted_offset - true_offset)

    result = {
        'n': int(prediction.shape[0]),
        'mae_left': mae(prediction[:, 0], target[:, 0]),
        'mae_right': mae(prediction[:, 1], target[:, 1]),
        'mae_offset': float(error.mean()),
        'rmse_offset': rmse(predicted_offset, true_offset),
        'p50_offset': float(np.percentile(error, 50)),
        'p90_offset': float(np.percentile(error, 90)),
        'max_offset': float(error.max()),
    }
    if speeds is not None:
        result['by_speed'] = _by_bins(np.asarray(speeds), error)
    return result


def by_abs_offset(true_offset, diff, edges=(0.3, 0.6, 0.9, 1.2, 1.6)):
    """按真值偏移大小分档，小偏移看精度，大偏移看能不能报出来。

    以前 --max-offset 默认 0.8，比 depart_enter 还小，超过它的帧直接被算成
    skipped，统计里只剩好做的样本。现在默认放宽到 1.6，同时按 |e| 分档把
    各段的真实误差摆出来。
    """
    values = np.abs(np.asarray(true_offset, dtype=np.float64))
    diff = np.asarray(diff, dtype=np.float64)
    buckets = {}
    for index in range(len(edges) + 1):
        low = 0.0 if index == 0 else edges[index - 1]
        high = float('inf') if index == len(edges) else edges[index]
        mask = (values >= low) & (values < high)
        if mask.sum() == 0:
            continue
        label = '>=%g m' % low if math.isinf(high) else '%g-%g m' % (low, high)
        buckets[label] = {
            'n': int(mask.sum()),
            'mae_offset': float(np.abs(diff[mask]).mean()),
            'rmse_offset': float(np.sqrt(np.mean(diff[mask] ** 2))),
        }
    return buckets


def evaluate_offsets(predicted_offset, true_offset, speeds=None):
    """直接拿两条偏移序列算误差；真值是垂距时用这个。"""
    predicted = np.asarray(predicted_offset, dtype=np.float64)
    truth = np.asarray(true_offset, dtype=np.float64)
    diff = predicted - truth
    error = np.abs(diff)
    result = {
        'n': int(truth.size),
        'mae_offset': float(error.mean()),
        'rmse_offset': float(np.sqrt(np.mean(diff ** 2))),
        'p50_offset': float(np.percentile(error, 50)),
        'p90_offset': float(np.percentile(error, 90)),
        'max_offset': float(error.max()),
        'by_abs_offset': by_abs_offset(truth, diff),
    }
    if speeds is not None:
        result['by_speed'] = _by_bins(np.asarray(speeds), error)
    return result


def _by_bins(values, error, edges=(0.0, 4.0, 8.0, 12.0, float('inf'))):
    """按车速分档，看高速时误差是否明显变大。"""
    buckets = {}
    for index in range(len(edges) - 1):
        low, high = edges[index], edges[index + 1]
        mask = (values >= low) & (values < high)
        if mask.sum() == 0:
            continue
        if math.isinf(high):
            label = '>=%g m/s' % low
        else:
            label = '%g-%g m/s' % (low, high)
        buckets[label] = {
            'n': int(mask.sum()),
            'mae_offset': float(error[mask].mean()),
        }
    return buckets


def format_report(result):
    """把指标整理成能直接贴进文档的几行文本。"""
    lines = ['样本 %d 帧' % result['n']]
    if 'mae_left' in result:
        lines.append('左边界 MAE %.3f m，右边界 MAE %.3f m'
                     % (result['mae_left'], result['mae_right']))
    lines += [
        '横向偏移 MAE %.3f m，RMSE %.3f m' % (result['mae_offset'],
                                              result['rmse_offset']),
        '偏移误差 p50 %.3f m / p90 %.3f m / 最大 %.3f m'
        % (result['p50_offset'], result['p90_offset'], result['max_offset']),
    ]
    for name, value in result.get('by_speed', {}).items():
        lines.append('  %-10s %4d 帧  MAE %.3f m'
                     % (name, value['n'], value['mae_offset']))
    for name, value in result.get('by_abs_offset', {}).items():
        lines.append('  |e| %-9s %3d 帧  MAE %.3f m  RMSE %.3f m'
                     % (name, value['n'], value['mae_offset'],
                        value['rmse_offset']))
    return '\n'.join(lines)