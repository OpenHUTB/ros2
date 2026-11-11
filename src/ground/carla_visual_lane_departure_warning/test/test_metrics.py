"""指标层离线测试：误差口径、符号、分档统计。"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from carla_visual_lane_departure_warning import metrics

CASES = []


def case(func):
    CASES.append(func)
    return func


@case
def mae_and_rmse_basic():
    """平均绝对误差和均方根误差的基本定义。"""
    prediction = np.array([1.0, 2.0, 3.0])
    target = np.array([1.0, 2.0, 5.0])
    assert abs(metrics.mae(prediction, target) - 2.0 / 3.0) < 1e-9
    assert abs(metrics.rmse(prediction, target) - np.sqrt(4.0 / 3.0)) < 1e-9


@case
def perfect_prediction_gives_zero():
    """完全预测对时所有误差都是 0。"""
    prediction = np.array([[1.7, -1.7], [0.5, -3.5]])
    result = metrics.evaluate(prediction, prediction.copy())
    assert result['mae_offset'] < 1e-12
    assert result['rmse_offset'] < 1e-12
    assert result['max_offset'] < 1e-12


@case
def offset_uses_midpoint_with_left_positive_sign():
    """偏移 = -(左+右)/2，左偏为正，与仓库 LDW 口径一致。"""
    prediction, true = metrics.offset_error(np.array([[0.0, 4.0]]),
                                            np.array([[0.0, 4.0]]))
    assert abs(prediction[0] + 2.0) < 1e-12

    # 左右各 -1 时，中点右侧 1 m，偏移应为 -1
    predicted_offset, _ = metrics.offset_error(np.array([[2.0, 0.0]]),
                                               np.array([[2.0, 0.0]]))
    assert abs(predicted_offset[0] + 1.0) < 1e-12


@case
def symmetric_error_cancels_in_offset():
    """左右同时外扩时中点不变，偏移误差为 0，但边界误差不为 0。"""
    prediction = np.array([[-2.0, 2.0]])
    target = np.array([[-1.5, 1.5]])
    result = metrics.evaluate(prediction, target)
    assert result['mae_offset'] < 1e-12
    assert result['mae_left'] > 0.4


@case
def by_speed_bins_partition_samples():
    """按车速分档后，各档样本数之和等于总数。"""
    prediction = np.array([[0.0, 4.0], [0.0, 4.0], [0.0, 4.0], [0.0, 4.0]])
    target = np.array([[0.2, 3.8], [0.4, 3.6], [0.1, 3.9], [0.3, 3.7]])
    speeds = np.array([1.0, 5.0, 9.0, 15.0])
    result = metrics.evaluate(prediction, target, speeds=speeds)
    total = sum(item['n'] for item in result['by_speed'].values())
    assert total == 4, '分档样本数之和 %d 不等于 4' % total


@case
def report_contains_key_numbers():
    """生成的文本报告里必须带上样本数和两个核心误差。"""
    prediction = np.array([[0.0, 4.0], [0.5, 3.5]])
    target = np.array([[0.1, 3.9], [0.4, 3.6]])
    text = metrics.format_report(metrics.evaluate(prediction, target))
    assert '样本 2 帧' in text
    assert 'MAE' in text and 'RMSE' in text


def main():
    failures = 0
    for func in CASES:
        try:
            func()
            print('[通过] %s' % func.__name__)
        except AssertionError as error:
            failures += 1
            print('[失败] %s：%s' % (func.__name__, error))
    print('=' * 60)
    if failures:
        print('[结果] %d 项失败，%d 项通过' % (failures, len(CASES) - failures))
        return 1
    print('[结果] %d 项全部通过' % len(CASES))
    return 0


if __name__ == '__main__':
    sys.exit(main())