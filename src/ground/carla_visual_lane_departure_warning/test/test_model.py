"""网络层离线测试：形状、参数量、能否学会、存取一致性。"""
import os
import sys
import shutil

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from carla_visual_lane_departure_warning import model as net
from carla_visual_lane_departure_warning import selfcheck

CASES = []


def case(func):
    CASES.append(func)
    return func


@case
def flatten_images_shape():
    """灰度图被压成 (N, 2304) 并归一化到 0~1。"""
    images = np.full((4, net.INPUT_H, net.INPUT_W), 255, dtype=np.uint8)
    features = net.flatten_images(images)
    assert features.shape == (4, net.INPUT_DIM), features.shape
    assert abs(features.max() - 1.0) < 1e-9


@case
def forward_output_shape():
    """前向输出是每个样本两个数：左右边界位置。"""
    regressor = net.LaneRegressor()
    x = net.flatten_images(np.zeros((5, net.INPUT_H, net.INPUT_W), dtype=np.uint8))
    prediction = regressor.predict(x)
    assert prediction.shape == (5, 2), prediction.shape


@case
def parameter_count_is_small():
    """参数量要小到能在 CPU 上训练，又不能小到学不动。"""
    total = net.count_parameters(net.LaneRegressor())
    assert 100000 < total < 500000, '参数量 %d 不在预期范围' % total


@case
def training_reduces_loss():
    """同样的数据上多轮训练，损失必须下降。"""
    images, targets = selfcheck.synthetic_dataset(count=120, seed=1)
    regressor = net.LaneRegressor(seed=0)
    history = regressor.fit(images, targets, epochs=8, verbose=False)
    assert history[-1] < history[0] * 0.5, '损失没降下来：%.4f -> %.4f' % (
        history[0], history[-1])


@case
def learned_model_beats_zero_baseline():
    """训练后的偏移误差要明显小于「全预测 0」。"""
    images, targets = selfcheck.synthetic_dataset(count=200, seed=2)
    regressor = net.LaneRegressor(seed=0)
    regressor.fit(images, targets, epochs=20, verbose=False)
    prediction = regressor.predict(net.flatten_images(images))

    model_error = np.mean(np.abs(-(prediction[:, 0] + prediction[:, 1]) / 2.0
                                 + (targets[:, 0] + targets[:, 1]) / 2.0))
    zero_error = np.mean(np.abs((targets[:, 0] + targets[:, 1]) / 2.0))
    assert model_error < zero_error * 0.5, '模型 %.3f 没明显好于基线 %.3f' % (
        model_error, zero_error)


@case
def save_load_roundtrip():
    """存盘再读回，预测必须逐位相同。"""
    regressor = net.LaneRegressor(seed=3)
    images, _ = selfcheck.synthetic_dataset(count=10, seed=4)
    x = net.flatten_images(images)
    before = regressor.predict(x)

    workdir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           '_tmp_model_%d' % os.getpid())
    os.makedirs(workdir, exist_ok=True)
    path = os.path.join(workdir, 'w.npz')
    try:
        regressor.save(path)
        reloaded = net.LaneRegressor.load(path)
    finally:
        shutil.rmtree(workdir, ignore_errors=True)
    after = reloaded.predict(x)
    assert np.allclose(before, after), '读回的模型预测不一致'
    assert net.count_parameters(reloaded) == net.count_parameters(regressor)


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