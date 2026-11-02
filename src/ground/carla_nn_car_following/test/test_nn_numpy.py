#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线单元测试：纯 NumPy 的 MLP、标准化器，以及"合成数据 → 训练 → 保存/载入 → 推理"全流程。

不需要 CARLA、不需要 ROS、不需要 PyTorch，只要 Python + NumPy。

运行方式::

    cd src/ground/carla_nn_car_following
    python3 test/test_nn_numpy.py
"""

import os
import shutil
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from carla_nn_car_following.nn_numpy import MLP, Normalizer        # noqa: E402
from carla_nn_car_following.train import train_models              # noqa: E402
from carla_nn_car_following.nn_controller import NNController      # noqa: E402
from carla_nn_car_following.reference_acc import ReferenceACC      # noqa: E402

PASSED = 0


def check(condition, message):
    global PASSED
    assert condition, '未通过：' + message
    PASSED += 1
    print('  [通过] %s' % message)


def make_synthetic_dataset(path, samples=600, seed=7):
    """合成一份与采集格式完全一致的 npz 数据（用经典 ACC 生成控制标签）。"""
    rng = np.random.default_rng(seed)
    acc = ReferenceACC()

    gap = rng.uniform(8.0, 30.0, samples)
    v_ego = rng.uniform(0.0, 17.0, samples)
    v_rel = rng.uniform(-5.0, 5.0, samples)
    throttle = np.zeros(samples)
    brake = np.zeros(samples)
    for index in range(samples):
        throttle[index], brake[index] = acc.labels(gap[index], v_rel[index], v_ego[index])

    # 雷达看到的是前车车尾，比"车距真值"近约 2.3 m；再加少量测量噪声
    radar_d = gap - 2.3 + rng.normal(0.0, 0.15, samples)
    radar_closing = -v_rel + rng.normal(0.0, 0.1, samples)
    radar_mean_depth = radar_d + rng.uniform(0.5, 3.0, samples)
    radar_n_points = rng.integers(20, 200, samples).astype(float)

    np.savez(
        path,
        scenario='synthetic',
        columns=np.array(['t']),
        t=np.arange(samples) * 0.05,
        radar_d=radar_d,
        radar_azimuth=rng.normal(0.0, 0.01, samples),
        radar_closing=radar_closing,
        radar_mean_depth=radar_mean_depth,
        radar_n_points=radar_n_points,
        gap_true=gap,
        v_rel_true=v_rel,
        v_ego=v_ego,
        label_throttle=throttle,
        label_brake=brake,
        follow_state=np.zeros(samples, dtype=int),
    )


def main():
    print('1) 前向传播')
    model = MLP([6, 8, 2], seed=0)
    features = np.random.default_rng(0).normal(size=(20, 6))
    output = model.predict(features)
    check(output.shape == (20, 2), '前向输出形状为 (20, 2)')
    check(np.isfinite(output).all(), '前向输出无 NaN/Inf')

    print('2) 标准化器')
    normalizer = Normalizer().fit(features)
    check(np.allclose(normalizer.inverse(normalizer.transform(features)), features, atol=1e-8),
          '标准化与反标准化往返一致')

    print('3) 训练能力（拟合线性关系）')
    rng = np.random.default_rng(1)
    data_x = rng.normal(size=(400, 3))
    true_w = np.array([[1.0, -0.5], [0.3, 0.8], [-0.2, 0.6]])
    data_y = data_x @ true_w + 0.05 * rng.normal(size=(400, 2))
    model = MLP([3, 16, 2], seed=2, lr=0.02)
    history = model.fit(data_x, data_y, epochs=200, batch_size=32, verbose=False)
    check(history[-1] < history[0] * 0.2, 'MSE 下降到初值的 20% 以下')
    check(history[-1] < 0.02, '最终 MSE < 0.02')

    print('4) 保存 / 载入')
    # 自测产生的临时文件统一放在 build/nn_selftest/ 下，结束时删除文件、保留目录
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    tmpdir = os.path.join(package_dir, 'build', 'nn_selftest')
    os.makedirs(tmpdir, exist_ok=True)
    try:
        path = os.path.join(tmpdir, 'm.npz')
        model.save(path, meta={'tag': np.array([1.5, 2.5])})
        loaded, meta = MLP.load(path)
        check(np.allclose(model.predict(data_x), loaded.predict(data_x), atol=1e-10),
              '保存载入后预测结果完全一致')
        check(np.allclose(meta['tag'], [1.5, 2.5]), '元数据（标准化参数）一并保存')
        check(loaded.layer_sizes == model.layer_sizes, '网络结构一并保存')

        print('5) 端到端：合成数据 → 训练两个网络 → 载入控制器')
        data_dir = os.path.join(tmpdir, 'data')
        models_dir = os.path.join(tmpdir, 'models')
        os.makedirs(data_dir, exist_ok=True)
        os.makedirs(models_dir, exist_ok=True)
        make_synthetic_dataset(os.path.join(data_dir, 'dataset_synthetic.npz'))

        metrics = train_models(data_dir=data_dir, models_dir=models_dir,
                               epochs=120, lr=0.02, batch_size=32, verbose=False)
        check(os.path.isfile(os.path.join(models_dir, 'perception.npz')), '感知网络已保存')
        check(os.path.isfile(os.path.join(models_dir, 'control.npz')), '控制网络已保存')
        check(metrics['perception_rmse_gap'] < 1.5,
              '感知网络车距 RMSE %.3f m < 1.5 m' % metrics['perception_rmse_gap'])
        check(metrics['perception_rmse_vrel'] < 0.5,
              '感知网络相对速度 RMSE %.3f m/s < 0.5 m/s' % metrics['perception_rmse_vrel'])
        check(metrics['control_mse'] < 0.01,
              '控制网络验证 MSE %.5f < 0.01' % metrics['control_mse'])

        controller = NNController.load(models_dir)
        throttle, brake, gap_hat, v_rel_hat = controller.labels(
            {'d_radar': 22.0, 'azimuth': 0.0, 'closing_rate': 3.0,
             'mean_depth': 23.0, 'n_points': 120.0}, 10.0)
        check(0.0 <= throttle <= 1.0 and 0.0 <= brake <= 1.0,
              '控制器输出油门/刹车均在 [0, 1] 内')
        check(throttle == 0.0 or brake == 0.0, '油门与刹车互斥（不同时踩下）')
        check(abs(gap_hat - 24.0) < 2.0,
              '车距估计 %.2f m 与"雷达 22 m + 车尾偏移 2.3 m"相符' % gap_hat)
        check(abs(v_rel_hat + 3.0) < 1.5,
              '相对速度估计 %.2f m/s 与接近速度 3 m/s 相符' % v_rel_hat)
    finally:
        # 删除自测产生的文件（保留目录，方便重复运行）
        for root, _dirs, files in os.walk(tmpdir):
            for name in files:
                if name == '.keep':
                    continue
                try:
                    os.remove(os.path.join(root, name))
                except OSError:
                    pass

    print('\n[结果] %d 项全部通过' % PASSED)
    return 0


if __name__ == '__main__':
    sys.exit(main())
