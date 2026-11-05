# -*- coding: utf-8 -*-
"""训练感知网络与控制网络。

**本文件不需要 CARLA**，只要 Python + NumPy 就能运行，
因此可以在普通电脑上复现训练过程（数据由 ``main.py --mode collect`` 采集）。

数据来源：``data/dataset_<scenario>.npz``（每种前车工况一个文件）

* 感知网络：雷达特征(6 维) → 车距、相对速度(2 维)
* 控制网络：车距、相对速度、自车速度(3 维) → 油门、刹车(2 维)
"""

import glob
import os

import numpy as np

from .nn_numpy import MLP, Normalizer

PERCEPTION_INPUTS = ('radar_d', 'radar_azimuth', 'radar_closing',
                     'radar_mean_depth', 'radar_n_points', 'v_ego')
PERCEPTION_TARGETS = ('gap_true', 'v_rel_true')
CONTROL_INPUTS = ('gap_true', 'v_rel_true', 'v_ego')
CONTROL_TARGETS = ('label_throttle', 'label_brake')


# ----------------------------------------------------------------------------
# 数据
# ----------------------------------------------------------------------------
def load_datasets(data_dir):
    paths = sorted(glob.glob(os.path.join(data_dir, 'dataset_*.npz')))
    if not paths:
        raise RuntimeError('在 %s 下找不到 dataset_*.npz，请先运行：'
                           'python3 main.py --mode collect --scenario all' % data_dir)
    datasets = []
    for path in paths:
        with np.load(path, allow_pickle=False) as data:
            datasets.append({key: np.array(data[key]) for key in data.files})
    return paths, datasets


def build_samples(datasets):
    """把每个数据集转成 (感知输入, 感知目标, 控制输入, 控制目标)。"""
    samples = []
    for data in datasets:
        samples.append((
            np.stack([data[key] for key in PERCEPTION_INPUTS], axis=1),
            np.stack([data[key] for key in PERCEPTION_TARGETS], axis=1),
            np.stack([data[key] for key in CONTROL_INPUTS], axis=1),
            np.stack([data[key] for key in CONTROL_TARGETS], axis=1),
        ))
    return samples


def split_train_val(samples, val_ratio=0.2):
    """按时间顺序切分：每个数据集的后 ``val_ratio`` 作为验证集。

    注意不能随机打乱后再切分——相邻帧高度相关，随机切分会让验证集"泄题"。
    """
    train_parts, val_parts = [], []
    for perception_x, perception_y, control_x, control_y in samples:
        cut = int(len(perception_x) * (1.0 - float(val_ratio)))
        train_parts.append((perception_x[:cut], perception_y[:cut], control_x[:cut], control_y[:cut]))
        val_parts.append((perception_x[cut:], perception_y[cut:], control_x[cut:], control_y[cut:]))

    def merge(parts, index):
        arrays = [part[index] for part in parts if len(part[index])]
        return np.concatenate(arrays, axis=0) if arrays else np.zeros((0, 1))

    return ([merge(train_parts, i) for i in range(4)],
            [merge(val_parts, i) for i in range(4)])


def describe_samples(samples):
    """打印数据质量统计，用于发现"采集时场景异常"造成的脏数据。

    最关键的判据：**雷达最短回波距离与真实车距的相关系数**。
    正常情况下两者几乎线性相关（相差约半个车身长度），相关系数应 > 0.95；
    如果采集时自车撞墙、跟丢前车或车辆被困在原地，雷达测到的是墙或者根本测不到前车，
    相关系数会显著下降，训练出来的感知网络也不可用。
    """
    radar = np.concatenate([sample[0] for sample in samples], axis=0)
    target = np.concatenate([sample[1] for sample in samples], axis=0)
    gap = target[:, 0]
    v_rel = target[:, 1]
    radar_d = radar[:, 0]

    print('[数据质量] 真实车距     ：最小 %6.1f / 均值 %6.1f / 最大 %6.1f m'
          % (gap.min(), gap.mean(), gap.max()))
    print('[数据质量] 相对速度     ：最小 %+6.1f / 均值 %+6.1f / 最大 %+6.1f m/s'
          % (v_rel.min(), v_rel.mean(), v_rel.max()))
    print('[数据质量] 雷达最短回波 ：最小 %6.1f / 均值 %6.1f / 最大 %6.1f m'
          % (radar_d.min(), radar_d.mean(), radar_d.max()))
    correlation = float(np.corrcoef(radar_d, gap)[0, 1])
    print('[数据质量] 雷达回波与真实车距的相关系数：%.3f（应 > 0.95）' % correlation)
    outliers = int(np.sum(np.abs(gap - radar_d) > 20.0))
    print('[数据质量] 两者相差超过 20 m 的帧：%d（%.1f%%，应接近 0）'
          % (outliers, 100.0 * outliers / len(gap)))

    if correlation < 0.9 or outliers > 0.05 * len(gap) or abs(v_rel).max() > 40.0:
        print('[警告] 数据可能来自异常场景（自车撞墙 / 跟丢前车 / 车辆被困在原地）。')
        print('        请确认 CarlaUE4.exe 已启动，删除旧数据后重新采集：')
        print('        rm -f data/dataset_*.npz && python3 main.py --mode collect --scenario all')
    return correlation


# ----------------------------------------------------------------------------
# 训练
# ----------------------------------------------------------------------------
def filter_frames(samples, max_diff=15.0, max_range=60.0, min_frames=30):
    """按帧剔除明显异常的样本，而不是整包丢掉一个工况。

    判据（三者同时满足才保留）：

    1. ``|真值车距 - 雷达读数| <= max_diff``：雷达跟到远处杂波时该差值会非常大；
    2. 雷达读数与真值车距都不超过 ``max_range``：超出 ACC 有效工作区间的样本不参与训练；
    3. 剔除后该工况至少还剩 ``min_frames`` 帧，否则整个工况丢弃。

    这样既保住每个工况 90% 以上的正常帧，又不会让被杂波污染的帧进入训练集
    （实测这类脏帧会让感知网络的车距 RMSE 从 0.8 m 恶化到 8 m）。
    """
    filtered, dropped, total = [], 0, 0
    for perception_x, perception_y, control_x, control_y in samples:
        gap = perception_y[:, 0]
        radar = perception_x[:, 0]
        mask = ((np.abs(gap - radar) <= float(max_diff))
                & (radar <= float(max_range))
                & (gap <= float(max_range)))
        total += len(mask)
        dropped += int((~mask).sum())
        if int(mask.sum()) >= int(min_frames):
            filtered.append((perception_x[mask], perception_y[mask],
                             control_x[mask], control_y[mask]))
    if total:
        print('[数据质量] 按帧剔除异常样本：%d / %d（%.1f%%），保留 %d 个工况'
              % (dropped, total, 100.0 * dropped / total, len(filtered)))
    return filtered


def train_models(data_dir='data', models_dir='models', epochs=300, lr=0.01,
                 batch_size=64, seed=0, hidden=(24, 16), val_ratio=0.2, verbose=True,
                 max_diff=15.0, max_range=60.0):
    paths, datasets = load_datasets(data_dir)
    print('[数据] 载入 %d 个文件：%s'
          % (len(paths), ', '.join(os.path.basename(p) for p in paths)))

    samples = filter_frames(build_samples(datasets), max_diff, max_range)
    if not samples:
        raise RuntimeError('剔除异常样本后没有可用数据，请先检查采集质量：'
                           'python3 analyze_dataset.py')

    train_set, val_set = split_train_val(samples, val_ratio)
    p_x_train, p_y_train, c_x_train, c_y_train = train_set
    p_x_val, p_y_val, c_x_val, c_y_val = val_set
    print('[数据] 样本总数 %d（训练 %d / 验证 %d）'
          % (len(p_x_train) + len(p_x_val), len(p_x_train), len(p_x_val)))
    describe_samples(samples)

    os.makedirs(models_dir, exist_ok=True)

    # ---------------- 感知网络 ----------------
    print('\n[感知网络] 输入 %s → 输出 %s' % ('+'.join(PERCEPTION_INPUTS),
                                              '+'.join(PERCEPTION_TARGETS)))
    p_in = Normalizer().fit(p_x_train)
    p_out = Normalizer().fit(p_y_train)
    perception = MLP([p_x_train.shape[1]] + list(hidden) + [p_y_train.shape[1]],
                     seed=seed, lr=lr)
    print('  网络结构：%s，参数量 %d'
          % ('-'.join(str(n) for n in perception.layer_sizes),
             sum(w.size for w in perception.W) + sum(b.size for b in perception.b)))
    history_p = perception.fit(p_in.transform(p_x_train), p_out.transform(p_y_train),
                               epochs=epochs, batch_size=batch_size, seed=seed, verbose=verbose)

    prediction = p_out.inverse(perception.predict(p_in.transform(p_x_val)))
    rmse_gap = float(np.sqrt(np.mean((prediction[:, 0] - p_y_val[:, 0]) ** 2)))
    rmse_vrel = float(np.sqrt(np.mean((prediction[:, 1] - p_y_val[:, 1]) ** 2)))
    print('  验证集：车距 RMSE = %.3f m，相对速度 RMSE = %.3f m/s' % (rmse_gap, rmse_vrel))
    perception.save(os.path.join(models_dir, 'perception.npz'), meta={
        'in_mean': p_in.mean, 'in_std': p_in.std,
        'out_mean': p_out.mean, 'out_std': p_out.std,
        'in_min': p_x_train.min(axis=0), 'in_max': p_x_train.max(axis=0),
        'out_min': p_y_train.min(axis=0), 'out_max': p_y_train.max(axis=0),
        'val_rmse_gap': rmse_gap, 'val_rmse_vrel': rmse_vrel,
    })

    # ---------------- 控制网络 ----------------
    print('\n[控制网络] 输入 %s → 输出 %s' % ('+'.join(CONTROL_INPUTS),
                                              '+'.join(CONTROL_TARGETS)))
    c_in = Normalizer().fit(c_x_train)
    c_out = Normalizer().fit(c_y_train)
    control = MLP([c_x_train.shape[1]] + list(hidden) + [c_y_train.shape[1]],
                  seed=seed + 1, lr=lr)
    print('  网络结构：%s'
          % '-'.join(str(n) for n in control.layer_sizes))
    history_c = control.fit(c_in.transform(c_x_train), c_out.transform(c_y_train),
                            epochs=epochs, batch_size=batch_size, seed=seed + 1, verbose=verbose)

    prediction = c_out.inverse(control.predict(c_in.transform(c_x_val)))
    rmse_throttle = float(np.sqrt(np.mean((prediction[:, 0] - c_y_val[:, 0]) ** 2)))
    rmse_brake = float(np.sqrt(np.mean((prediction[:, 1] - c_y_val[:, 1]) ** 2)))
    mse_total = float(np.mean((prediction - c_y_val) ** 2))
    print('  验证集：油门 RMSE = %.4f，刹车 RMSE = %.4f，总 MSE = %.6f'
          % (rmse_throttle, rmse_brake, mse_total))
    control.save(os.path.join(models_dir, 'control.npz'), meta={
        'in_mean': c_in.mean, 'in_std': c_in.std,
        'out_mean': c_out.mean, 'out_std': c_out.std,
        'in_min': c_x_train.min(axis=0), 'in_max': c_x_train.max(axis=0),
        'out_min': c_y_train.min(axis=0), 'out_max': c_y_train.max(axis=0),
        'val_rmse_throttle': rmse_throttle, 'val_rmse_brake': rmse_brake,
        'val_mse': mse_total,
    })

    # ---------------- 训练日志 ----------------
    log_path = os.path.join(data_dir, 'training_log.csv')
    with open(log_path, 'w', encoding='utf-8') as fp:
        fp.write('epoch,perception_mse,control_mse\n')
        for index in range(len(history_p)):
            fp.write('%d,%.8f,%.8f\n' % (index + 1, history_p[index], history_c[index]))
    print('\n[日志] 训练曲线数据已写入 %s' % log_path)

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        figure, axes = plt.subplots(1, 2, figsize=(10, 4))
        axes[0].plot(history_p)
        axes[0].set_title('perception MSE')
        axes[0].set_xlabel('epoch')
        axes[1].plot(history_c)
        axes[1].set_title('control MSE')
        axes[1].set_xlabel('epoch')
        figure.tight_layout()
        curve_path = os.path.join(data_dir, 'training_curve.png')
        figure.savefig(curve_path, dpi=110)
        print('[日志] 训练曲线图已保存：%s' % curve_path)
    except Exception as exc:                      # matplotlib 缺失时不影响训练
        print('[提示] 未生成曲线图（%s），可执行 sudo apt install -y python3-matplotlib' % exc)

    return {'perception_rmse_gap': rmse_gap, 'perception_rmse_vrel': rmse_vrel,
            'control_mse': mse_total}
