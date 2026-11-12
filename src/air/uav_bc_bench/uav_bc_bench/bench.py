"""对比实验主流程：结构 × 增强策略 × 随机种子。"""

import argparse
import io
import json
import math
import os
import time

import numpy as np

try:                      # torch 只在真正训练时才需要，_split 之类的纯逻辑要能脱离它导入
    import torch
    from torch import nn
    from torch.utils.data import DataLoader, TensorDataset
except ImportError:       # pragma: no cover - CI 里没有 torch 也要能跑 test_split
    torch = None
    nn = None
    DataLoader = None
    TensorDataset = None

# data 依赖 cv2、models 依赖 torch：都放到函数内导入，
# 让 CI 在没有 opencv/torch 的环境里也能 import `_split` 跑 test_split.py。


def _require_torch():
    """训练前确认 torch 可用，给出比 ImportError 更清楚的提示。"""
    if torch is None:
        raise RuntimeError('未安装 PyTorch：请先 `sudo apt install python3-torch` '
                           '或 `pip install torch --index-url ...`（CPU 版即可）')


def _augment_batch(batch, rng):
    """对一批张量做随机平移 + 亮度扰动 + 高斯噪声。

    在训练循环里逐轮调用（而不是训练前做一次固定变换），
    这样每个 epoch 看到的样本都不一样，与文档描述的"按轮随机增强"一致。
    """
    count = batch.shape[0]
    side = batch.shape[-1]
    max_shift = max(1, side // 8)
    out = batch.clone()
    for i in range(count):
        dx = int(rng.integers(-max_shift, max_shift + 1))
        dy = int(rng.integers(-max_shift, max_shift + 1))
        if dx or dy:
            out[i] = torch.roll(out[i], shifts=(dy, dx), dims=(1, 2))
    gain = torch.from_numpy(
        rng.uniform(0.8, 1.2, size=(count, 1, 1, 1)).astype('float32'))
    out = torch.clamp(out * gain, 0.0, 1.0)
    noise = torch.from_numpy(rng.normal(0.0, 0.01, out.shape).astype('float32'))
    return torch.clamp(out + noise, 0.0, 1.0)


def lap_groups(dataset_dir):
    """按 `controls.csv` 里的 x/y 给每帧打圈号；没有位置列时返回 None。

    圆轨迹跑两圈时，第 1 圈和第 2 圈经过的是同一片空间。若按时间切成
    "前 80% / 中 10% / 后 10%"，训练段就覆盖了测试段要走的几乎整圈，
    测试指标会被空间重叠抬高。按圈分组可以让测试集落在没见过的圈上。
    """
    import csv

    path = os.path.join(dataset_dir, 'controls.csv')
    if not os.path.exists(path):
        return None
    xs, ys = [], []
    with io.open(path, encoding='utf-8') as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                xs.append(float(row['x']))
                ys.append(float(row['y']))
            except (KeyError, TypeError, ValueError):
                return None
    if len(xs) < 10:
        return None
    cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
    thetas = [math.atan2(y - cy, x - cx) for x, y in zip(xs, ys)]
    travel, last = 0.0, None
    laps = []
    for theta in thetas:
        if last is not None:
            delta = math.atan2(
                math.sin(theta - last),
                math.cos(theta - last))
            travel += abs(delta)
        last = theta
        laps.append(int(travel / (2.0 * math.pi)))
    if laps[-1] < 1:
        return None
    return laps


def _split(n, seed, groups=None):
    """切分训练/验证/测试。

    优先按圈分组（最后一圈做测试、倒数第二圈做验证，其余训练）；
    拿不到位置信息时退回"按时间连续切分"，同样不做随机打乱。
    """
    if groups is not None:
        uniq = sorted(set(groups))
        if len(uniq) >= 3:
            test_laps = {uniq[-1]}
            val_laps = {uniq[-2]}
            train_laps = set(uniq[:-2])
            train_idx = np.array([i for i, g in enumerate(groups) if g in train_laps])
            val_idx = np.array([i for i, g in enumerate(groups) if g in val_laps])
            test_idx = np.array([i for i, g in enumerate(groups) if g in test_laps])
            if len(train_idx) >= 2 and len(val_idx) >= 1 and len(test_idx) >= 1:
                return train_idx, val_idx, test_idx, 'lap'
        # 圈号不足 3 个：静默退回按时间切分，调用方据返回的模式记录

    n_val = max(1, int(round(n * 0.1)))
    n_test = max(1, int(round(n * 0.1)))
    n_train = n - n_val - n_test
    if n_train < 2:
        raise ValueError(f'样本数 {n} 太少，无法切成 8:1:1')
    jitter = 0 if seed == 0 else int(np.random.default_rng(seed).integers(0, max(1, n_val)))
    a = max(2, n_train - jitter)
    b = min(n - n_test, a + n_val + (jitter // 2))
    return np.arange(0, a), np.arange(a, b), np.arange(b, n), 'time'


def _evaluate(model, loader, loss_fn):
    model.eval()
    total, count, abs_sum = 0.0, 0, 0.0
    with torch.no_grad():
        for xb, yb in loader:
            pred = model(xb)
            total += float(loss_fn(pred, yb)) * len(xb)
            abs_sum += float((pred - yb).abs().sum())
            count += len(xb)
    return total / max(1, count), abs_sum / max(1, 2 * count)


def train_one(structure, use_augment, seed, images, targets, epochs, image_size,
              batch_size, groups=None):
    """训练并评估一个配置，返回指标字典。"""
    torch.manual_seed(seed)
    train_idx, val_idx, test_idx, _mode = _split(len(images), seed, groups)

    def loader(idx, shuffle):
        ds = TensorDataset(torch.from_numpy(images[idx]),
                           torch.from_numpy(targets[idx]))
        return DataLoader(ds, batch_size=batch_size, shuffle=shuffle)

    train_loader = loader(train_idx, True)
    val_loader = loader(val_idx, False)
    test_loader = loader(test_idx, False)

    from uav_bc_bench.models import build, num_parameters
    model = build(structure, image_size=image_size)
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
    loss_fn = nn.MSELoss()
    rng = np.random.default_rng(seed)

    started = time.time()
    best_val, best_state = float('inf'), None
    for _ in range(epochs):
        model.train()
        for xb, yb in train_loader:
            if use_augment:
                xb = _augment_batch(xb, rng)
            optimizer.zero_grad()
            loss = loss_fn(model(xb), yb)
            loss.backward()
            optimizer.step()
        val_mse, _ = _evaluate(model, val_loader, loss_fn)
        if val_mse < best_val:
            best_val = val_mse
            best_state = {k: v.detach().clone() for k, v in model.state_dict().items()}

    if best_state is not None:
        model.load_state_dict(best_state)
    test_mse, test_mae = _evaluate(model, test_loader, loss_fn)
    # 恒零基线：一直输出 (0, 0) 的 MAE，用来判断"网络是否真的学到了东西"。
    zero_mae = float(np.abs(targets[test_idx]).mean())

    return {
        'structure': structure,
        'augment': bool(use_augment),
        'seed': seed,
        'params': num_parameters(model),
        'train_seconds': round(time.time() - started, 1),
        'split': {'train': int(len(train_idx)), 'val': int(len(val_idx)),
                  'test': int(len(test_idx))},
        'zero_baseline_mae': round(zero_mae, 4),
        'val_mse': round(best_val, 4),
        'test_mse': round(test_mse, 4),
        'test_mae': round(test_mae, 4),
    }


def _font(size):
    """优先用 DejaVu/文泉驿字体，找不到就退回 PIL 默认字体。"""
    from PIL import ImageFont

    for path in ('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
                 '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
                 'C:/Windows/Fonts/arial.ttf'):
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default()


def _draw_chart(rows, path):
    """用 PIL 画柱状图，避免额外引入 matplotlib。

    没装 Pillow 时返回 False，由调用方降级（指标照样写 json / md），
    不让画图这个附属功能把整轮实验弄崩。
    """
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return False

    width, height = 900, 480
    im = Image.new('RGB', (width, height), (255, 255, 255))
    draw = ImageDraw.Draw(im)
    title_font = _font(18)
    label_font = _font(13)
    draw.text((28, 16), 'Behaviour cloning benchmark: test MAE (m/s), lower is better',
              fill=(20, 20, 20), font=title_font)

    labels = sorted({(r['structure'], r['augment']) for r in rows})
    values = []
    for label in labels:
        vals = [r['test_mae'] for r in rows if (r['structure'], r['augment']) == label]
        values.append(sum(vals) / len(vals))

    top = max(values) if values else 1.0
    left, bottom, plot_top = 70, height - 90, 80
    slot = (width - left - 40) // max(1, len(labels))
    bar_w = int(slot * 0.55)
    for i, (label, value) in enumerate(zip(labels, values)):
        base_x = left + i * slot + (slot - bar_w) // 2
        h = int((bottom - plot_top) * value / top) if top else 0
        color = (70, 120, 200) if not label[1] else (200, 130, 70)
        draw.rectangle([base_x, bottom - h, base_x + bar_w, bottom], fill=color)
        text = '%.3f' % value
        tw = draw.textlength(text, font=label_font)
        draw.text((base_x + (bar_w - tw) / 2, bottom - h - 20), text, fill=(20, 20, 20), font=label_font)
        name = label[0] + ('\n+aug' if label[1] else '')
        for j, part in enumerate(name.split('\n')):
            pw = draw.textlength(part, font=label_font)
            draw.text((base_x + (bar_w - pw) / 2, bottom + 10 + j * 16), part,
                      fill=(40, 40, 40), font=label_font)

    draw.line([left - 12, bottom, width - 24, bottom], fill=(120, 120, 120))
    draw.rectangle([width - 190, 16, width - 178, 28], fill=(70, 120, 200))
    draw.text((width - 172, 14), 'without augmentation', fill=(40, 40, 40), font=label_font)
    draw.rectangle([width - 190, 38, width - 178, 50], fill=(200, 130, 70))
    draw.text((width - 172, 36), 'with augmentation', fill=(40, 40, 40), font=label_font)
    im.save(path)
    return True


def run_benchmark(dataset_dir, output_dir, epochs=30, seeds=3, structures=None,
                  augment=True, image_size=64, batch_size=32, synthetic=0, log=print):
    """跑完整对比实验，写出 metrics.json / comparison.md / comparison.png。

    `synthetic > 0` 时先现场生成一份合成数据集（不依赖其它模块），
    这样没有真实飞行数据也能把整条流水线跑通。
    """
    _require_torch()
    structures = structures or ['mlp', 'cnn', 'cnn_wide']
    os.makedirs(output_dir, exist_ok=True)

    if synthetic and synthetic > 0:
        from uav_bc_bench.data import synthesize_dataset
        dataset_dir = synthesize_dataset(os.path.join(output_dir, 'synthetic'),
                                         samples=int(synthetic),
                                         image_size=image_size, seed=0)
        log(f'使用合成数据集：{dataset_dir}（{synthetic} 个样本）')

    if not os.path.isdir(dataset_dir) or not os.path.isfile(os.path.join(dataset_dir, 'controls.csv')):
        raise FileNotFoundError(
            '数据集目录不可用：' + str(dataset_dir) + '。'
            '  要么先采集数据（见 uav_depth_track_nn 的 collect），'
            '要么用 synthetic:=200 现场生成一份合成数据集。')

    from uav_bc_bench.data import load_dataset
    images, targets = load_dataset(dataset_dir, image_size=image_size)
    groups = lap_groups(dataset_dir)
    split_mode = 'time'
    if groups is not None and len(set(groups)) >= 3:
        split_mode = 'lap'
    log(f'数据集 {dataset_dir}：{len(images)} 个样本，图像 {images.shape[1:]}，'
        f'切分方式 {split_mode}（圈号 {0 if groups is None else len(set(groups))} 个）')

    rows = []
    for structure in structures:
        for use_augment in ([False, True] if augment else [False]):
            for seed in range(seeds):
                row = train_one(structure, use_augment, seed, images, targets,
                                epochs, image_size, batch_size, groups)
                rows.append(row)
                log('  %-8s augment=%-5s seed=%d  val_mse=%.4f  test_mse=%.4f  test_mae=%.4f'
                    % (structure, use_augment, seed, row['val_mse'], row['test_mse'], row['test_mae']))

    summary = []
    for structure in structures:
        for use_augment in ([False, True] if augment else [False]):
            group = [r for r in rows if r['structure'] == structure and r['augment'] == use_augment]
            maes = [r['test_mae'] for r in group]
            summary.append({
                'structure': structure,
                'augment': bool(use_augment),
                'params': group[0]['params'],
                'test_mae_mean': round(float(np.mean(maes)), 4),
                'test_mae_std': round(float(np.std(maes)), 4),
                'train_seconds': round(float(np.mean([r['train_seconds'] for r in group])), 1),
            })

    summary.sort(key=lambda r: r['test_mae_mean'])
    zero_baseline = round(float(np.mean([r['zero_baseline_mae'] for r in rows])), 4)
    with open(os.path.join(output_dir, 'metrics.json'), 'w', encoding='utf-8') as fh:
        json.dump({'samples': int(len(images)), 'epochs': epochs, 'seeds': seeds,
                   'image_size': image_size, 'batch_size': batch_size,
                   'split_mode': split_mode, 'zero_baseline_mae': zero_baseline,
                   'dataset_dir': dataset_dir,
                   'runs': rows, 'summary': summary}, fh, ensure_ascii=False, indent=2)

    lines = ['| 结构 | 图像增强 | 参数量 | 测试 MAE (m/s) | 训练耗时 (s) |',
             '| --- | --- | --- | --- | --- |']
    for row in summary:
        lines.append('| %s | %s | %d | %.4f ± %.4f | %.1f |' % (
            row['structure'], '是' if row['augment'] else '否', row['params'],
            row['test_mae_mean'], row['test_mae_std'], row['train_seconds']))
    table = '\n'.join(lines)
    with open(os.path.join(output_dir, 'comparison.md'), 'w', encoding='utf-8') as fh:
        fh.write(table + '\n')

    if _draw_chart(rows, os.path.join(output_dir, 'comparison.png')):
        log(f'柱状图已写到 {os.path.join(output_dir, "comparison.png")}')
    else:
        log('未安装 Pillow，跳过 comparison.png（指标已写入 metrics.json / comparison.md）')
    log('最优配置：%s（MAE %.4f）' % (summary[0]['structure'], summary[0]['test_mae_mean']))
    return {'rows': rows, 'summary': summary, 'table': table}


def main(argv=None):
    """命令行入口，方便脱离 ROS 直接跑。"""
    parser = argparse.ArgumentParser(description='行为克隆模型对比基准')
    parser.add_argument('--dataset-dir', default=os.path.expanduser('~/.ros/uav_depth_track_nn/data'))
    parser.add_argument('--output-dir', default=os.path.expanduser('~/.ros/uav_bc_bench'))
    parser.add_argument('--epochs', type=int, default=30)
    parser.add_argument('--seeds', type=int, default=3)
    parser.add_argument('--batch-size', type=int, default=32)
    parser.add_argument('--structures', default='mlp,cnn,cnn_wide')
    parser.add_argument('--no-augment', action='store_true')
    parser.add_argument('--image-size', type=int, default=64)
    parser.add_argument('--synthetic', type=int, default=0,
                        help='大于 0 时先在该目录生成等量合成样本，不依赖任何仿真数据')
    args = parser.parse_args(argv)
    args.dataset_dir = os.path.expanduser(args.dataset_dir)
    args.output_dir = os.path.expanduser(args.output_dir)
    run_benchmark(args.dataset_dir, args.output_dir, epochs=args.epochs, seeds=args.seeds,
                  structures=[s for s in args.structures.split(',') if s],
                  augment=not args.no_augment, batch_size=args.batch_size,
                  image_size=args.image_size, synthetic=args.synthetic,
                  log=lambda m: print('[bench] ' + str(m)))


if __name__ == '__main__':
    main()
