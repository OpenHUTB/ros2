"""对比实验主流程：结构 × 增强策略 × 随机种子。"""

import argparse
import json
import os
import time

import cv2
import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, TensorDataset

from uav_bc_bench.data import load_dataset
from uav_bc_bench.models import build, num_parameters


def _augment(images, rng):
    """随机裁剪 + 亮度扰动 + 高斯噪声，只在训练集上使用。"""
    out = np.empty_like(images)
    for i, img in enumerate(images):
        h, w = img.shape[1:]
        crop = int(rng.integers(0, 8))
        if crop > 0:
            img = img[:, crop:h - crop, crop:w - crop]
            img = cv2.resize(img[0], (w, h), interpolation=cv2.INTER_LINEAR)[None]
        gain = float(rng.uniform(0.8, 1.2))
        img = np.clip(img * gain, 0.0, 1.0)
        img = np.clip(img + rng.normal(0.0, 0.01, img.shape).astype(np.float32), 0.0, 1.0)
        out[i] = img
    return out


def _split(n, seed):
    """按时间顺序切成连续三段。

    连续飞行的相邻帧高度相关，随机打乱会让邻帧同时落进训练集和测试集，
    评估结果被时间泄漏抬高。这里 train/val/test = 前 80%/中 10%/后 10%，
    每段的起始点再按 seed 做一点抖动，避免固定切点带来的偶然性。
    """
    n_val = max(1, int(round(n * 0.1)))
    n_test = max(1, int(round(n * 0.1)))
    n_train = n - n_val - n_test
    if n_train < 2:
        raise ValueError(f'样本数 {n} 太少，无法切成 8:1:1')
    jitter = 0 if seed == 0 else int(np.random.default_rng(seed).integers(0, max(1, n_val)))
    a = max(2, n_train - jitter)
    b = min(n - n_test, a + n_val + (jitter // 2))
    return (np.arange(0, a), np.arange(a, b), np.arange(b, n))


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


def train_one(structure, use_augment, seed, images, targets, epochs, image_size, batch_size):
    """训练并评估一个配置，返回指标字典。"""
    torch.manual_seed(seed)
    train_idx, val_idx, test_idx = _split(len(images), seed)

    x_train = images[train_idx]
    if use_augment:
        rng = np.random.default_rng(seed)
        x_train = _augment(x_train, rng)

    def loader(idx, xs, shuffle):
        ds = TensorDataset(torch.from_numpy(xs), torch.from_numpy(targets[idx]))
        return DataLoader(ds, batch_size=batch_size, shuffle=shuffle)

    train_loader = loader(train_idx, x_train, True)
    val_loader = loader(val_idx, images[val_idx], False)
    test_loader = loader(test_idx, images[test_idx], False)

    model = build(structure, image_size=image_size)
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
    loss_fn = nn.MSELoss()

    started = time.time()
    best_val, best_state = float('inf'), None
    for _ in range(epochs):
        model.train()
        for xb, yb in train_loader:
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

    return {
        'structure': structure,
        'augment': bool(use_augment),
        'seed': seed,
        'params': num_parameters(model),
        'train_seconds': round(time.time() - started, 1),
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
    """用 PIL 画柱状图，避免额外引入 matplotlib。"""
    from PIL import Image, ImageDraw

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


def run_benchmark(dataset_dir, output_dir, epochs=30, seeds=3, structures=None,
                  augment=True, image_size=64, batch_size=32, log=print):
    """跑完整对比实验，写出 metrics.json / comparison.md / comparison.png。"""
    structures = structures or ['mlp', 'cnn', 'cnn_wide']
    os.makedirs(output_dir, exist_ok=True)

    images, targets = load_dataset(dataset_dir, image_size=image_size)
    log(f'数据集 {dataset_dir}：{len(images)} 个样本，图像 {images.shape[1:]}')

    rows = []
    for structure in structures:
        for use_augment in ([False, True] if augment else [False]):
            for seed in range(seeds):
                row = train_one(structure, use_augment, seed, images, targets,
                                epochs, image_size, batch_size)
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
    with open(os.path.join(output_dir, 'metrics.json'), 'w', encoding='utf-8') as fh:
        json.dump({'samples': int(len(images)), 'epochs': epochs, 'seeds': seeds,
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

    _draw_chart(rows, os.path.join(output_dir, 'comparison.png'))
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
    dataset_dir = args.dataset_dir
    if args.synthetic > 0:
        from .data import synthesize_dataset
        dataset_dir = synthesize_dataset(os.path.join(args.output_dir, 'synthetic'),
                                         samples=args.synthetic, seed=0)
        print(f'[bench] 使用合成数据集 {dataset_dir}')
    run_benchmark(dataset_dir, args.output_dir, epochs=args.epochs, seeds=args.seeds,
                  structures=[s for s in args.structures.split(',') if s],
                  augment=not args.no_augment, batch_size=args.batch_size,
                  image_size=args.image_size)


if __name__ == '__main__':
    main()
