"""
open_set.py —— 开集声呐分类：在礁石 / 管道 / 残骸之外增加"背景"类，并按置信度拒识。

为什么需要：
  原三分类实现要求每一帧必须三选一。真实巡检中绝大多数画面没有目标，
  这样的模型会把空画面判成某类目标（误报）。本模块增加第 4 类"背景"，
  并在置信度低于阈值时输出 ``unknown``，把"我不确定"作为合法的输出。

与原实现的两个差别（都是为了让指标可信）：
  1. **训练/测试用互不相同的随机种子**。原实现两处都从 ``RandomState(42)`` 起，
     测试集实际上是训练集的前 200 张，1.000 的准确率来自数据泄漏；
  2. **目标位置逐样本随机**。原实现三类亮斑几乎固定在相同像素上，
     CNN 只要看"哪个固定位置最亮"就能分类，学不到纹理。

数据集、拒识判定、指标计算都只依赖 numpy，只有训练/推理需要 torch，
因此上述逻辑可以在没有 PyTorch 的机器上单元测试。
"""
import json

import numpy as np

from . import common

CLASS_NAMES = ['rock', 'pipe', 'wreck', 'background']
BACKGROUND = 3
NUM_CLASSES = 4

STRONG = {'rock': 0.85, 'pipe': 0.90, 'wreck': 0.70}
WEAK = 0.14


def random_targets(rng):
    """在 10m 水池内随机摆放目标，使 CNN 无法依赖固定像素位置。"""
    def position():
        return [float(rng.uniform(-4.0, 4.0)), float(rng.uniform(-4.0, 4.0))]

    return [position(), position()], [position()], [position()]


def add_clutter(image, rng, count, size=64):
    """叠加不属于任何类别的随机亮斑（假目标）。

    没有杂波时"背景"与"有目标"只差一个整体亮度阈值，任务是平凡的；
    加入杂波后，模型必须区分"随机亮斑"与"成形的目标回波"。
    """
    if count <= 0:
        return image
    out = image.astype(np.float32)
    for _ in range(count):
        cx, cy = int(rng.randint(0, size)), int(rng.randint(0, size))
        sigma = float(rng.uniform(1.0, 4.0))
        magnitude = float(rng.uniform(0.20, 0.50)) * 255.0
        reach = int(np.ceil(sigma * 2))
        for dy in range(-reach, reach + 1):
            for dx in range(-reach, reach + 1):
                x, y = cx + dx, cy + dy
                if 0 <= x < size and 0 <= y < size:
                    out[y, x] += magnitude * np.exp(
                        -(dx * dx + dy * dy) / (2 * sigma * sigma))
    return np.clip(out, 0, 255).astype(np.uint8)


def make_sample(rng, label, size=64, clutter=0):
    """按类别标签合成一张声呐图：只有该类目标回波强，其余为弱干扰。"""
    if not 0 <= label < NUM_CLASSES:
        raise ValueError(f'label must be in [0, {NUM_CLASSES}), got {label}')

    rock, pipe, wreck = random_targets(rng)
    if label == BACKGROUND:
        rock_i = pipe_i = wreck_i = WEAK
    elif label == 0:
        rock_i, pipe_i, wreck_i = STRONG['rock'], WEAK, WEAK * 0.8
    elif label == 1:
        rock_i, pipe_i, wreck_i = WEAK, STRONG['pipe'], WEAK * 0.8
    else:
        rock_i, pipe_i, wreck_i = WEAK, WEAK, STRONG['wreck']

    image = common.generate_sonar_image(
        None, rock, pipe, wreck, size=size,
        rock_intensity=rock_i, pipe_intensity=pipe_i, wreck_intensity=wreck_i,
        rng=rng)
    if clutter:
        image = add_clutter(image, rng, clutter, size=size)
    return image


def build_dataset(n_samples, seed, size=64, clutter=0):
    """生成均衡的 ``(images, labels)``；类别按顺序轮转后打乱，避免与索引相关。"""
    if n_samples <= 0:
        raise ValueError('n_samples must be positive')
    rng = np.random.RandomState(seed)
    labels = np.arange(n_samples) % NUM_CLASSES
    rng.shuffle(labels)

    images = np.empty((n_samples, size, size), dtype=np.uint8)
    for index, label in enumerate(labels):
        images[index] = make_sample(rng, int(label), size=size, clutter=clutter)
    return images, labels


def reject_from_probs(probs, threshold=0.0):
    """按置信度决定是否拒识；返回 ``(name, confidence, index)``。

    ``threshold <= 0`` 时退化为普通的 argmax 分类（与原实现一致）。
    """
    probs = np.asarray(probs, dtype=np.float64).reshape(-1)
    if probs.size != NUM_CLASSES:
        raise ValueError(f'expected {NUM_CLASSES} probabilities, got {probs.size}')
    index = int(np.argmax(probs))
    confidence = float(probs[index])
    if threshold > 0.0 and confidence < threshold:
        return 'unknown', confidence, index
    return CLASS_NAMES[index], confidence, index


def metrics_from_probs(probs, labels, threshold=0.0):
    """在给定拒识阈值下统计覆盖率、被接受样本的准确率与背景误报率。"""
    probs = np.asarray(probs, dtype=np.float64)
    labels = np.asarray(labels, dtype=np.int64).reshape(-1)
    if probs.shape[0] != labels.shape[0]:
        raise ValueError('probs and labels must have the same length')

    predicted = probs.argmax(axis=1)
    confidence = probs.max(axis=1)
    accepted = np.ones(labels.shape, dtype=bool) if threshold <= 0.0 else confidence >= threshold

    total = int(labels.size)
    n_accepted = int(accepted.sum())
    if n_accepted:
        correct = int((predicted[accepted] == labels[accepted]).sum())
        background = labels[accepted] == BACKGROUND
        false_alarms = int(((predicted[accepted] != BACKGROUND) & background).sum())
        accuracy = correct / n_accepted
        background_fp = false_alarms / int(background.sum()) if background.any() else 0.0
    else:
        accuracy = float('nan')
        background_fp = float('nan')

    return {
        'threshold': round(float(threshold), 3),
        'samples': total,
        'accepted': n_accepted,
        'coverage': round(n_accepted / total, 4),
        'accuracy': round(accuracy, 4) if n_accepted else None,
        'background_fp_rate': round(background_fp, 4) if n_accepted else None,
    }


def sweep_thresholds(probs, labels, thresholds=(0.0, 0.5, 0.7, 0.9, 0.95, 0.99)):
    """逐阈值评估，形成"覆盖率–准确率"曲线。"""
    return [metrics_from_probs(probs, labels, threshold=t) for t in thresholds]


# ---------------------------------------------------------------- torch 部分
def _to_tensor(images):
    import torch
    return torch.tensor(images, dtype=torch.float32).unsqueeze(1) / 255.0


def train_open_set(epochs=30, lr=1e-3, seed=42, n_train=800, clutter=2):
    """训练 4 类 CNN，返回 ``(net, history)``，权重存到 sonar_cnn_openset.pt。"""
    import torch
    import torch.nn as nn
    from torch.utils.data import DataLoader, TensorDataset

    from .cnn_model import SonarCNN

    images, labels = build_dataset(n_train, seed, clutter=clutter)
    x = _to_tensor(images)
    y = torch.tensor(labels, dtype=torch.long)

    torch.manual_seed(seed)
    net = SonarCNN(num_classes=NUM_CLASSES)
    optimiser = torch.optim.Adam(net.parameters(), lr=lr)
    loss_fn = nn.CrossEntropyLoss()
    loader = DataLoader(TensorDataset(x, y), batch_size=32, shuffle=True)

    history = []
    for epoch in range(epochs):
        total = 0.0
        for batch_x, batch_y in loader:
            optimiser.zero_grad()
            loss = loss_fn(net(batch_x), batch_y)
            loss.backward()
            optimiser.step()
            total += float(loss.item())
        history.append(total / max(1, len(loader)))
        if epoch % 10 == 0 or epoch == epochs - 1:
            print(f'[openset] epoch {epoch:3d}/{epochs}  loss={history[-1]:.4f}')

    weights_dir = common.get_package_root() / 'weights'
    weights_dir.mkdir(exist_ok=True)
    path = weights_dir / 'sonar_cnn_openset.pt'
    torch.save(net.state_dict(), path)
    print(f'[openset] 权重保存到 {path}')
    return net, history


def predict_probs(net, images):
    """批量推理，返回 ``(N, 4)`` 的 softmax 概率。"""
    import torch

    net.eval()
    with torch.no_grad():
        logits = net(_to_tensor(images))
        return torch.softmax(logits, dim=1).numpy()


def train_and_evaluate(epochs=30, seed_train=42, seed_test=1234,
                       n_train=800, n_test=400, clutter=2):
    """训练 + 在**独立种子**的测试集上评估，打印阈值扫描表并保存报告。"""
    net, _ = train_open_set(epochs=epochs, seed=seed_train,
                            n_train=n_train, clutter=clutter)

    test_images, test_labels = build_dataset(n_test, seed_test, clutter=clutter)
    probs = predict_probs(net, test_images)

    rows = sweep_thresholds(probs, test_labels)
    print(f"[openset] 测试集 seeds: train={seed_train} test={seed_test} "
          f"n={n_test} clutter={clutter}")
    print(f"{'threshold':>9} {'coverage':>9} {'accuracy':>9} {'bg_fp':>7}")
    for row in rows:
        accuracy = 'n/a' if row['accuracy'] is None else f"{row['accuracy']:.3f}"
        bg_fp = 'n/a' if row['background_fp_rate'] is None else f"{row['background_fp_rate']:.3f}"
        print(f"{row['threshold']:>9.3f} {row['coverage']:>9.3f} {accuracy:>9} {bg_fp:>7}")

    report = {
        'seed_train': seed_train,
        'seed_test': seed_test,
        'n_train': n_train,
        'n_test': n_test,
        'clutter': clutter,
        'rows': rows,
    }
    path = common.get_package_root() / 'data' / 'openset_report.json'
    path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'[openset] 报告: {path}')
    return report
