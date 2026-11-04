#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把评价结果画成图，供文档和 PR 使用（纯计算 + matplotlib，不依赖 CARLA）。

两张图：

* 混淆矩阵热力图 —— 看完能知道错在哪：是车被当成路面，还是行人被当成柱子；
* 分距离 IoU 曲线 —— 远处点稀疏，精度下滑多少，一张图看出来。

matplotlib 是可选依赖：没装就返回空列表，由调用方提示，不阻断训练。

**图里的文字一律用英文。** matplotlib 默认字体没有中文字形，直接写中文会变成
方框（终端刷一屏 "Glyph missing from current font"）。装 Noto CJK 能解决，
但要求每个复现的人先去装字体不合理；中文说明放到文档图注里，图本身保持英文。
"""

import os

from . import labels as L


def _pyplot():
    """延迟导入 matplotlib，并用无界面后端，避免虚拟机里没有显示服务时报错。"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    return plt


def save_confusion(matrix, path, title='Confusion matrix (rows = truth, cols = prediction)'):
    """画混淆矩阵热力图，数字写在格子里，颜色越深表示数量越多。"""
    plt = _pyplot()
    names = list(L.CLASS_NAMES)
    figure, axes = plt.subplots(figsize=(5.2, 4.4), dpi=140)
    image = axes.imshow(matrix, cmap='Blues')
    figure.colorbar(image, ax=axes, fraction=0.046, pad=0.04)

    axes.set_xticks(range(len(names)))
    axes.set_yticks(range(len(names)))
    axes.set_xticklabels(names, rotation=30, ha='right')
    axes.set_yticklabels(names)
    axes.set_xlabel('predicted class')
    axes.set_ylabel('true class')
    axes.set_title(title, fontsize=11)

    total = float(matrix.sum()) or 1.0
    threshold = matrix.max() * 0.5
    for row in range(len(names)):
        for col in range(len(names)):
            value = int(matrix[row][col])
            share = value / total
            axes.text(col, row, '%d\n%.1f%%' % (value, share * 100.0),
                      ha='center', va='center', fontsize=7,
                      color='white' if value > threshold else '#222222')

    figure.tight_layout()
    figure.savefig(path)
    plt.close(figure)
    return path


def save_range_curve(by_range, path, title='Accuracy and macro IoU by range'):
    """画分距离档的准确率与宏平均 IoU。"""
    plt = _pyplot()
    keys = list(by_range.keys())
    accuracy = [by_range[k]['accuracy'] for k in keys]
    macro = [by_range[k]['macro_iou'] for k in keys]

    figure, axes = plt.subplots(figsize=(5.2, 3.6), dpi=140)
    positions = range(len(keys))
    axes.plot(positions, accuracy, marker='o', label='overall accuracy')
    axes.plot(positions, macro, marker='s', label='macro IoU')
    axes.set_xticks(list(positions))
    axes.set_xticklabels(keys)
    axes.set_ylim(0.0, 1.05)
    axes.set_xlabel('range bin')
    axes.set_ylabel('score')
    axes.set_title(title, fontsize=11)
    axes.grid(True, linestyle='--', alpha=0.4)
    axes.legend(loc='lower left')

    for x, value in zip(positions, macro):
        axes.annotate('%.2f' % value, (x, value), textcoords='offset points',
                      xytext=(0, -14), ha='center', fontsize=8)

    figure.tight_layout()
    figure.savefig(path)
    plt.close(figure)
    return path


def save_all(report, by_range, directory):
    """一次画完两张图，返回生成的文件路径列表；没装 matplotlib 时返回空列表。"""
    try:
        _pyplot()
    except ImportError:
        print('[出图] 没有安装 matplotlib，跳过。需要时执行 '
              'python3.10 -m pip install matplotlib')
        return []
    os.makedirs(directory, exist_ok=True)
    paths = [
        save_confusion(report['confusion_matrix'],
                       os.path.join(directory, 'lidar_seg_confusion.png')),
        save_range_curve(by_range,
                         os.path.join(directory, 'lidar_seg_range_iou.png')),
    ]
    return paths