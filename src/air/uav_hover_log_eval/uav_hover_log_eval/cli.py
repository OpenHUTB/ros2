"""将定高 CSV 整理为误差摘要和时间曲线。"""
import argparse
import json
from pathlib import Path

from .metrics import load_log, summarize
from .paths import resource


def evaluate(path, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    from PIL import Image

    data = load_log(path)
    result = summarize(data)
    output.mkdir(parents=True, exist_ok=True)
    (output / 'summary.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    fig, axes = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
    axes[0].plot(data[:, 0], data[:, 1], label='measured height')
    axes[0].plot(data[:, 0], data[:, 2], '--', label='target')
    axes[0].set_ylabel('Height (m)')
    axes[0].legend()
    axes[1].plot(data[:, 0], data[:, 1] - data[:, 2], label='height error')
    axes[1].axhspan(-0.15, 0.15, alpha=0.15, color='green')
    axes[1].axvline(3.0, linestyle='--', color='gray')
    axes[1].set(xlabel='Time (s)', ylabel='Error (m)')
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(output / 'flight_error.png', dpi=140)
    cursor = axes[1].axvline(data[0, 0], color='red', alpha=0.6)
    frames = []
    for now in data[::10, 0]:
        cursor.set_xdata([now, now])
        fig.canvas.draw()
        frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()))
    frames[0].save(output / 'flight_error.gif', save_all=True, append_images=frames[1:], duration=120, loop=0)
    plt.close(fig)
    print(json.dumps(result, indent=2))


def main():
    parser = argparse.ArgumentParser(description='定高日志误差与抗扰恢复时间')
    parser.add_argument('--input', type=Path, default=resource('data/gust_ppo_101.csv'))
    parser.add_argument('--output', type=Path, default=Path('outputs'))
    parser.add_argument('--ros', action='store_true')
    args = parser.parse_args()
    if args.ros:
        from .ros_node import main as ros_main
        ros_main()
    else:
        evaluate(args.input, args.output)
