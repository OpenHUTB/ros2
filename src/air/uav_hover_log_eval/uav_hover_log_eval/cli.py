"""将定高 CSV 整理为误差摘要和时间曲线。"""
import argparse
import json
from pathlib import Path

from .metrics import load_log, summarize
from .paths import resource


def evaluate(path, output, disturbance_end=3.0, disturbance_window=1.0,
             tolerance=0.15, hold_s=0.5):
    """计算指标并输出 summary.json、PNG 与 GIF."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    from PIL import Image

    data = load_log(path)
    result = summarize(data, disturbance_end=disturbance_end,
                       tolerance=tolerance, hold_s=hold_s,
                       disturbance_window=disturbance_window)
    output.mkdir(parents=True, exist_ok=True)
    (output / 'summary.json').write_text(
        json.dumps(result, indent=2), encoding='utf-8')
    fig, axes = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
    axes[0].plot(data[:, 0], data[:, 1], label='measured height')
    axes[0].plot(data[:, 0], data[:, 2], '--', label='target')
    axes[0].set_ylabel('Height (m)')
    axes[0].legend()
    axes[1].plot(data[:, 0], data[:, 1] - data[:, 2], label='height error')
    axes[1].axhspan(-tolerance, tolerance, alpha=0.15, color='green')
    axes[1].axvline(disturbance_end, linestyle='--', color='gray')
    axes[1].set(xlabel='Time (s)', ylabel='Error (m)')
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(output / 'flight_error.png', dpi=140)
    cursor = axes[1].axvline(data[0, 0], color='red', alpha=0.6)
    frames = []
    for now in data[::10, 0]:
        cursor.set_xdata([now, now])
        fig.canvas.draw()
        frames.append(
            Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()))
    frames[0].save(output / 'flight_error.gif', save_all=True,
                   append_images=frames[1:], duration=120, loop=0)
    plt.close(fig)
    print(json.dumps(result, indent=2))


def main():
    """解析命令行参数，选择离线评估或 ROS 2 发布."""
    parser = argparse.ArgumentParser(description='定高日志误差与抗扰指标')
    parser.add_argument('--input', type=Path,
                        default=resource('data/gust_ppo_101.csv'))
    parser.add_argument('--output', type=Path, default=Path('outputs'))
    parser.add_argument('--disturbance-end', type=float, default=3.0,
                        help='扰动结束时刻（秒），恢复时间从此起算')
    parser.add_argument('--disturbance-window', type=float, default=1.0,
                        help='扰动窗口长度（秒），用于统计窗口峰值偏差')
    parser.add_argument('--tolerance', type=float, default=0.15,
                        help='恢复判定的容差带半宽（米）')
    parser.add_argument('--hold', dest='hold_s', type=float, default=0.5,
                        help='确认恢复所需的连续保持时间（秒）')
    parser.add_argument('--ros', action='store_true')
    args = parser.parse_args()
    if args.ros:
        from .ros_node import main as ros_main
        ros_main()
    else:
        evaluate(args.input, args.output, args.disturbance_end,
                 args.disturbance_window, args.tolerance, args.hold_s)
