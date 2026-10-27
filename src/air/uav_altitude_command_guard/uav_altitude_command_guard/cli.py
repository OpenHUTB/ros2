"""回放正常、越界、无效和超时指令."""
import argparse
import csv
from pathlib import Path

import numpy as np

from .core import AltitudeGuard


def replay(output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from PIL import Image

    output.mkdir(parents=True, exist_ok=True)
    guard = AltitudeGuard()
    events = {0: 2.0, 10: 2.0, 20: 2.0, 30: 9.0, 40: 9.0,
              50: float('nan'), 100: 0.0, 110: 0.0, 120: 0.0}
    rows = []
    for index in range(151):
        now = index * 0.02
        accepted = ''
        if index in events:
            accepted = guard.command(events[index], now)
        value, state = guard.tick(now)
        rows.append({'time_s': now, 'target_m': guard.target, 'output_m': value,
                     'state': state, 'accepted': accepted})
    with (output / 'commands.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    times = [r['time_s'] for r in rows]
    values = [r['output_m'] for r in rows]
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(times, [r['target_m'] for r in rows], '--', label='accepted target')
    ax.plot(times, values, label='rate limited output')
    ax.set(xlabel='Time (s)', ylabel='Height (m)', ylim=(0.2, 3.2))
    ax.legend()
    fig.tight_layout()
    fig.savefig(output / 'command_guard.png', dpi=140)
    marker, = ax.plot([], [], 'ro')
    frames = []
    for i in range(0, len(rows), 3):
        marker.set_data([times[i]], [values[i]])
        fig.canvas.draw()
        frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()))
    frames[0].save(output / 'command_guard.gif', save_all=True, append_images=frames[1:], duration=60, loop=0)
    plt.close(fig)
    print(f'已回放 {len(rows)} 个时刻，结果位于 {output}')


def main():
    parser = argparse.ArgumentParser(description='无人机定高指令限幅与超时保持')
    parser.add_argument('--output', type=Path, default=Path('outputs'))
    parser.add_argument('--ros', action='store_true')
    args = parser.parse_args()
    if args.ros:
        from .ros_node import main as ros_main
        ros_main()
    else:
        replay(args.output)
