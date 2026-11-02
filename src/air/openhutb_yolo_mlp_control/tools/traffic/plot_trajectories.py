"""Plot CARLA ground-truth trajectories without connecting to the simulator."""
import argparse
import csv
import json
import math
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True)
    args = parser.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np

    root = args.data.resolve()
    meta = json.loads((root / 'metadata.json').read_text(encoding='utf-8'))
    groups = {}
    with (root / 'vehicle_trajectories.csv').open(encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            groups.setdefault(int(row['actor_id']), []).append(row)
    if not groups:
        raise RuntimeError('No managed vehicle samples')
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False,
                         'axes.spines.right': False, 'pdf.fonttype': 42})
    fig, (ax, speeds) = plt.subplots(1, 2, figsize=(13.5, 6),
                                    gridspec_kw={'width_ratios': [1.25, 1]})
    stats = []
    colors = plt.get_cmap('tab10')
    for i, (actor_id, rows) in enumerate(sorted(groups.items())):
        x = np.array([float(row['x_m']) for row in rows])
        y = np.array([float(row['y_m']) for row in rows])
        t = np.array([float(row['sim_time']) for row in rows])
        speed = np.array([math.hypot(float(row['vx_mps']), float(row['vy_mps'])) * 3.6
                          for row in rows])
        color = colors(i % 10)
        kind = rows[0]['vehicle_type'].removeprefix('vehicle.')
        ax.plot(x, y, color=color, lw=1.6, alpha=0.85, label=f'{actor_id}: {kind}')
        ax.scatter(x[0], y[0], s=28, marker='o', color=color, zorder=4)
        ax.scatter(x[-1], y[-1], s=40, marker='x', color=color, zorder=5)
        speeds.plot(t - t[0], speed, color=color, lw=1.1, alpha=0.8)
        stats.append({'actor_id': actor_id, 'vehicle_type': rows[0]['vehicle_type'],
                      'samples': len(rows),
                      'distance_m': round(float(np.hypot(np.diff(x), np.diff(y)).sum()), 2),
                      'mean_speed_kmh': round(float(speed.mean()), 2),
                      'max_speed_kmh': round(float(speed.max()), 2),
                      'simulation_duration_s': round(float(t[-1] - t[0]), 2)})
    ax.set(xlabel='CARLA world X (m)', ylabel='CARLA world Y (m)',
           title='A  |  Vehicle trajectories (circle: start; cross: end)')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(alpha=0.2)
    ax.legend(loc='best', fontsize=7.5, framealpha=0.9)
    speeds.set(xlabel='Elapsed simulation time (s)', ylabel='Speed (km/h)',
               title='B  |  Vehicle speed')
    speeds.grid(alpha=0.2)
    fig.suptitle(f'{meta["map"]} — Traffic Manager vehicle trajectory recording', fontsize=15)
    rate = meta.get('sampling_hz_actual', meta.get('sampling_hz_requested', meta.get('sampling_hz_wall', 10)))
    fig.text(0.5, 0.015, f'{len(groups)} vehicles | {meta["duration_wall_s"]:.1f} s capture'
             f' | CARLA ground truth | approx. {rate:g} Hz wall-clock sampling',
             ha='center', fontsize=9, color='#444444')
    fig.tight_layout(rect=(0, 0.04, 1, 0.94))
    fig.savefig(root / 'traffic_trajectory_snapshot.png', dpi=220)
    fig.savefig(root / 'traffic_trajectory_snapshot.pdf')
    plt.close(fig)
    with (root / 'vehicle_summary.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(stats[0]))
        writer.writeheader()
        writer.writerows(stats)
    print(json.dumps(stats, indent=2))


if __name__ == '__main__':
    main()
