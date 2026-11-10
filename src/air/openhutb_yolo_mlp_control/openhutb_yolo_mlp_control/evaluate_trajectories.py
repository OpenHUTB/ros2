import argparse
import math
import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


TRAJECTORIES = {
    'square': 'square_trajectory.csv',
    'circle': 'circle_trajectory.csv',
    'eight': 'eight_trajectory.csv',
}


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--result-dir', default='results')
    parsed = parser.parse_args(args)
    result_dir = parsed.result_dir
    summary_rows = []

    for name, filename in TRAJECTORIES.items():
        path = os.path.join(result_dir, filename)
        if not os.path.exists(path):
            print('文件不存在:', path)
            continue

        df = pd.read_csv(path)
        dx = df['target_x'] - df['actual_x']
        dy = df['target_y'] - df['actual_y']
        dz = df['target_z'] - df['actual_z']
        error = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        rmse = math.sqrt(float(np.mean(error ** 2)))
        mean_error = float(np.mean(error))
        max_error = float(np.max(error))
        final_error = float(error.iloc[-1])
        duration = float(df['time'].iloc[-1] - df['time'].iloc[0])

        summary_rows.append({
            'trajectory': name,
            'rmse_m': rmse,
            'mean_error_m': mean_error,
            'max_error_m': max_error,
            'final_error_m': final_error,
            'duration_s': duration,
            'samples': len(df),
        })

        plt.figure(figsize=(8, 7))
        plt.plot(df['target_x'], df['target_y'], '--', label='Target trajectory')
        plt.plot(df['actual_x'], df['actual_y'], label='MLP actual trajectory')
        plt.xlabel('X / m')
        plt.ylabel('Y / m')
        plt.title(f'{name.capitalize()} Trajectory Tracking')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.tight_layout()
        plt.savefig(os.path.join(result_dir, f'{name}_xy.png'), dpi=200)
        plt.close()

        plt.figure(figsize=(9, 5))
        plt.plot(df['time'], error)
        plt.xlabel('Time / s')
        plt.ylabel('Position Error / m')
        plt.title(f'{name.capitalize()} Tracking Error')
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(os.path.join(result_dir, f'{name}_error.png'), dpi=200)
        plt.close()

        print(
            f'{name}: RMSE={rmse:.3f} m, mean={mean_error:.3f} m, '
            f'max={max_error:.3f} m, final={final_error:.3f} m, duration={duration:.2f} s'
        )

    summary_df = pd.DataFrame(summary_rows)
    summary_path = os.path.join(result_dir, 'metrics_summary.csv')
    summary_df.to_csv(summary_path, index=False, encoding='utf-8-sig')
    print(summary_df)


if __name__ == '__main__':
    main()
