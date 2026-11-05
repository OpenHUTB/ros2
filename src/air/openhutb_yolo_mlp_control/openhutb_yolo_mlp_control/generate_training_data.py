import argparse
import csv
import random


NUM_SAMPLES = 50000
KP_XY = 0.8
KP_Z = 0.8
KD_XY = 0.35
KD_Z = 0.35
MAX_XY_SPEED = 4.0
MAX_Z_SPEED = 2.0


def clip(value, limit):
    return max(-limit, min(limit, value))


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--samples', type=int, default=NUM_SAMPLES)
    parser.add_argument('--output', default='controller_dataset.csv')
    parsed = parser.parse_args(args)

    with open(parsed.output, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([
            'ex', 'ey', 'ez', 'vx', 'vy', 'vz', 'vx_cmd', 'vy_cmd', 'vz_cmd'
        ])
        for _ in range(parsed.samples):
            ex = random.uniform(-15, 15)
            ey = random.uniform(-15, 15)
            ez = random.uniform(-6, 6)
            vx = random.uniform(-4, 4)
            vy = random.uniform(-4, 4)
            vz = random.uniform(-2, 2)
            vx_cmd = clip(KP_XY * ex - KD_XY * vx, MAX_XY_SPEED)
            vy_cmd = clip(KP_XY * ey - KD_XY * vy, MAX_XY_SPEED)
            vz_cmd = clip(KP_Z * ez - KD_Z * vz, MAX_Z_SPEED)
            writer.writerow([ex, ey, ez, vx, vy, vz, vx_cmd, vy_cmd, vz_cmd])

    print(f'训练数据生成完成: {parsed.samples} 条')
    print(f'文件: {parsed.output}')


if __name__ == '__main__':
    main()
