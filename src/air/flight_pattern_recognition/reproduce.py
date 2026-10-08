"""Re-train the fixed recipe in a fresh directory, preserving submitted weights."""
import argparse
import subprocess
import sys
from pathlib import Path


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, default=Path('models/retrained_run'))
    args = p.parse_args()
    if args.output.exists():
        raise FileExistsError('Choose a new output folder: ' + str(args.output))
    args.output.mkdir(parents=True)
    for variant, seed in [('mlp', 42), ('mlp', 43), ('mlp', 44), ('gru', 42), ('mlp_position_only', 42)]:
        subprocess.run([sys.executable, 'train.py', '--variant', variant, '--seed', str(seed),
                        '--output', str(args.output / ('%s_%d' % (variant, seed)))], check=True)
    for split in ['test', 'final']:
        subprocess.run([sys.executable, 'evaluate.py', '--split', split, '--models', str(args.output),
                        '--output', str(args.output / ('evaluation_' + split))], check=True)


if __name__ == '__main__':
    main()
