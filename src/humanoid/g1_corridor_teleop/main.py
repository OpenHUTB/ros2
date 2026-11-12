#!/usr/bin/env python3
"""通过键盘复现 G1 预训练步态在室内走廊的前进、转向和停止。"""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets', required=True, help='Path to unitree_rl_gym checkout')
    parser.add_argument('--mode', choices=['teleop', 'smoke'], default='teleop')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--dataset', type=Path, default=Path('data/visual_patrol'))
    parser.add_argument('--weights', type=Path, default=Path(__file__).resolve().parent / 'models')
    parser.add_argument('--samples', type=int, default=100)
    parser.add_argument('--epochs', type=int, default=15)
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--seconds', type=float, default=45)
    parser.add_argument('--record', action='store_true')
    parser.add_argument('--controller',choices=['navigation','perception','end2end'],default='navigation')
    args = parser.parse_args()
    if args.mode == 'teleop':
        from patrol.runtime import teleop
        teleop(args.assets,args.seed,args.seconds,args.output)
        return
    from patrol.locomotion import Walker
    result = Walker(args.assets).smoke_test()
    report = json.dumps(result, indent=2)
    print(report)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(report+'\n', encoding='utf-8')
    if result['fallen'] or result['displacement_m'] < 0.5:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
