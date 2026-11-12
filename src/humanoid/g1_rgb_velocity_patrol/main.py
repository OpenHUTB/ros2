#!/usr/bin/env python3
"""验证直接由 RGB 图像预测速度的 G1 巡检策略，并提供近墙行为复测入口。"""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets', required=True, help='Path to unitree_rl_gym checkout')
    parser.add_argument('--mode', choices=['end2end', 'collect', 'train', 'teacher'], default='end2end')
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
    if args.mode == 'collect':
        from patrol.learning import collect
        collect(args.assets, args.dataset, args.samples)
        return
    if args.mode == 'train':
        from patrol.learning import train
        train(args.dataset, args.weights, args.epochs)
        return
    if args.mode in ('teacher','perception','end2end'):
        from patrol.runtime import rollout
        result = rollout(args.assets,args.mode,args.weights,args.output or Path('results'),
                         args.seed,args.seconds,args.record)
        print(json.dumps(result,indent=2))
        return


if __name__ == '__main__':
    main()
