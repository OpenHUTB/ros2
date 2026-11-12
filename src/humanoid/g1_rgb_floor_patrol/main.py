#!/usr/bin/env python3
"""用地面分割和语义控制网络为 G1 生成通行速度，并在地面掩码为空时停车。"""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets', required=True, help='Path to unitree_rl_gym checkout')
    parser.add_argument('--mode', choices=['perception', 'collect', 'train', 'teacher'], default='perception')
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
        from patrol.learning import train, train_mask_controller
        train(args.dataset, args.weights, args.epochs)
        train_mask_controller(args.dataset,args.weights)
        return
    if args.mode in ('teacher','perception','end2end'):
        from patrol.runtime import rollout
        result = rollout(args.assets,args.mode,args.weights,args.output or Path('results'),
                         args.seed,args.seconds,args.record)
        print(json.dumps(result,indent=2))
        return


if __name__ == '__main__':
    main()
