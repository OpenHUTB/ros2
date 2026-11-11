#!/usr/bin/env python3
"""在独立栅格示例中比较 CostNet 辅助 A* 和曼哈顿 A* 的路径代价与搜索扩展数。"""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets', help=argparse.SUPPRESS)
    parser.add_argument('--mode', choices=['demo', 'train_planner', 'plan_stdio'], default='demo')
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
    if args.mode == 'train_planner':
        from patrol.planning import train_planner
        print(json.dumps(train_planner(args.weights),indent=2))
        return
    if args.mode == 'plan_stdio':
        import sys
        from patrol.planning import plan_request
        print(json.dumps(plan_request(json.load(sys.stdin),args.weights)))
        return
    from patrol.demo import run
    print(json.dumps(run(args.weights, args.output or Path("results")), indent=2))


if __name__ == '__main__':
    main()
