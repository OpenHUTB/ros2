#!/usr/bin/env python3
"""Train and run humanoid visual patrol, teleoperation and ROS bridge tasks."""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets', required=True, help='Path to unitree_rl_gym checkout')
    parser.add_argument('--mode', choices=['smoke', 'collect', 'train', 'teacher', 'perception', 'end2end', 'teleop', 'bridge', 'train_planner', 'plan_stdio'], default='smoke')
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
    if args.mode == 'bridge':
        from patrol.bridge_server import serve
        serve(args.assets,args.seed,args.controller,args.weights)
        return
    if args.mode == 'collect':
        from patrol.learning import collect
        collect(args.assets, args.dataset, args.samples)
        return
    if args.mode == 'train':
        from patrol.learning import train, train_mask_controller
        train(args.dataset, args.weights, args.epochs)
        train_mask_controller(args.dataset,args.weights)
        return
    if args.mode == 'teleop':
        from patrol.runtime import teleop
        teleop(args.assets,args.seed,args.seconds,args.output)
        return
    if args.mode in ('teacher','perception','end2end'):
        from patrol.runtime import rollout
        result = rollout(args.assets,args.mode,args.weights,args.output or Path('results'),
                         args.seed,args.seconds,args.record)
        print(json.dumps(result,indent=2))
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
