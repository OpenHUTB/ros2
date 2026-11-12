#!/usr/bin/env python3
import argparse
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description='Run G1 gmapping and move_base in ROS Noetic.')
    parser.add_argument('--assets', required=True)
    parser.add_argument('--sim-python', default=sys.executable)
    parser.add_argument('--mode', choices=['navigation', 'bridge'], default='navigation')
    parser.add_argument('--seed', type=int, default=7)
    args = parser.parse_args()
    if args.mode == 'bridge':
        from patrol.bridge_server import serve
        serve(args.assets, args.seed)
        return
    script = Path(__file__).resolve().parent / 'scripts/run_navigation.sh'
    subprocess.run(['bash', str(script), args.sim_python, args.assets, str(args.seed)], check=True)


if __name__ == '__main__':
    main()
