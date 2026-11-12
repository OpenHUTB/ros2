#!/usr/bin/env python3
"""Fetch the exact upstream model/policy revision; never overwrite an existing path."""
import argparse
from pathlib import Path
import subprocess

REVISION = '276801e46c5d433564f24658bac64f254b7d2d4b'
URL = 'https://github.com/unitreerobotics/unitree_rl_gym.git'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination',type=Path)
    args = parser.parse_args()
    target = args.destination.resolve()
    if target.exists():
        raise SystemExit('Destination exists; use a new directory or validate the existing revision manually.')
    subprocess.run(['git', 'init', str(target)],check=True)
    subprocess.run(['git', '-C', str(target), 'remote', 'add', 'origin', URL], check=True)
    subprocess.run(['git', '-C', str(target), 'fetch', '--depth', '1', 'origin', REVISION], check=True)
    subprocess.run(['git','-C',str(target),'sparse-checkout','set','deploy','resources/robots/g1_description'],check=True)
    subprocess.run(['git','-C',str(target),'checkout','--detach',REVISION],check=True)
    print(target)


if __name__ == '__main__':
    main()
