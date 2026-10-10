#!/usr/bin/env python3
"""carla_pilotnet_ros 入口脚本"""
import os, sys, subprocess, argparse

def main():
    parser = argparse.ArgumentParser(description='PilotNet ROS launcher')
    parser.add_argument('--image-dir', default=None)
    parser.add_argument('--model-path', default=None)
    parser.add_argument('--rate', type=float, default=2.0)
    args = parser.parse_args()

    pkg_dir = os.path.dirname(os.path.abspath(__file__))
    launch_file = os.path.join(pkg_dir, 'launch', 'main.launch')

    cmd = ['roslaunch', launch_file]
    if args.image_dir: cmd.append(f'image_dir:={args.image_dir}')
    if args.model_path: cmd.append(f'model_path:={args.model_path}')
    cmd.append(f'publish_rate:={args.rate}')

    print('Running:', ' '.join(cmd))
    try:
        subprocess.run(cmd, check=True)
    except FileNotFoundError:
        print('ERROR: roslaunch not found. Run: source /opt/ros/noetic/setup.bash')
        sys.exit(1)
    except subprocess.CalledProcessError as e:
        sys.exit(e.returncode)

if __name__ == '__main__':
    main()
