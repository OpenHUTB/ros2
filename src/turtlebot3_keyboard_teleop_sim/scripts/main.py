#!/usr/bin/env python3
"""Task 1 entry: start world plus keyboard teleop, verify startup, monitor."""
import argparse
import os
import subprocess
import sys
import time


def log(msg):
    print('[main] ' + msg, flush=True)


def env_setup():
    env = dict(os.environ)
    env.setdefault('TURTLEBOT3_MODEL', 'burger')
    here = os.path.dirname(os.path.abspath(__file__))
    src = os.path.abspath(os.path.join(here, '..'))
    env['ROS_PACKAGE_PATH'] = src + os.pathsep + env.get('ROS_PACKAGE_PATH', '')
    env.setdefault('ROS_MASTER_URI', 'http://localhost:11311')
    return env


def gazebo_ready(env, timeout=5):
    deadline = time.time() + timeout
    while time.time() < deadline:
        out = subprocess.run(['rosservice', 'list'], capture_output=True,
                             text=True, env=env).stdout
        if '/gazebo/get_world_properties' in out:
            return True
        time.sleep(1)
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--gl', choices=['software', 'hardware'],
                    default='software',
                    help='rendering backend; software suits VMs without GPU')
    args = ap.parse_args()
    env = env_setup()
    if args.gl == 'software' and 'LIBGL_ALWAYS_SOFTWARE' not in os.environ:
        env['LIBGL_ALWAYS_SOFTWARE'] = '1'
    proc = subprocess.Popen(
        ['roslaunch', 'turtlebot3_keyboard_teleop_sim',
         'keyboard_teleop_sim.launch'], env=env)
    deadline = time.time() + 120
    while time.time() < deadline:
        if proc.poll() is not None:
            log('ERROR: roslaunch exited early with code %s' % proc.returncode)
            return 1
        if gazebo_ready(env):
            log('Gazebo world ready; drive the robot in the xterm window '
                'with w/a/s/d/x.')
            return proc.wait()
        time.sleep(1)
    proc.terminate()
    proc.wait()
    log('ERROR: Gazebo did not become ready within timeout')
    return 1


if __name__ == '__main__':
    sys.exit(main())
