#!/usr/bin/env python3
"""Smoke-test the installed catkin roslaunch entry with a private ROS master."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

import rosgraph
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    folder = Path(args.output).resolve()
    folder.mkdir(parents=True, exist_ok=False)
    sock = socket.socket()
    sock.bind(('127.0.0.1', 0))
    port = sock.getsockname()[1]
    sock.close()
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:' + str(port)
    os.environ['ROS_HOSTNAME'] = 'localhost'
    os.environ.pop('ROS_IP', None)
    os.environ['ROS_LOG_DIR'] = str(folder / 'ros_logs')
    os.environ['QT_QPA_PLATFORM'] = 'offscreen'
    states = []
    success = False
    log = (folder / 'roslaunch.txt').open('w')
    process = subprocess.Popen(['roslaunch', 'turtle_waypoint_patrol', 'main.launch'],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def wait(check, seconds=10):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if process.poll() is not None:
                raise AssertionError('roslaunch exited during the test')
            try:
                if check():
                    return
            except (OSError, rospy.ROSException):
                pass
            time.sleep(.05)
        raise AssertionError('launch readiness/state timed out')

    try:
        wait(lambda: rosgraph.Master('/launch_probe').getPid())
        rospy.init_node('patrol_launch_smoke', anonymous=True)
        subscriber = rospy.Subscriber('/waypoint_patrol/status', String,
            lambda msg: states.append(json.loads(msg.data)), queue_size=20)
        wait(lambda: states and states[-1]['state'] == 'RUNNING')
        for action, state in [('pause', 'PAUSED'), ('resume', 'RUNNING'), ('cancel', 'CANCELED')]:
            rospy.wait_for_service('/waypoint_patrol/' + action, timeout=5)
            reply = rospy.ServiceProxy('/waypoint_patrol/' + action, Trigger)()
            assert reply.success, (action, reply.message)
            wait(lambda: states[-1]['state'] == state)
        subscriber.unregister()
        success = True
    finally:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            code = process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
            raise
        finally:
            log.close()
        result = {'launch_pass': success, 'roslaunch_exit_code': code,
                  'observed_states': sorted({state['state'] for state in states})}
        (folder / 'result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result), flush=True)
    assert success and code == 0
    assert 'EXIT ros_shutdown_done' in (folder / 'roslaunch.txt').read_text()
    print('INSTALLED_CATKIN_ROSLAUNCH_SMOKE_PASS', flush=True)


if __name__ == '__main__':
    main()
