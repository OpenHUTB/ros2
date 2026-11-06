"""Humble demo: real launch, separate recorder process, Action and offline exports."""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from std_msgs.msg import String
from std_srvs.srv import Trigger
from uav_mission_interfaces.action import ExecuteMission


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True, help='New evidence directory')
    output = parser.parse_args().output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('mission_recording_smoke')
    latest = {}

    def status(message):
        latest.update(json.loads(message.data))

    subscription = node.create_subscription(String, '/uav_mission_safety_manager/status', status, 10)
    publisher = node.create_publisher(String, '/uav_mission_safety_manager/command', 10)
    action = ActionClient(node, ExecuteMission, '/uav_mission_safety_manager/execute_mission')
    console = (output / 'launch.txt').open('w')
    launch = subprocess.Popen(
        ['ros2', 'launch', 'uav_mission_safety_manager', 'main.launch.py',
         'use_demo_adapter:=true', 'record_missions:=true',
         'recording_directory:=' + str(output / 'logs')],
        stdout=console, stderr=subprocess.STDOUT, start_new_session=True)

    def wait(predicate, timeout=25.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
            if launch.poll() is not None:
                raise AssertionError('Launch exited; inspect ' + str(output / 'launch.txt'))
        raise AssertionError('Timeout; latest=' + json.dumps(latest))

    def resolve(future):
        wait(future.done)
        return future.result()

    def service(name):
        client = node.create_client(Trigger, '/uav_mission_safety_manager/' + name)
        try:
            assert client.wait_for_service(timeout_sec=5.)
            response = resolve(client.call_async(Trigger.Request()))
            assert response.success, response.message
        finally:
            node.destroy_client(client)

    def send(xs, speed):
        goal = ExecuteMission.Goal(waypoints=[Point(x=x, z=-2.) for x in xs], speeds=[speed] * len(xs))
        handle = resolve(action.send_goal_async(goal))
        assert handle.accepted
        return handle

    try:
        assert action.wait_for_server(timeout_sec=10.)
        wait(lambda: latest.get('state') == 'IDLE' and publisher.get_subscription_count() > 0)
        # Let the independently discovered recorder receive idle samples before takeoff.
        wait(lambda: list((output / 'logs').glob('*.jsonl')))
        until = time.monotonic() + 1.
        wait(lambda: time.monotonic() >= until)
        for event, state in [('connect', 'ARMING'), ('armed', 'TAKEOFF'), ('airborne', 'EXECUTING')]:
            publisher.publish(String(data=json.dumps(dict(event=event))))
            wait(lambda: latest.get('state') == state and not latest.get('command_pending'))
        handle = send([2., 4., 6.], 4.)
        result = resolve(handle.get_result_async())
        assert result.status == 4 and result.result.completed == 3
        print('three_waypoints: SUCCEEDED, completed=3', flush=True)
        wait(lambda: not latest.get('command_pending'))
        handle = send([30.], 0.5)
        wait(lambda: latest['mission_queue']['state'] == 'RUNNING' and not latest['command_pending'])
        service('pause')
        wait(lambda: latest['mission_queue']['state'] == 'PAUSED' and not latest['command_pending'])
        until = time.monotonic() + 1.
        wait(lambda: time.monotonic() >= until)
        service('resume')
        wait(lambda: latest['mission_queue']['state'] == 'RUNNING' and not latest['command_pending'])
        assert resolve(handle.cancel_goal_async()).goals_canceling
        assert resolve(handle.get_result_async()).status == 5
        print('pause_resume_client_cancel: CANCELED', flush=True)
        wait(lambda: not latest['command_pending'])
        handle = send([30.], 0.5)
        wait(lambda: latest['mission_queue']['state'] == 'RUNNING' and not latest['command_pending'])
        service('return_home')
        assert resolve(handle.get_result_async()).status == 6
        wait(lambda: latest.get('state') == 'LANDED', timeout=35.)
        print('return_interrupt: Action ABORTED, queue CANCELED, then LANDED', flush=True)
    finally:
        # Let launch forward one signal to its children.
        launch.send_signal(signal.SIGINT)
        try:
            launch.wait(timeout=15.)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
            launch.wait()
        console.close()
        action.destroy()
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    logs = list((output / 'logs').glob('*.jsonl'))
    assert len(logs) == 1
    subprocess.run(['ros2', 'run', 'uav_mission_safety_manager', 'mission_report', str(logs[0]),
                    '--output-dir', str(output / 'report')], check=True)
    report = json.loads((output / 'report/summary.json').read_text())
    missions = report['missions']
    assert [m['outcome'] for m in missions] == ['COMPLETED', 'CANCELED', 'CANCELED'], missions
    assert missions[0]['completed'] == 3 and missions[0]['sampled_distance_m'] > 0
    assert missions[1]['observed_pause_s'] >= 0.8
    assert all(not m['start_partial'] for m in missions)
    with (output / 'report/missions.csv').open() as stream:
        assert len(list(csv.DictReader(stream))) == 3
    print('PASS: separate recorder, JSONL, pause timing, terminal deduplication, JSON/CSV exports', flush=True)


if __name__ == '__main__':
    main()
