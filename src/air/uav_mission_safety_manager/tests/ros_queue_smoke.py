"""Launch a separate Humble demo node and exercise real ROS topic/service I/O."""
import json
import os
import signal
import subprocess
import tempfile
import time

import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = rclpy.create_node('waypoint_queue_smoke')
    status = {}
    node.create_subscription(String, '/uav_mission_safety_manager/status',
                             lambda msg: status.update(json.loads(msg.data)), 10)
    publisher = node.create_publisher(String, '/uav_mission_safety_manager/command', 10)
    launch_log = tempfile.TemporaryFile(mode='w+')
    process = subprocess.Popen([
        'ros2', 'launch', 'uav_mission_safety_manager', 'main.launch.py',
        'use_demo_adapter:=true'], stdout=launch_log, stderr=subprocess.STDOUT,
        start_new_session=True)

    def wait_for(predicate, timeout=12.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if process.poll() is not None:
                launch_log.seek(0)
                raise AssertionError('launch exited: ' + launch_log.read())
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError('Timed out; latest status: ' + json.dumps(status))

    def record(label):
        print(json.dumps({'check': label, 'status': status}, ensure_ascii=False), flush=True)

    def command(event, **data):
        msg = String()
        msg.data = json.dumps(dict(event=event, **data))
        publisher.publish(msg)

    def service(name):
        client = node.create_client(Trigger, '/uav_mission_safety_manager/' + name)
        try:
            assert client.wait_for_service(timeout_sec=5)
            future = client.call_async(Trigger.Request())
            wait_for(future.done)
            assert future.result().success, future.result().message
        finally:
            node.destroy_client(client)

    try:
        wait_for(lambda: status.get('state') == 'IDLE' and publisher.get_subscription_count() > 0)
        command('connect')
        wait_for(lambda: status.get('state') == 'ARMING')
        command('armed')
        wait_for(lambda: status.get('state') == 'TAKEOFF')
        command('airborne')
        wait_for(lambda: status.get('state') == 'EXECUTING' and not status['command_pending'])
        points = [{'x': x, 'y': 0, 'z': -2, 'speed': 2} for x in (4, 8, 12)]
        command('mission', waypoints=points)
        wait_for(lambda: status.get('mission_queue', {}).get('completed') == 1)
        record('first_waypoint_reached')
        service('pause')
        wait_for(lambda: status['mission_queue']['state'] == 'PAUSED' and not status['command_pending'])
        paused = status['mission_queue']['completed']
        start = time.monotonic()
        while time.monotonic() - start < 0.8:
            rclpy.spin_once(node, timeout_sec=0.05)
            assert status['mission_queue']['completed'] == paused
        record('pause_preserves_progress')
        service('resume')
        wait_for(lambda: status['mission_queue']['state'] == 'COMPLETED' and not status['command_pending'])
        assert status['mission_queue']['completed'] == 3
        record('three_waypoints_completed')
        command('mission', waypoints=[{'x': 20, 'y': 0, 'z': -2}])
        wait_for(lambda: status['mission_queue']['state'] == 'RUNNING' and not status['command_pending'])
        service('cancel')
        wait_for(lambda: status['mission_queue']['state'] == 'CANCELED' and not status['command_pending'])
        assert status['target_ned'] is None
        record('cancel_clears_target')
        command('mission', waypoints=[{'x': 25, 'y': 0, 'z': -2}])
        wait_for(lambda: status['mission_queue']['state'] == 'RUNNING' and not status['command_pending'])
        service('return_home')
        wait_for(lambda: status['state'] == 'RETURNING' and not status['command_pending'])
        assert status['mission_queue']['state'] == 'CANCELED'
        record('return_preempts_queue')
        service('land')
        wait_for(lambda: status['state'] == 'LANDED')
        record('demo_landed')
        print('PASS: separate ROS launch, 3 waypoints, pause/resume, cancel, return, land', flush=True)
    finally:
        os.killpg(process.pid, signal.SIGINT) if process.poll() is None else None
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        launch_log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
