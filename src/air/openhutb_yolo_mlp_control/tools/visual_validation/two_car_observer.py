"""Run the production ROS visual loop and independently record a two-car test."""
import argparse
import json
import os
import math
from pathlib import Path
import signal
import socket
import subprocess
import time

from scenario import DURATION, output_directory, read_json, summarize, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, help='Same shared directory as scene driver')
    parser.add_argument('--airsim-ip', required=True)
    parser.add_argument('--airsim-port', type=int, default=41451)
    parser.add_argument('--model', required=True)
    parser.add_argument('--prepare-pose', action='store_true',
        help='Explicit Town10HD one-time placement; preserves body yaw, fixes camera forward')
    parser.add_argument('--show-window', action='store_true')
    args = parser.parse_args()
    root = output_directory(args.output)
    if not Path(args.model).is_file():
        parser.error('Model must be an existing local file')
    if not (root / 'scene_ready.json').exists():
        parser.error('Start the scene driver first, with the same shared directory')
    if (root / 'ready.json').exists() or (root / 'stop.json').exists():
        parser.error('Session already used; start both tools with a new directory')
    scene = read_json(root / 'scene_ready.json')
    from openhutb_yolo_mlp_control.vendor import airsim
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    import cv2
    import numpy as np
    with socket.create_connection((args.airsim_ip, args.airsim_port), timeout=3):
        pass
    client = airsim.MultirotorClient(ip=args.airsim_ip, port=args.airsim_port,
                                    timeout_value=5)
    api = client.isApiControlEnabled()
    rclpy.init()
    node = Node('two_car_validation_observer')
    rows, observations, odoms, events, exit_samples = [], [], [], [], []
    images, saved = {}, set()
    proc, log, ready, sent = None, None, None, False
    failure = None
    try:
        discovery_start = time.monotonic()
        while time.monotonic() - discovery_start < .5:
            rclpy.spin_once(node, timeout_sec=.05)
        # Refuse competing control and perception graphs in this ROS domain.
        if any(name in ('airsim_bridge', 'visual_centering', 'yolo_detector')
               for name in node.get_node_names()):
            raise RuntimeError('Stop existing visual nodes before validation')
        pose = client.simGetVehiclePose()
        yaw = airsim.to_eularian_angles(pose.orientation)[2]
        write_json(root / 'geometry.json', {'yaw': yaw})
        if args.prepare_pose:
            s = scene['arguments']
            # Town10HD CARLA-to-AirSim translation used only for initial scene setup.
            pose.position = airsim.Vector3r(s['x'] - 36.2034 - 12*math.cos(yaw)
                + .8*math.sin(yaw), s['y'] + 7.5409 - 12*math.sin(yaw)
                - .8*math.cos(yaw), 47)
            client.enableApiControl(True)
            try:
                client.cancelLastTask()
                client.simSetCameraPose('0', airsim.Pose(airsim.Vector3r(.5, 0, .1),
                                                       airsim.to_quaternion(0, 0, 0)))
                client.simSetVehiclePose(pose, True)
                time.sleep(.5)
                actual = client.simGetVehiclePose().position
                if math.dist([pose.position.x_val, pose.position.y_val, pose.position.z_val],
                             [actual.x_val, actual.y_val, actual.z_val]) > 1:
                    raise RuntimeError('Initial placement failed')
            finally:
                client.enableApiControl(api)

        def status(msg):
            rows.append(dict(t=time.time(), **json.loads(msg.data)))

        def observation(msg):
            observations.append(dict(t=time.time(), **json.loads(msg.data)))

        def odom(msg):
            p = msg.pose.pose.position
            odoms.append({'t': time.time(), 'p': [p.x, p.y, p.z]})

        def image(msg):
            stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
            if msg.encoding != 'bgr8':
                return
            images[stamp] = (msg.height, msg.width, msg.step, bytes(msg.data))
            while len(images) > 20:
                del images[next(iter(images))]

        node.create_subscription(String, '/openhutb/visual_centering/status', status, 100)
        node.create_subscription(String, '/openhutb/visual_observation', observation, 100)
        node.create_subscription(Odometry, '/openhutb/odom', odom, qos_profile_sensor_data)
        node.create_subscription(Image, '/openhutb/yolo/image', image, qos_profile_sensor_data)
        actions = node.create_publisher(String, '/openhutb/visual_centering/action', 1)
        log = (root / 'launch.log').open('w', encoding='utf-8')
        proc = subprocess.Popen(['ros2', 'launch', 'openhutb_yolo_mlp_control',
            'visual_centering.launch.py', 'airsim_ip:=' + args.airsim_ip,
            'airsim_port:=' + str(args.airsim_port), 'yolo_model_path:=' + str(Path(args.model).resolve()),
            'dry_run:=false', 'show_window:=' + str(args.show_window).lower(),
            'lock_timeout:=3.0', 'detection_timeout:=1.5',
            'target_area:=0.0', 'vertical_gain:=0.0'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        started = time.monotonic()
        while proc.poll() is None:
            rclpy.spin_once(node, timeout_sec=.02)
            now = time.time()
            if ready is None:
                if rows and rows[-1]['status'] in ('tracking', 'centered'):
                    ready = now
                    write_json(root / 'ready.json', {'epoch': ready, 'run_id': scene['run_id']})
                elif time.monotonic() - started > 35:
                    raise TimeoutError('No confirmed target in 35 seconds')
                continue
            elapsed = now - ready
            if (root / 'scene_cleaned.json').exists():
                raise RuntimeError('Scene driver exited before observer completed')
            if elapsed >= 45 and not sent:
                msg = String(); msg.data = 'reselect'; actions.publish(msg)
                events.append({'t': now, 'action': 'reselect'}); sent = True
            # Match image and status by capture stamp; do not fake status overlays.
            if rows:
                state = rows[-1]
                stamp = state.get('observation_stamp_ns')
                windows = [('two_cars_locked', 7, 14, ('tracking', 'centered')),
                    ('short_loss', 15, 16.6, ('target_lost',)),
                    ('short_recovered', 19, 28, ('tracking', 'centered')),
                    ('long_loss', 33, 35, ('reselect_required',)),
                    ('returned_waiting', 38, 44, ('reselect_required',)),
                    ('explicit_reselect', 48, 54, ('tracking', 'centered'))]
                for name, low, high, statuses in windows:
                    if low < elapsed < high and state['status'] in statuses and name not in saved and stamp in images:
                        h, w, step, data = images[stamp]
                        frame = np.frombuffer(data, np.uint8).reshape(h, step)[:, :w*3].reshape(h, w, 3)
                        if not cv2.imwrite(str(root / (name + '.png')), frame):
                            raise RuntimeError('Could not save image')
                        write_json(root / (name + '.json'), {'image_stamp_ns': stamp, 'status': state})
                        saved.add(name)
            if elapsed >= DURATION:
                break
        if proc.poll() is not None:
            raise RuntimeError('Production launch exited before test completed')
    except KeyboardInterrupt:
        failure = 'Interrupted by operator'
    except Exception as exc:
        failure = str(exc)
        raise
    finally:
        # Keep the scene stable until the production bridge has stopped.
        try:
            if proc is not None and proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    try:
                        proc.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait(timeout=5)
            for _ in range(11):
                p = client.simGetVehiclePose().position
                exit_samples.append([time.time(), p.x_val, p.y_val, p.z_val])
                time.sleep(.2)
        finally:
            if proc is not None:
                try:
                    client.cancelLastTask()
                    client.enableApiControl(api)
                except Exception as exc:
                    failure = (failure or '') + '; control cleanup failed: ' + str(exc)
            try:
                api_final = client.isApiControlEnabled()
            except Exception:
                api_final = None
            write_json(root / 'stop.json', {'run_id': scene['run_id']})
            write_json(root / 'result.json', {'ready': ready, 'failure': failure,
                'rows': rows, 'observations': observations, 'odom': odoms,
                'events': events, 'exit_samples': exit_samples,
                'api_initial': api, 'api_final': api_final})
            if ready is not None:
                write_json(root / 'summary.json', summarize(rows, observations, ready))
            if log is not None:
                log.close()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
