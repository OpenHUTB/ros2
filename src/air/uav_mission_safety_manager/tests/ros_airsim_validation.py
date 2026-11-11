"""Opt-in real AirSim tests through the installed ROS launch/topic/services.

Run from a sourced Foxy workspace with no other vehicle controllers active.
The disconnect case cuts a private TCP relay, not the simulator process.
"""
import argparse
import json
import math
import os
from pathlib import Path
import select
import socket
import subprocess
import threading
import time

import airsim
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger


class Relay:
    """Disposable loopback relay; closing it breaks existing and new RPCs."""
    def __init__(self, host):
        self.host = host
        self.stopped = threading.Event()
        self.listener = socket.socket()
        self.listener.bind(('127.0.0.1', 0))
        self.port = self.listener.getsockname()[1]
        self.listener.listen()
        self.listener.settimeout(0.2)
        self.thread = threading.Thread(target=self.accept, daemon=True)
        self.thread.start()

    def accept(self):
        while not self.stopped.is_set():
            try:
                client, _ = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            threading.Thread(target=self.forward, args=(client,), daemon=True).start()

    def forward(self, client):
        upstream = None
        try:
            upstream = socket.create_connection((self.host, 41451), timeout=3)
            while not self.stopped.is_set():
                readable, _, _ = select.select([client, upstream], [], [], 0.1)
                for source in readable:
                    data = source.recv(65536)
                    if not data:
                        return
                    (upstream if source is client else client).sendall(data)
        except OSError:
            pass
        finally:
            client.close()
            if upstream:
                upstream.close()

    def close(self):
        self.stopped.set()
        self.listener.close()
        self.thread.join(timeout=1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--case', choices=['mission', 'return_land', 'disconnect'], required=True)
    parser.add_argument('--log', required=True)
    parser.add_argument('--fly', action='store_true')
    args = parser.parse_args()
    if not args.fly:
        parser.error('--fly is required: resets and flies the simulated vehicle')
    # Fail quickly before reset if Air mode/RPC is unavailable.
    with socket.create_connection((args.host, 41451), timeout=3):
        pass
    probe = airsim.MultirotorClient(ip=args.host, timeout_value=125)
    if probe.simIsPause():
        raise RuntimeError('Simulator paused; resume it before this test')
    probe.reset()
    deadline, settled, previous = time.monotonic()+30, None, None
    while time.monotonic() < deadline:
        s = probe.getMultirotorState()
        p, v = s.kinematics_estimated.position, s.kinematics_estimated.linear_velocity
        position = (p.x_val, p.y_val, p.z_val)
        stable = (s.landed_state == airsim.LandedState.Landed and v.get_length() <= 0.1
                  and previous is not None and math.dist(position, previous) <= 0.02)
        settled = (settled or time.monotonic()) if stable else None
        if settled and time.monotonic()-settled >= 3:
            break
        previous = position
        time.sleep(0.2)
    else:
        raise RuntimeError('Preflight failed: no stable landed vehicle')
    home = position
    relay = Relay(args.host) if args.case == 'disconnect' else None
    host, port = ('127.0.0.1', relay.port) if relay else (args.host, 41451)
    Path(args.log).parent.mkdir(parents=True, exist_ok=True)
    node_log = open(args.log+'.node.log', 'w', encoding='utf-8')
    process = subprocess.Popen([
        'ros2', 'launch', 'uav_mission_safety_manager', 'main.launch.py',
        'use_airsim:=true', 'airsim_host:='+host, 'airsim_port:='+str(port)],
        stdout=node_log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('airsim_validation_observer')
    latest, statuses = {}, []
    log = open(args.log, 'w', encoding='utf-8')

    def record(event, **data):
        row = dict(time=time.monotonic(), case=args.case, event=event, **data)
        log.write(json.dumps(row)+'\n')
        log.flush()
        if event != 'status':
            print(json.dumps(row), flush=True)

    def status_cb(msg):
        latest.clear()
        latest.update(json.loads(msg.data))
        statuses.append(dict(latest))
        record('status', **latest)

    subscription = node.create_subscription(
        String, '/uav_mission_safety_manager/status', status_cb, 20)
    publisher = node.create_publisher(String, '/uav_mission_safety_manager/command', 10)
    clients = {name: node.create_client(Trigger, '/uav_mission_safety_manager/'+name)
               for name in ['pause', 'resume', 'return_home', 'land']}

    def wait(predicate, timeout=120, allow_failsafe=False):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(node, timeout_sec=0.1)
            if latest.get('state') == 'FAILSAFE' and not allow_failsafe:
                raise RuntimeError(latest.get('reason'))
            if predicate():
                return
            if process.poll() is not None:
                raise RuntimeError('ROS launch process exited')
        raise TimeoutError('Stage timeout: '+json.dumps(latest))

    def command(event, expected=None, **data):
        wait(lambda: not latest.get('command_pending'), 5)
        count = len(statuses)
        publisher.publish(String(data=json.dumps(dict(event=event, **data))))
        record('command', command=event, request=data)
        wait(lambda: len(statuses) > count and
             (latest.get('state') == expected if expected else latest.get('target_ned') is not None))
        wait(lambda: not latest.get('command_pending'), 5)

    def service(name, required_state=None):
        if required_state is None:
            wait(lambda: not latest.get('command_pending'), 5)
        if not clients[name].wait_for_service(timeout_sec=3):
            raise RuntimeError('Service unavailable: '+name)
        if required_state is not None and latest.get('state') != required_state:
            raise RuntimeError('Missed service interruption window: '+json.dumps(latest))
        record('service_request', name=name, state_before=latest.get('state'))
        start = time.monotonic()
        while True:
            if required_state is not None and latest.get('state') != required_state:
                raise RuntimeError('Missed service interruption window: '+json.dumps(latest))
            future = clients[name].call_async(Trigger.Request())
            wait(future.done, 3)
            response = future.result()
            if (response.success or required_state is None or
                    'RPC dispatch pending' not in response.message or time.monotonic()-start > 5):
                break
            record('service_retry', name=name, message=response.message)
            rclpy.spin_once(node, timeout_sec=0.05)
        record('service', name=name, success=response.success, message=response.message,
               response_seconds=time.monotonic()-start)
        if not response.success:
            raise RuntimeError(response.message)
        wait(lambda: not latest.get('command_pending'), 5)

    try:
        wait(lambda: latest.get('state') == 'IDLE' and publisher.get_subscription_count() > 0, 20)
        record('preflight', home=home, ros_domain=os.environ.get('ROS_DOMAIN_ID'))
        command('connect', expected='ARMING')
        command('armed', expected='TAKEOFF')
        command('airborne', expected='EXECUTING')
        wait(lambda: latest.get('takeoff_rpc_result') is not None, 25)
        record('takeoff_result', result=latest['takeoff_rpc_result'])
        x, y, z = home
        if args.case == 'disconnect':
            command('goto', x=x, y=y, z=z-3, speed=0.5)
            wait(lambda: latest.get('target_ned') is None)
            cut_time, count = time.monotonic(), len(statuses)
            relay.close()
            record('rpc_transport_disconnected')
            wait(lambda: latest.get('state') == 'FAILSAFE', 10, allow_failsafe=True)
            assert len(statuses)-count >= 3, 'Status publishing stopped on RPC loss'
            record('PASS', detection_seconds=time.monotonic()-cut_time,
                   reason=latest.get('reason'), statuses_after_disconnect=len(statuses)-count)
        else:
            command('goto', x=x, y=y, z=z-5, speed=1.0)
            wait(lambda: latest.get('target_ned') is None)
            command('goto', x=x+3, y=y, z=z-5, speed=0.5)
            if args.case == 'mission':
                wait(lambda: latest['position_ned'][0]-x >= 0.5, 20)
                original = latest['target_ned']
                assert original is not None
                service('pause')
                wait(lambda: latest.get('state') == 'PAUSED')
                paused = time.monotonic()
                wait(lambda: time.monotonic()-paused >= 3, 6)
                speed = probe.getMultirotorState().kinematics_estimated.linear_velocity.get_length()
                assert speed <= 0.1 and latest['target_ned'] == original
                record('pause_verified', speed=speed, target=original)
                service('resume')
                wait(lambda: latest.get('state') == 'EXECUTING')
                assert latest['target_speed'] == 0.5 and latest['target_ned'] == original
                record('resume_verified', commanded_speed=latest['target_speed'])
            wait(lambda: latest.get('target_ned') is None)
            service('return_home')
            if args.case == 'return_land':
                # Interrupt RETURNING after reaching the recorded landing
                # column, before its automatic 1s stability gate completes.
                wait(lambda: latest.get('state') == 'RETURNING' and
                     math.hypot(latest['position_ned'][0]-x, latest['position_ned'][1]-y) < 0.07, 40)
                record('return_interruption_position', position=latest['position_ned'])
                service('land', required_state='RETURNING')
            wait(lambda: latest.get('state') == 'LANDED')
            s = probe.getMultirotorState()
            p = s.kinematics_estimated.position
            error = math.dist((p.x_val, p.y_val, p.z_val), home)
            assert s.landed_state == airsim.LandedState.Landed and error <= 0.15
            record('PASS', final_home_error=error, states=sorted(set(s['state'] for s in statuses)))
    except Exception as exc:
        record('FAIL', error=str(exc))
        raise
    finally:
        # Stop the node before independent recovery; never mix flight owners.
        import signal
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        try:
            process.wait(timeout=6)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        if relay:
            relay.close()
        s = probe.getMultirotorState()
        if s.landed_state != airsim.LandedState.Landed:
            probe.enableApiControl(True)
            result = probe.landAsync(timeout_sec=60).get()
            record('independent_recovery_land', result=result)
            settled, deadline = None, time.monotonic()+10
            while result is True and time.monotonic() < deadline:
                s = probe.getMultirotorState()
                p, v = s.kinematics_estimated.position, s.kinematics_estimated.linear_velocity
                stable = math.dist((p.x_val, p.y_val, p.z_val), home) <= 0.15 and v.get_length() <= 0.1
                settled = (settled or time.monotonic()) if stable else None
                if settled and time.monotonic()-settled >= 3:
                    record('guarded_recovery_disarm', result=probe.armDisarm(False))
                    # SimpleFlight updates Landed asynchronously after disarm.
                    confirmation_deadline = time.monotonic()+3
                    while time.monotonic() < confirmation_deadline:
                        if probe.getMultirotorState().landed_state == airsim.LandedState.Landed:
                            break
                        time.sleep(0.1)
                    break
                time.sleep(0.2)
            record('recovery_final_state', landed_state=probe.getMultirotorState().landed_state)
        node.destroy_node()
        rclpy.shutdown()
        node_log.close()
        log.close()


if __name__ == '__main__':
    main()
