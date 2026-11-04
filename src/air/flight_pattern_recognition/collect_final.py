"""Fresh final-test flights, adapted from the existing OpenHUTB state-prediction collector.
Uses the same verified flight bounds and landing routine.
"""
import argparse
import csv
import hashlib
import json
import math
import time
from pathlib import Path

import airsim
import numpy as np

FIELDS = ['timestamp_ns', 'px', 'py', 'pz', 'vx', 'vy', 'vz', 'ax', 'ay', 'az',
          'qw', 'qx', 'qy', 'qz', 'wx', 'wy', 'wz', 'cmd_vx', 'cmd_vy', 'cmd_vz',
          'collision', 'wall_elapsed_s']
FAMILIES = ['line', 'circle', 'figure_eight', 'climb', 'stop_go']


def pack_state(state):
    k = state.kinematics_estimated
    return [state.timestamp] + [getattr(getattr(k, f), a + '_val')
        for f in ['position', 'linear_velocity', 'linear_acceleration'] for a in 'xyz'] + [
        getattr(k.orientation, a + '_val') for a in 'wxyz'] + [
        getattr(k.angular_velocity, a + '_val') for a in 'xyz']


def trajectory(family, t, amplitude, omega, phase):
    s, c = math.sin(omega*t+phase), math.cos(omega*t+phase)
    p, v = np.zeros(3), np.zeros(3)
    if family in ('line', 'stop_go'):
        p[0], v[0] = amplitude*s, amplitude*omega*c
        if family == 'stop_go':
            # Smooth dwell/reversal pattern, with a matching analytic derivative.
            p[0] = amplitude*math.tanh(2*s)
            v[0] = amplitude*2*omega*c/(math.cosh(2*s)**2)
    elif family == 'circle':
        p[:2] = amplitude*np.array([s,c])
        v[:2] = amplitude*omega*np.array([c,-s])
    elif family == 'figure_eight':
        p[:2] = amplitude*np.array([s,0.5*math.sin(2*(omega*t+phase))])
        v[:2] = amplitude*omega*np.array([c,math.cos(2*(omega*t+phase))])
    else:
        p[[0,2]] = amplitude*np.array([0.4*s,0.55*c])
        v[[0,2]] = amplitude*omega*np.array([0.4*c,-0.55*s])
    p[2] -= 8.0
    return p, v


def wait_landed(client, vehicle, ground_z, timeout=45):
    # Descend over the clear central flight area before the landing controller.
    client.moveToPositionAsync(0,0,-8,2,timeout_sec=20,vehicle_name=vehicle).join()
    client.moveToZAsync(-2,1,timeout_sec=20,vehicle_name=vehicle).join()
    client.landAsync(timeout_sec=timeout, vehicle_name=vehicle).join()
    deadline = time.monotonic()+10
    stable_since = None
    while time.monotonic() < deadline:
        state = client.getMultirotorState(vehicle_name=vehicle)
        packed = pack_state(state)
        p, velocity = np.array(packed[1:4]), np.array(packed[4:7])
        # SimpleFlight's landed flag may remain Flying while the motors are armed.
        # Stop motors only after stable contact at the known takeoff elevation.
        collision=client.simGetCollisionInfo(vehicle_name=vehicle)
        ground_contact = (abs(p[2]-ground_z)<0.05 and np.linalg.norm(velocity)<0.02
            and collision.object_name.startswith('Ground')
            and collision.normal.z_val < -0.8)
        if ground_contact:
            if stable_since is None: stable_since=time.monotonic()
        else: stable_since=None
        if state.landed_state == airsim.LandedState.Landed or (stable_since is not None and time.monotonic()-stable_since>1.0):
            client.armDisarm(False, vehicle_name=vehicle)
            client.enableApiControl(False, vehicle_name=vehicle)
            time.sleep(0.5)
            return client.getMultirotorState(vehicle_name=vehicle).landed_state == airsim.LandedState.Landed
        time.sleep(0.2)
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='192.168.239.1')
    parser.add_argument('--vehicle', default='PredictionDrone')
    parser.add_argument('--output', type=Path, default=Path('data/final_raw'))
    parser.add_argument('--start', type=int, default=300)
    parser.add_argument('--episodes', type=int, default=15)
    parser.add_argument('--duration', type=float, default=18)
    parser.add_argument('--rate', type=float, default=20)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    client = airsim.MultirotorClient(ip=args.host, timeout_value=10)
    client.confirmConnection()
    client.simPause(False)
    initial=client.getMultirotorState(vehicle_name=args.vehicle)
    if initial.landed_state != airsim.LandedState.Landed:
        raise RuntimeError('Start collection only with the simulated drone on the ground')
    ground_z=initial.kinematics_estimated.position.z_val
    client.enableApiControl(True, vehicle_name=args.vehicle)
    client.armDisarm(True, vehicle_name=args.vehicle)
    try:
        client.takeoffAsync(timeout_sec=20, vehicle_name=args.vehicle).join()
        for episode in range(args.start, args.start+args.episodes):
            name = 'episode_%03d' % episode
            path = args.output / (name+'.csv')
            if path.exists():
                raise FileExistsError('Refusing to overwrite '+str(path))
            family = FAMILIES[episode % len(FAMILIES)]
            repeat = episode // len(FAMILIES)
            split = 'final'
            seed = 20261002+episode
            rng = np.random.default_rng(seed)
            amplitude, omega, phase = float(rng.uniform(0.8,1.6)), float(rng.uniform(0.35,0.65)), float(rng.uniform(-math.pi,math.pi))
            client.moveToPositionAsync(0,0,-8,2,timeout_sec=20,vehicle_name=args.vehicle).join()
            # AirSim Async completion can precede settling after a fast turn.
            # Confirm a stable start instead of loosening the flight acceptance bounds.
            settle_deadline = time.monotonic() + 15
            while time.monotonic() < settle_deadline:
                current = np.array(pack_state(client.getMultirotorState(vehicle_name=args.vehicle))[1:7])
                error = np.array([0., 0., -8.]) - current[:3]
                if np.linalg.norm(error) < .5 and np.linalg.norm(current[3:]) < .2:
                    break
                velocity = np.clip(.8 * error - .3 * current[3:], -.8, .8)
                client.moveByVelocityAsync(*velocity.tolist(), duration=.25, vehicle_name=args.vehicle)
                time.sleep(.1)
            # Verify actual altitude rather than trusting Async.join()'s return value.
            state = client.getMultirotorState(vehicle_name=args.vehicle)
            pos = np.array(pack_state(state)[1:4])
            if np.linalg.norm(pos-np.array([0,0,-8])) > 1.2:
                raise RuntimeError('Failed to reach episode start: '+str(pos))
            start_ns = state.timestamp
            wall_start = time.monotonic()
            deadline = wall_start
            rows = []
            while True:
                state = client.getMultirotorState(vehicle_name=args.vehicle)
                elapsed = (state.timestamp-start_ns)*1e-9
                if elapsed >= args.duration:
                    break
                if time.monotonic()-wall_start > args.duration*3+15:
                    raise TimeoutError('Simulation time stopped advancing')
                packed = pack_state(state)
                p = np.array(packed[1:4])
                target, feedforward = trajectory(family,elapsed,amplitude,omega,phase)
                command = np.clip(feedforward+0.65*(target-p),-2.0,2.0)
                client.moveByVelocityAsync(*command.tolist(),duration=0.3,vehicle_name=args.vehicle)
                collision = client.simGetCollisionInfo(vehicle_name=args.vehicle)
                hit = bool(collision.has_collided and collision.time_stamp > start_ns)
                rows.append(packed+command.tolist()+[int(hit),time.monotonic()-wall_start])
                if hit or abs(p[0]) > 6 or abs(p[1]) > 6 or not -12 < p[2] < -4:
                    raise RuntimeError('Collision or flight-boundary violation; episode rejected')
                deadline += 1/args.rate
                time.sleep(max(0,deadline-time.monotonic()))
            with path.open('w',newline='') as f:
                writer=csv.writer(f); writer.writerow(FIELDS); writer.writerows(rows)
            stamps = np.array([row[0] for row in rows],dtype=np.int64)
            dt = np.diff(stamps)*1e-9
            meta = dict(episode_id=name, family=family, split=split, seed=seed,
                amplitude=amplitude, omega=omega, phase=phase, samples=len(rows),
                duration_sim_s=float((stamps[-1]-stamps[0])*1e-9),
                median_dt_s=float(np.median(dt)),max_dt_s=float(dt.max()),
                timestamps_strictly_increasing=bool((dt>0).all()),
                collision_count=sum(row[-2] for row in rows),
                coordinate_frame='NED world; quaternion body FRD to NED; angular velocity body FRD',
                source='Windows Blocks / AirSim RPC getMultirotorState',
                airsim_python_version=airsim.__version__,
                sha256=hashlib.sha256(path.read_bytes()).hexdigest())
            path.with_suffix('.json').write_text(json.dumps(meta,indent=2))
            print(json.dumps(meta),flush=True)
            if episode == args.start:
                img=client.simGetImages([airsim.ImageRequest('0',airsim.ImageType.Scene,False,True)],vehicle_name=args.vehicle)[0]
                if img.width:
                    (args.output/(name+'.png')).write_bytes(bytes(img.image_data_uint8))
    finally:
        try:
            client.simPause(False)
            landed = wait_landed(client,args.vehicle,ground_z)
            print('LANDED_VERIFIED',landed,flush=True)
            if not landed:
                client.hoverAsync(vehicle_name=args.vehicle)
                raise RuntimeError('Landing not confirmed; inspect simulator')
        finally:
            client.enableApiControl(False,vehicle_name=args.vehicle)


if __name__ == '__main__':
    main()
