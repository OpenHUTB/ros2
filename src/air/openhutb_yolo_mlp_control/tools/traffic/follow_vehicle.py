"""Opt-in AirSim pose following of CARLA telemetry; not flight dynamics validation."""

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

from common import (REFERENCE_LIMIT, calibration_offset, follow_phase, following_point,
                    positive, pose_clearance, read_live, follow_step, step_towards,
                    velocity_towards, write_json)
from obstacle_avoidance import LocalAvoidance, SceneQueries


def follow(args):
    # Also support a source checkout; installed copies use the installed package.
    package_root = Path(__file__).resolve().parents[2]
    if (package_root / 'openhutb_yolo_mlp_control' / '__init__.py').is_file():
        sys.path.insert(0, str(package_root))
    from openhutb_yolo_mlp_control.vendor import airsim

    directory = args.data.resolve()
    exit_file = directory / 'follow_exit'
    pause_file = directory / 'follow_pause'
    if exit_file.exists():
        raise RuntimeError('Previous follow_exit exists; remove it only after the old follower exits')
    # Exclusive output also prevents two follower processes owning the drone.
    with (directory / 'drone_follow.csv').open('x', newline='', encoding='utf-8') as stream:
        client = airsim.MultirotorClient(ip=args.host, port=args.port, timeout_value=5)
        if not client.ping():
            raise RuntimeError('AirSim unavailable')
        live = read_live(directory)
        references = json.loads((directory / 'references.json').read_text(encoding='utf-8'))
        if references['map'] != live['map']:
            raise RuntimeError('References and telemetry belong to different maps')
        names = set(client.simListSceneObjects())
        pairs = []
        for reference in references['references']:
            if reference['name'] not in names:
                continue
            p = client.simGetObjectPose(reference['name']).position
            values = [p.x_val, p.y_val, p.z_val]
            if all(math.isfinite(value) for value in values):
                pairs.append((reference['carla'], values))
            if len(pairs) >= REFERENCE_LIMIT:
                break
        offset, residual = calibration_offset(pairs)
        write_json(directory / 'calibration.json',
                   {'map': live['map'], 'offset': offset, 'references': len(pairs),
                    'max_residual_m': residual})
        # Calibration can take time; refresh before selecting a target.
        live = read_live(directory)
        pose = client.simGetVehiclePose()
        vehicles = live['vehicles']
        if not vehicles:
            raise RuntimeError('No managed vehicles in telemetry')
        if args.target_id is None:
            def cost(v):
                return math.hypot(v['x'] + offset[0] - pose.position.x_val,
                                  v['y'] + offset[1] - pose.position.y_val)
            moving = [v for v in vehicles if math.hypot(v['vx'], v['vy']) > 1]
            target_id = min(moving or vehicles, key=cost)['actor_id']
        else:
            target_id = args.target_id
        if not any(v['actor_id'] == target_id for v in vehicles):
            raise RuntimeError('Requested target ID is absent')
        control = airsim.MultirotorClient(ip=args.host, port=args.port, timeout_value=5)
        initial_api_control = control.isApiControlEnabled()
        initial_collision_timestamp = client.simGetCollisionInfo().time_stamp
        writer = csv.writer(stream)
        writer.writerow(['wall_time', 'target_id', 'drone_x_ned', 'drone_y_ned',
                         'drone_z_ned', 'desired_x_ned', 'desired_y_ned', 'desired_z_ned',
                         'horizontal_error_m', 'observed_speed_mps',
                         'observed_horizontal_speed_mps', 'phase', 'collision',
                         'navigation_x_ned', 'navigation_y_ned', 'navigation_z_ned',
                         'command_vx_ned_mps', 'command_vy_ned_mps',
                         'command_vz_ned_mps', 'command_speed_mps'])
        start = previous = time.monotonic()
        previous_pose = None
        previous_pose_time = None
        held_position = None
        previous_command = [0.0, 0.0, 0.0]
        reason = 'duration'
        queries = None
        avoidance = None
        requested_control_mode = getattr(args, 'control_mode', 'auto')
        flight_state = control.getMultirotorState()
        flight_ready = bool(getattr(flight_state, 'ready', False))
        control_mode = ('velocity' if flight_ready else 'pose') \
            if requested_control_mode == 'auto' else requested_control_mode
        print(f'Target {target_id}; backend={control_mode}; '
              f'requested={requested_control_mode}; flight_ready={flight_ready}; '
              f'calibration residual={residual:.6f} m', flush=True)
        control_started = False
        try:
            if (control_mode == 'velocity'
                    and flight_state.landed_state != airsim.LandedState.Flying):
                raise RuntimeError('Follow requires the drone to be airborne; '
                                   'the follower will not take off automatically')
            if control_mode == 'velocity':
                if not flight_ready:
                    raise RuntimeError(
                        'AirSim native velocity control is unavailable '
                        '(flight controller ready=false); use --control-mode auto or pose')
                if args.ignore_collision:
                    raise RuntimeError('--ignore-collision applies only to pose control')
            if control_mode == 'pose':
                p = client.simGetVehiclePose().position
                target = next(v for v in vehicles if v['actor_id'] == target_id)
                pose_clearance([p.x_val, p.y_val, p.z_val], target, offset)
            if getattr(args, 'avoid_obstacles', False):
                queries = SceneQueries(args.obstacle_host or args.host,
                                       args.obstacle_port, offset)
                avoidance = LocalAvoidance(queries, args.obstacle_margin)
            control_started = True
            control.enableApiControl(True)
            control.cancelLastTask()
            while args.duration == 0 or time.monotonic() - start < args.duration:
                if exit_file.exists():
                    reason = 'manual_exit'
                    break
                live = read_live(directory)
                if live['map'] != references['map']:
                    raise RuntimeError('Map changed')
                vehicle = next((v for v in live['vehicles'] if v['actor_id'] == target_id), None)
                if vehicle is None:
                    raise RuntimeError('Target disappeared')
                collision = client.simGetCollisionInfo()
                if (not args.ignore_collision and collision.has_collided
                        and collision.time_stamp > initial_collision_timestamp):
                    reason = 'collision'
                    raise RuntimeError(f'Collision with {collision.object_name}; ending follow')
                pose = client.simGetVehiclePose()
                p = pose.position
                current = [p.x_val, p.y_val, p.z_val]
                pose_time = time.monotonic()
                observed_speed = (math.dist(current, previous_pose)
                                  / max(pose_time - previous_pose_time, 1e-9)
                                  if previous_pose is not None else 0.0)
                observed_horizontal_speed = (
                    math.dist(current[:2], previous_pose[:2])
                    / max(pose_time - previous_pose_time, 1e-9)
                    if previous_pose is not None else 0.0)
                previous_pose = current
                previous_pose_time = pose_time
                desired = following_point(vehicle, offset, args.distance, args.height)
                phase = follow_phase(current, desired)
                paused = pause_file.exists()
                if paused:
                    if held_position is None:
                        held_position = current[:]
                    desired = held_position
                    phase = 'hold'
                    target_velocity = [0.0, 0.0, 0.0]
                else:
                    held_position = None
                    # CARLA x/y axes align with the calibrated AirSim axes;
                    # NED reverses the vertical velocity sign.
                    target_velocity = ([vehicle['vx'], vehicle['vy'], -vehicle['vz']]
                                       if control_mode == 'velocity'
                                       else [0.0, 0.0, 0.0])
                navigation = desired
                if avoidance is not None and phase != 'hold':
                    navigation, avoidance_phase = avoidance.select(current, desired)
                    if avoidance_phase != 'follow':
                        phase = avoidance_phase
                    # Queries take time; reject a plan if the vehicle moved away
                    # from its checked starting position while querying the scene.
                    checked = client.simGetVehiclePose().position
                    if math.dist(current, [checked.x_val, checked.y_val, checked.z_val]) > .5:
                        raise RuntimeError('Vehicle moved during obstacle query; ending follow')
                now = time.monotonic()
                dt = min(max(now - previous, 0), 0.25)
                previous = now
                if control_mode == 'velocity':
                    if phase == 'blocked':
                        command = [0.0, 0.0, 0.0]
                    else:
                        command = velocity_towards(
                            current, navigation, args.speed,
                            previous=previous_command, dt=dt,
                            target_velocity=(target_velocity
                                             if phase in ('climb', 'follow')
                                             else [0.0, 0.0, 0.0]))
                    previous_command = command
                    command_speed = math.sqrt(sum(value * value for value in command))
                    control.moveByVelocityAsync(
                        *command, 0.2, airsim.DrivetrainType.MaxDegreeOfFreedom,
                        airsim.YawMode(True, 0.0)).join()
                else:
                    position = (step_towards(current, navigation,
                                             min(args.speed * dt, avoidance.horizon))
                                if avoidance else follow_step(current, navigation, args.speed * dt))
                    command = [(b - a) / max(dt, 1e-9)
                               for a, b in zip(current, position)]
                    command_speed = math.sqrt(sum(value * value for value in command))
                    # Legacy geometric mode: this is a pose update, not a velocity command.
                    control.simSetVehiclePose(
                        airsim.Pose(airsim.Vector3r(*position), pose.orientation),
                        args.ignore_collision)
                error = math.hypot(current[0] - desired[0], current[1] - desired[1])
                writer.writerow([time.time(), target_id, *current, *desired,
                                 error, observed_speed, observed_horizontal_speed,
                                 phase, collision.has_collided
                                 and collision.time_stamp > initial_collision_timestamp,
                                 *navigation, *command, command_speed])
                stream.flush()
                write_json(directory / 'follow_status.json',
                           {'running': True, 'target_id': target_id, 'phase': phase,
                            'backend': control_mode, 'heading_mode': 'preserve_current',
                            'requested_control_mode': requested_control_mode,
                            'control_mode': control_mode,
                            'horizontal_error_m': error,
                            'command_speed_mps': command_speed,
                            'command_speed_limit_mps': args.speed,
                            'observed_speed_mps': observed_speed,
                            'observed_horizontal_speed_mps': observed_horizontal_speed,
                            'ignore_collision': args.ignore_collision,
                            'obstacle_check': 'carla_scene_rays' if avoidance else 'disabled',
                            'time': time.time()})
                time.sleep(0.05 if control_mode == 'velocity' else 0.2)
        except KeyboardInterrupt:
            reason = 'keyboard_interrupt'
        except Exception:
            if reason != 'collision':
                reason = 'error'
            raise
        finally:
            if queries is not None:
                try:
                    queries.close()
                except Exception as exc:
                    print(f'Obstacle helper cleanup failed: {exc}', flush=True)
            released = True
            if control_started and control_mode == 'velocity':
                try:
                    control.moveByVelocityAsync(
                        0.0, 0.0, 0.0, 0.5, airsim.DrivetrainType.MaxDegreeOfFreedom,
                        airsim.YawMode(True, 0.0)).join()
                except Exception as exc:
                    released = False
                    print(f'Velocity stop failed: {exc}', flush=True)
            if control_started:
                for operation in (control.cancelLastTask,
                                  lambda: control.enableApiControl(initial_api_control)):
                    try:
                        operation()
                    except Exception as exc:
                        released = False
                        print(f'Control release failed: {exc}', flush=True)
            final_api_control = None
            try:
                final_api_control = control.isApiControlEnabled()
                released = released and final_api_control == initial_api_control
            except Exception as exc:
                released = False
                print(f'Control state verification failed: {exc}', flush=True)
            write_json(directory / 'follow_status.json',
                       {'running': False, 'target_id': target_id, 'reason': reason,
                        'control_released': released,
                        'initial_api_control': initial_api_control,
                        'final_api_control': final_api_control,
                        'api_control_restored': released, 'time': time.time()})
            print(f'Follow ended; control_released={released}; '
                  f'api_control_restored={released}; api_enabled={final_api_control}. '
                  'No further pose updates.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--host', required=True, help='Verified AirSim host address')
    parser.add_argument('--port', type=int, default=41451)
    parser.add_argument('--target-id', type=int)
    parser.add_argument('--height', type=positive, default=25)
    parser.add_argument('--distance', type=positive, default=12)
    parser.add_argument('--speed', type=positive, default=12,
                        help='Speed limit for native control; maximum pose-step rate in fallback mode')
    parser.add_argument('--control-mode', choices=('auto', 'velocity', 'pose'), default='auto',
                        help='Auto-select native velocity or pose fallback (default)')
    parser.add_argument('--ignore-collision', action='store_true',
                        help='Allow pose updates through collisions and disable collision-report exits')
    parser.add_argument('--avoid-obstacles', action='store_true',
                        help='Check CARLA scene rays and try short local detours')
    parser.add_argument('--obstacle-host', help='Read-only CARLA ray query server; defaults to --host')
    parser.add_argument('--obstacle-port', type=int, default=41452)
    parser.add_argument('--obstacle-margin', type=positive, default=1.5,
                        help='Half-width of sampled obstacle corridor in metres')
    parser.add_argument('--duration', type=float, default=90, help='Seconds; 0 until manual exit')
    parser.add_argument('--enable-follow-control', '--enable-pose-control',
                        dest='enable_follow_control', action='store_true',
                        help='Explicitly enable simulated vehicle following')
    args = parser.parse_args()
    if not args.enable_follow_control:
        parser.error('Following requires --enable-follow-control; this is not native flight control')
    if args.control_mode == 'velocity' and args.ignore_collision:
        parser.error('--ignore-collision applies only to pose control; velocity mode uses physics')
    if args.avoid_obstacles and args.ignore_collision:
        parser.error('--avoid-obstacles requires collision checks enabled')
    if args.ignore_collision:
        print('WARNING: --ignore-collision may move the simulated vehicle through scenery.',
              file=sys.stderr, flush=True)
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be finite and nonnegative')
    follow(args)


if __name__ == '__main__':
    main()
