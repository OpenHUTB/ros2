"""Optional CARLA-only scene driver. Run with the simulator-matched API."""
import argparse
import math
import time
import uuid

from scenario import DURATION, output_directory, phase, read_json, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, help='Shared directory outside Git')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--x', type=float, default=-54.64)
    parser.add_argument('--y', type=float, default=24.47101)
    parser.add_argument('--z', type=float, default=.6)
    parser.add_argument('--yaw-degrees', type=float, default=.159198)
    parser.add_argument('--separation', type=float, default=5.0)
    parser.add_argument('--wait-timeout', type=float, default=120)
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.x, args.y, args.z,
            args.yaw_degrees, args.separation, args.wait_timeout)):
        parser.error('Scene coordinates and limits must be finite')
    if args.separation <= 0 or args.wait_timeout <= 0:
        parser.error('Separation and wait timeout must be positive')
    root = output_directory(args.output)
    if any(root.iterdir()):
        parser.error('Use a new empty output directory for each run')
    import carla
    client = carla.Client(args.host, args.port)
    client.set_timeout(5)
    if client.get_client_version() != client.get_server_version():
        raise RuntimeError('Use the Python API matching the simulator server')
    world = client.get_world()
    if not world.get_map().name.endswith('Town10HD'):
        raise RuntimeError('Default scene is calibrated for Town10HD only')
    if world.get_settings().synchronous_mode:
        raise RuntimeError('Requires an asynchronous world; never ticks the world')
    actors, samples = [], []
    run_id = uuid.uuid4().hex
    started = time.monotonic()
    try:
        for name, color, x in [('A', '0,0,255', args.x),
                               ('B', '255,0,0', args.x + args.separation)]:
            bp = world.get_blueprint_library().find('vehicle.tesla.model3')
            bp.set_attribute('role_name', 'yolo_validation_' + run_id + '_' + name)
            bp.set_attribute('color', color)
            transform = carla.Transform(carla.Location(x=x, y=args.y,
                z=args.z if name == 'A' else -100),
                carla.Rotation(yaw=args.yaw_degrees))
            actor = world.try_spawn_actor(bp, transform)
            if actor is None:
                raise RuntimeError('Spawn occupied for ' + name)
            actors.append((name, actor, transform))
            actor.set_simulate_physics(False)
        write_json(root / 'scene_ready.json', {'run_id': run_id,
            'actors': {name: a.id for name, a, _ in actors}, 'arguments': vars(args)})
        while not (root / 'stop.json').exists():
            if time.monotonic() - started > args.wait_timeout + DURATION + 20:
                raise TimeoutError('Observer did not finish; cleaning owned vehicles')
            geometry = read_json(root / 'geometry.json') if (root / 'geometry.json').exists() else None
            ready = read_json(root / 'ready.json') if (root / 'ready.json').exists() else None
            if ready and ready['run_id'] != run_id:
                raise RuntimeError('Observer run ID mismatch')
            elapsed = time.time() - ready['epoch'] if ready else 0
            state = phase(elapsed)
            sample = {'t': time.time(), 'elapsed': elapsed}
            for name, actor, transform in actors:
                hidden = state['hide_a'] if name == 'A' else state['hide_b']
                transform.location.x = args.x + state['offset_a'] if name == 'A' else args.x
                transform.location.y = args.y
                if name == 'B' and geometry:
                    yaw = geometry['yaw']
                    transform.location.x -= args.separation * math.sin(yaw)
                    transform.location.y += args.separation * math.cos(yaw)
                transform.location.z = -100 if hidden else args.z
                actor.set_transform(transform)
                sample[name] = {'hidden': hidden, 'x': transform.location.x,
                    'y': transform.location.y, 'z': transform.location.z}
            samples.append(sample)
            time.sleep(.05)
    except KeyboardInterrupt:
        pass
    finally:
        errors = []
        for name, actor, _ in actors:
            try:
                if not actor.destroy():
                    errors.append(name)
            except Exception as exc:
                errors.append(str(exc))
        write_json(root / 'scene.json', samples)
        write_json(root / 'scene_cleaned.json', {'errors': errors})
        print('Owned test vehicles cleaned; errors:', errors)


if __name__ == '__main__':
    main()
