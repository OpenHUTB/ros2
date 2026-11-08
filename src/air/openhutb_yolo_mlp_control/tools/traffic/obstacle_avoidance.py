"""Local detours using CARLA scene ray queries, not sensor-based avoidance."""

import argparse
import json
import math
import socket
import sys
import time


def _add(a, b):
    return [x + y for x, y in zip(a, b)]


def _scaled(a, scale):
    return [x * scale for x in a]


def corridor_rays(start, end, margin):
    """Nine parallel samples across the square swept corridor (not a solid sweep)."""
    delta = [b - a for a, b in zip(start, end)]
    length = math.sqrt(sum(x*x for x in delta))
    if length < 1e-6:
        return []
    direction = _scaled(delta, 1 / length)
    horizontal = math.hypot(direction[0], direction[1])
    side = ([-direction[1]/horizontal, direction[0]/horizontal, 0]
            if horizontal > 1e-6 else [1, 0, 0])
    up = [direction[1]*side[2] - direction[2]*side[1],
          direction[2]*side[0] - direction[0]*side[2],
          direction[0]*side[1] - direction[1]*side[0]]
    rays = []
    for i in (-1, 0, 1):
        for j in (-1, 0, 1):
            shift = _add(_scaled(side, i*margin), _scaled(up, j*margin))
            rays.append([_add(start, shift), _add(end, shift)])
    return rays


class SceneQueries:
    """Read-only geometry queries served by simulator-matched CARLA Python."""

    def __init__(self, host, port, offset, timeout=5):
        self.offset = offset
        deadline = time.monotonic() + timeout
        while True:
            try:
                self.socket = socket.create_connection((host, port), timeout=timeout)
                break
            except OSError:
                if time.monotonic() >= deadline:
                    raise RuntimeError('Obstacle query server unavailable; follow not started')
                time.sleep(.1)
        self.stream = self.socket.makefile('rw', encoding='utf-8')
        try:
            ready = json.loads(self.stream.readline())
            if not ready.get('ready'):
                raise RuntimeError(ready.get('error', 'Scene query helper unavailable'))
        except Exception:
            self.close()
            raise

    def clear(self, segments):
        rays = []
        for segment in segments:
            rays.append([[p[0]-self.offset[0], p[1]-self.offset[1], self.offset[2]-p[2]]
                         for p in segment])
        self.stream.write(json.dumps({'rays': rays}) + '\n')
        self.stream.flush()
        try:
            reply = json.loads(self.stream.readline())
        except (OSError, ValueError) as exc:
            raise RuntimeError('Scene query failed; stopping follow') from exc
        if 'error' in reply:
            raise RuntimeError(reply['error'])
        return reply['clear']

    def close(self):
        self.stream.close()
        self.socket.close()


class LocalAvoidance:
    def __init__(self, queries, margin=1.5, horizon=20):
        self.queries = queries
        self.margin = margin
        self.horizon = horizon
        self.waypoint = None

    def clear(self, start, end):
        rays = corridor_rays(start, end, self.margin)
        return not rays or self.queries.clear(rays)

    def select(self, current, desired):
        distance = math.dist(current, desired)
        if distance < 1e-6:
            return current[:], 'follow'
        direction = _scaled([b-a for a, b in zip(current, desired)], 1/distance)
        local_goal = _add(current, _scaled(direction, min(distance, self.horizon)))
        if self.waypoint is not None:
            if math.dist(current, self.waypoint) < 1:
                self.waypoint = None
            elif self.clear(current, self.waypoint):
                return self.waypoint[:], 'avoid'
            else:
                self.waypoint = None
        if self.clear(current, local_goal):
            return desired[:], 'follow'
        horizontal = math.hypot(direction[0], direction[1])
        side = ([-direction[1]/horizontal, direction[0]/horizontal, 0]
                if horizontal > 1e-6 else [1, 0, 0])
        forward = _add(current, _scaled(direction, min(distance, 6)))
        candidates = [_add(forward, _scaled(side, amount)) for amount in (8, -8, 16, -16)]
        # NED negative Z is upward. Never choose a downward detour.
        candidates += [_add(current, [0, 0, -height]) for height in (8, 16)]
        candidates.sort(key=lambda p: math.dist(current, p) + math.dist(p, local_goal))
        for waypoint in candidates:
            if self.clear(current, waypoint) and self.clear(waypoint, local_goal):
                self.waypoint = waypoint
                return waypoint[:], 'avoid'
        return current[:], 'blocked'


def serve(host, port, stream):
    import carla
    client = carla.Client(host, port)
    client.set_timeout(3)
    if client.get_client_version() != client.get_server_version():
        raise RuntimeError('CARLA ray queries require a simulator-matched Python API')
    world = client.get_world()
    world_id = world.id
    def reply(value):
        stream.write(json.dumps(value) + '\n')
        stream.flush()

    reply({'ready': True})
    for line in stream:
        try:
            request = json.loads(line)
            if client.get_world().id != world_id:
                raise RuntimeError('CARLA world changed')
            clear = True
            for start, end in request['rays']:
                a, b = carla.Location(*start), carla.Location(*end)
                for hit in world.cast_ray(a, b):
                    # AirSim's own body can be reported at the ray origin as NONE.
                    if hit.label == carla.CityObjectLabel.NONE and hit.location.distance(a) < .05:
                        continue
                    clear = False
                    break
                if not clear:
                    break
            reply({'clear': clear})
        except Exception as exc:
            reply({'error': str(exc)})


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--server', action='store_true', required=True)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--listen', default='127.0.0.1')
    parser.add_argument('--query-port', type=int, default=41452)
    options = parser.parse_args()
    try:
        with socket.create_server((options.listen, options.query_port)) as listener:
            listener.settimeout(60)
            connection, _ = listener.accept()
            with connection, connection.makefile('rw', encoding='utf-8') as stream:
                serve(options.host, options.port, stream)
    except Exception as exc:
        print(json.dumps({'error': str(exc)}), flush=True)
        sys.exit(1)
