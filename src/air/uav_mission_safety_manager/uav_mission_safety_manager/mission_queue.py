"""Validated, bounded waypoint sequences; independent of ROS and AirSim."""
import math


class MissionQueue:
    MAX_WAYPOINTS = 100

    def __init__(self):
        self.waypoints = ()
        self.completed = 0
        self.state = 'IDLE'

    @property
    def active(self):
        return self.state in ('RUNNING', 'PAUSED')

    def load(self, waypoints, home_z, max_altitude):
        if (isinstance(home_z, bool) or isinstance(max_altitude, bool) or
                not isinstance(home_z, (int, float)) or
                not isinstance(max_altitude, (int, float))):
            raise ValueError('home_z and max_altitude must be numbers')
        try:
            home_z = float(home_z)
            max_altitude = float(max_altitude)
        except OverflowError as exc:
            raise ValueError('home_z and max_altitude must be finite') from exc
        if (not math.isfinite(home_z) or not math.isfinite(max_altitude) or
                max_altitude <= 0):
            raise ValueError('home_z must be finite and max_altitude must be positive')
        if not isinstance(waypoints, list) or not 1 <= len(waypoints) <= self.MAX_WAYPOINTS:
            raise ValueError('waypoints must be a list of 1..100 NED waypoints')
        validated = []
        for point in waypoints:
            if not isinstance(point, dict) or set(point) - {'x', 'y', 'z', 'speed'}:
                raise ValueError('Each waypoint requires x/y/z and optional speed')
            values = [point.get(key) for key in ('x', 'y', 'z')]
            values.append(point.get('speed', 3.0))
            if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in values):
                raise ValueError('Waypoint coordinates and speed must be numbers')
            try:
                x, y, z, speed = map(float, values)
            except OverflowError as exc:
                raise ValueError('Waypoint numbers must be finite') from exc
            if (not all(math.isfinite(v) for v in (x, y, z, speed)) or speed <= 0 or
                    not home_z - max_altitude <= z < home_z):
                raise ValueError('Invalid NED waypoint altitude or speed')
            validated.append(((x, y, z), speed))
        self.waypoints = tuple(validated)
        self.completed = 0
        self.state = 'RUNNING'

    def stop(self, state):
        if self.active:
            self.state = state

    def status(self):
        return {'state': self.state, 'total': len(self.waypoints),
                'completed': self.completed,
                'current_index': self.completed if self.active else None,
                'remaining': len(self.waypoints) - self.completed if self.active else 0}
