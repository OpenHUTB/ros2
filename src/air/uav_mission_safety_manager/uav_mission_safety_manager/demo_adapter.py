"""Local kinematic demonstration, NOT an AirSim or flight dynamics simulator."""
import math
import time
from concurrent.futures import Future


def ready(value=None):
    future = Future()
    future.set_result(value)
    return future


class DemoAdapter:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.last = clock()
        self.position = (0., 0., 0.)
        self.target = None
        self.speed = 0.
        self.landed = True
        self.landing = False
        self.velocity = (0., 0., 0.)

    def _advance(self):
        now = self.clock()
        elapsed, self.last = max(0., now - self.last), now
        self.velocity = (0., 0., 0.)
        if self.target is not None:
            error = tuple(b - a for a, b in zip(self.position, self.target))
            distance = math.sqrt(sum(v * v for v in error))
            if distance <= 1e-12:
                self.target = None
                if self.landing:
                    self.landed, self.landing = True, False
                return
            fraction = min(1., self.speed * elapsed / max(distance, 1e-12))
            self.position = tuple(a + fraction * b for a, b in zip(self.position, error))
            if fraction < 1.:
                self.velocity = tuple(self.speed * v / distance for v in error)
            else:
                self.target = None
                if self.landing:
                    self.landed = True
                    self.landing = False

    def submit(self, command, **data):
        self._advance()
        if command == 'connect':
            return self.sample()
        if command in ('arm', 'disarm'):
            return ready()
        if command == 'takeoff':
            self.landed = False
            self.target, self.speed = (0., 0., -2.), 2.
        elif command == 'goto':
            self.target, self.speed = data['target'], data['speed']
        elif command == 'hover':
            self.target = None
        elif command == 'land':
            self.target = (self.position[0], self.position[1], 0.)
            self.speed, self.landing = 2., True
        elif command == 'velocity':
            self.target = tuple(p + v * data['duration']
                                for p, v in zip(self.position, data['velocity']))
            self.speed = math.sqrt(sum(v * v for v in data['velocity']))
        else:
            raise ValueError('Unknown demo command: ' + command)
        return ready(ready(True))

    def sample(self):
        self._advance()
        return ready({'position': self.position, 'velocity': self.velocity,
                      'landed': self.landed})

    def close(self):
        pass
