"""ROS-independent asynchronous flight controller. Call tick from a ROS timer."""
import math
import time

from .state_machine import SafetyMachine, State
from .mission_queue import MissionQueue


class FlightController:
    ACTIVE = (State.TAKEOFF, State.EXECUTING, State.PAUSED,
              State.RETURNING, State.LANDING)

    def __init__(self, adapter, max_altitude=50.0, telemetry_timeout=2.0,
                 clock=time.monotonic, operation_timeout=120.0):
        self.adapter, self.clock = adapter, clock
        self.machine = SafetyMachine(max_altitude, telemetry_timeout)
        self.operation_timeout = operation_timeout
        self.pending = self.sample = None
        self.last_telemetry = clock()
        self.home = self.position = self.target = None
        self.started = clock()
        self.flight_started = False
        self.connected = False
        self.last_error = ''
        self.revision = self.sample_revision = 0
        self.landing_result = None
        self.land_succeeded = self.disarmed = False
        self.settled_since = None
        self.target_speed = None
        self.return_settled_since = None
        self.return_precision = False
        self.takeoff_result = None
        self.takeoff_rpc_result = None
        self.mission_queue = MissionQueue()

    def _dispatch_waypoint(self):
        self.target, self.target_speed = self.mission_queue.waypoints[self.mission_queue.completed]
        self.started = self.clock()
        self._submit('goto', target=self.target, speed=self.target_speed)

    def _submit(self, command, **data):
        self.revision += 1
        self.pending = (command, self.adapter.submit(command, **data))

    def _begin_landing(self):
        self.landing_result = None
        self.land_succeeded = self.disarmed = False
        self.settled_since = None
        self.started = self.clock()
        self._submit('land')

    def request(self, event, **data):
        # A completed dispatch must not reject a service just because the
        # timer has not consumed its Future yet. This permits interruption
        # without adding gaps between precision velocity corrections.
        self._finish_dispatch(self.clock())
        s = self.machine.state
        if self.pending:
            raise ValueError('RPC dispatch pending; retry after it completes')
        allowed = {'connect': (State.IDLE,), 'armed': (State.ARMING,),
                   'airborne': (State.TAKEOFF,), 'goto': (State.EXECUTING,),
                   'mission': (State.EXECUTING,),
                   'cancel': (State.EXECUTING, State.PAUSED),
                   'pause': (State.EXECUTING,), 'resume': (State.PAUSED,),
                   'return': (State.EXECUTING, State.PAUSED),
                   'land': self.ACTIVE + (State.FAILSAFE,), 'reset': (State.LANDED,)}
        if event not in allowed or s not in allowed[event]:
            raise ValueError('Invalid event {} in {}'.format(event, s.value))
        if event == 'reset':
            self.mission_queue = MissionQueue()
            self.machine.transition('reset')
            self.target = None
            self.target_speed = None
            self.takeoff_result = self.takeoff_rpc_result = None
            self.flight_started = False
            return
        if event == 'connect':
            self.machine.transition('connect')
            self._submit('connect')
        elif event == 'armed':
            self._submit('arm')
        elif event == 'airborne':
            if self.flight_started:
                raise ValueError('Takeoff already dispatched')
            self.flight_started = True
            self.started = self.clock()
            self._submit('takeoff')
        elif event == 'mission':
            if self.target is not None or self.mission_queue.active:
                raise ValueError('Cancel or finish the current target before starting a mission')
            if self.home is None:
                raise ValueError('Home position unavailable')
            if set(data) != {'waypoints'}:
                raise ValueError('mission requires only waypoints')
            self.mission_queue.load(data['waypoints'], self.home[2], self.machine.max_altitude)
            self._dispatch_waypoint()
        elif event == 'cancel':
            if not self.mission_queue.active:
                raise ValueError('No active waypoint mission to cancel')
            self.mission_queue.stop('CANCELED')
            self.target = self.target_speed = None
            self._submit('hover')
        elif event == 'goto':
            if self.mission_queue.active:
                raise ValueError('Cancel the waypoint mission before sending goto')
            target = tuple(float(data[k]) for k in ('x', 'y', 'z'))
            speed = float(data.get('speed', 3.0))
            if (not all(math.isfinite(v) for v in target + (speed,)) or
                    speed <= 0 or not self.home[2] - self.machine.max_altitude <= target[2] < self.home[2]):
                raise ValueError('Invalid NED waypoint altitude or speed')
            self.target = target
            self.target_speed = speed
            self.started = self.clock()
            self._submit('goto', target=target, speed=speed)
        elif event == 'pause':
            self.machine.transition('pause')
            if self.mission_queue.active:
                self.mission_queue.state = 'PAUSED'
            self._submit('hover')
        elif event == 'resume':
            self.machine.transition('resume')
            if self.mission_queue.active:
                self.mission_queue.state = 'RUNNING'
            self.started = self.clock()
            if self.target:
                self._submit('goto', target=self.target, speed=self.target_speed)
            else:
                self._submit('hover')
        elif event == 'return':
            if self.home is None:
                raise ValueError('Home position unavailable')
            self.mission_queue.stop('CANCELED')
            self.target = (self.home[0], self.home[1],
                           min(self.position[2], self.home[2] - min(5.0, self.machine.max_altitude)))
            self.machine.transition('return')
            self.return_settled_since = None
            self.return_precision = False
            self.target_speed = 0.5
            self.started = self.clock()
            self._submit('goto', target=self.target, speed=self.target_speed)
        elif event == 'land':
            self.mission_queue.stop('CANCELED')
            self.machine.transition('land')
            self.target = None
            self.target_speed = None
            self._begin_landing()

    def _fail(self, reason):
        self.mission_queue.stop('ABORTED')
        self.last_error = reason
        self.machine.transition('failsafe', reason=reason)
        # No guaranteed automatic landing on connection loss. Explicit land may be retried.

    def _finish_dispatch(self, now):
        if self.pending and self.pending[1].done():
            command, future = self.pending
            self.pending = None
            try:
                result = future.result()
                if command == 'connect':
                    if not result['landed']:
                        raise ValueError('Connect requires a landed vehicle to capture home')
                    velocity = result.get('velocity', ())
                    if (len(velocity) != 3 or
                            not all(math.isfinite(v) for v in velocity) or
                            math.sqrt(sum(v*v for v in velocity)) > 0.1):
                        raise ValueError('Connect requires stationary landed telemetry')
                    self.home = self.position = result['position']
                    self.connected = True
                    self.last_telemetry = now
                    self.machine.transition('connected')
                elif command == 'arm':
                    self.machine.transition('armed')
                elif command == 'land':
                    self.landing_result = result
                elif command == 'takeoff':
                    self.takeoff_result = result
                elif command == 'disarm':
                    self.disarmed = True
            except Exception as exc:
                self._fail('AirSim {} failed: {}'.format(command, exc))

    def tick(self):
        now = self.clock()
        self._finish_dispatch(now)
        if self.takeoff_result is not None and self.takeoff_result.done():
            result, self.takeoff_result = self.takeoff_result, None
            try:
                self.takeoff_rpc_result = result.result()
                if self.takeoff_rpc_result is not True:
                    self.last_error = 'AirSim takeoff returned False; airborne state requires telemetry'
            except Exception as exc:
                self._fail('AirSim takeoff completion failed: {}'.format(exc))
        if self.landing_result is not None and self.landing_result.done():
            result, self.landing_result = self.landing_result, None
            try:
                if result.result() is not True:
                    raise RuntimeError('AirSim did not confirm landing command success')
                self.land_succeeded = True
            except Exception as exc:
                if self.machine.state == State.LANDING:
                    self._fail('AirSim landing completion failed: {}'.format(exc))
        if self.sample and self.sample.done():
            future, self.sample = self.sample, None
            try:
                result = future.result()
                if self.sample_revision == self.revision:
                    self._observe(result, now)
            except Exception as exc:
                self.last_error = 'AirSim telemetry unavailable: {}'.format(exc)
        if self.connected and self.sample is None:
            self.sample_revision = self.revision
            self.sample = self.adapter.sample()
        if self.machine.state in self.ACTIVE:
            self.machine.check_safety(0, now - self.last_telemetry)
            moving = (self.machine.state in (State.RETURNING, State.LANDING) or
                      self.machine.state == State.TAKEOFF and self.flight_started or
                      self.machine.state == State.EXECUTING and self.target is not None)
            if moving and now - self.started > self.operation_timeout:
                self._fail('Flight operation timed out')
        if self.machine.state == State.FAILSAFE:
            self.mission_queue.stop('ABORTED')

    def _observe(self, data, now):
        position = tuple(float(v) for v in data['position'])
        if len(position) != 3 or not all(math.isfinite(v) for v in position):
            raise ValueError('Invalid AirSim position')
        self.position = position
        if now - self.last_telemetry > self.machine.telemetry_timeout:
            self.settled_since = None
            self.return_settled_since = None
        self.last_telemetry = now
        s = self.machine.state
        if s not in self.ACTIVE:
            return
        altitude = self.home[2] - position[2]
        self.machine.check_safety(altitude, 0)
        if self.machine.state == State.FAILSAFE or self.pending:
            return
        if s == State.TAKEOFF and self.flight_started and not data['landed'] and altitude > 0.5:
            self.machine.transition('airborne')
        elif s in (State.RETURNING, State.EXECUTING) and self.target:
            distance = math.sqrt(sum((a-b)**2 for a, b in zip(position, self.target)))
            if s == State.RETURNING:
                # AirSim position moves may stop short. Close the final metre
                # with bounded velocity pulses, then require a stable stop.
                velocity = data.get('velocity', ())
                self.return_precision = self.return_precision or distance <= 1.0
                stationary = (len(velocity) == 3 and
                              all(math.isfinite(v) for v in velocity) and
                              math.sqrt(sum(v*v for v in velocity)) <= 0.05)
                if distance <= 0.08 and stationary:
                    if self.return_settled_since is None:
                        self.return_settled_since = now
                else:
                    self.return_settled_since = None
                if (self.return_settled_since is not None and
                        now - self.return_settled_since >= 1.0):
                    self.target = None
                    self.target_speed = None
                    self.machine.transition('at_home')
                    self._begin_landing()
                elif self.return_precision:
                    error = tuple(b-a for a, b in zip(position, self.target))
                    gain = min(0.6, 0.3 / max(distance, 1e-9))
                    correction = ((0., 0., 0.) if distance <= 0.08 else
                                  tuple(gain*v for v in error))
                    self._submit('velocity', velocity=correction, duration=0.3)
            elif distance <= 1.0:
                if self.mission_queue.active:
                    if now - self.started > self.operation_timeout:
                        self._fail('Flight operation timed out')
                        return
                    self.mission_queue.completed += 1
                    if self.mission_queue.completed < len(self.mission_queue.waypoints):
                        self._dispatch_waypoint()
                        return
                    self.mission_queue.state = 'COMPLETED'
                    self._submit('hover')
                self.target = None
                self.target_speed = None
        elif s == State.LANDING:
            if self.disarmed:
                if data['landed']:
                    self.machine.transition('landed')
                return
            # Some OpenHUTB/SimpleFlight builds report Flying after contact until
            # disarmed. Never disarm just because land() returned True or z froze.
            velocity = data.get('velocity', ())
            near_home = math.dist(position, self.home) <= 0.15
            stationary = (len(velocity) == 3 and
                          all(math.isfinite(v) for v in velocity) and
                          math.sqrt(sum(v*v for v in velocity)) <= 0.1)
            if self.land_succeeded and near_home and stationary:
                if self.settled_since is None:
                    self.settled_since = now
            else:
                self.settled_since = None
            settled = (self.settled_since is not None and
                       now - self.settled_since >= 3.0)
            if data['landed'] or settled:
                self._submit('disarm')
