"""Bounded RPC dispatch; flight completion is determined from telemetry."""
from concurrent.futures import ThreadPoolExecutor, wait
from threading import Event, local


class AirSimAdapter:
    def __init__(self, host='127.0.0.1', port=41451, vehicle_name=''):
        self.host, self.port, self.vehicle_name = host, int(port), vehicle_name
        self._clients = local()
        # msgpack clients/event loops must not be shared between threads.
        self._commands = ThreadPoolExecutor(max_workers=1)
        self._telemetry = ThreadPoolExecutor(max_workers=1)
        self._flight = ThreadPoolExecutor(max_workers=1)
        self._active = None
        self._flight_started = None

    def submit(self, command, **data):
        return self._commands.submit(self._dispatch, command, data)

    def sample(self):
        return self._telemetry.submit(self._read)

    def _client(self, timeout=1):
        if not hasattr(self._clients, 'client'):
            import airsim
            self._clients.client = airsim.MultirotorClient(
                ip=self.host, port=self.port, timeout_value=timeout)
        return self._clients.client

    def _dispatch(self, command, data):
        c, v = self._client(), self.vehicle_name
        if command == 'connect':
            if not c.ping():
                raise RuntimeError('AirSim ping failed')
            c.enableApiControl(True, v)
            return self._read()
        if command == 'arm':
            if not c.armDisarm(True, v):
                raise RuntimeError('AirSim rejected arming')
        elif command == 'disarm':
            if not c.armDisarm(False, v):
                raise RuntimeError('AirSim rejected disarming')
        elif command in ('takeoff', 'goto', 'hover', 'land', 'velocity'):
            # cancelLastTask only requests cancellation. Wait for the previous
            # task's response before starting its successor on the same worker.
            # This also pumps the RPC event loop instead of abandoning Futures.
            if self._active is not None:
                if not self._active.done():
                    if not self._flight_started.wait(1):
                        raise RuntimeError('Previous flight RPC did not start')
                    c.cancelLastTask(vehicle_name=v)
                if wait([self._active], timeout=2).not_done:
                    raise RuntimeError('Previous flight did not finish cancellation')
            self._flight_started = Event()
            self._active = self._flight.submit(
                self._run_flight, command, data, self._flight_started)
            return self._active
        else:
            raise ValueError('Unknown AirSim command: ' + command)
        # Non-flight commands have no separate completion Future.
        return None

    def _run_flight(self, command, data, started):
        # Only this dedicated worker waits for flight completion. Its client and
        # event loop belong to this thread; both RPC and flight waits are bounded.
        c, v = self._client(timeout=125), self.vehicle_name
        try:
            if command == 'takeoff':
                future = c.takeoffAsync(timeout_sec=20, vehicle_name=v)
            elif command == 'goto':
                future = c.moveToPositionAsync(
                    *data['target'], data['speed'], timeout_sec=120, vehicle_name=v)
            elif command == 'velocity':
                future = c.moveByVelocityAsync(
                    *data['velocity'], data['duration'], vehicle_name=v)
            elif command == 'hover':
                future = c.hoverAsync(vehicle_name=v)
            else:
                future = c.landAsync(timeout_sec=120, vehicle_name=v)
        finally:
            started.set()
        return future.get()

    def _read(self):
        import airsim
        state = self._client().getMultirotorState(vehicle_name=self.vehicle_name)
        p = state.kinematics_estimated.position
        velocity = state.kinematics_estimated.linear_velocity
        return {'position': (float(p.x_val), float(p.y_val), float(p.z_val)),
                'velocity': (float(velocity.x_val), float(velocity.y_val), float(velocity.z_val)),
                'landed': state.landed_state == airsim.LandedState.Landed}

    def close(self):
        self._commands.shutdown(wait=False)
        self._telemetry.shutdown(wait=False)
        self._flight.shutdown(wait=False)
