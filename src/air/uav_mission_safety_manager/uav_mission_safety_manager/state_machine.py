from enum import Enum

class State(str, Enum):
    IDLE = 'IDLE'
    CONNECTING = 'CONNECTING'
    ARMING = 'ARMING'
    TAKEOFF = 'TAKEOFF'
    EXECUTING = 'EXECUTING'
    PAUSED = 'PAUSED'
    RETURNING = 'RETURNING'
    LANDING = 'LANDING'
    LANDED = 'LANDED'
    FAILSAFE = 'FAILSAFE'

class SafetyMachine:
    """Pure Python state machine; independent of ROS and AirSim for easy testing."""
    def __init__(self, max_altitude=50.0, telemetry_timeout=2.0, on_transition=None):
        self.state = State.IDLE
        self.max_altitude = max_altitude
        self.telemetry_timeout = telemetry_timeout
        self.reason = ''
        self.on_transition = on_transition

    def transition(self, event, **data):
        previous = self.state
        s = self.state
        allowed = {
            (State.IDLE,'connect'): State.CONNECTING,
            (State.CONNECTING,'connected'): State.ARMING,
            (State.ARMING,'armed'): State.TAKEOFF,
            (State.TAKEOFF,'airborne'): State.EXECUTING,
            (State.EXECUTING,'pause'): State.PAUSED,
            (State.PAUSED,'resume'): State.EXECUTING,
            (State.EXECUTING,'return'): State.RETURNING,
            (State.PAUSED,'return'): State.RETURNING,
            (State.RETURNING,'at_home'): State.LANDING,
            (State.TAKEOFF,'land'): State.LANDING,
            (State.EXECUTING,'land'): State.LANDING,
            (State.PAUSED,'land'): State.LANDING,
            (State.RETURNING,'land'): State.LANDING,
            (State.LANDING,'land'): State.LANDING,
            (State.FAILSAFE,'land'): State.LANDING,
            (State.LANDING,'landed'): State.LANDED,
            (State.LANDED,'reset'): State.IDLE,
        }
        if event == 'failsafe':
            self.reason = data.get('reason', 'safety condition')
            self.state = State.FAILSAFE
            self._notify(previous, event, self.reason)
        elif event == 'landed' and s == State.FAILSAFE:
            self.state = State.LANDED
            self._notify(previous, event)
        elif (s, event) in allowed:
            self.state = allowed[(s, event)]
            self.reason = ''
            self._notify(previous, event)
        else:
            raise ValueError(f'event {event!r} is invalid in {s.value}')
        return self.state

    def _notify(self, previous, event, reason=''):
        """Invoke the optional observer after a real state change."""
        if self.on_transition is not None and previous is not self.state:
            self.on_transition(previous, self.state, event, reason)

    def check_safety(self, altitude, telemetry_age, inside_fence=True):
        if altitude > self.max_altitude:
            self.transition('failsafe', reason=f'altitude {altitude:.1f}m exceeds limit')
        elif telemetry_age > self.telemetry_timeout:
            self.transition('failsafe', reason=f'telemetry timeout {telemetry_age:.1f}s')
        elif not inside_fence:
            self.transition('failsafe', reason='geofence violation')
        return self.state
