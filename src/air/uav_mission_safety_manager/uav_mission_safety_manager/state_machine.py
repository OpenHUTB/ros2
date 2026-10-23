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
    def __init__(self, max_altitude=50.0, telemetry_timeout=2.0):
        self.state = State.IDLE
        self.max_altitude = max_altitude
        self.telemetry_timeout = telemetry_timeout
        self.reason = ''

    def transition(self, event, **data):
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
        elif event == 'landed' and s == State.FAILSAFE:
            self.state = State.LANDED
        elif (s, event) in allowed:
            self.state = allowed[(s, event)]
            self.reason = ''
        else:
            raise ValueError(f'event {event!r} is invalid in {s.value}')
        return self.state

    def check_safety(self, altitude, telemetry_age, inside_fence=True):
        if altitude > self.max_altitude:
            self.transition('failsafe', reason=f'altitude {altitude:.1f}m exceeds limit')
        elif telemetry_age > self.telemetry_timeout:
            self.transition('failsafe', reason=f'telemetry timeout {telemetry_age:.1f}s')
        elif not inside_fence:
            self.transition('failsafe', reason='geofence violation')
        return self.state
