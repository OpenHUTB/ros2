"""ROS-independent visual servo for a calibrated, zero-roll camera.

Camera is fixed facing forward; commands use world NED.

Horizontal centering is always active. Apparent-size distance keeping and
vertical centering are opt-in: leaving ``target_area`` and ``vertical_gain`` at
zero keeps the original behaviour (no forward motion, altitude hold).
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Box:
    x: float
    y: float
    width: float
    height: float
    score: float
    label: str


@dataclass(frozen=True)
class Settings:
    target_class: str = 'car'
    confidence: float = 0.5
    tracking_confidence: float = 0.5
    gain: float = 0.6
    deadband: float = 0.08
    max_speed: float = 0.4
    altitude_gain: float = 0.8
    max_vertical_speed: float = 0.3
    detection_timeout: float = 0.8
    odom_timeout: float = 0.5
    lock_timeout: float = 1.0
    association_gate: float = 0.25
    confirmation_frames: int = 3
    max_size_ratio: float = 1.8
    max_displacement: float = 3.0
    max_yaw_error: float = 0.35
    max_acceleration: float = 0.0

    # Opt-in depth control: keep the target box at ``target_area`` of the frame.
    # Zero disables forward motion entirely.
    target_area: float = 0.0
    distance_gain: float = 1.0
    distance_deadband: float = 0.15
    max_forward_speed: float = 0.5
    # Opt-in vertical centering. Zero keeps the original altitude hold.
    vertical_gain: float = 0.0
    vertical_deadband: float = 0.08
    # Maximum |z - z0| in metres, measured from the pose captured when tracking
    # started. It bounds the altitude change that vertical centering may
    # command, so it only takes effect while vertical_gain > 0.
    max_altitude_change: float = 3.0

    def __post_init__(self):
        if (isinstance(self.confirmation_frames, bool) or
                not isinstance(self.confirmation_frames, int) or self.confirmation_frames < 1):
            raise ValueError('confirmation_frames must be a positive integer')
        if self.max_size_ratio <= 1:
            raise ValueError('max_size_ratio must exceed one')
        # Zero is meaningful for these: it disables the associated feature.
        nonnegative = {'max_acceleration', 'target_area', 'distance_gain',
                       'vertical_gain', 'distance_deadband', 'vertical_deadband'}
        for name, value in vars(self).items():
            if name == 'target_class':
                continue
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
            if name in nonnegative:
                if value < 0:
                    raise ValueError(f'{name} must be finite and nonnegative')
            elif value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not self.target_class.strip():
            raise ValueError('target_class must not be empty')
        if (self.confidence > 1 or self.deadband >= 1 or self.association_gate > 2
                or self.distance_deadband >= 1 or self.vertical_deadband >= 1):
            raise ValueError('invalid confidence, deadband or association_gate')
        if not 0 < self.tracking_confidence <= self.confidence:
            raise ValueError('tracking_confidence must be positive and <= confidence')


class CenteringController:
    def __init__(self, settings=None):
        self.settings = settings or Settings()
        self.pose = None
        self.origin = None
        self.pose_time = None
        self.target = None
        self.target_time = None
        self.last_lock = None
        self.lock_time = None
        self.last_step_time = None
        self.lateral_speed = 0.0
        self.seen_area = None
        self._last_distance_error = None
        self._last_forward_speed = 0.0
        self.pending = None
        self.pending_time = None
        self.confirmations = 0
        self.last_frame_time = None
        self.lock_expired = False
        self.paused = False
        self.needs_confirmation = True
        self.selection_reason = 'waiting'
        self.association_diagnostics = {}

    @property
    def last_distance_error(self):
        """Relative area error from the most recent step, or None when disabled."""
        return self._last_distance_error

    @property
    def last_forward_speed(self):
        """Forward speed commanded by the most recent step, in m/s."""
        return self._last_forward_speed

    def reset_target(self):
        """Explicitly allow selecting a new vehicle; keep flight safety origin."""
        self.clear_detection()
        self.last_lock = self.lock_time = None
        self.lock_expired = False

    def set_paused(self, paused):
        self.paused = paused
        self.clear_detection()

    def update_pose(self, x, y, z, yaw, now):
        if not all(math.isfinite(v) for v in (x, y, z, yaw, now)):
            self.pose = None
            return
        self.pose = (x, y, z, yaw)
        self.pose_time = now

    def clear_detection(self):
        self.target = None
        self.seen_area = None
        self.pending = None
        self.confirmations = 0
        self.needs_confirmation = True

    def _matches(self, candidate, reference):
        s = self.settings
        return (math.hypot(candidate[0]-reference[0], candidate[1]-reference[1])
                <= s.association_gate and all(
                    1/s.max_size_ratio <= candidate[i]/reference[i] <= s.max_size_ratio
                    for i in (3, 4)))

    def update_detections(self, boxes, width, height, now):
        self.association_diagnostics = {'capture_time': now, 'candidates': [],
                                        'reference': None}
        previous_target = self.target
        self.target = None
        self.seen_area = None
        if (not math.isfinite(now) or
                (self.last_frame_time is not None and now <= self.last_frame_time)):
            self.selection_reason = 'non_monotonic_frame'
            self.clear_detection()
            return
        self.last_frame_time = now
        if (self.paused or not all(math.isfinite(v) and v > 0 for v in (width, height))):
            self.clear_detection()
            return
        s = self.settings
        if (self.pending is not None and self.pending_time is not None
                and now-self.pending_time > s.detection_timeout):
            self.pending = None
            self.confirmations = 0
        if self.last_lock is not None and now-self.lock_time > s.lock_timeout:
            self.lock_expired = True
        if self.lock_expired:
            self.selection_reason = 'lock_expired'
            self.clear_detection()
            return
        candidates = []
        for b in boxes:
            if (b.label != s.target_class or not all(math.isfinite(v) for v in
                    (b.x, b.y, b.width, b.height, b.score))):
                continue
            threshold = s.tracking_confidence if (
                self.last_lock is not None or self.pending is not None) else s.confidence
            if not (threshold <= b.score <= 1 and b.width > 0 and b.height > 0
                    and 0 <= b.x <= width and 0 <= b.y <= height):
                continue
            candidates.append((2*b.x/width - 1, 2*b.y/height - 1, b.score,
                               b.width/width, b.height/height))
        if self.last_lock is not None:
            valid_count = len(candidates)
            reference = self.last_lock
            if (self.needs_confirmation and self.pending is not None and self.pending_time is not None
                    and 0 <= now-self.pending_time <= s.detection_timeout):
                reference = self.pending
            self.association_diagnostics['reference'] = reference
            self.association_diagnostics['candidates'] = [
                {'box': p,
                 'center_distance': math.hypot(p[0]-reference[0], p[1]-reference[1]),
                 'width_ratio': p[3]/reference[3],
                 'height_ratio': p[4]/reference[4],
                 'center_ok': math.hypot(p[0]-reference[0], p[1]-reference[1]) <= s.association_gate,
                 'size_ok': all(1/s.max_size_ratio <= p[i]/reference[i] <= s.max_size_ratio
                                for i in (3, 4))}
                for p in candidates]
            candidates = [p for p in candidates if self._matches(p, reference)]
            if len(candidates) > 1:
                self.selection_reason = 'ambiguous_association'
                self.clear_detection()
                return
            key = lambda p: math.hypot(p[0]-reference[0], p[1]-reference[1])
        else:
            # Once acquiring, associate the candidate instead of allowing
            # alternating confidence rankings to restart selection each frame.
            if self.pending is not None:
                associated = [p for p in candidates if self._matches(p, self.pending)]
                if associated:
                    candidates = associated
                    key = lambda p: math.hypot(p[0]-self.pending[0], p[1]-self.pending[1])
                else:
                    candidates = [p for p in candidates if p[2] >= s.confidence]
                    key = lambda p: -p[2]
            else:
                key = lambda p: -p[2]
        if not candidates:
            self.selection_reason = ('association_rejected' if self.last_lock is not None
                                     and valid_count else 'no_valid_box')
            self.clear_detection()
            return
        candidate = min(candidates, key=key)
        # Matching observations keep the association memory alive while
        # confirmation is pending; only confirmed boxes move its anchor.
        if self.last_lock is not None:
            self.lock_time = now
        if previous_target is None or now-self.target_time > s.detection_timeout:
            self.needs_confirmation = True
        if self.needs_confirmation:
            if (self.pending is not None and now-self.pending_time <= s.detection_timeout
                    and self._matches(candidate, self.pending)):
                self.confirmations += 1
            else:
                self.confirmations = 1
            self.pending, self.pending_time = candidate, now
            if self.confirmations < s.confirmation_frames:
                self.selection_reason = 'confirming'
                return
        self.selection_reason = 'accepted'
        self.needs_confirmation = False
        self.target = candidate
        self.seen_area = candidate[3]*candidate[4]
        self.target_time = now
        self.last_lock = candidate
        self.lock_time = now

    def step(self, now):
        """Return (world_vx, world_vy, world_vz), status, horizontal error."""
        stop = (0.0, 0.0, 0.0)
        s = self.settings
        dt = 0.0 if self.last_step_time is None else max(0.0, min(0.1, now-self.last_step_time))
        self.last_step_time = now
        # Every guard and the centered state bypass the acceleration ramp.
        previous_speed = self.lateral_speed
        self.lateral_speed = 0.0
        self._last_forward_speed = 0.0
        self._last_distance_error = None
        if self.pose is None or not 0 <= now-self.pose_time <= s.odom_timeout:
            return stop, 'odom_stale', None
        if self.paused:
            return stop, 'paused', None
        if self.last_lock is not None and now-self.lock_time > s.lock_timeout:
            self.lock_expired = True
        if self.lock_expired:
            return stop, 'reselect_required', None
        if self.needs_confirmation and self.pending is not None:
            # Confirmation continuity is measured between capture times, while
            # motion freshness includes inference age. A timer must not erase
            # a still-continuous sequence simply because its last capture aged.
            if 0 <= now-self.pending_time <= s.detection_timeout:
                return stop, 'acquiring', None
            self.target = None
            return stop, 'target_lost', None
        if self.target is None or not 0 <= now-self.target_time <= s.detection_timeout:
            self.clear_detection()
            return stop, 'target_lost', None
        # Capture reference only when both inputs first become usable.
        if self.origin is None:
            self.origin = self.pose
        x, y, z, yaw = self.pose
        ox, oy, oz, initial_yaw = self.origin
        error = self.target[0]
        if math.hypot(x-ox, y-oy) >= s.max_displacement:
            return stop, 'displacement_limit', error
        yaw_error = math.atan2(math.sin(yaw-initial_yaw), math.cos(yaw-initial_yaw))
        if abs(yaw_error) > s.max_yaw_error:
            return stop, 'heading_changed', error
        # Bounds only the altitude change commanded by vertical centering, so the
        # default (vertical_gain == 0) keeps the original behaviour untouched.
        if s.vertical_gain > 0 and abs(z-oz) > s.max_altitude_change:
            return stop, 'altitude_limit', error
        lateral = 0.0 if abs(error) <= s.deadband else max(
            -s.max_speed, min(s.max_speed, s.gain*error))
        if lateral != 0.0 and s.max_acceleration > 0:
            # A fresh start ramps from zero; repeated timer calls do not add speed.
            change = s.max_acceleration * dt
            lateral = max(previous_speed-change, min(previous_speed+change, lateral))
        self.lateral_speed = lateral
        # Apparent size is only a proxy for range: it needs a known target class
        # and a scene where the target is never partly occluded.
        forward = 0.0
        area_error = None
        if s.target_area > 0 and self.seen_area:
            area_error = (s.target_area - self.seen_area)/s.target_area
            self._last_distance_error = area_error
            if abs(area_error) > s.distance_deadband:
                forward = max(-s.max_forward_speed,
                              min(s.max_forward_speed, s.distance_gain*area_error))
        self._last_forward_speed = forward
        if s.vertical_gain > 0:
            elevation = self.target[1]
            vertical = 0.0 if abs(elevation) <= s.vertical_deadband else max(
                -s.max_vertical_speed, min(s.max_vertical_speed,
                                           s.vertical_gain*elevation))
        else:
            vertical = max(-s.max_vertical_speed, min(s.max_vertical_speed,
                                                      s.altitude_gain*(oz-z)))
        # Fixed forward camera: +x forward, +y right in world NED.
        camera_yaw = yaw
        velocity = (math.cos(camera_yaw)*forward - math.sin(camera_yaw)*lateral,
                    math.sin(camera_yaw)*forward + math.cos(camera_yaw)*lateral,
                    vertical)
        horizontal_ok = abs(error) <= s.deadband
        distance_ok = area_error is None or abs(area_error) <= s.distance_deadband
        return velocity, ('centered' if horizontal_ok and distance_ok else 'tracking'), error
