"""AirSim height-aware obstacle negotiation: overfly low obstacles, bypass tall ones.

NED metres; yaw in radians. The camera must be level, forward facing and centred
on the vehicle.

The single-pillar (PR #597) and double-pillar modules scan only the central band
of the depth image, so altitude is fixed and every obstacle is bypassed
laterally. That is expensive for a low wall: the vehicle flies a long detour
around something it could simply climb over. This planner reads the full image
height instead, reconstructs each obstacle's top edge above the camera, and
chooses between three behaviours:

  advance  - corridor ahead is clear
  overfly  - obstacle top is low enough and the space above it is free, so climb
  curve    - obstacle is too tall (or overfly is disarmed), bypass laterally

Overflying is a deliberate climb into space the camera has verified is free, so
it is guarded three ways: the required climb (top edge plus the camera mount
offset plus body radius and safety margin) must fit under ``--max-climb`` and
the absolute ``--max-altitude``,
free space above the top edge must actually be visible so the top can be
trusted, and repeated failed climbs disarm overfly for the rest of the mission
rather than letting the vehicle keep climbing at an obstacle. A committed climb
is flown in place first and only then crosses the obstacle.

This is a local planner for static obstacles, not a collision-free guarantee.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import time
import tempfile
from dataclasses import dataclass
from pathlib import Path


MAX_YAW_RATE = 12.0  # degrees/s, shared by planner, smoothing and blocked scan.
OVERFLY_HOLD_FRAMES = 3  # consecutive frames the climb target must hold before trusting it.
OVERFLY_DONE_FRAMES = 6  # consecutive clear frames that end an overfly.
OVERFLY_MAX_FAILS = 2  # failed climbs after which overfly is disarmed for the run.
OVERFLY_CLIMB_GRACE = 10.0  # seconds a committed climb may take before it counts as stalled
OVERFLY_GAIN_RATIO = 0.7  # share of the committed height that must be gained in that time
GROUND_ROWS_DEFAULT = 12  # bottom rows excluded as ground, not obstacles.


@dataclass
class ProgressWatchdog:
    """Track cruise motion separately from continuous blocked scanning."""
    position: tuple[float, float]
    progress_time: float
    blocked_since: float | None = None

    def update(self, now: float, position: tuple[float, float], blocked: bool) -> None:
        if blocked:
            if self.blocked_since is None:
                self.blocked_since = now
            if now - self.blocked_since >= 30:
                raise TimeoutError("No clear corridor after 30 seconds of scanning")
            return
        if self.blocked_since is not None:
            self.progress_time = now
            self.position = position
            self.blocked_since = None
        if math.hypot(position[0] - self.position[0], position[1] - self.position[1]) >= 0.4:
            self.position = position
            self.progress_time = now
        elif now - self.progress_time >= 20:
            raise TimeoutError("No horizontal progress for 20 seconds; stop and inspect depth/route")


@dataclass
class OverflyState:
    """Commit to one climb so the vehicle cannot oscillate at an obstacle top.

    ``armed`` starts True and is cleared permanently after OVERFLY_MAX_FAILS
    unsuccessful climbs, which degrades the mission to lateral bypass instead of
    letting it retry a climb that has already failed twice.

    ``climb_started`` is a monotonic timestamp set by the caller when a climb is
    committed or when the committed height is extended, so ``check_progress``
    can judge the climb only after a grace period. Checking the first frames of
    a legitimate climb would disarm overfly before the vehicle had time to rise.

    After a climb completes, ``hold`` keeps the vehicle at the climbed altitude
    until it has flown the configured holdover distance past the obstacle, so
    the body does not descend straight back onto the obstacle top it just
    crossed.
    """
    armed: bool = True
    active: bool = False
    climb_target: float = 0.0  # metres above cruise altitude
    climb_started: float = 0.0  # monotonic seconds; 0 means no committed climb
    hold: bool = False  # keep the climbed altitude after completion
    hold_altitude: float = 0.0  # height above cruise to hold during holdover
    hold_start: tuple[float, float] = (0.0, 0.0)  # NED position where the hold began
    hold_frames: int = 0
    clear_frames: int = 0
    fails: int = 0
    completed: int = 0

    def request(self, climb: float) -> bool:
        """Ask for a climb; require it to be stable before committing.

        Returns True when a climb was just committed or the committed height was
        extended by at least 0.2 m, so the caller can restart its progress timer.
        """
        if not self.armed:
            return False
        if self.active:
            # Only ever extend the climb; never lower the target mid-climb, or
            # the vehicle could descend back onto the obstacle it is passing.
            previous = self.climb_target
            self.climb_target = max(self.climb_target, climb)
            self.hold_frames += 1
            self.clear_frames = 0
            return self.climb_target - previous >= 0.2
        if abs(climb - self.climb_target) < 0.15:
            self.hold_frames += 1
        else:
            self.climb_target = climb
            self.hold_frames = 1
        if self.hold_frames >= OVERFLY_HOLD_FRAMES:
            self.active = True
            self.clear_frames = 0
            self.cancel_hold()  # a new climb supersedes any pending holdover
            return True
        return False

    def begin_hold(self, altitude: float, position: tuple[float, float]) -> None:
        """Keep ``altitude`` above cruise after completion until ``end_hold``."""
        self.hold = True
        self.hold_altitude = max(0.0, altitude)
        self.hold_start = position

    def cancel_hold(self) -> None:
        self.hold = False
        self.hold_altitude = 0.0
        self.hold_start = (0.0, 0.0)

    def end_hold(self) -> None:
        self.cancel_hold()

    def check_progress(self, now: float, lift: float) -> bool:
        """Return True when a committed climb has failed to gain its height.

        ``lift`` is the measured height above cruise altitude in metres. The
        grace period is judged from the commit (or last extension), so a climb
        that is still in progress is never mistaken for a stalled one.
        """
        if not self.active or self.climb_started <= 0.0:
            return False
        if now - self.climb_started < OVERFLY_CLIMB_GRACE:
            return False
        return lift < OVERFLY_GAIN_RATIO * self.climb_target

    def suspend_clear(self) -> None:
        """Drop the clear-frame streak without finishing the climb.

        A lateral bypass or a blocked scan means the corridor is not clear, so
        clear frames counted before it must not add up to a false completion.
        """
        self.clear_frames = 0

    def note_clear(self) -> bool:
        """Count a clear frame; return True when the overfly has just finished."""
        if not self.active:
            self.climb_target = 0.0
            self.climb_started = 0.0
            self.hold_frames = 0
            return False
        self.clear_frames += 1
        if self.clear_frames >= OVERFLY_DONE_FRAMES:
            self.active = False
            self.completed += 1
            self.climb_target = 0.0
            self.climb_started = 0.0
            self.hold_frames = 0
            self.clear_frames = 0
            return True
        return False

    def fail(self) -> None:
        """Record a climb that did not work; disarm overfly after the limit."""
        self.fails += 1
        self.active = False
        self.climb_target = 0.0
        self.climb_started = 0.0
        self.cancel_hold()
        self.hold_frames = 0
        self.clear_frames = 0
        if self.fails >= OVERFLY_MAX_FAILS:
            self.armed = False


@dataclass(frozen=True)
class Command:
    mode: str  # advance | overfly | curve | blocked
    speed: float  # body-forward m/s; never strafe into unseen space
    climb: float  # m/s upward, positive rises (NED vz is the negation)
    yaw_rate: float  # degrees/s, positive turns right
    clearance: float  # nearest forward corridor distance
    overhead: float  # free height above the obstacle top, metres
    top_height: float  # obstacle top above the camera, metres
    reason: str
    steer_angle: float = 0.0  # selected camera-relative direction, radians


@dataclass(frozen=True)
class ColumnProfile:
    """One image column reduced to what the height decision needs."""
    distance: float  # nearest obstacle along the optical axis, metres
    top_height: float  # that obstacle's top edge above the camera, metres
    overhead: float  # free vertical space above the top edge, metres
    valid: bool


def create_run_directory(root: Path, args: argparse.Namespace) -> Path:
    """Keep each run's evidence and exact requested settings together."""
    root.mkdir(parents=True, exist_ok=True)
    directory = Path(tempfile.mkdtemp(prefix=time.strftime("%Y%m%d_%H%M%S_"), dir=root))
    settings = {key: str(value) if isinstance(value, Path) else value
                for key, value in vars(args).items()}
    (directory / "settings.json").write_text(
        json.dumps(settings, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return directory


def record_event(path: Path, state: str, reason: str = "") -> None:
    """Append diagnostic events without replacing the original flight failure."""
    try:
        with path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps({"time_unix_s": time.time(), "state": state,
                                     "reason": reason}, ensure_ascii=False) + "\n")
    except OSError as error:
        print(f"Event log unavailable: {error}")


def route_from_start(start_x: float, start_y: float, goal_x: float,
                     goal_y: float) -> list[tuple[float, float]]:
    """Return a rectangular inspection route in AirSim world N/E metres."""
    return [(start_x + goal_x, start_y),
            (start_x + goal_x, start_y + goal_y),
            (start_x, start_y + goal_y),
            (start_x, start_y)]


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def camera_geometry(width: int, height: int, fov: float, ground_rows: int):
    """Focal length in pixels plus the row band used for obstacle reasoning.

    AirSim Unity's ``CameraInfo.fov`` is ``Camera.fieldOfView``, i.e. the
    **vertical** field of view (confirmed in DataCaptureScript). With square
    pixels the same focal length applies to rows and columns, so it is derived
    from the image height; deriving it from the width would under-estimate
    obstacle heights by the aspect ratio and could let a tall obstacle look
    climbable.

    Rows below ``row_bottom`` are treated as ground: with a level camera the
    near floor fills the bottom of the image at short range and would otherwise
    be mistaken for a low obstacle to overfly.
    """
    focal = height / (2 * math.tan(math.radians(fov) / 2))
    centre = (height - 1) / 2
    row_top = height // 8
    row_bottom = max(row_top + 1, height - 1 - max(0, ground_rows))
    return focal, centre, row_top, row_bottom


def analyze_columns(depth: list[float], width: int, height: int, fov: float,
                    ground_rows: int = GROUND_ROWS_DEFAULT,
                    free_distance: float = 25.0) -> list[ColumnProfile]:
    """Reduce the depth image to per-column obstacle distance and top height.

    For each column the nearest obstacle is found inside the analysed band, then
    the search walks upward while pixels still belong to that same obstacle
    (distance within 30%), stopping at free space. The stopping row gives the
    top edge, which is converted to metres above the camera by the pinhole
    relation. Invalid pixels are treated as blocked so a dead sensor cannot be
    read as free space.

    ``free_distance`` is what an unobstructed ray reports; rays farther than this
    are considered free for the overhead measurement.
    """
    if width < 16 or height < 8 or len(depth) != width * height:
        raise ValueError("Invalid or empty depth image")
    focal, centre, row_top, row_bottom = camera_geometry(width, height, fov, ground_rows)
    profiles = []
    any_valid = False
    for col in range(width):
        base = col
        nearest_row = -1
        nearest = float("inf")
        for row in range(row_top, row_bottom + 1):
            value = depth[row * width + base]
            if not math.isfinite(value) or value <= 0:
                # Dead pixel: conservative, counts as an obstacle at zero range.
                nearest_row, nearest = row, 0.0
                break
            any_valid = True
            if value < nearest:
                nearest_row, nearest = row, value
        if nearest_row < 0:
            profiles.append(ColumnProfile(float("inf"), 0.0, float("inf"), False))
            continue
        if nearest <= 0:
            profiles.append(ColumnProfile(0.0, 0.0, 0.0, True))
            continue
        # Walk up while the pixels still belong to this obstacle.
        top_row = nearest_row
        while top_row - 1 >= row_top:
            above = depth[(top_row - 1) * width + base]
            if not math.isfinite(above) or above <= 0:
                break  # unknown above: do not assume it is free
            if above > free_distance or above > nearest * 1.3:
                break  # free space above the obstacle: this is its top edge
            top_row -= 1
        # Height above the camera of the top edge, at the obstacle's own range.
        top_height = nearest * (centre - top_row) / focal
        # Free height from the top edge up to the analysed band's upper limit.
        band_top_height = nearest * (centre - row_top) / focal
        overhead = max(0.0, band_top_height - top_height)
        profiles.append(ColumnProfile(nearest, top_height, overhead, True))
    if not any_valid:
        raise ValueError("No valid depth pixels; stop the mission")
    return profiles


def lateral_clearance(profiles: list[ColumnProfile], width: int, height: int,
                      fov: float, angle: float, radius: float) -> float:
    """Nearest entry distance of the swept body corridor along ``angle``."""
    focal = height / (2 * math.tan(math.radians(fov) / 2))
    centre = (width - 1) / 2
    cosine, sine = math.cos(angle), math.sin(angle)
    clearance = float("inf")
    for col, profile in enumerate(profiles):
        if not profile.valid or not math.isfinite(profile.distance):
            continue
        distance = profile.distance
        if distance <= 0:
            return 0.0
        x = distance * (col - centre) / focal
        along = distance * cosine + x * sine
        lateral = abs(x * cosine - distance * sine)
        if along >= 0 and lateral <= radius:
            entry = along - math.sqrt(max(0.0, radius * radius - lateral * lateral))
            clearance = min(clearance, max(0.0, entry))
    return clearance


def committed_climb(top_height: float, camera_offset: float, radius: float) -> float:
    """Climb above cruise needed for the body to clear an obstacle top.

    ``top_height`` is measured above the **camera**, so the camera's mount
    offset above the body origin must be added before comparing with the body
    radius and margin. Without it the vehicle commits ``top + radius``, which
    leaves only ``radius - camera_offset`` of real clearance (0.49 m less on
    this Unity build).
    """
    return max(0.0, top_height + camera_offset + radius)


def plan_height_aware(profiles: list[ColumnProfile], width: int, height: int, fov: float,
                      goal_angle: float, max_speed: float = 1.0,
                      stop_distance: float = 3.0, safety_margin: float = 1.5,
                      vehicle_radius: float = 0.6, preferred_angle: float = 0.0,
                      avoidance_side: int = 0, curve_distance: float = 2.0,
                      max_climb: float = 2.0, max_altitude: float = 6.0,
                      cruise_altitude: float = 3.0, overfly_armed: bool = True,
                      camera_offset: float = 0.0,
                      free_distance: float = 25.0) -> Command:
    """Choose advance / overfly / curve / blocked from the column profiles.

    The forward corridor uses the same inflated swept-body test as the lateral
    planner. Overfly is only offered when every obstacle inside that swept
    corridor has a top edge low enough to clear within the remaining altitude
    budget, and the free space above the highest such top edge covers the body
    radius plus margin. Obstacles are collected by corridor entry distance, not
    by a narrow image slice: a tall pole 1.8 m to the side is inside the inflated
    body radius even though it is outside the central slice, and must veto a
    climb directly at it.
    """
    if width < 16 or height < 8 or len(profiles) != width:
        raise ValueError("Invalid or empty depth image")
    if (not 30 <= fov <= 140 or max_speed <= 0 or stop_distance < 1
            or safety_margin < 0 or vehicle_radius <= 0 or camera_offset < 0):
        raise ValueError("Invalid planner settings")
    if max_climb < 0 or max_altitude <= cruise_altitude:
        raise ValueError("Altitude budget must allow cruise plus a positive climb")
    if free_distance <= stop_distance:
        raise ValueError("free-distance must exceed stop-distance")

    if any(p.valid and p.distance <= 0 for p in profiles):
        return Command("blocked", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "blocked_hover")

    radius = vehicle_radius + safety_margin
    focal = height / (2 * math.tan(math.radians(fov) / 2))
    half_window = max(2, math.ceil(focal * 0.6 / stop_distance))
    centre_col = width // 2
    front_slice = profiles[max(0, centre_col - half_window):
                           min(width, centre_col + half_window + 1)]
    front = min((p.distance for p in front_slice if p.valid and math.isfinite(p.distance)),
                default=float("inf"))
    front = min(front, lateral_clearance(profiles, width, height, fov, 0.0, radius))

    # Obstacles that the inflated body would actually sweep through when flying
    # straight, i.e. those whose corridor entry distance is within the stopping
    # horizon. This uses the whole image, not just the central slice, because a
    # tall obstacle just outside the slice can still be inside the body radius.
    blocking = []
    for col, profile in enumerate(profiles):
        if not profile.valid or not math.isfinite(profile.distance):
            continue
        lateral = abs(profile.distance * (col - (width - 1) / 2) / focal)
        if lateral > radius:
            continue
        entry = profile.distance - math.sqrt(max(0.0, radius * radius - lateral * lateral))
        if entry <= stop_distance + curve_distance:
            blocking.append(profile)
    top_height = max((p.top_height for p in blocking), default=0.0)
    overhead = min((p.overhead for p in blocking), default=float("inf"))

    # An obstacle already below the camera by more than the body radius needs no
    # climb at all; clamping keeps the committed target from going negative and
    # turning an overfly into a descent. The camera mount offset is part of the
    # budget: top_height is camera-relative, the radius is body-relative.
    climb_needed = committed_climb(top_height, camera_offset, radius)
    headroom = max_altitude - cruise_altitude
    # overhead > 0 means the camera actually saw free space above the top edge.
    # If the obstacle fills the analysed band, top_row stops at row_top and
    # overhead is 0, so an unverifiable top never triggers a climb. The vertical
    # safety margin comes from climb_needed (radius above the top), not from
    # pre-existing free space in the image, which the vertical FOV cannot cover.
    overfly_ok = (overfly_armed and bool(blocking) and math.isfinite(top_height)
                  and climb_needed <= min(max_climb, headroom) and overhead > 0)

    if overfly_ok:
        # The flight loop holds position until the committed climb is gained and
        # only then advances, so this speed is the crossing speed over the top.
        speed = min(max_speed, max(0.25, (front - stop_distance) * 0.35))
        climb = 0.0 if climb_needed <= 0 else min(0.6, max(0.2, climb_needed * 0.3))
        return Command("overfly", max(0.0, speed), climb,
                       0.0, front, overhead, top_height, "overfly_obstacle")

    if not blocking and front > stop_distance + curve_distance:
        speed = min(max_speed, max(0.0, (front - stop_distance) * 0.5))
        return Command("advance", speed, 0.0, 0.0, front, overhead, top_height, "advance")

    options = []
    for col in range(half_window, width - half_window, max(1, width // 32)):
        angle = math.atan2(col - (width - 1) / 2, focal)
        window = profiles[col - half_window:col + half_window + 1]
        clearance = min((p.distance for p in window
                         if p.valid and math.isfinite(p.distance)), default=0.0)
        clearance = min(clearance, lateral_clearance(profiles, width, height, fov,
                                                     angle, radius))
        if clearance > stop_distance:
            score = (abs(wrap(angle - goal_angle))
                     + 0.8 * abs(wrap(angle - preferred_angle))
                     + 0.15 / min(clearance, 20))
            score += 1.5 / (clearance - stop_distance + 0.1)
            if avoidance_side and angle * avoidance_side < -math.radians(3):
                score += 1.5  # Hysteresis: keep the selected side of the obstacle.
            if avoidance_side and abs(angle) < math.radians(8):
                score += 0.6  # Do not aim at the obstacle centre while close.
            options.append((score, angle, clearance))
    if not options:
        return Command("blocked", 0.0, 0.0, 0.0, front, overhead, top_height, "blocked_hover")
    _, heading, clearance = min(options)
    yaw_rate = max(-MAX_YAW_RATE, min(MAX_YAW_RATE, math.degrees(heading) * 0.8))
    if abs(heading) > math.radians(12) or front <= stop_distance + curve_distance:
        speed = min(max_speed, max(0.3, (clearance - stop_distance) * 0.6))
        return Command("curve", speed, 0.0, yaw_rate, clearance, overhead, top_height,
                       "curve_around_obstacle", heading)
    speed = min(max_speed, max(0.0, (front - stop_distance) * 0.5))
    return Command("advance", speed, 0.0, yaw_rate, front, overhead, top_height,
                   "advance", heading)


def smooth_yaw_rate(previous: float, desired: float, dt: float) -> float:
    """Limit yaw acceleration and rate, including switches of turn direction."""
    step = MAX_YAW_RATE * min(max(dt, 0.01), 0.5)
    desired = max(-MAX_YAW_RATE, min(MAX_YAW_RATE, desired))
    return previous + max(-step, min(step, desired - previous))


def decode_unity_depth(values: list[float], near: float = 0.3,
                       far: float = 1000.0, compression: float = 0.25,
                       srgb: bool = False) -> list[float]:
    """Decode AirSim Unity's compressed DepthPerspective pixels to metres.

    The Unity CameraEffects shader returns ``pow(linear_depth, 0.25)`` for
    floating-point depth requests. The decoded value is the optical-axis
    depth in metres; invalid pixels remain invalid for the planner.
    """
    if not 0 < near < far or compression <= 0:
        raise ValueError("Invalid Unity depth calibration")
    near_over_far = near / far
    scale = 1.0 + near_over_far
    decoded = []
    for value in values:
        if not math.isfinite(value) or value <= 0:
            decoded.append(float("nan"))
            continue
        if value > 1:
            decoded.append(float("nan"))
            continue
        if srgb:
            value = value / 12.92 if value <= 0.04045 else ((value + 0.055) / 1.055) ** 2.4
        linear = value ** (1.0 / compression)
        normalized = linear / scale + near_over_far
        decoded.append(normalized * far)
    return decoded


def flip_rows(values: list[float], width: int, height: int) -> list[float]:
    """Reorder a bottom-up row-major buffer so row 0 is the image top.

    Unity textures use a bottom-left origin, and AirSim Unity's float depth
    buffer inherits that order. Verified on the live build: buffer row 5 reads
    the floor 1.4 m under a level, landed camera while row 131 reads the sky,
    i.e. exactly inverted relative to a top-down image. Every other module that
    only takes per-column minima is unaffected; the height logic here is not.
    """
    if len(values) != width * height:
        raise ValueError("Buffer size does not match the image")
    flipped = []
    for row in range(height - 1, -1, -1):
        flipped.extend(values[row * width:(row + 1) * width])
    return flipped


def check_new_collision(collision, initial_timestamp: int) -> None:
    """Ignore a latched pre-flight report; abort on any subsequent contact."""
    if collision.has_collided and collision.time_stamp != initial_timestamp:
        raise RuntimeError(
            f"New collision with {collision.object_name!r}; "
            f"timestamp={collision.time_stamp}, "
            f"penetration_depth={collision.penetration_depth}; aborting"
        )


def fly(args: argparse.Namespace) -> None:
    # Lazy import keeps offline tests independent of legacy RPC dependencies.
    try:
        import airsim
    except ModuleNotFoundError as error:
        raise SystemExit(
            f"Missing Python dependency: {error.name}. Install requirements.txt and "
            "set PYTHONPATH to AirSim 1.8.1/PythonClient; see README.md."
        ) from error

    client = airsim.MultirotorClient(ip=args.host, port=args.port, timeout_value=10)
    client.confirmConnection()
    vehicle = {"vehicle_name": args.vehicle}
    # Fail before arming if the simulator cannot supply float depth.
    request = airsim.ImageRequest(args.camera, airsim.ImageType.DepthVis,
                                  pixels_as_float=True, compress=False)
    info = client.simGetCameraInfo(args.camera, **vehicle)
    fov = info.fov
    # The camera is mounted above the body origin, while top_height is measured
    # relative to the camera. Measure the offset once (NED z down: the camera is
    # more negative than the body) and add it to every climb budget.
    camera_offset = 0.0
    try:
        camera_z = info.pose.position.z_val
        body_z = client.getMultirotorState(**vehicle).kinematics_estimated.position.z_val
        camera_offset = max(0.0, body_z - camera_z)
    except AttributeError:
        camera_offset = 0.0
    print(f"Camera mount offset: {camera_offset:.2f} m above the body origin")

    def read_frame():
        responses = client.simGetImages([request], **vehicle)
        if not responses or not responses[0].pixels_as_float:
            raise RuntimeError("Camera did not return float depth")
        frame = responses[0]
        decoded = decode_unity_depth(frame.image_data_float, args.depth_near,
                                     args.depth_far, srgb=args.depth_color_space == "srgb")
        decoded = flip_rows(decoded, frame.width, frame.height)
        return decoded, frame.width, frame.height, frame.time_stamp

    data, width, height, _ = read_frame()
    profiles = analyze_columns(data, width, height, fov, args.ground_rows)
    plan_height_aware(profiles, width, height, fov, 0, args.speed, args.stop_distance,
                      args.safety_margin, args.vehicle_radius,
                      curve_distance=args.curve_distance, max_climb=args.max_climb,
                      max_altitude=args.max_altitude, cruise_altitude=args.altitude,
                      camera_offset=camera_offset)
    print(f"Obstacle buffer: body radius {args.vehicle_radius:.2f} m + "
          f"surface clearance {args.safety_margin:.2f} m; "
          f"climb budget {args.max_climb:.2f} m under ceiling {args.max_altitude:.2f} m")
    if args.check_only:
        state = client.getMultirotorState(**vehicle)
        pos = state.kinematics_estimated.position
        print(f"Preflight OK: camera={width}x{height}, FOV={fov}, "
              f"position=({pos.x_val:.3f},{pos.y_val:.3f},{pos.z_val:.3f})")
        print("No arming or movement requested; verify the low obstacle height before flight.")
        return
    run_directory = create_run_directory(args.output, args)
    print(f"Run evidence: {run_directory.resolve()}")
    event_path = run_directory / "events.jsonl"
    record_event(event_path, "mission_started")
    controlled = False
    airborne = False
    initial_collision = client.simGetCollisionInfo(**vehicle)
    initial_collision_timestamp = initial_collision.time_stamp
    if initial_collision.has_collided:
        print(f"Pre-flight contact: {initial_collision.object_name!r}; "
              "will establish airborne collision baseline after climb")

    try:
        ground_z = client.getMultirotorState(**vehicle).kinematics_estimated.position.z_val
        client.enableApiControl(True, **vehicle)
        controlled = True
        if not client.isApiControlEnabled(**vehicle):
            raise RuntimeError("API control rejected; check Unity vehicle and RC settings")
        if not client.armDisarm(True, **vehicle):
            raise RuntimeError("Arming rejected")
        airborne = True  # Attempt landing in cleanup even if the climb fails midway.
        # This Unity build reports landed_state=Landed while airborne, so the
        # closed-loop takeoffAsync/moveToZAsync gate on a false state and return
        # False. Trigger takeoff to start the rotors, then climb through the same
        # open-loop velocity channel the cruise loop uses and judge by measured
        # altitude, which is trustworthy here (verified on the double-pillar run).
        client.takeoffAsync(timeout_sec=15, **vehicle).get()
        start = client.getMultirotorState(**vehicle).kinematics_estimated.position
        print(f"After takeoff request: x={start.x_val:.2f}, y={start.y_val:.2f}, "
              f"z={start.z_val:.2f}")
        route = (route_from_start(start.x_val, start.y_val, args.goal_x, args.goal_y)
                 if args.route == "rectangle" else
                 [(start.x_val + args.goal_x, start.y_val + args.goal_y)])
        print(f"Route targets (N/E metres): {route}")
        route_index = 0
        overfly = OverflyState()
        avoidance_side = 0
        preferred_heading = None
        clear_frames = 0
        previous_yaw_rate = 0.0
        previous_command_time = time.monotonic()
        cruise_z = start.z_val - args.altitude
        ceiling_z = start.z_val - args.max_altitude

        def climb_to(z_target):
            """Open-loop proportional climb judged by measured altitude."""
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                z_now = client.getMultirotorState(**vehicle).kinematics_estimated.position.z_val
                error = z_target - z_now
                if abs(error) < 0.25:
                    break
                vz = max(-0.6, min(0.6, error * 0.5))
                client.moveByVelocityAsync(
                    0.0, 0.0, vz, 0.3,
                    drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                    yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=0.0),
                    **vehicle).join()
            client.hoverAsync(**vehicle).join()
            return client.getMultirotorState(**vehicle).kinematics_estimated.position

        climb_pos = climb_to(cruise_z)
        print(f"After climb: z={climb_pos.z_val:.2f}, cruise_z={cruise_z:.2f}, "
              f"ceiling_z={ceiling_z:.2f}")
        if climb_pos.z_val > cruise_z + 0.6:
            raise RuntimeError("Climb failed; vehicle did not reach requested altitude; "
                               "check the spawn area is clear and Unity is stepping physics")
        # Unity stamps a fresh collision on every ground-contact frame, so set the
        # flight baseline after the climb or cleanup lands it right after liftoff.
        airborne_collision = client.simGetCollisionInfo(**vehicle)
        initial_collision_timestamp = airborne_collision.time_stamp
        if airborne_collision.has_collided:
            print(f"Airborne collision baseline: {airborne_collision.object_name!r}; "
                  "ground contact during takeoff is ignored")

        def check_collision():
            check_new_collision(client.simGetCollisionInfo(**vehicle),
                                initial_collision_timestamp)

        start_time = time.monotonic()
        watchdog = ProgressWatchdog((climb_pos.x_val, climb_pos.y_val), start_time)
        last_frame_stamp = None
        with (run_directory / "flight.csv").open("w", newline="", encoding="utf-8") as stream:
            log = csv.writer(stream)
            log.writerow(["elapsed_s", "x", "y", "z", "lift_m", "speed", "climb", "yaw_rate",
                          "clearance_m", "top_height_m", "overhead_m", "mode",
                          "route_index", "target_x", "target_y", "yaw_rad", "steer_rad",
                          "avoidance_side", "overfly_active", "overfly_hold",
                          "overfly_done", "overfly_armed"])
            while time.monotonic() - start_time < args.timeout:
                if not client.isApiControlEnabled(**vehicle):
                    raise RuntimeError("API control lost during flight; check Unity/RC or other scripts")
                state = client.getMultirotorState(**vehicle)
                pos = state.kinematics_estimated.position
                lift = max(0.0, cruise_z - pos.z_val)
                check_collision()
                target_x, target_y = route[route_index]
                dx, dy = target_x - pos.x_val, target_y - pos.y_val
                if math.hypot(dx, dy) < 0.8:
                    if route_index == len(route) - 1:
                        print("Goal reached")
                        record_event(event_path, "goal_reached",
                                     f"overfly_completed={overfly.completed}")
                        log.writerow([round(time.monotonic() - start_time, 3), pos.x_val,
                                      pos.y_val, pos.z_val, lift, 0, 0, 0, "", "", "",
                                      "goal_reached", route_index, target_x, target_y,
                                      "", "", avoidance_side, int(overfly.active),
                                      int(overfly.hold), overfly.completed,
                                      int(overfly.armed)])
                        stream.flush()
                        break
                    route_index += 1
                    watchdog = ProgressWatchdog((pos.x_val, pos.y_val), time.monotonic())
                    print(f"route waypoint {route_index}/{len(route) - 1} reached")
                    continue
                yaw = airsim.to_eularian_angles(state.kinematics_estimated.orientation)[2]
                # One frame per control step: depth and its timestamp must pair,
                # so a frozen camera is detected on the same image being planned.
                depth, width, height, stamp = read_frame()
                if stamp == last_frame_stamp:
                    raise RuntimeError("Repeated depth timestamp; camera may be frozen")
                last_frame_stamp = stamp
                profiles = analyze_columns(depth, width, height, fov, args.ground_rows)
                cmd = plan_height_aware(
                    profiles, width, height, fov, wrap(math.atan2(dy, dx) - yaw),
                    args.speed, args.stop_distance, args.safety_margin, args.vehicle_radius,
                    wrap(preferred_heading - yaw) if preferred_heading is not None else 0,
                    avoidance_side, args.curve_distance, args.max_climb, args.max_altitude,
                    args.altitude, overfly.armed, camera_offset=camera_offset)

                # Commit to, hold, and finish a climb; disarm after failures.
                if cmd.mode == "overfly":
                    was_active = overfly.active
                    climb = committed_climb(cmd.top_height, camera_offset,
                                            args.vehicle_radius + args.safety_margin)
                    if overfly.request(climb):
                        overfly.climb_started = time.monotonic()
                        if not was_active:
                            print(f"Overfly committed: climb {overfly.climb_target:.2f} m "
                                  f"above cruise (top={cmd.top_height:.2f} m)")
                            record_event(event_path, "overfly_started",
                                         f"climb={overfly.climb_target:.2f} "
                                         f"top={cmd.top_height:.2f}")
                    clear_frames = 0
                elif cmd.mode == "advance":
                    held = overfly.climb_target
                    if overfly.note_clear():
                        overfly.begin_hold(held, (pos.x_val, pos.y_val))
                        print(f"Overfly completed ({overfly.completed}); holding "
                              f"{overfly.hold_altitude:.2f} m for "
                              f"{args.overfly_hold_distance:.1f} m past the obstacle")
                        record_event(event_path, "overfly_completed",
                                     f"count={overfly.completed} "
                                     f"hold={overfly.hold_altitude:.2f}")
                    clear_frames += 1
                    if clear_frames >= 6:
                        avoidance_side = 0
                elif cmd.mode == "curve":
                    overfly.suspend_clear()
                    if not avoidance_side:
                        avoidance_side = 1 if cmd.steer_angle >= 0 else -1
                        print(f"Avoidance side locked: {avoidance_side:+d}; bypassing tall obstacle")
                        record_event(event_path, "lateral_bypass",
                                     f"side={avoidance_side:+d} top={cmd.top_height:.2f}")
                    clear_frames = 0

                if cmd.mode == "blocked":
                    overfly.suspend_clear()
                    scan_sign = avoidance_side or (1 if wrap(math.atan2(dy, dx) - yaw) >= 0 else -1)
                    cmd = Command("blocked", 0.0, 0.0, scan_sign * MAX_YAW_RATE, 0.0,
                                  cmd.overhead, cmd.top_height, "scan_for_exit")
                now = time.monotonic()
                watchdog.update(now, (pos.x_val, pos.y_val), cmd.mode == "blocked")

                # A committed climb only fails when the vehicle has not gained the
                # committed height after the grace period. Judging the first frames
                # of a climb would disarm overfly before it had time to rise.
                if overfly.check_progress(now, lift):
                    print(f"Overfly not gaining (lift={lift:.2f} m, "
                          f"target={overfly.climb_target:.2f} m); counting a failed climb")
                    record_event(event_path, "overfly_failed",
                                 f"lift={lift:.2f} target={overfly.climb_target:.2f}")
                    overfly.fail()
                    if not overfly.armed:
                        print("Overfly disarmed for this mission; lateral bypass only")
                        record_event(event_path, "overfly_disarmed")

                # Keep the climbed altitude for the configured distance past the
                # obstacle before descending back to cruise.
                if overfly.hold:
                    travelled = math.hypot(pos.x_val - overfly.hold_start[0],
                                           pos.y_val - overfly.hold_start[1])
                    if travelled >= args.overfly_hold_distance:
                        overfly.end_hold()
                        print("Overfly hold complete; descending to cruise altitude")
                        record_event(event_path, "overfly_hold_done",
                                     f"travelled={travelled:.2f}")

                # Absolute ceiling: never trade the climb budget for a collision.
                if overfly.active:
                    target_z = cruise_z - overfly.climb_target
                elif overfly.hold:
                    target_z = cruise_z - overfly.hold_altitude
                else:
                    target_z = cruise_z
                target_z = max(target_z, ceiling_z)
                vz = max(-0.6, min(0.6, (target_z - pos.z_val) * 0.8))

                speed = min(cmd.speed, math.hypot(dx, dy) * 0.5)
                if overfly.active and overfly.climb_target - lift > 0.3:
                    # Rise in place first: crossing while still below the top
                    # would trade the climb for forward motion into the obstacle.
                    speed = 0.0
                yaw_rate = smooth_yaw_rate(previous_yaw_rate, cmd.yaw_rate,
                                           now - previous_command_time)
                previous_yaw_rate, previous_command_time = yaw_rate, now
                travel_heading = yaw + cmd.steer_angle if speed > 0 else yaw
                client.moveByVelocityAsync(
                    speed * math.cos(travel_heading), speed * math.sin(travel_heading), vz, 0.3,
                    drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                    yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate),
                    **vehicle).join()
                log.writerow([round(time.monotonic() - start_time, 3), pos.x_val, pos.y_val,
                              pos.z_val, lift, speed, cmd.climb, yaw_rate, cmd.clearance,
                              cmd.top_height, cmd.overhead, cmd.mode, route_index,
                              target_x, target_y, yaw, cmd.steer_angle, avoidance_side,
                              int(overfly.active), int(overfly.hold),
                              overfly.completed, int(overfly.armed)])
                stream.flush()
                print(f"{cmd.reason}: forward={speed:.2f}, climb={cmd.climb:.2f}, "
                      f"clearance={cmd.clearance:.2f} m, top={cmd.top_height:.2f} m, "
                      f"overhead={cmd.overhead:.2f} m, lift={lift:.2f} m")
            else:
                raise TimeoutError("Mission timeout; goal not reached")
    except KeyboardInterrupt:
        record_event(event_path, "interrupted", "Ctrl+C")
        raise
    except Exception as error:
        record_event(event_path, "timeout" if isinstance(error, TimeoutError) else "abort",
                     f"{type(error).__name__}: {error}")
        raise
    finally:
        if controlled:
            landed = not airborne
            try:
                if airborne:
                    client.enableApiControl(True, **vehicle)
                    if not client.isApiControlEnabled(**vehicle):
                        raise RuntimeError("Cannot recover API control for landing; stop Unity Play")
                    client.cancelLastTask(**vehicle)
                    client.hoverAsync(**vehicle).join()
                    # landed_state is unreliable on this build and landAsync timed
                    # out after a verified goal on the double-pillar run, so
                    # descend through the open-loop channel and judge by measured
                    # altitude, keeping landAsync only as a best-effort disarm.
                    descent_deadline = time.monotonic() + 25
                    while time.monotonic() < descent_deadline:
                        z_now = client.getMultirotorState(**vehicle).kinematics_estimated.position.z_val
                        if z_now > ground_z - 0.3:
                            break
                        client.moveByVelocityAsync(
                            0.0, 0.0, 0.6, 0.3,
                            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                            yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=0.0),
                            **vehicle).join()
                    try:
                        land_result = bool(client.landAsync(timeout_sec=5, **vehicle).get())
                    except Exception as error:
                        land_result = False
                        print(f"landAsync best-effort failed: {type(error).__name__}: {error}")
                    final_z = client.getMultirotorState(**vehicle).kinematics_estimated.position.z_val
                    landed = final_z > ground_z - 0.5
                    print(f"Landing confirmed: {landed} (final_z={final_z:.2f}, "
                          f"ground_z={ground_z:.2f}, landAsync={land_result})")
                if landed:
                    client.armDisarm(False, **vehicle)
                else:
                    print("Landing unconfirmed: API control retained; stop Unity Play before retrying")
            except Exception as error:
                record_event(event_path, "landing_error", f"{type(error).__name__}: {error}")
                raise
            finally:
                record_event(event_path, "landing_confirmed" if landed else "landing_unconfirmed")
                if landed:
                    client.enableApiControl(False, **vehicle)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--check-only", action="store_true",
                        help="Check RPC, vehicle and depth without arming or flying")
    parser.add_argument("--port", type=int, default=41451)
    parser.add_argument("--vehicle", default="SimpleFlight", help="AirSim vehicle name")
    parser.add_argument("--camera", default="0")
    parser.add_argument("--goal-x", type=float, default=30.0)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--altitude", type=float, default=3.0,
                        help="巡航高度，相对起飞点，米")
    parser.add_argument("--max-climb", type=float, default=3.5,
                        help="单次越障允许的最大抬升，米；须大于机体半径+额外间距才可能越障")
    parser.add_argument("--max-altitude", type=float, default=7.0,
                        help="相对起飞点的绝对高度上限，米；须大于 altitude+max_climb 才不截断越障")
    parser.add_argument("--overfly-hold-distance", type=float, default=5.0,
                        help="越过障碍后保持越障高度继续飞行的水平距离，米；0 表示越过即下降")
    parser.add_argument("--speed", type=float, default=1.2)
    parser.add_argument("--stop-distance", type=float, default=3.0)
    parser.add_argument("--curve-distance", type=float, default=2.0,
                        help="距离障碍多早开始侧向绕行，米")
    parser.add_argument("--safety-margin", type=float, default=1.5,
                        help="障碍面之外的额外间距，米")
    parser.add_argument("--vehicle-radius", type=float, default=0.6,
                        help="含桨叶的机体半径，米")
    parser.add_argument("--ground-rows", type=int, default=GROUND_ROWS_DEFAULT,
                        help="图像底部按地面处理而不当作障碍的行数")
    parser.add_argument("--depth-near", type=float, default=0.3,
                        help="Unity 相机近平面距离，米")
    parser.add_argument("--depth-far", type=float, default=1000.0,
                        help="Unity 相机远平面距离，米")
    parser.add_argument("--depth-color-space", choices=("srgb", "linear"), default="srgb",
                        help="Linear 色彩空间的 Unity 工程 ARGB32 采集用 srgb")
    parser.add_argument("--route", choices=("goal", "rectangle"), default="goal",
                        help="goal: 飞往偏移目标; rectangle: 绕行矩形返回")
    parser.add_argument("--timeout", type=float, default=180)
    parser.add_argument("--output", type=Path, default=Path("artifacts/height_aware_overfly"))
    args = parser.parse_args(argv)
    finite = (args.goal_x, args.goal_y, args.altitude, args.speed, args.stop_distance,
              args.timeout, args.safety_margin, args.vehicle_radius, args.curve_distance,
              args.max_climb, args.max_altitude, args.overfly_hold_distance)
    if (not all(math.isfinite(v) for v in finite)
            or not 0 < args.speed <= 1.5 or not 1 <= args.altitude <= 10
            or args.stop_distance < 2 or args.timeout <= 0
            or args.safety_margin < 0 or args.vehicle_radius <= 0
            or args.curve_distance < 0 or args.max_climb < 0
            or args.max_altitude <= args.altitude
            or args.overfly_hold_distance < 0
            or args.ground_rows < 0
            or not 0 < args.depth_near < args.depth_far):
        parser.error("Require speed (0,1.5], altitude [1,10], stop-distance >=2, "
                     "max-altitude > altitude, non-negative max-climb and hold-distance, "
                     "and positive timeout")
    return args


def main():
    fly(parse_args())


if __name__ == "__main__":
    main()
