"""AirSim front-depth reactive navigation for a two-pillar slalom. NED metres; yaw radians.

The camera must be level, forward facing and centred on the vehicle.
This is a local planner for static obstacles, not a collision-free guarantee.

Compared with the single-pillar reference (PR #597), the planner re-selects the
bypass side once the vehicle has genuinely cleared one pillar, so a second
pillar standing on the other side of the route produces an S-shaped path instead
of a same-side loop. Side switching is guarded by a short cooldown so a narrow
gap between the two pillars does not trigger a premature swap.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import time
import tempfile
from dataclasses import dataclass, field
from pathlib import Path


MAX_YAW_RATE = 12.0  # degrees/s, shared by planner, smoothing and blocked scan.
RELEASE_FRAMES = 6  # consecutive clear frames needed before a pillar counts as cleared.
SIDE_COOLDOWN_FRAMES = 4  # extra frames after clearing a pillar before the side may flip again.


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
class SlalomState:
    """Remember which pillar side is locked and count cleared pillars.

    Pure state kept out of the AirSim client so it can be tested offline.
    ``side`` is -1/0/+1. After a pillar is cleared the side unlocks so a
    pillar on the opposite side gets a fresh choice (an S path). A short
    ``cooldown`` keeps the side unlocked for a few frames while the vehicle
    crosses the corridor between two pillars, so residual depth of the pillar
    it just passed does not immediately re-lock a contradicting side and make
    the vehicle wobble.
    """
    side: int = 0
    clear_frames: int = 0
    cooldown: int = 0
    pillars_cleared: int = 0

    def update(self, reason: str, steer_angle: float,
               release_frames: int = RELEASE_FRAMES) -> bool:
        """Advance one control step; return True when a pillar was just cleared."""
        if reason == "curve_around_obstacle":
            self.clear_frames = 0
            if self.cooldown > 0:
                # Still crossing the gap: stay unlocked, let the planner pick
                # freely without committing to a side yet.
                self.cooldown -= 1
            elif self.side == 0:
                self.side = 1 if steer_angle >= 0 else -1
            return False
        if reason == "advance":
            if self.cooldown > 0:
                self.cooldown -= 1
            self.clear_frames += 1
            if self.side and self.clear_frames >= release_frames:
                # The locked side has served this pillar; free the choice so a
                # pillar on the opposite side is bypassed the other way (an S).
                self.side = 0
                self.cooldown = SIDE_COOLDOWN_FRAMES
                self.pillars_cleared += 1
                self.clear_frames = 0
                return True
        return False


@dataclass
class Command:
    speed: float  # body-forward m/s; never strafe into unseen space
    yaw_rate: float  # degrees/s, positive turns right
    clearance: float
    reason: str
    steer_angle: float = 0.0  # selected camera-relative direction, radians
    front_clearance: float = 0.0


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


def plan(depth: list[float], width: int, height: int, fov: float,
         goal_angle: float, max_speed: float = 1.0,
         stop_distance: float = 2.0, safety_margin: float = 1.0,
         vehicle_radius: float = 0.6, preferred_angle: float = 0.0,
         avoidance_side: int = 0, curve_distance: float = 1.5) -> Command:
    """Use DepthPerspective metres, conservatively checking corridor minima.

    Invalid pixels are treated as blocked. All sampled rows must be clear.
    Inflate observed obstacles by vehicle_radius + safety_margin. Scan the central
    half of the image vertically: obstacles outside that band are not covered.
    """
    if width < 16 or height < 8 or len(depth) != width * height:
        raise ValueError("Invalid or empty depth image")
    if (not 30 <= fov <= 140 or max_speed <= 0 or stop_distance < 1
            or safety_margin < 0 or vehicle_radius <= 0):
        raise ValueError("Invalid planner settings")
    focal = width / (2 * math.tan(math.radians(fov) / 2))
    # Radial depth -> optical-axis depth, conservative for off-axis pixels.
    columns = []
    valid = 0
    for col in range(width):
        minimum = float("inf")
        for row in range(height // 4, 3 * height // 4):
            distance = depth[row * width + col]
            if not math.isfinite(distance) or distance <= 0:
                minimum = 0.0
            else:
                valid += 1
                cosine = 1 / math.sqrt(1 + ((col - (width - 1) / 2) / focal) ** 2
                                       + ((row - (height - 1) / 2) / focal) ** 2)
                minimum = min(minimum, distance * cosine)
        columns.append(minimum)
    if valid == 0:
        raise ValueError("No valid depth pixels; stop the mission")
    if any(distance <= 0 for distance in columns):
        return Command(0, 0, 0, "blocked_hover")
    half_window = max(2, math.ceil(focal * 0.6 / stop_distance))
    radius = vehicle_radius + safety_margin
    # Project observed points onto each proposed travel direction. This checks
    # a swept body corridor, including near obstacles beside the image centre.
    points = [(distance * (col - (width - 1) / 2) / focal, distance)
              for col, distance in enumerate(columns) if distance > 0]

    def corridor_clearance(angle):
        cosine, sine = math.cos(angle), math.sin(angle)
        clearance = float("inf")
        for x, z in points:
            along = z * cosine + x * sine
            lateral = abs(x * cosine - z * sine)
            if along >= 0 and lateral <= radius:
                entry = along - math.sqrt(max(0, radius * radius - lateral * lateral))
                clearance = min(clearance, max(0, entry))
        return clearance

    options = []
    for col in range(half_window, width - half_window, max(1, width // 32)):
        angle = math.atan2(col - (width - 1) / 2, focal)
        clearance = min(columns[col - half_window:col + half_window + 1])
        clearance = min(clearance, corridor_clearance(angle))
        if clearance > stop_distance:
            score = (abs(wrap(angle - goal_angle))
                     + 0.8 * abs(wrap(angle - preferred_angle))
                     + 0.15 / min(clearance, 20))
            score += 1.5 / (clearance - stop_distance + 0.1)
            if avoidance_side and angle * avoidance_side < -math.radians(3):
                score += 1.5  # Hysteresis: prefer the selected side of the obstacle.
            if avoidance_side and abs(angle) < math.radians(8):
                score += 0.6  # Do not aim at the obstacle centre while close.
            options.append((score, angle, clearance))
    if not options:
        return Command(0, 0, 0, "blocked_hover")
    _, heading, clearance = min(options)
    yaw_rate = max(-MAX_YAW_RATE, min(MAX_YAW_RATE, math.degrees(heading) * 0.8))
    front = min(columns[width // 2 - half_window:width // 2 + half_window + 1])
    front = min(front, corridor_clearance(0))
    if abs(heading) > math.radians(12) or front <= stop_distance + curve_distance:
        # The selected direction has its own inflated, scanned corridor. Fly
        # along it while yaw follows gently, instead of waiting for front to clear.
        speed = min(max_speed, max(0.3, (clearance - stop_distance) * 0.6))
        return Command(speed, yaw_rate, clearance, "curve_around_obstacle", heading, front)
    speed = min(max_speed, max(0.0, (front - stop_distance) * 0.5))
    return Command(speed, yaw_rate, front, "advance", heading)


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
    # This Unity plugin only enables the depth shader for DepthVis.
    request = airsim.ImageRequest(args.camera, airsim.ImageType.DepthVis,
                                  pixels_as_float=True, compress=False)
    info = client.simGetCameraInfo(args.camera, **vehicle)
    fov = info.fov

    def read_depth():
        responses = client.simGetImages([request], **vehicle)
        if not responses or not responses[0].pixels_as_float:
            raise RuntimeError("Camera did not return float depth")
        frame = responses[0]
        return (decode_unity_depth(frame.image_data_float, args.depth_near,
                                   args.depth_far, srgb=args.depth_color_space == "srgb"),
                frame.width, frame.height)

    data, width, height = read_depth()
    plan(data, width, height, fov, 0, args.speed, args.stop_distance,
         args.safety_margin, args.vehicle_radius)
    print(f"Obstacle buffer: body radius {args.vehicle_radius:.2f} m + "
          f"surface clearance {args.safety_margin:.2f} m")
    if args.check_only:
        state = client.getMultirotorState(**vehicle)
        pos = state.kinematics_estimated.position
        print(f"Preflight OK: camera={width}x{height}, FOV={fov}, "
              f"position=({pos.x_val:.3f},{pos.y_val:.3f},{pos.z_val:.3f})")
        print("No arming or movement requested; check the known-distance obstacle before flight.")
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
        # AirSim Unity reports landed_state unreliably (it stays Landed even several
        # metres airborne), so takeoffAsync/moveToZAsync, which gate success on that
        # state, falsely return False in this environment. Trigger the takeoff to
        # arm liftoff, then climb with the same open-loop velocity channel the
        # cruise loop uses and judge the result from the measured altitude, which
        # the probe confirmed is trustworthy here.
        client.takeoffAsync(timeout_sec=15, **vehicle).get()
        start = client.getMultirotorState(**vehicle).kinematics_estimated.position
        print(f"After takeoff request: x={start.x_val:.2f}, y={start.y_val:.2f}, z={start.z_val:.2f}")
        route = (route_from_start(start.x_val, start.y_val, args.goal_x, args.goal_y)
                 if args.route == "rectangle" else
                 [(start.x_val + args.goal_x, start.y_val + args.goal_y)])
        print(f"Route targets (N/E metres): {route}")
        route_index = 0
        slalom = SlalomState()
        preferred_heading = None
        previous_yaw_rate = 0.0
        previous_command_time = time.monotonic()
        target_z = start.z_val - args.altitude

        def climb_to(z_target):
            """Open-loop proportional climb; returns the measured kinematic state.

            Uses negative world-frame z velocity to rise, the direction the
            Unity plugin honours, instead of the state-checked moveToZAsync.
            """
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                z_now = client.getMultirotorState(**vehicle).kinematics_estimated.position.z_val
                error = z_target - z_now  # negative means still below target (must rise)
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

        climb_pos = climb_to(target_z)
        print(f"After climb: z={climb_pos.z_val:.2f}, target_z={target_z:.2f}")
        if climb_pos.z_val > target_z + 0.6:
            raise RuntimeError("Climb failed; vehicle did not reach requested altitude; "
                               "check the spawn area is clear and Unity is stepping physics")
        # Unity emits a fresh collision timestamp on every ground-contact frame.
        # Establish the flight baseline after takeoff/climb, so floor contact
        # does not make cleanup land the vehicle immediately after liftoff.
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
            log.writerow(["elapsed_s", "x", "y", "z", "speed", "yaw_rate", "clearance_m",
                          "state", "route_index", "target_x", "target_y", "yaw_rad",
                          "steer_rad", "avoidance_side", "pillars_cleared"])
            while time.monotonic() - start_time < args.timeout:
                if not client.isApiControlEnabled(**vehicle):
                    raise RuntimeError("API control lost during flight; check Unity/RC or other scripts")
                state = client.getMultirotorState(**vehicle)
                pos = state.kinematics_estimated.position
                check_collision()
                target_x, target_y = route[route_index]
                dx, dy = target_x - pos.x_val, target_y - pos.y_val
                if math.hypot(dx, dy) < 0.8:
                    if route_index == len(route) - 1:
                        print("Goal reached")
                        record_event(event_path, "goal_reached",
                                     f"pillars_cleared={slalom.pillars_cleared}")
                        log.writerow([round(time.monotonic() - start_time, 3), pos.x_val,
                                      pos.y_val, pos.z_val, 0, 0, "", "goal_reached",
                                      route_index, target_x, target_y, "", "",
                                      slalom.side, slalom.pillars_cleared])
                        stream.flush()
                        break
                    route_index += 1
                    watchdog = ProgressWatchdog((pos.x_val, pos.y_val), time.monotonic())
                    print(f"route waypoint {route_index}/{len(route) - 1} reached")
                    continue
                yaw = airsim.to_eularian_angles(state.kinematics_estimated.orientation)[2]
                # Include depth timestamp so a frozen image is not accepted.
                responses = client.simGetImages([request], **vehicle)
                if not responses or not responses[0].pixels_as_float:
                    raise RuntimeError("Depth unavailable")
                frame = responses[0]
                if frame.time_stamp == last_frame_stamp:
                    raise RuntimeError("Repeated depth timestamp; camera may be frozen")
                last_frame_stamp = frame.time_stamp
                depth = decode_unity_depth(frame.image_data_float, args.depth_near,
                                           args.depth_far, srgb=args.depth_color_space == "srgb")
                cmd = plan(depth, frame.width, frame.height, fov,
                           wrap(math.atan2(dy, dx) - yaw), args.speed, args.stop_distance,
                           args.safety_margin, args.vehicle_radius,
                           wrap(preferred_heading - yaw) if preferred_heading is not None else 0,
                           # side is 0 while unlocked/cooling down, so the planner
                           # re-selects freely across the gap between pillars.
                           slalom.side, args.curve_distance)
                side_before = slalom.side
                if slalom.update(cmd.reason, cmd.steer_angle):
                    print(f"Pillar cleared ({slalom.pillars_cleared}); side unlocked for re-selection")
                    record_event(event_path, "pillar_cleared",
                                 f"pillars_cleared={slalom.pillars_cleared}")
                if side_before == 0 and slalom.side != 0:
                    print(f"Avoidance side locked: {slalom.side:+d}; following scanned corridor")
                if cmd.speed > 0:
                    preferred_heading = yaw + cmd.steer_angle
                watchdog.update(time.monotonic(), (pos.x_val, pos.y_val),
                                cmd.reason == "blocked_hover")
                if cmd.reason == "blocked_hover":
                    # Rotate toward the chosen bypass side to bring a free sector
                    # into view instead of repeatedly turning back toward the goal.
                    scan_sign = slalom.side or (1 if wrap(math.atan2(dy, dx) - yaw) >= 0 else -1)
                    cmd = Command(0, scan_sign * MAX_YAW_RATE, 0, "scan_for_exit")
                vz = max(-0.4, min(0.4, (target_z - pos.z_val) * 0.5))
                now = time.monotonic()
                yaw_rate = smooth_yaw_rate(previous_yaw_rate, cmd.yaw_rate,
                                           now - previous_command_time)
                previous_yaw_rate, previous_command_time = yaw_rate, now
                travel_heading = yaw + cmd.steer_angle if cmd.speed > 0 else yaw
                speed = min(cmd.speed, math.hypot(dx, dy) * 0.5)
                client.moveByVelocityAsync(
                    speed * math.cos(travel_heading), speed * math.sin(travel_heading), vz, 0.3,
                    drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                    yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate),
                    **vehicle).join()
                log.writerow([round(time.monotonic() - start_time, 3), pos.x_val, pos.y_val,
                              pos.z_val, speed, yaw_rate, cmd.clearance, cmd.reason,
                              route_index, target_x, target_y, yaw, cmd.steer_angle,
                              slalom.side, slalom.pillars_cleared])
                stream.flush()
                print(f"{cmd.reason}: forward={cmd.speed:.2f}, clearance={cmd.clearance:.2f} m, "
                      f"side={slalom.side:+d}, cleared={slalom.pillars_cleared}")
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
                    # Cancel any outstanding motion before issuing cleanup commands.
                    client.enableApiControl(True, **vehicle)
                    if not client.isApiControlEnabled(**vehicle):
                        raise RuntimeError("Cannot recover API control for landing; stop Unity Play")
                    client.cancelLastTask(**vehicle)
                    client.hoverAsync(**vehicle).join()
                    # The verified run reached the goal but landAsync then timed out
                    # after 30 s, so descend with the open-loop velocity channel that
                    # the climb already proved works, and keep landAsync only as a
                    # best-effort disarm request whose failure must not mask the
                    # measured altitude. landed_state is unreliable on this build.
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
                    # Within 0.5 m of the pre-takeoff ground height means it came down.
                    landed = final_z > (ground_z - 0.5)
                    print(f"Landing confirmed: {landed} "
                          f"(final_z={final_z:.2f}, ground_z={ground_z:.2f}, "
                          f"landAsync={land_result})")
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
    parser.add_argument("--vehicle", default="SimpleFlight",
                        help="AirSim vehicle name")
    parser.add_argument("--camera", default="0")
    parser.add_argument("--goal-x", type=float, default=38.0)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--altitude", type=float, default=3.0)
    parser.add_argument("--speed", type=float, default=1.2)
    parser.add_argument("--stop-distance", type=float, default=3.0)
    parser.add_argument("--curve-distance", type=float, default=2.0,
                        help="距离障碍多早开始侧向曲线绕行，米")
    parser.add_argument("--safety-margin", type=float, default=1.5,
                        help="障碍面之外的额外间距，米")
    parser.add_argument("--vehicle-radius", type=float, default=0.6,
                        help="含桨叶的机体半径，米")
    parser.add_argument("--depth-near", type=float, default=0.3,
                        help="Unity 相机近平面距离，米")
    parser.add_argument("--depth-far", type=float, default=1000.0,
                        help="Unity 相机远平面距离，米")
    parser.add_argument("--depth-color-space", choices=("srgb", "linear"), default="srgb",
                        help="Linear 色彩空间的 Unity 工程 ARGB32 采集用 srgb")
    parser.add_argument("--route", choices=("goal", "rectangle"), default="goal",
                        help="goal: 飞往偏移目标; rectangle: 绕行矩形返回")
    parser.add_argument("--timeout", type=float, default=180)
    parser.add_argument("--output", type=Path, default=Path("artifacts/double_pillar_avoidance"))
    args = parser.parse_args(argv)
    if (not all(math.isfinite(v) for v in (args.goal_x, args.goal_y, args.altitude,
                                          args.speed, args.stop_distance, args.timeout,
                                          args.safety_margin, args.vehicle_radius, args.curve_distance))
            or not 0 < args.speed <= 1.5 or not 1 <= args.altitude <= 10
            or args.stop_distance < 2 or args.timeout <= 0
            or args.safety_margin < 0 or args.vehicle_radius <= 0
            or args.curve_distance < 0
            or not 0 < args.depth_near < args.depth_far):
        parser.error("Use speed (0,1.5], altitude [1,10], stop-distance >=2 and positive timeout")
    return args


def main():
    fly(parse_args())


if __name__ == "__main__":
    main()
