"""AirSim front-depth reactive navigation. NED metres; yaw in radians.

The camera must be level, forward facing and centred on the vehicle.
This is a local planner for static obstacles, not a collision-free guarantee.
"""
from __future__ import annotations

import argparse
import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Command:
    speed: float  # body-forward m/s; never strafe into unseen space
    yaw_rate: float  # degrees/s, positive turns right
    clearance: float
    reason: str
    steer_angle: float = 0.0  # selected camera-relative direction, radians
    front_clearance: float = 0.0


def route_from_start(start_x: float, start_y: float, goal_x: float,
                     goal_y: float) -> list[tuple[float, float]]:
    """Return a rectangular inspection route in AirSim world N/E metres."""
    return [(start_x + goal_x, start_y),
            (start_x + goal_x, start_y + goal_y),
            (start_x, start_y + goal_y),
            (start_x, start_y)]


def insert_detour(route: list[tuple[float, float]], index: int,
                  x: float, y: float) -> None:
    """Insert one temporary waypoint before the active route waypoint."""
    if index < 0 or index >= len(route):
        raise IndexError("active route index is out of range")
    route.insert(index, (x, y))


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
    yaw_rate = max(-12.0, min(12.0, math.degrees(heading) * 0.8))
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
    step = 12.0 * min(max(dt, 0.01), 0.5)
    desired = max(-12.0, min(12.0, desired))
    return previous + max(-step, min(step, desired - previous))


def detour_point(position_x: float, position_y: float, yaw: float,
                 command: Command, distance: float = 3.0) -> tuple[float, float]:
    """Convert the selected free camera sector into a temporary N/E waypoint."""
    if command.reason != "turn_to_clear_sector" or command.clearance <= 0:
        raise ValueError("A clear turning sector is required for a detour")
    distance = min(distance, command.clearance - 1.0, 6.0)
    if distance < 1.0:
        raise ValueError("Insufficient clearance for a detour")
    heading = yaw + command.steer_angle
    return position_x + distance * math.cos(heading), position_y + distance * math.sin(heading)


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
        import sys
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
                                   args.depth_far, srgb=args.depth_color_space == "srgb"), frame.width, frame.height)

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
    args.output.mkdir(parents=True, exist_ok=True)
    controlled = False
    airborne = False
    initial_collision = client.simGetCollisionInfo(**vehicle)
    initial_collision_timestamp = initial_collision.time_stamp
    if initial_collision.has_collided:
        print(f"Pre-flight contact: {initial_collision.object_name!r}; "
              "will establish airborne collision baseline after climb")

    try:
        client.enableApiControl(True, **vehicle)
        controlled = True
        if not client.isApiControlEnabled(**vehicle):
            raise RuntimeError("API control rejected; check Unity vehicle and RC settings")
        if not client.armDisarm(True, **vehicle):
            raise RuntimeError("Arming rejected")
        airborne = True  # Also attempt landing if takeoff RPC fails midway.
        takeoff_result = client.takeoffAsync(timeout_sec=15, **vehicle).get()
        if takeoff_result is False:
            raise RuntimeError("Takeoff failed; check Unity ground contact and API control")
        start = client.getMultirotorState(**vehicle).kinematics_estimated.position
        print(f"After takeoff: x={start.x_val:.2f}, y={start.y_val:.2f}, z={start.z_val:.2f}")
        route = (route_from_start(start.x_val, start.y_val, args.goal_x, args.goal_y)
                 if args.route == "rectangle" else
                 [(start.x_val + args.goal_x, start.y_val + args.goal_y)])
        print(f"Route targets (N/E metres): {route}")
        route_index = 0
        avoidance_side = 0
        preferred_heading = None
        clear_frames = 0
        previous_yaw_rate = 0.0
        previous_command_time = time.monotonic()
        progress_position = (start.x_val, start.y_val)
        progress_time = time.monotonic()
        target_z = start.z_val - args.altitude
        climb_result = client.moveToZAsync(target_z, 0.5, timeout_sec=30, **vehicle).get()
        climb_pos = client.getMultirotorState(**vehicle).kinematics_estimated.position
        print(f"After climb: z={climb_pos.z_val:.2f}, target_z={target_z:.2f}")
        if climb_result is False or abs(climb_pos.z_val - target_z) > 0.6:
            raise RuntimeError("Climb failed; vehicle did not reach requested altitude")
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
        blocked_since = None
        last_frame_stamp = None
        with (args.output / "flight.csv").open("w", newline="", encoding="utf-8") as stream:
            log = csv.writer(stream)
            log.writerow(["elapsed_s", "x", "y", "z", "speed", "yaw_rate", "clearance_m", "state", "route_index", "target_x", "target_y", "yaw_rad", "steer_rad", "avoidance_side"])
            while time.monotonic() - start_time < args.timeout:
                if not client.isApiControlEnabled(**vehicle):
                    raise RuntimeError("API control lost during flight; check Unity/RC or other scripts")
                state = client.getMultirotorState(**vehicle)
                pos = state.kinematics_estimated.position
                check_collision()
                if math.hypot(pos.x_val - progress_position[0], pos.y_val - progress_position[1]) >= 0.4:
                    progress_position = (pos.x_val, pos.y_val)
                    progress_time = time.monotonic()
                elif time.monotonic() - progress_time > 20:
                    raise RuntimeError("No horizontal progress for 20 seconds; stop and inspect depth/route")
                target_x, target_y = route[route_index]
                dx, dy = target_x - pos.x_val, target_y - pos.y_val
                if math.hypot(dx, dy) < 0.8:
                    if route_index == len(route) - 1:
                        print("Goal reached")
                        log.writerow([round(time.monotonic() - start_time, 3), pos.x_val,
                                      pos.y_val, pos.z_val, 0, 0, "", "goal_reached",
                                      route_index, target_x, target_y, "", "", avoidance_side])
                        stream.flush()
                        break
                    route_index += 1
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
                           avoidance_side, args.curve_distance)
                if cmd.reason == "curve_around_obstacle":
                    if not avoidance_side:
                        avoidance_side = 1 if cmd.steer_angle >= 0 else -1
                        print(f"Avoidance side locked: {avoidance_side}; following scanned corridor")
                    clear_frames = 0
                elif cmd.reason == "advance":
                    clear_frames += 1
                    if clear_frames >= 6:
                        avoidance_side = 0
                if cmd.speed > 0:
                    preferred_heading = yaw + cmd.steer_angle
                if cmd.reason == "blocked_hover":
                    blocked_since = blocked_since or time.monotonic()
                    if time.monotonic() - blocked_since > 30:
                        raise RuntimeError("No clear corridor after 30 seconds of scanning")
                    # Rotate toward the chosen bypass side to bring a free sector
                    # into view instead of repeatedly turning back toward the goal.
                    scan_sign = avoidance_side or (1 if wrap(math.atan2(dy, dx) - yaw) >= 0 else -1)
                    cmd = Command(0, scan_sign * 15, 0, "scan_for_exit")
                else:
                    blocked_since = None
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
                              route_index, target_x, target_y, yaw, cmd.steer_angle, avoidance_side])
                stream.flush()
                print(f"{cmd.reason}: forward={cmd.speed:.2f}, clearance={cmd.clearance:.2f} m")
            else:
                raise RuntimeError("Mission timeout; goal not reached")
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
                    client.landAsync(timeout_sec=30, **vehicle).join()
                    landed = client.getMultirotorState(**vehicle).landed_state == airsim.LandedState.Landed
                    print(f"Landing confirmed: {landed}")
                if landed:
                    client.armDisarm(False, **vehicle)
                else:
                    print("Landing unconfirmed: API control retained; stop Unity Play before retrying")
            finally:
                if landed:
                    client.enableApiControl(False, **vehicle)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--check-only", action="store_true",
                        help="Check RPC, vehicle and depth without arming or flying")
    parser.add_argument("--port", type=int, default=41451)
    parser.add_argument("--vehicle", default="SimpleFlight",
                        help="AirSim vehicle name")
    parser.add_argument("--camera", default="0")
    parser.add_argument("--goal-x", type=float, default=12.0)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--altitude", type=float, default=3.0)
    parser.add_argument("--speed", type=float, default=1.2)
    parser.add_argument("--stop-distance", type=float, default=2.0)
    parser.add_argument("--curve-distance", type=float, default=1.5,
                        help="开始侧向曲线绕行的提前距离，米")
    parser.add_argument("--safety-margin", type=float, default=1.5,
                        help="Extra obstacle surface clearance beyond the body radius, metres")
    parser.add_argument("--vehicle-radius", type=float, default=0.6,
                        help="Body radius including propellers, metres")
    parser.add_argument("--depth-near", type=float, default=0.3,
                        help="Unity camera near clip distance in metres")
    parser.add_argument("--depth-far", type=float, default=1000.0,
                        help="Unity camera far clip distance in metres")
    parser.add_argument("--depth-color-space", choices=("srgb", "linear"), default="srgb",
                        help="srgb for this Unity project's Linear color-space ARGB32 capture")
    parser.add_argument("--route", choices=("goal", "rectangle"), default="goal",
                        help="goal: fly to the offset target; rectangle: return around a rectangle")
    parser.add_argument("--timeout", type=float, default=180)
    parser.add_argument("--max-detours", type=int, default=8,
                        help="最多插入的临时绕行航点")
    parser.add_argument("--output", type=Path, default=Path("artifacts/obstacle_avoidance"))
    args = parser.parse_args()
    if (not all(math.isfinite(v) for v in (args.goal_x, args.goal_y, args.altitude,
                                          args.speed, args.stop_distance, args.timeout,
                                          args.safety_margin, args.vehicle_radius, args.curve_distance))
            or not 0 < args.speed <= 1.5 or not 1 <= args.altitude <= 10
            or args.stop_distance < 2 or args.timeout <= 0 or args.max_detours < 0
            or args.safety_margin < 0 or args.vehicle_radius <= 0
            or args.curve_distance < 0
            or not 0 < args.depth_near < args.depth_far):
        parser.error("Use speed (0,1.5], altitude [1,10], stop-distance >=2 and positive timeout")
    fly(args)


if __name__ == "__main__":
    main()
