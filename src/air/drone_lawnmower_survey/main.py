#!/usr/bin/env python3
"""Run a rectangular lawnmower survey mission in AirSim."""

import argparse
import math
import sys

from survey_core import build_lawnmower_path, path_length, translate_path


def build_parser():
    parser = argparse.ArgumentParser(
        description="Generate and optionally fly an AirSim lawnmower survey path"
    )
    parser.add_argument("--length", type=float, default=60.0,
                        help="survey length along NED x in metres (default: 60)")
    parser.add_argument("--width", type=float, default=40.0,
                        help="survey width along NED y in metres (default: 40)")
    parser.add_argument("--lane-spacing", type=float, default=8.0,
                        help="maximum spacing between lanes in metres (default: 8)")
    parser.add_argument("--altitude", type=float, default=20.0,
                        help="height above the takeoff point in metres (default: 20)")
    parser.add_argument("--speed", type=float, default=5.0,
                        help="path speed in metres per second (default: 5)")
    parser.add_argument("--ip", default="127.0.0.1",
                        help="AirSim RPC address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=41451,
                        help="AirSim RPC port (default: 41451)")
    parser.add_argument("--vehicle", default="",
                        help="vehicle name; empty selects the first available vehicle")
    parser.add_argument(
        "--execute",
        action="store_true",
        help="connect to AirSim and fly; without this flag only preview the path",
    )
    return parser


def validate_speed(speed):
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError("speed must be a positive finite number")


def resolve_vehicle_name(client, requested):
    if requested:
        return requested
    vehicles = client.listVehicles()
    if not vehicles:
        raise RuntimeError("AirSim did not report any multirotor vehicles")
    return vehicles[0]


def print_preview(path, speed):
    distance = path_length(path)
    print("Survey waypoints: %d" % len(path))
    print("Path distance: %.2f m" % distance)
    print("Estimated path time: %.1f s" % (distance / speed))
    for index, (x, y, z) in enumerate(path, start=1):
        print("%02d: x=%8.2f  y=%8.2f  z=%8.2f" % (index, x, y, z))


def execute_mission(args, local_path):
    try:
        import airsim
    except ImportError as exc:
        raise RuntimeError(
            "AirSim Python package is required for --execute; run: pip install airsim"
        ) from exc

    client = airsim.MultirotorClient(ip=args.ip, port=args.port)
    client.confirmConnection()
    vehicle_name = resolve_vehicle_name(client, args.vehicle)
    api_enabled = False
    armed = False
    took_off = False

    try:
        client.enableApiControl(True, vehicle_name=vehicle_name)
        api_enabled = True
        state = client.getMultirotorState(vehicle_name=vehicle_name)
        home = state.kinematics_estimated.position

        client.armDisarm(True, vehicle_name=vehicle_name)
        armed = True
        took_off = True
        client.takeoffAsync(vehicle_name=vehicle_name).join()

        world_path = translate_path(local_path, home.x_val, home.y_val, home.z_val)
        target_z = home.z_val - args.altitude
        climb_speed = min(args.speed, 3.0)
        client.moveToZAsync(
            target_z,
            climb_speed,
            timeout_sec=max(20.0, args.altitude / climb_speed * 2.0),
            vehicle_name=vehicle_name,
        ).join()

        vectors = [airsim.Vector3r(x, y, z) for x, y, z in world_path]
        timeout = max(30.0, path_length(world_path) / args.speed * 2.0)
        print("Flying %d waypoints with vehicle %s" % (len(vectors), vehicle_name))
        client.moveOnPathAsync(
            vectors,
            args.speed,
            timeout_sec=timeout,
            drivetrain=airsim.DrivetrainType.ForwardOnly,
            yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0.0),
            lookahead=-1,
            adaptive_lookahead=1,
            vehicle_name=vehicle_name,
        ).join()
        client.hoverAsync(vehicle_name=vehicle_name).join()
    finally:
        if took_off:
            try:
                client.hoverAsync(vehicle_name=vehicle_name).join()
                client.landAsync(timeout_sec=30, vehicle_name=vehicle_name).join()
            except Exception as exc:
                print("Warning: automatic landing failed: %s" % exc, file=sys.stderr)
        if armed:
            try:
                client.armDisarm(False, vehicle_name=vehicle_name)
            except Exception as exc:
                print("Warning: could not disarm vehicle: %s" % exc, file=sys.stderr)
        if api_enabled:
            try:
                client.enableApiControl(False, vehicle_name=vehicle_name)
            except Exception as exc:
                print("Warning: could not release API control: %s" % exc, file=sys.stderr)


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if not 1 <= args.port <= 65535:
            raise ValueError("port must be between 1 and 65535")
        validate_speed(args.speed)
        local_path = build_lawnmower_path(
            args.length, args.width, args.lane_spacing, args.altitude
        )
    except ValueError as exc:
        parser.error(str(exc))

    print_preview(local_path, args.speed)
    if not args.execute:
        print("Preview only. Add --execute after checking the waypoints.")
        return 0

    try:
        execute_mission(args, local_path)
    except (RuntimeError, OSError) as exc:
        parser.exit(1, "error: %s\n" % exc)
    return 0


if __name__ == "__main__":
    sys.exit(main())
