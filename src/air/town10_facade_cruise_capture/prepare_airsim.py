"""显式设置一次仿真实验起点；巡航运行不调用该脚本。"""

import argparse
import json
import math
import time

import numpy as np


def main():
    parser = argparse.ArgumentParser(
        description="一次性设置AirSim实验起点及前向相机外参"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=41451)
    parser.add_argument("--vehicle", default="drone1")
    parser.add_argument("--camera", default="front_cam")
    parser.add_argument(
        "--position", nargs=3, type=float, required=True, metavar=("N", "E", "D")
    )
    parser.add_argument("--yaw", type=float, required=True, help="NED偏航角，单位度")
    parser.add_argument(
        "--camera-offset",
        nargs=3,
        type=float,
        default=(1.5, 0.0, -0.2),
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument(
        "--takeoff", action="store_true", help="先执行解锁起飞，再设置实验起点"
    )
    args = parser.parse_args()
    if not np.isfinite([*args.position, args.yaw, *args.camera_offset]).all():
        parser.error("位置、偏航和相机偏移必须为有限数值")
    import airsim

    client = airsim.MultirotorClient(ip=args.host, port=args.port, timeout_value=10)
    client.confirmConnection()
    client.enableApiControl(True, vehicle_name=args.vehicle)
    try:
        if args.takeoff:
            client.armDisarm(True, vehicle_name=args.vehicle)
            client.takeoffAsync(timeout_sec=20, vehicle_name=args.vehicle).join()
        if (
            client.getMultirotorState(vehicle_name=args.vehicle).landed_state
            == airsim.LandedState.Landed
        ):
            parser.error("需先起飞悬停，或显式指定 --takeoff")
        client.hoverAsync(vehicle_name=args.vehicle).join()
        time.sleep(1.0)
        pose = airsim.Pose(
            airsim.Vector3r(*args.position),
            airsim.to_quaternion(0.0, 0.0, math.radians(args.yaw)),
        )
        client.simSetVehiclePose(pose, ignore_collision=True, vehicle_name=args.vehicle)
        client.simSetCameraPose(
            args.camera,
            airsim.Pose(
                airsim.Vector3r(*args.camera_offset),
                airsim.to_quaternion(0.0, 0.0, 0.0),
            ),
            vehicle_name=args.vehicle,
        )
        client.hoverAsync(vehicle_name=args.vehicle).join()
        state = client.getMultirotorState(vehicle_name=args.vehicle)
        p = state.kinematics_estimated.position
        print(
            json.dumps(
                {
                    "vehicle": args.vehicle,
                    "requested_start_ned_m": args.position,
                    "requested_yaw_deg": args.yaw,
                    "actual_hover_ned_m": [p.x_val, p.y_val, p.z_val],
                    "timestamp_ns": int(state.timestamp),
                    "initialization_pose_writes": 1,
                },
                indent=2,
                ensure_ascii=False,
            )
        )
    finally:
        try:
            client.hoverAsync(vehicle_name=args.vehicle).join()
        finally:
            client.enableApiControl(False, vehicle_name=args.vehicle)


if __name__ == "__main__":
    main()
