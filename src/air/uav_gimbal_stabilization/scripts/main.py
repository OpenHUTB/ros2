#!/usr/bin/env python3
"""UAV gimbal stabilization entry point.

Pure simulation:
    python3 main.py --duration 12 --plot response.png

ROS:
    roslaunch uav_gimbal_stabilization main.launch
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gimbal_sim import GimbalSimulation, run_simulation  # noqa: E402


def _ros_main(args, ros_args=None):
    import rospy
    from geometry_msgs.msg import Vector3
    from std_msgs.msg import String

    node_argv = None
    if ros_args is not None:
        node_argv = [sys.argv[0]] + list(ros_args)
    rospy.init_node(
        "uav_gimbal_stabilization", anonymous=False, argv=node_argv
    )
    simulation = GimbalSimulation(args.duration, args.rate)
    target_pub = rospy.Publisher("/uav/gimbal/target", Vector3, queue_size=1)
    cmd_pub = rospy.Publisher("/uav/gimbal/cmd", Vector3, queue_size=1)
    state_pub = rospy.Publisher("/uav/gimbal/state", Vector3, queue_size=1)
    status_pub = rospy.Publisher("/uav/gimbal/status", String, queue_size=1, latch=True)
    rate = rospy.Rate(args.rate)
    while not rospy.is_shutdown() and simulation.elapsed + simulation.dt * 0.5 < simulation.duration:
        row = simulation.step()
        target_pub.publish(Vector3(row["target_pan"], row["target_tilt"], 0.0))
        cmd_pub.publish(Vector3(row["pan_rate"], row["tilt_rate"], 0.0))
        state_pub.publish(Vector3(row["pan"], row["tilt"], row["error_deg"]))
        if len(simulation.rows) % max(1, int(args.rate)) == 0:
            status_pub.publish(String(data="RUN t=%.2f error=%.3f" % (
                row["t"], row["error_deg"])))
        rate.sleep()
    metrics = simulation.metrics()
    simulation.write_csv(args.csv)
    simulation.plot(args.plot)
    status = (
        "DONE samples=%d rmse_deg=%.3f max_error_deg=%.3f "
        "saturation_ratio=%.3f"
        % (
            metrics["samples"],
            metrics["rmse_deg"],
            metrics["max_error_deg"],
            metrics["saturation_ratio"],
        )
    )
    status_pub.publish(String(data=status))
    rospy.loginfo(status)


def main(argv=None):
    parser = argparse.ArgumentParser(description="UAV gimbal stabilization")
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--csv", default="")
    parser.add_argument("--plot", default="")
    parser.add_argument("--ros", action="store_true", help=argparse.SUPPRESS)
    raw_argv = list(sys.argv[1:] if argv is None else argv)
    if "--ros" in raw_argv:
        args, ros_args = parser.parse_known_args(raw_argv)
        if not math.isfinite(args.duration) or args.duration <= 0.0:
            parser.error("--duration must be finite and positive")
        if not math.isfinite(args.rate) or args.rate <= 0.0:
            parser.error("--rate must be finite and positive")
        _ros_main(args, ros_args)
        return 0
    args = parser.parse_args(raw_argv)
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        parser.error("--duration must be finite and positive")
    if not math.isfinite(args.rate) or args.rate <= 0.0:
        parser.error("--rate must be finite and positive")
    _, metrics = run_simulation(
        duration=args.duration,
        rate_hz=args.rate,
        csv_path=args.csv,
        plot_path=args.plot,
    )
    print("samples=%d rmse_deg=%.3f max_error_deg=%.3f saturation_ratio=%.3f" % (
        metrics["samples"], metrics["rmse_deg"], metrics["max_error_deg"],
        metrics["saturation_ratio"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
