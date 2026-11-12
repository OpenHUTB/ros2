#!/usr/bin/env python3
"""ROS and CLI entry for IMU/barometer altitude fusion."""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from altitude_sim import AltitudeSimulation, run_simulation  # noqa: E402


def ros_main(args, ros_args=None):
    import rospy
    from std_msgs.msg import Float32, String

    node_argv = None
    if ros_args is not None:
        node_argv = [sys.argv[0]] + list(ros_args)
    rospy.init_node(
        "uav_altitude_sensor_fusion", anonymous=False, argv=node_argv
    )
    simulation = AltitudeSimulation(args.duration, args.rate, args.seed)
    baro_pub = rospy.Publisher("/uav/altitude/barometer", Float32, queue_size=1)
    estimate_pub = rospy.Publisher("/uav/altitude/estimate", Float32, queue_size=1)
    error_pub = rospy.Publisher("/uav/altitude/error", Float32, queue_size=1)
    status_pub = rospy.Publisher("/uav/altitude/status", String, queue_size=1, latch=True)
    rate = rospy.Rate(args.rate)
    while not rospy.is_shutdown() and simulation.elapsed + simulation.dt * 0.5 < simulation.duration:
        row = simulation.step()
        baro_pub.publish(Float32(data=row["barometer"]))
        estimate_pub.publish(Float32(data=row["fused"]))
        error_pub.publish(Float32(data=row["fused_error"]))
        if len(simulation.rows) % max(1, int(args.rate)) == 0:
            status_pub.publish(String(data="RUN t=%.2f altitude=%.3f error=%.3f" % (
                row["t"], row["fused"], row["fused_error"])))
        rate.sleep()
    metrics = simulation.metrics()
    simulation.write_csv(args.csv)
    simulation.plot(args.plot)
    status = "DONE samples=%d fused_rmse=%.3f final_error=%.3f" % (
        metrics["samples"], metrics["fused_rmse"], metrics["final_fused_error"])
    status_pub.publish(String(data=status))
    rospy.loginfo(status)


def main(argv=None):
    parser = argparse.ArgumentParser(description="UAV altitude sensor fusion")
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--seed", type=int, default=3)
    parser.add_argument("--csv", default="")
    parser.add_argument("--plot", default="")
    parser.add_argument("--ros", action="store_true", help=argparse.SUPPRESS)
    raw_argv = list(sys.argv[1:] if argv is None else argv)
    if "--ros" in raw_argv:
        args, ros_args = parser.parse_known_args(raw_argv)
        ros_main(args, ros_args)
        return 0
    args = parser.parse_args(raw_argv)
    _, metrics = run_simulation(
        duration=args.duration,
        rate_hz=args.rate,
        csv_path=args.csv,
        plot_path=args.plot,
        seed=args.seed,
    )
    print(
        "samples=%d baro_rmse=%.3f integrated_rmse=%.3f fused_rmse=%.3f "
        "final_fused_error=%.3f"
        % (
            metrics["samples"],
            metrics["baro_rmse"],
            metrics["integrated_rmse"],
            metrics["fused_rmse"],
            metrics["final_fused_error"],
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
