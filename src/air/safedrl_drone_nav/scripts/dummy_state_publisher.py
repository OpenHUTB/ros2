#!/usr/bin/env python3
"""
``dummy_state_publisher`` - stand-in drone for closing the ROS loop offline.

Purpose
-------
``drl_navigator_node`` subscribes to odometry and a laser scan and publishes a
velocity command.  On a laptop/VM without a flight controller or a Gazebo
instance there is nothing to provide those topics, so this node

  1. integrates the published ``cmd_vel`` with a simple point-mass model
     (so the loop is genuinely *closed*: the navigator really does drive
     something),
  2. publishes ``nav_msgs/Odometry`` at ``~publish_rate``,
  3. publishes a ``sensor_msgs/LaserScan`` computed by ray-casting against a
     configurable list of cylindrical obstacles,
  4. publishes a TF-like ``/clock``-free timeline (no extra deps).

It is intentionally dependency-light: only ``rospy``, ``numpy`` and the
standard message packages are required, which keeps it usable inside a
minimal ``ros:noetic`` container or a resource-constrained VM.

Quick check without ROS at all::

    python scripts/dummy_state_publisher.py --selftest
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

try:
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    ROS_AVAILABLE = True
except ImportError:  # pragma: no cover
    ROS_AVAILABLE = False


DEFAULT_OBSTACLES = [
    {"centre": [3.4, 1.1, 1.5], "radius": 0.32},
    {"centre": [5.9, -1.3, 1.4], "radius": 0.38},
    {"centre": [7.6, 1.4, 1.6], "radius": 0.28},
    {"centre": [9.8, -0.6, 1.5], "radius": 0.35},
]


class DummyDrone:
    """Point-mass drone that turns ``cmd_vel`` into a plausible odometry stream."""

    def __init__(self, start=(0.0, 0.0, 1.5), rate=30.0, obstacles=None,
                 n_rays=180, scan_range=3.0):
        self.pos = np.asarray(start, dtype=float)
        self.vel = np.zeros(3)
        self.yaw = 0.0
        self.rate = float(rate)
        self.obstacles = obstacles if obstacles is not None else DEFAULT_OBSTACLES
        self.n_rays = int(n_rays)
        self.scan_range = float(scan_range)
        self._cmd = np.zeros(3)
        self._yaw_rate = 0.0
        self.t0 = time.time()
        self.steps = 0

    # ------------------------------------------------------------------
    def on_cmd(self, msg: "Twist"):
        self._cmd = np.array([msg.linear.x, msg.linear.y, msg.linear.z], dtype=float)
        self._yaw_rate = float(msg.angular.z)

    def integrate(self, dt: float):
        # first-order velocity response, as a real position controller would show
        tau = 0.18
        alpha = 1.0 - math.exp(-dt / tau)
        self.vel += alpha * (self._cmd - self.vel)
        self.pos = self.pos + self.vel * dt
        self.yaw += self._yaw_rate * dt
        self.pos[2] = max(self.pos[2], 0.05)   # never sink below the floor
        self.steps += 1

    # ------------------------------------------------------------------
    def scan_ranges(self) -> np.ndarray:
        """2-D ray-cast against the obstacle cylinders."""
        angles = np.linspace(-math.pi, math.pi, self.n_rays, endpoint=False)
        ranges = np.full(self.n_rays, self.scan_range, dtype=float)
        for i, a in enumerate(angles):
            d = np.array([math.cos(a + self.yaw), math.sin(a + self.yaw), 0.0])
            best = self.scan_range
            for ob in self.obstacles:
                c = np.asarray(ob["centre"], float)
                r = float(ob["radius"])
                # only obstacles within the horizontal band of the scan plane
                if abs(c[2] - self.pos[2]) > 0.75:
                    continue
                oc = self.pos - c
                oc[2] = 0.0
                b = float(d @ oc)
                cc = float(oc @ oc) - r * r
                disc = b * b - cc
                if disc < 0:
                    continue
                t = -b - math.sqrt(disc)
                if 1e-6 < t < best:
                    best = t
            ranges[i] = best
        return ranges

    def odom_msg(self, topic_frame="map", child_frame="base_link"):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now() if ROS_AVAILABLE else None
        msg.header.frame_id = topic_frame
        msg.child_frame_id = child_frame
        msg.pose.pose.position.x = float(self.pos[0])
        msg.pose.pose.position.y = float(self.pos[1])
        msg.pose.pose.position.z = float(self.pos[2])
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        msg.twist.twist.linear.x = float(self.vel[0])
        msg.twist.twist.linear.y = float(self.vel[1])
        msg.twist.twist.linear.z = float(self.vel[2])
        msg.twist.twist.angular.z = float(self._yaw_rate)
        return msg

    def scan_msg(self):
        msg = LaserScan()
        msg.header.stamp = rospy.Time.now() if ROS_AVAILABLE else None
        msg.header.frame_id = "base_link"
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 2 * math.pi / self.n_rays
        msg.range_min = 0.05
        msg.range_max = self.scan_range
        msg.ranges = [float(x) for x in self.scan_ranges()]
        return msg


# ==========================================================================
# selftest (no ROS required)
# ==========================================================================


def selftest():
    print("dummy_state_publisher selftest")
    print("-" * 60)
    drone = DummyDrone()

    class _Twist:
        def __init__(self, x, y, z, wz):
            self.linear = type("L", (), {"x": x, "y": y, "z": z})()
            self.angular = type("A", (), {"z": wz})()

    # drive forward for 2 s and confirm the model actually moves + sees things
    for _ in range(60):
        drone.on_cmd(_Twist(1.5, 0.0, 0.0, 0.0))
        drone.integrate(1.0 / 30.0)
    ranges = drone.scan_ranges()
    print(f"  final position      : {np.round(drone.pos, 3).tolist()}")
    print(f"  final velocity      : {np.round(drone.vel, 3).tolist()}")
    print(f"  min laser range     : {ranges.min():.3f} m")
    print(f"  rays below 3 m      : {int((ranges < drone.scan_range - 1e-6).sum())}/{len(ranges)}")

    assert drone.pos[0] > 2.0, "drone did not move forward"
    assert ranges.min() < drone.scan_range, "ray-casting found no obstacle"
    assert abs(drone.pos[2] - 1.5) < 1e-6, "altitude should be held (no z command)"

    # altitude command must be honoured
    for _ in range(30):
        drone.on_cmd(_Twist(0.0, 0.0, 0.8, 0.0))
        drone.integrate(1.0 / 30.0)
    assert drone.pos[2] > 1.5, "altitude command ignored"
    print(f"  altitude after climb: {drone.pos[2]:.3f} m")
    print("-" * 60)
    print("OK - dummy drone model integrates commands and senses obstacles.")
    return 0


# ==========================================================================


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--selftest", action="store_true", help="run without ROS and exit")
    ap.add_argument("--publish_rate", type=float, default=30.0)
    ap.add_argument("--start", type=float, nargs=3, default=[0.0, 0.0, 1.5])
    ap.add_argument("--n_rays", type=int, default=180)
    ap.add_argument("--scan_range", type=float, default=3.0)
    ap.add_argument("--odom_topic", default="/drone/odom")
    ap.add_argument("--scan_topic", default="/drone/scan")
    ap.add_argument("--cmd_topic", default="/drone/cmd_vel")
    ap.add_argument("--echo_cmd", action="store_true")
    args, _ = ap.parse_known_args()

    if args.selftest:
        return selftest()
    if not ROS_AVAILABLE:
        print("ERROR: rospy not found; source /opt/ros/noetic/setup.bash first.",
              file=sys.stderr)
        return 1

    rospy.init_node("dummy_state_publisher", anonymous=False)
    obstacles = rospy.get_param("~obstacles", DEFAULT_OBSTACLES)
    drone = DummyDrone(
        start=args.start, rate=args.publish_rate, obstacles=obstacles,
        n_rays=args.n_rays, scan_range=args.scan_range,
    )

    pub_odom = rospy.Publisher(args.odom_topic, Odometry, queue_size=1)
    pub_scan = rospy.Publisher(args.scan_topic, LaserScan, queue_size=1)
    rospy.Subscriber(args.cmd_topic, Twist, drone.on_cmd, queue_size=1)

    if args.echo_cmd:
        def _echo(msg):
            rospy.loginfo("cmd_vel  v=(%.2f, %.2f, %.2f)  yaw_rate=%.2f",
                          msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        rospy.Subscriber(args.cmd_topic, Twist, _echo, queue_size=1)

    rospy.loginfo(
        "dummy drone up: publishing %s and %s at %.1f Hz, listening on %s",
        args.odom_topic, args.scan_topic, args.publish_rate, args.cmd_topic,
    )

    rate = rospy.Rate(args.publish_rate)
    dt = 1.0 / args.publish_rate
    while not rospy.is_shutdown():
        drone.integrate(dt)
        pub_odom.publish(drone.odom_msg())
        pub_scan.publish(drone.scan_msg())
        rate.sleep()
    return 0


if __name__ == "__main__":
    sys.exit(main())
