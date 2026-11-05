"""Sonar-driven VFH+ reactive obstacle avoidance node.

The node simulates a vehicle carrying a 360 degree scanning sonar.  Every
control tick it:

1. generates a sonar revolution at the current pose,
2. turns the echoes into a polar obstacle density and smooths it,
3. picks the free valley closest to the goal (with hysteresis),
4. publishes the resulting ``Twist`` and integrates a kinematic model so the
   node is self-contained and can be demonstrated without a physics engine.

Topics
    ``/sonar/scan``       ``sensor_msgs/LaserScan``   one revolution
    ``/vfh/histogram``    ``std_msgs/Float32MultiArray`` polar obstacle density
    ``/cmd_vel``          ``geometry_msgs/Twist``     steering command
    ``/odom``             ``nav_msgs/Odometry``       simulated pose
    ``/vfh/status``       ``std_msgs/String``         JSON diagnostics
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String

from .sonar import ScanningSonar
from .terrain import BoxObstacle, Seabed, World
from .vfh import (angle_difference_deg, clearance_along, polar_histogram,
                  select_direction, smooth_histogram, speed_scale)


class SonarVfhNode(Node):
    def __init__(self):
        super().__init__('sonar_vfh')

        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw_deg', 0.0)
        self.declare_parameter('goal_x', 20.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('depth', 6.0)
        self.declare_parameter('v_max', 1.2)
        self.declare_parameter('yaw_gain', 1.5)
        self.declare_parameter('max_yaw_rate', 0.8)
        self.declare_parameter('control_dt', 0.1)
        self.declare_parameter('goal_tolerance', 0.8)
        self.declare_parameter('max_steps', 400)
        self.declare_parameter('beams', 72)
        self.declare_parameter('fov_deg', 360.0)
        self.declare_parameter('max_range', 30.0)
        self.declare_parameter('range_sigma', 0.02)
        self.declare_parameter('dropout', 0.0)
        self.declare_parameter('sectors', 72)
        self.declare_parameter('robot_radius', 0.6)
        self.declare_parameter('safety_margin', 0.5)
        self.declare_parameter('threshold', 0.35)
        self.declare_parameter('min_valley_deg', 15.0)
        self.declare_parameter('hysteresis_deg', 15.0)
        self.declare_parameter('seed', 7)

        p = lambda name: self.get_parameter(name).value
        self.x = float(p('start_x'))
        self.y = float(p('start_y'))
        self.yaw = math.radians(float(p('start_yaw_deg')))
        self.goal = (float(p('goal_x')), float(p('goal_y')))
        self.depth = float(p('depth'))
        self.v_max = float(p('v_max'))
        self.yaw_gain = float(p('yaw_gain'))
        self.max_yaw_rate = float(p('max_yaw_rate'))
        self.dt = float(p('control_dt'))
        self.goal_tolerance = float(p('goal_tolerance'))
        self.max_steps = int(p('max_steps'))
        self.threshold = float(p('threshold'))
        self.min_valley_deg = float(p('min_valley_deg'))
        self.hysteresis_deg = float(p('hysteresis_deg'))
        self.robot_radius = float(p('robot_radius'))
        self.safety_margin = float(p('safety_margin'))

        self.sonar = ScanningSonar(
            beams=int(p('beams')), fov_deg=float(p('fov_deg')),
            max_range=float(p('max_range')), range_sigma=float(p('range_sigma')),
            dropout=float(p('dropout')))
        self.sectors = int(p('sectors'))
        self.rng = np.random.default_rng(int(p('seed')))
        self.world = self._build_world()
        self.previous_steering = None
        self.step_count = 0
        self.mode = 'driving'
        self.last_steering_deg = 0.0
        self.last_speed = 0.0

        self.scan_pub = self.create_publisher(LaserScan, '/sonar/scan', 10)
        self.hist_pub = self.create_publisher(Float32MultiArray, '/vfh/histogram', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.status_pub = self.create_publisher(String, '/vfh/status', 10)

        self.timer = self.create_timer(self.dt, self.control_step)
        self.get_logger().info(
            f'sonar_vfh ready: goal=({self.goal[0]:.1f}, {self.goal[1]:.1f}) '
            f'beams={self.sonar.beams} sectors={self.sectors}')

    @staticmethod
    def _build_world():
        """Scene from the module description: three blocks on an undulating bed.

        The seabed spans 7-13 m depth while the vehicle cruises at 6 m, so every
        beam clears the floor unless a block reaches its depth.  Block heights
        are chosen so their tops sit above the cruise depth.
        """
        seabed = Seabed()
        obstacles = []
        for cx, cy, size, height in ((6.0, 0.0, 1.5, 8.0),
                                     (12.0, 4.5, 1.2, 7.5),
                                     (14.0, -4.0, 1.2, 8.0)):
            obstacles.append(BoxObstacle(cx, cy, size, size, height))
        return World(seabed, obstacles)

    # ------------------------------------------------------------------ helpers
    def _goal_bearing_deg(self):
        angle = math.degrees(math.atan2(self.goal[1] - self.y, self.goal[0] - self.x))
        return angle_difference_deg(angle, math.degrees(self.yaw))

    def _goal_distance(self):
        return math.hypot(self.goal[0] - self.x, self.goal[1] - self.y)

    # -------------------------------------------------------------------- loop
    def control_step(self):
        if self.mode == 'reached' or self.step_count >= self.max_steps:
            return
        self.step_count += 1

        bearings, ranges = self.sonar.scan(
            self.world, self.x, self.y, self.depth, self.yaw, self.rng)
        hist = smooth_histogram(polar_histogram(
            bearings, ranges, sectors=self.sectors, max_range=self.sonar.max_range,
            robot_radius=self.robot_radius, safety_margin=self.safety_margin))

        goal_bearing = self._goal_bearing_deg()
        steering = select_direction(
            hist, goal_bearing, threshold=self.threshold,
            min_valley_deg=self.min_valley_deg,
            hysteresis_deg=self.hysteresis_deg, previous_deg=self.previous_steering)

        vx, wz = self._command(steering, bearings, ranges)
        self._integrate(vx, wz)
        self._publish(bearings, ranges, hist, steering, vx, wz)

        if self._goal_distance() <= self.goal_tolerance:
            self.mode = 'reached'
            self.get_logger().info(
                f'goal reached after {self.step_count} steps '
                f'(dist={self._goal_distance():.2f} m)')

    def _command(self, steering, bearings, ranges):
        if steering.blocked:
            self.mode = 'blocked'
            self.last_steering_deg = steering.angle_deg
            self.previous_steering = None
            return 0.0, self.max_yaw_rate  # rotate in place and rescan

        error = math.radians(steering.angle_deg)
        wz = max(-self.max_yaw_rate, min(self.max_yaw_rate, self.yaw_gain * error))
        inflation = self.robot_radius + self.safety_margin
        clearance = clearance_along(
            bearings, ranges, steering.angle_deg,
            max_range=self.sonar.max_range) - inflation
        scale = speed_scale(clearance)
        vx = self.v_max * scale * max(0.0, math.cos(error))

        self.mode = 'avoiding' if abs(steering.angle_deg) > 10.0 else 'driving'
        self.last_steering_deg = steering.angle_deg
        self.last_speed = vx
        self.previous_steering = steering.angle_deg
        return vx, wz

    def _integrate(self, vx, wz):
        self.yaw = math.atan2(math.sin(self.yaw + wz * self.dt),
                              math.cos(self.yaw + wz * self.dt))
        self.x += vx * math.cos(self.yaw) * self.dt
        self.y += vx * math.sin(self.yaw) * self.dt

    def _publish(self, bearings, ranges, hist, steering, vx, wz):
        stamp = self.get_clock().now().to_msg()

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'sonar_link'
        scan.angle_min = float(bearings[0])
        scan.angle_increment = float(math.radians(self.sonar.fov_deg) / self.sonar.beams)
        scan.angle_max = float(bearings[-1] + scan.angle_increment)
        scan.range_min = float(self.sonar.min_range)
        scan.range_max = float(self.sonar.max_range)
        scan.ranges = [float(r) for r in ranges]
        self.scan_pub.publish(scan)

        histogram = Float32MultiArray()
        histogram.data = [float(v) for v in hist]
        self.hist_pub.publish(histogram)

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.angular.z = float(wz)
        self.cmd_pub.publish(cmd)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = float(self.depth)
        odom.pose.pose.orientation.z = float(math.sin(self.yaw / 2.0))
        odom.pose.pose.orientation.w = float(math.cos(self.yaw / 2.0))
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.angular.z = float(wz)
        self.odom_pub.publish(odom)

        status = String()
        status.data = json.dumps({
            'mode': self.mode,
            'step': self.step_count,
            'goal_distance': round(self._goal_distance(), 3),
            'goal_bearing_deg': round(self._goal_bearing_deg(), 2),
            'steering_deg': round(float(steering.angle_deg), 2),
            'valley_deg': round(float(steering.valley_deg), 1),
            'speed': round(float(vx), 3),
            'yaw_rate': round(float(wz), 3),
            'min_range': round(float(ranges.min()), 2),
        })
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = SonarVfhNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
