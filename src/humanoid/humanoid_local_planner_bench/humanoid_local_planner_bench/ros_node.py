# -*- coding: utf-8 -*-
"""ROS 2 节点：在走廊场景里跑局部规划器，发布位姿 / 激光 / 状态。

话题：
    ~/scan        sensor_msgs/LaserScan   2D 激光（来自 MuJoCo 射线求交）
    ~/odom        nav_msgs/Odometry       机器人位姿
    ~/status      std_msgs/String         JSON：规划器、时间、间距、是否到达
    TF            world -> planner_bench/base_link（动态）
                  planner_bench/base_link -> planner_bench/laser（静态，z=SCAN_HEIGHT）
"""

import json
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from . import config
from .planners import PLANNERS
from .sim_core import CorridorSim


class PlannerBenchNode(Node):
    """局部避障对比节点。"""

    def __init__(self):
        super().__init__('planner_bench_node')

        self.declare_parameter('planner', 'dwa')
        self.declare_parameter('control_hz', config.CONTROL_HZ)
        self.declare_parameter('n_rays', config.N_RAYS)
        self.declare_parameter('ray_range', config.RAY_RANGE)
        self.declare_parameter('time_limit', config.TIME_LIMIT)
        self.declare_parameter('goal_tolerance', config.GOAL_TOL)
        self.declare_parameter('scene', '')

        name = self.get_parameter('planner').get_parameter_value().string_value
        hz = self.get_parameter('control_hz').get_parameter_value().double_value
        n_rays = self.get_parameter('n_rays').get_parameter_value().integer_value
        rng = self.get_parameter('ray_range').get_parameter_value().double_value
        self.time_limit = self.get_parameter('time_limit').get_parameter_value().double_value
        self.goal_tol = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        scene = self.get_parameter('scene').get_parameter_value().string_value

        if name not in PLANNERS:
            raise ValueError("未知规划器 {}，可选 {}".format(name, list(PLANNERS)))
        self.planner_name = name
        self.planner = PLANNERS[name]()
        self.planner.reset()

        self.sim = CorridorSim(model_path=scene or None, n_rays=n_rays, ray_range=rng)
        self.sim.reset(*config.START)
        self.goal = config.GOAL
        self.dt = 1.0 / hz
        self.t = 0.0
        self.phase = 0.0
        self.reached = False

        self.pub_scan = self.create_publisher(LaserScan, '~/scan', 10)
        self.pub_odom = self.create_publisher(Odometry, '~/odom', 10)
        self.pub_status = self.create_publisher(String, '~/status', 10)
        self.tf = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)
        self._publish_static_tf()
        self.create_timer(self.dt, self._tick)

        self.get_logger().info(
            '规划器对比节点已启动 | planner={} | {} 束激光 | {:.0f} Hz | 门洞净宽 0.90 m'
            .format(name, n_rays, hz))

    def _publish_static_tf(self):
        """补 planner_bench/base_link -> planner_bench/laser 的静态 TF。

        激光从机器人中心上方 SCAN_HEIGHT 处发出，LaserScan 的 frame_id 用的是
        planner_bench/laser；不补这条 RViz 会报 No transform，激光画不出来。
        """
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'planner_bench/base_link'
        tf.child_frame_id = 'planner_bench/laser'
        tf.transform.translation.z = config.SCAN_HEIGHT
        tf.transform.rotation.w = 1.0
        self.tf_static.sendTransform(tf)

    # ------------------------------------------------------------------
    def _tick(self):
        pose = self.sim.pose()
        scan = self.sim.scan()
        v, w = self.planner.step(pose, scan, self.goal, self.dt)

        x = pose[0] + v * math.cos(pose[2]) * self.dt
        y = pose[1] + v * math.sin(pose[2]) * self.dt
        yaw = pose[2] + w * self.dt
        self.phase += (v * self.dt) / 0.7 * 2.0 * math.pi
        self.sim.set_pose(x, y, yaw, self.phase)

        now = self.get_clock().now().to_msg()
        self._publish_scan(scan, now)
        self._publish_odom(now, x, y, yaw, v, w)

        if not self.reached and math.hypot(x - self.goal[0], y - self.goal[1]) <= self.goal_tol:
            self.reached = True
            self.get_logger().info(
                '到达终点 | 用时 {:.2f}s | 规划器 {}'.format(self.t, self.planner_name))

        self._publish_status(x, y)

        if self.reached or self.t >= self.time_limit:
            self.get_logger().info(
                '实验结束 | reached={} t={:.2f}s clearance={:.3f}m'.format(
                    self.reached, self.t, self.sim.clearance()))
            self.planner.reset()
            self.sim.reset(*config.START)
            self.t = 0.0
            self.phase = 0.0
            self.reached = False
            return
        self.t += self.dt

    # ------------------------------------------------------------------
    def _publish_scan(self, scan, stamp):
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = 'planner_bench/laser'
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = 2.0 * math.pi / len(scan)
        msg.range_min = 0.05
        msg.range_max = self.sim.ray_range
        msg.scan_time = self.dt
        msg.ranges = [d for _, d in scan]
        self.pub_scan.publish(msg)

    def _publish_odom(self, stamp, x, y, yaw, v, w):
        q = (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = 'world'
        od.child_frame_id = 'planner_bench/base_link'
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.orientation.z = q[2]
        od.pose.pose.orientation.w = q[3]
        od.twist.twist.linear.x = v
        od.twist.twist.angular.z = w
        self.pub_odom.publish(od)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'world'
        tf.child_frame_id = 'planner_bench/base_link'
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        self.tf.sendTransform(tf)

    def _publish_status(self, x, y):
        msg = String()
        msg.data = json.dumps({
            "planner": self.planner_name,
            "t": round(self.t, 2),
            "x": round(x, 3),
            "y": round(y, 3),
            "clearance_m": round(self.sim.clearance(), 3),
            "collided": self.sim.collided(),
            "reached": self.reached,
        }, ensure_ascii=False)
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerBenchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
