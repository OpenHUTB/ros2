"""ROS 2（Humble）节点：MPPI / 蒸馏网络 / 纯追踪 三种控制器任选，跑 8 字跟踪。

发布：
  /car/scan     sensor_msgs/LaserScan      32 线锥桶雷达（frame_id=laser）
  /car/odom     nav_msgs/Odometry          编码器+陀螺估计（frame_id=map）
  /car/cmd      geometry_msgs/Twist        线速度 + 偏航率
  /car/steering std_msgs/Float32           前轮转角 δ（弧度）
  /car/status   std_msgs/String            controller/rms/进度
  /tf           tf2_msgs/TFMessage         map->base_link、base_link->laser
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from . import track
from .env import DT, LIDAR_BEAMS, LIDAR_RANGE, AckermannEnv, WHEELBASE
from .mppi import MPPIConfig, MPPIController, pure_pursuit
from .nav import PoseEstimator
from .net import TrackerNet, build_features


def _find_asset(name: str) -> Path:
    from .env import _find_asset as f
    return f(name)


def _import_ros():
    import rclpy
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Float32, String
    from tf2_ros import TransformBroadcaster
    return (rclpy, Node, LaserScan, Odometry, Twist, Float32, String,
            TransformBroadcaster, TransformStamped)


def main(argv=None) -> int:
    (rclpy, Node, LaserScan, Odometry, Twist, Float32, String,
     TransformBroadcaster, TransformStamped) = _import_ros()

    class TrackerNode(Node):
        def __init__(self):
            super().__init__("mujoco_mppi_neural_tracker")
            self.declare_parameter("controller", "nn")
            self.declare_parameter("weights", "")
            self.declare_parameter("rate_hz", 20.0)
            self.declare_parameter("max_steps", 1200)
            ctrl = str(self.get_parameter("controller").value)
            rate = float(self.get_parameter("rate_hz").value)
            self.max_steps = int(self.get_parameter("max_steps").value)

            self.env = AckermannEnv(0)
            self.path = self.env.path
            self.est = PoseEstimator(self.env)
            self.ctrl = ctrl
            self.mppi = MPPIController(self.path, MPPIConfig()) if ctrl == "mppi" else None
            self.net = None
            if ctrl == "nn":
                w = self.get_parameter("weights").value or str(_find_asset("policy.npz"))
                self.net = TrackerNet()
                if Path(w).exists():
                    self.net.load(w)
                    self.get_logger().info(f"已加载权重 {w}")
                else:
                    self.get_logger().error(
                        "找不到 policy.npz：先 python main.py --demo，或用 weights 指定")

            self.step_i = 0
            self.done = False
            self.delta_prev, self.v_prev = 0.0, 0.9
            self.prev_state = self.env.state()

            self.pub_scan = self.create_publisher(LaserScan, "car/scan", 10)
            self.pub_odom = self.create_publisher(Odometry, "car/odom", 10)
            self.pub_cmd = self.create_publisher(Twist, "car/cmd", 10)
            self.pub_delta = self.create_publisher(Float32, "car/steering", 10)
            self.pub_status = self.create_publisher(String, "car/status", 10)
            self.tf = TransformBroadcaster(self)
            self.timer = self.create_timer(1.0 / rate, self.tick)
            self.get_logger().info(
                f"控制器={ctrl} 目标轨迹=8 字 (半宽 {track.A} m) "

                f"车道半宽={track.LANE_HALF} m")

        def _stamp(self):
            return self.get_clock().now().to_msg()

        def _pub_scan(self, scan):
            m = LaserScan()
            m.header.stamp = self._stamp()
            m.header.frame_id = "laser"
            inc = 2 * np.pi / LIDAR_BEAMS
            m.angle_min = -np.pi
            m.angle_max = float(np.pi - inc)
            m.angle_increment = float(inc)
            m.range_min = 0.05
            m.range_max = float(LIDAR_RANGE)
            m.time_increment = 0.0
            m.scan_time = 0.05
            m.ranges = [float(r) for r in scan]
            self.pub_scan.publish(m)

        def _pub_odom(self, pose, v, w):
            m = Odometry()
            m.header.stamp = self._stamp()
            m.header.frame_id = "map"
            m.child_frame_id = "base_link"
            m.pose.pose.position.x = float(pose[0])
            m.pose.pose.position.y = float(pose[1])
            m.pose.pose.orientation.z = float(np.sin(pose[2] / 2))
            m.pose.pose.orientation.w = float(np.cos(pose[2] / 2))
            m.twist.twist.linear.x = float(v)
            m.twist.twist.angular.z = float(w)
            self.pub_odom.publish(m)

            t = TransformStamped()
            t.header.stamp = m.header.stamp
            t.header.frame_id = "map"
            t.child_frame_id = "base_link"
            t.transform.translation.x = float(pose[0])
            t.transform.translation.y = float(pose[1])
            t.transform.rotation.z = float(np.sin(pose[2] / 2))
            t.transform.rotation.w = float(np.cos(pose[2] / 2))
            self.tf.sendTransform(t)

            t2 = TransformStamped()
            t2.header.stamp = m.header.stamp
            t2.header.frame_id = "base_link"
            t2.child_frame_id = "laser"
            t2.transform.rotation.w = 1.0
            self.tf.sendTransform(t2)

        def tick(self):
            if self.done:
                return
            e_true, psi_true = self.env.errors()
            e_meas = e_true + self.env.rng.normal(0, 0.04)
            self.est.correct(self.path, e_meas)
            pose = self.est.pose
            idx = track.nearest_index(self.path, pose[:2], self.est.idx)
            e_est, psi_est = track.track_errors(self.path, pose[:2], pose[2], idx)

            if self.ctrl == "mppi":
                v_cmd, delta = self.mppi.act(pose, v_now=self.prev_state.v)
            elif self.ctrl == "nn":
                feats = build_features(e_est, psi_est, self.prev_state.v,
                                       self.delta_prev, self.v_prev, self.path, idx, pose)
                v_cmd, delta = self.net.act(feats)
            else:
                v_cmd, delta = pure_pursuit(pose, self.path, idx)

            state_prev = self.env.state()
            new = self.env.step(v_cmd, delta)
            v_o, w_o = self.env.odom(state_prev, new)
            self.est.predict(v_o, w_o)
            self.prev_state = new
            self.delta_prev, self.v_prev = delta, v_cmd

            self._pub_scan(self.env.scan())
            self._pub_odom(pose, v_o, w_o)
            cmd = Twist()
            cmd.linear.x = float(v_cmd)
            cmd.angular.z = float(v_cmd * np.tan(delta) / WHEELBASE)
            self.pub_cmd.publish(cmd)
            d = Float32()
            d.data = float(delta)
            self.pub_delta.publish(d)

            st = String()
            st.data = (f"ctrl={self.ctrl} step={self.step_i} progress={self.env.progress:.2f} "
                       f"e={e_est:+.2f} cmd=({v_cmd:.2f},{delta:+.2f})")
            self.pub_status.publish(st)
            if self.step_i % 20 == 0:
                self.get_logger().info(st.data)

            self.step_i += 1
            if self.step_i >= self.max_steps or self.env.crashed or self.env.progress > 0.98:
                self.done = True
                self.timer.cancel()
                self.get_logger().info(
                    f"结束：crashed={self.env.crashed} progress={self.env.progress:.2f}")

        @property
        def cfg_dt(self):
            return DT

    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=argv)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
