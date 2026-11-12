"""ROS 2 节点：把「带延迟的神经网络跟踪」跑成真正的话题流。

参数（`config/params.yaml`）：
- `shape`      赛道：circle / kidney / peanut
- `delay_ms`   执行延迟，毫秒
- `comp`       补偿方式：none / extrap / model
- `rate_hz`    控制频率
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from .env import LIDAR_BEAMS, LIDAR_RANGE, LatencyEnv
from .nav import DelayLine, features, query_pose, scan_at
from .net import LatencyNet

try:                                  # 允许在没有 ROS 的环境里 import 本模块（pytest 用）
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster

    _HAS_ROS = True
except Exception:                     # pragma: no cover - 只有装好 ROS 才会走上面一支
    _HAS_ROS = False

    class Node:                       # type: ignore
        def __init__(self, *a, **k):
            raise RuntimeError("没有 rclpy：请先 source /opt/ros/humble/setup.bash")


def _asset(*candidates: Path) -> Path:
    """权重查找：环境变量 → 包 share 目录 → 源码目录。"""
    import os

    env = os.environ.get("LAT_NAV_POLICY")
    if env and Path(env).is_file():
        return Path(env)
    for c in candidates:
        if c.is_file():
            return c
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("mujoco_nn_latency_robustness")) / "models"
        p = share / "policy.npz"
        if p.is_file():
            return p
    except Exception:
        pass
    raise FileNotFoundError("找不到 policy.npz：设置 LAT_NAV_POLICY 或先跑 main.py --demo")


class LatencyNode(Node):
    def __init__(self) -> None:
        super().__init__("latency_nav_node")
        self.declare_parameter("shape", "kidney")
        self.declare_parameter("delay_ms", 300.0)
        self.declare_parameter("comp", "model")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("seed", 0)

        self.shape = self.get_parameter("shape").value
        self.delay_ms = float(self.get_parameter("delay_ms").value)
        self.comp = self.get_parameter("comp").value
        self.rate = float(self.get_parameter("rate_hz").value)
        seed = int(self.get_parameter("seed").value)

        self.env = LatencyEnv(self.shape, seed=seed)
        weights = _asset(Path(__file__).resolve().parents[1] / "models" / "policy.npz")
        self.net = LatencyNet().load(weights)
        self.ticks = int(round(self.delay_ms / 1000.0 * self.rate))
        self.delay = DelayLine(self.ticks)
        self.rng = np.random.default_rng(seed + 5)
        self.w_prev = 0.0
        self.step = 0
        self.substeps = max(1, int(round((1.0 / self.rate) / 0.005)))

        self.pub_scan = self.create_publisher(LaserScan, "car/scan", 10)
        self.pub_odom = self.create_publisher(Odometry, "car/odom", 10)
        self.pub_status = self.create_publisher(String, "car/status", 10)
        self.tf = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            f"赛道={self.shape} 延迟={self.delay_ms:.0f} ms 补偿={self.comp} 频率={self.rate:.0f} Hz")

    # ------------------------------------------------------------------ 一步
    def tick(self) -> None:
        pose = self.env.pose()
        v, w = self.env.velocity()
        scan_now = scan_at(self.env, pose, self.rng)

        q = query_pose(self.env, pose, v, w, self.delay.pending(), 1.0 / self.rate, self.comp)
        feat = features(scan_at(self.env, q, self.rng), v, self.w_prev)
        v_cmd, w_cmd = self.net.act(feat)

        v_app, w_app = self.delay.push((v_cmd, w_cmd))
        self.env.step_period(v_app, w_app, self.substeps)
        self.w_prev = w_cmd
        self.step += 1

        e, psi = self.env.errors()
        self.publish(pose, scan_now, v, w, e, psi, (v_app, w_app))

    def publish(self, pose, scan, v, w, e, psi, cmd) -> None:
        stamp = self.get_clock().now().to_msg()

        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = "laser"
        msg.angle_min = -math.pi
        msg.angle_increment = 2.0 * math.pi / LIDAR_BEAMS
        msg.angle_max = msg.angle_min + (LIDAR_BEAMS - 1) * msg.angle_increment
        msg.range_min = 0.02
        msg.range_max = LIDAR_RANGE
        msg.scan_time = 1.0 / self.rate
        msg.time_increment = msg.scan_time / LIDAR_BEAMS
        msg.ranges = [float(r) for r in scan]
        self.pub_scan.publish(msg)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = "map"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x = float(pose[0])
        od.pose.pose.position.y = float(pose[1])
        od.pose.pose.orientation.z = float(math.sin(pose[2] / 2.0))
        od.pose.pose.orientation.w = float(math.cos(pose[2] / 2.0))
        od.twist.twist.linear.x = float(v)
        od.twist.twist.angular.z = float(w)
        self.pub_odom.publish(od)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = float(pose[0])
        t.transform.translation.y = float(pose[1])
        t.transform.rotation.z = float(math.sin(pose[2] / 2.0))
        t.transform.rotation.w = float(math.cos(pose[2] / 2.0))
        self.tf.sendTransform(t)

        st = String()
        st.data = json.dumps({
            "step": self.step,
            "shape": self.shape,
            "delay_ms": self.delay_ms,
            "comp": self.comp,
            "rate_hz": self.rate,
            "e": round(float(e), 3),
            "psi": round(float(psi), 3),
            "v_cmd": round(float(cmd[0]), 3),
            "w_cmd": round(float(cmd[1]), 3),
            "progress": round(float(self.env.progress()), 3),
        }, ensure_ascii=False)
        self.pub_status.publish(st)


def main(args=None) -> int:
    try:
        from rclpy.executors import ExternalShutdownException
    except Exception:
        ExternalShutdownException = KeyboardInterrupt  # type: ignore
    rclpy.init(args=args)
    node = LatencyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if _HAS_ROS and __name__ == "__main__":   # pragma: no cover
    raise SystemExit(main())
