"""ROS2 节点：融合轮式里程计、IMU 航向与绝对位置观测，输出滤波后的位姿。

仅在安装了 rclpy 的环境中可用；未安装时导入本模块会抛出 ImportError，
离线仿真请直接使用 sim/run_sim.py。

话题约定：
    订阅 /odom              (nav_msgs/Odometry)                 轮式里程计 (v, w)
    订阅 /imu               (sensor_msgs/Imu)                   航向观测
    订阅 /pose_measurement  (geometry_msgs/PoseWithCovarianceStamped)  绝对位置观测
    发布 /odometry/filtered (nav_msgs/Odometry)                 滤波后的位姿与协方差
    发布 /ekf_info          (std_msgs/String，JSON 调试信息)

时序：预测按固定周期推进（用真实时间差 dt，而不是假定常数），
观测更新在回调中立即执行——观测不该等到下一个预测周期才生效。
"""

from __future__ import annotations

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from .ekf import EKFConfig, EKFLocalizer
from .safety import FreshnessTracker, sanitize_sigma


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """四元数转 yaw。"""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


class EKFLocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__("ekf_localization_node")

        # ---- 参数声明
        self.declare_parameter("sigma_v", 0.15)
        self.declare_parameter("sigma_w", 0.15)
        self.declare_parameter("sigma_imu_yaw", 0.05)
        self.declare_parameter("sigma_pos", 0.15)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("use_imu", True)
        self.declare_parameter("use_pose", True)
        self.declare_parameter("odom_timeout", 0.5)
        self.declare_parameter("imu_timeout", 0.5)
        self.declare_parameter("pose_timeout", 2.0)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")

        # ---- 噪声参数合法性校验（sigma=0 会让滤波器把预测当真理）
        sigma_v, warn = sanitize_sigma(self.get_parameter("sigma_v").value, 0.15)
        if warn:
            self.get_logger().warn(f"参数校验 sigma_v：{warn}")
        sigma_w, warn = sanitize_sigma(self.get_parameter("sigma_w").value, 0.15)
        if warn:
            self.get_logger().warn(f"参数校验 sigma_w：{warn}")
        sigma_yaw, warn = sanitize_sigma(self.get_parameter("sigma_imu_yaw").value, 0.05)
        if warn:
            self.get_logger().warn(f"参数校验 sigma_imu_yaw：{warn}")
        sigma_pos, warn = sanitize_sigma(self.get_parameter("sigma_pos").value, 0.15)
        if warn:
            self.get_logger().warn(f"参数校验 sigma_pos：{warn}")

        self.cfg = EKFConfig(
            sigma_v=sigma_v,
            sigma_w=sigma_w,
            sigma_imu_yaw=sigma_yaw,
            sigma_pos=sigma_pos,
        )
        self.ekf = EKFLocalizer(self.cfg, P0=np.eye(3) * 1e-3)

        self.use_imu = bool(self.get_parameter("use_imu").value)
        self.use_pose = bool(self.get_parameter("use_pose").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        rate = float(self.get_parameter("publish_rate").value)
        self.dt_nominal = 1.0 / rate if rate > 0 else 0.05

        # ---- 数据新鲜度：任一话题断流都不再参与递推
        self.freshness = FreshnessTracker()
        self.freshness.register("odom", float(self.get_parameter("odom_timeout").value))
        self.freshness.register("imu", float(self.get_parameter("imu_timeout").value))
        self.freshness.register("pose", float(self.get_parameter("pose_timeout").value))

        self.twist = None          # 最近一次里程计读数 (v, w)
        self.last_predict = None   # 上次预测时刻 (s)

        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.create_subscription(Imu, "/imu", self._on_imu, 20)
        self.create_subscription(PoseWithCovarianceStamped, "/pose_measurement", self._on_pose, 5)
        self.odom_pub = self.create_publisher(Odometry, "/odometry/filtered", 20)
        self.info_pub = self.create_publisher(String, "/ekf_info", 10)
        self.create_timer(self.dt_nominal, self._on_timer)

    # ---------------------------------------------------------------- 回调

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_odom(self, msg: Odometry) -> None:
        self.twist = (
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.angular.z),
        )
        self.freshness.mark("odom", self._now_sec())

    def _on_imu(self, msg: Imu) -> None:
        self.freshness.mark("imu", self._now_sec())
        if not self.use_imu:
            return
        q = msg.orientation
        if not all(map(math.isfinite, (q.x, q.y, q.z, q.w))):
            self.get_logger().warn("IMU 四元数非法，本帧忽略。", throttle_duration_sec=2.0)
            return
        self.ekf.update_yaw(quat_to_yaw(q.x, q.y, q.z, q.w))

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.freshness.mark("pose", self._now_sec())
        if not self.use_pose:
            return
        p = msg.pose.pose.position
        if not all(map(math.isfinite, (p.x, p.y))):
            self.get_logger().warn("位置观测非法，本帧忽略。", throttle_duration_sec=2.0)
            return
        # 消息自带协方差时优先采用（取 x/y 对角元），否则用参数
        cov = msg.pose.covariance
        r = None
        if len(cov) >= 7 and math.isfinite(cov[0]) and cov[0] > 0.0:
            r = math.sqrt(cov[0])
        self.ekf.update_position(float(p.x), float(p.y), r)

    # ---------------------------------------------------------------- 定时

    def _on_timer(self) -> None:
        now = self._now_sec()
        odom_stale = self.freshness.is_stale("odom", now)

        dt = self.dt_nominal
        if self.last_predict is not None:
            dt = now - self.last_predict
        if not math.isfinite(dt) or dt <= 0.0 or dt > 5.0 * self.dt_nominal:
            # 时钟异常或长时间未调度：退回标称周期，避免超大 dt 把状态推飞
            dt = self.dt_nominal

        if odom_stale or self.twist is None:
            # 里程计断流：不递推（递推只能在可信的控制量上进行）
            self.get_logger().warn(
                "里程计数据超时，本周期跳过预测，仅发布上一次估计。",
                throttle_duration_sec=2.0,
            )
        else:
            self.ekf.predict(self.twist[0], self.twist[1], dt)
            self.last_predict = now

        self._publish(now)

    # ---------------------------------------------------------------- 发布

    def _publish(self, now: float) -> None:
        x, y, th = float(self.ekf.x[0]), float(self.ekf.x[1]), float(self.ekf.x[2])

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(th / 2.0)
        msg.pose.pose.orientation.w = math.cos(th / 2.0)
        # 6x6 协方差只需填 x、y、yaw 三个对角元，其余保持 0
        msg.pose.covariance[0] = float(self.ekf.P[0, 0])
        msg.pose.covariance[7] = float(self.ekf.P[1, 1])
        msg.pose.covariance[35] = float(self.ekf.P[2, 2])
        if self.twist is not None:
            msg.twist.twist.linear.x = float(self.twist[0])
            msg.twist.twist.angular.z = float(self.twist[1])
        self.odom_pub.publish(msg)

        payload = {
            "x": x,
            "y": y,
            "theta": th,
            "trace_cov": self.ekf.trace_cov(),
            "cov_repairs": self.ekf.cov_repairs,
            "rejected_updates": self.ekf.rejected_updates,
            "dt": self.dt_nominal,
            "odom_stale": self.freshness.is_stale("odom", now),
            "imu_stale": self.freshness.is_stale("imu", now),
            "pose_stale": self.freshness.is_stale("pose", now),
            "imu_age": self.freshness.age("imu", now),
            "pose_age": self.freshness.age("pose", now),
        }
        self.info_pub.publish(String(data=json.dumps(payload)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = EKFLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
