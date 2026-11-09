"""ROS2 节点：``/cmd_vel`` 的守护层。

数据流（推荐接法）::

    任务层（导航/遥控） --/cmd_vel_nav--> 本节点 --/cmd_vel--> 底盘驱动
                                            ^
                              /odom  /imu  /scan

正常时把任务层指令透传给底盘；一旦检测到卡死，本节点接管指令输出、执行脱困动作，
结束后把控制权交回任务层。算法全部在 ``detect`` / ``recovery`` 里，节点本身不含逻辑，
便于在无 ROS 环境下做单元测试。
"""
from __future__ import annotations

import json
import math

from .detect import DetectorConfig, StuckDetector
from .guard import OdomWatch, evaluate, odom_available
from .recovery import RecoveryConfig, RecoverySupervisor
from .robot import Limits, Twist
from .sensors import FRONT_SECTOR, front_range


class StuckRecoveryNode:
    """把 ROS 话题桥接到算法层的薄封装（构造时导入 rclpy，便于离线测试其他模块）。"""

    def __init__(self) -> None:
        import rclpy
        from geometry_msgs.msg import Twist as TwistMsg
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from sensor_msgs.msg import Imu, LaserScan
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        self._rclpy = rclpy
        self._twist_msg = TwistMsg
        self._string_msg = String
        self.node: Node = Node("stuck_recovery")

        n = self.node
        self.declare = n.declare_parameter
        self.declare("odom_topic", "/odom")
        self.declare("imu_topic", "/imu")
        self.declare("scan_topic", "/scan")
        self.declare("task_topic", "/cmd_vel_nav")
        self.declare("cmd_topic", "/cmd_vel")
        self.declare("rate_hz", 20.0)
        self.declare("max_level", 3)

        self.limits = Limits()
        self.detector = StuckDetector(DetectorConfig())
        self.supervisor = RecoverySupervisor(
            RecoveryConfig(max_level=int(n.get_parameter("max_level").value))
        )

        self.pose = (0.0, 0.0, 0.0)
        self.v_odom = 0.0
        self.w_odom = 0.0
        self.w_imu = None
        self.range_front = None
        self.task_cmd = Twist()
        self.watch = OdomWatch()
        self._last_pose = (0.0, 0.0, 0.0)

        self.pub_cmd = n.create_publisher(TwistMsg, n.get_parameter("cmd_topic").value, 10)
        self.pub_status = n.create_publisher(String, "/stuck_recovery/status", 10)
        n.create_subscription(Odometry, n.get_parameter("odom_topic").value, self.on_odom, 20)
        n.create_subscription(Imu, n.get_parameter("imu_topic").value, self.on_imu, 20)
        n.create_subscription(LaserScan, n.get_parameter("scan_topic").value, self.on_scan, 10)
        n.create_subscription(TwistMsg, n.get_parameter("task_topic").value, self.on_task, 10)
        n.create_service(Trigger, "/stuck_recovery/clear_abort", self.on_clear_abort)

        period = 1.0 / float(n.get_parameter("rate_hz").value)
        self.timer = n.create_timer(period, self.tick)
        self._status_div = 0
        n.get_logger().info("stuck_recovery 已启动：透传任务指令，检测到卡死时接管 /cmd_vel")

    # ------------------------------------------------------------------ 回调
    def on_odom(self, msg) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._last_pose = self.pose
        self.pose = (p.x, p.y, yaw)
        self.v_odom = msg.twist.twist.linear.x
        self.w_odom = msg.twist.twist.angular.z
        self.watch.update(self._now())

    def on_imu(self, msg) -> None:
        self.w_imu = msg.angular_velocity.z

    def on_scan(self, msg) -> None:
        self.range_front = front_range(
            msg.ranges, msg.angle_min, msg.angle_increment, FRONT_SECTOR, msg.range_max
        )

    def on_task(self, msg) -> None:
        self.task_cmd = Twist(msg.linear.x, msg.angular.z)

    def on_clear_abort(self, request, response):
        self.supervisor.clear_abort()
        response.success = True
        response.message = "已解除 ABORT，恢复透传"
        return response

    # ------------------------------------------------------------------ 主循环
    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def tick(self) -> None:
        now = self._now()
        out, verdict, odom_age = evaluate(
            now,
            self.task_cmd,
            self.watch,
            self.detector,
            self.supervisor,
            v_odom=self.v_odom,
            w_odom=self.w_odom,
            w_imu=self.w_imu,
            range_front=self.range_front,
            pose=self.pose,
        )

        msg = self._twist_msg()
        msg.linear.x = out.v
        msg.angular.z = out.w
        self.pub_cmd.publish(msg)

        self._status_div += 1
        if self._status_div % 10 == 0 or out.override:
            status = self.supervisor.status()
            status.update(
                {
                    "cause_zh": verdict.cause_zh,
                    "slip_ratio": round(verdict.slip_ratio, 3),
                    "note": out.note,
                    "cmd": [round(out.v, 3), round(out.w, 3)],
                    # 未收到里程计时 age=inf，JSON 里用 null 表示"不可用"
                    "odom_age": round(odom_age, 2) if odom_age < float("inf") else None,
                    "odom_ok": odom_available(odom_age, self.supervisor.cfg.stale_odom),
                }
            )
            self.pub_status.publish(self._string_msg(data=json.dumps(status, ensure_ascii=False)))

    def stop(self) -> None:
        msg = self._twist_msg()
        self.pub_cmd.publish(msg)


def main(args=None) -> None:
    import rclpy

    rclpy.init(args=args)
    wrapper = StuckRecoveryNode()
    try:
        rclpy.spin(wrapper.node)
    except KeyboardInterrupt:
        pass
    finally:
        wrapper.stop()
        wrapper.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
