# -*- coding: utf-8 -*-
"""评测节点：把"手感"算成可判定的三个数。

发布 ``/rov/eval``（``Float64MultiArray``，5 Hz）：
    [0] 漂移率 D      —— 最近 10 s 的 |w|（机体系垂向速度）均值，m/s
    [1] η_x           —— 纵荡方向操控权限利用率（0～1）
    [2] η_y           —— 横荡
    [3] η_z           —— 升沉
    [4] 净浮力         —— N，正数表示会自己上浮

指标定义
--------
* 漂移率：判据 ``D ≤ 0.05 m/s`` 视为配平合格；
* 操控权限利用率：``η = (可用推力 − k·v_c²) / 可用推力``，``η < 0.2`` 视为
  在该流速下已失去该轴的有效操控能力。

注意：η 用的是**洋流真值**（``/rov/current``），控制器看不到它——
这样"抗流实验"考的是手操，而不是控制器偷看答案。
"""

import collections

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .dynamics import VehicleParams, current_authority


class EvalNode(Node):
    def __init__(self):
        super().__init__("eval_node")

        self.declare_parameter("window_sec", 10.0)
        self.declare_parameter("rate", 5.0)
        self.declare_parameter("mass", 11.0)
        self.declare_parameter("displacement", 0.0110)
        self.declare_parameter("ballast", 0.0)
        self.declare_parameter("rho", 1025.0)
        self.declare_parameter("drag_linear", [13.20, 35.69, 63.33])
        self.declare_parameter("max_force", [10.0, 10.0, 10.0])
        self.declare_parameter("csv_path", "")     # 非空则把每次输出追加到 CSV

        gp = self.get_parameter
        self.window = float(gp("window_sec").value)
        self.rate = float(gp("rate").value)
        self.csv_path = str(gp("csv_path").value).strip()

        self.params = VehicleParams(
            mass=float(gp("mass").value),
            displacement=float(gp("displacement").value),
            ballast=float(gp("ballast").value),
            rho=float(gp("rho").value),
            drag_linear=list(gp("drag_linear").value),
            max_force=list(gp("max_force").value),
        )

        # 漂移窗口：按"秒"而不是按样本数，采样率变了也不会算错
        self.samples = collections.deque()
        self.current_speed = 0.0

        self.create_subscription(Odometry, "/rov/odom", self.on_odom, 10)
        self.create_subscription(Vector3, "/rov/current", self.on_current, 10)
        self.pub = self.create_publisher(Float64MultiArray, "/rov/eval", 10)
        self.create_timer(1.0 / self.rate, self.on_timer)

        if self.csv_path:
            with open(self.csv_path, "w", encoding="utf-8") as f:
                f.write("t,drift,eta_x,eta_y,eta_z,net_buoyancy,current_speed\n")
            self.get_logger().info("评测结果写入 %s" % self.csv_path)

        self.get_logger().info(
            "eval_node 启动：窗口 %.0f s，净浮力 %+.2f N，配平判据 D≤0.05 m/s"
            % (self.window, self.params.net_buoyancy()))

    # ---------------------------------------------------------------- 回调
    def on_odom(self, msg: Odometry):
        w = abs(float(msg.twist.twist.linear.z))
        self.samples.append((self.get_clock().now().nanoseconds * 1e-9, w))
        # 丢掉窗口外的样本
        cutoff = self.get_clock().now().nanoseconds * 1e-9 - self.window
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()

    def on_current(self, msg: Vector3):
        self.current_speed = float(np.hypot(msg.x, msg.y))

    # ---------------------------------------------------------------- 输出
    def on_timer(self):
        drift = float(np.mean([s[1] for s in self.samples])) if self.samples else 0.0
        eta = current_authority(self.params, self.current_speed)

        msg = Float64MultiArray()
        msg.data = [drift, float(eta[0]), float(eta[1]), float(eta[2]),
                    float(self.params.net_buoyancy())]
        self.pub.publish(msg)

        if self.csv_path:
            t = self.get_clock().now().nanoseconds * 1e-9
            with open(self.csv_path, "a", encoding="utf-8") as f:
                f.write("%.3f,%.4f,%.3f,%.3f,%.3f,%.2f,%.3f\n"
                        % (t, drift, eta[0], eta[1], eta[2],
                           self.params.net_buoyancy(), self.current_speed))

        if drift > 0.05:
            self.get_logger().debug("漂移 %.3f m/s 超过判据 0.05，建议检查配平" % drift)


def main(args=None):
    rclpy.init(args=args)
    node = EvalNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
