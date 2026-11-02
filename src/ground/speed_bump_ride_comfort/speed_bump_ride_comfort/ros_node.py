"""ROS 2 适配节点：按当前车速给出减速带通过建议。

节点不订阅车辆状态以外的任何数据：收到 ``/vehicle/speed``（m/s，``Float64``）后
用同一套四分之一车模型算出峰值加速度与最小轮荷比，把 JSON 结果发布到
``/vehicle/ride_comfort``，供上层限速或提示模块使用。

模型运行是纯数值积分（1 ms 步长、约 2.2 s 仿真时长），单次约几十毫秒，因此做了
发布频率限制，默认最高 5 Hz。
"""

from __future__ import annotations

import json

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

from .analysis import DEFAULT_LIMITS_MPS2, recommend_max_speed, speed_curve
from .bump import SpeedBump
from .comfort import passage_metrics
from .vehicle import QuarterCar


class SpeedBumpComfortNode(Node):
    """订阅车速，发布平顺性指标与建议车速。"""

    def __init__(self) -> None:
        super().__init__("speed_bump_ride_comfort")
        self.declare_parameter("bump_height_m", 0.06)
        self.declare_parameter("bump_width_m", 1.2)
        self.declare_parameter("bump_kind", "half_sine")
        self.declare_parameter("comfort_limit_mps2", DEFAULT_LIMITS_MPS2[0][1])
        self.declare_parameter("publish_period_s", 0.2)
        self.declare_parameter("simulation_dt_s", 1.0e-3)

        self.vehicle = QuarterCar()
        self.bump = SpeedBump(
            height_m=float(self.get_parameter("bump_height_m").value),
            width_m=float(self.get_parameter("bump_width_m").value),
            kind=str(self.get_parameter("bump_kind").value),
        )
        self.limit_mps2 = float(self.get_parameter("comfort_limit_mps2").value)
        self.dt = float(self.get_parameter("simulation_dt_s").value)
        self.publish_period_s = float(self.get_parameter("publish_period_s").value)

        self.advice = self._compute_advice()
        self.last_speed_mps: float | None = None
        self.last_publish_s: float = -1.0e9
        self.last_result: dict | None = None

        self.create_subscription(Float64, "/vehicle/speed", self.on_speed, 10)
        self.publisher = self.create_publisher(String, "/vehicle/ride_comfort", 10)
        self.get_logger().info(
            f"减速带 {self.bump.height_m * 1000:.0f} mm × {self.bump.width_m:.2f} m，"
            f"舒适阈值 {self.limit_mps2:.1f} m/s²，"
            f"建议最高车速 {self._advice_text()}"
        )

    # ------------------------------------------------------------------ 内部计算
    def _compute_advice(self) -> dict:
        curve = speed_curve(self.vehicle, self.bump, dt=self.dt)
        return recommend_max_speed(curve, self.limit_mps2)

    def _advice_text(self) -> str:
        speed = self.advice.get("speed_mps")
        if speed is None:
            return "扫速范围内无满足项"
        return f"{speed * 3.6:.1f} km/h（{speed:.1f} m/s）"

    def evaluate(self, speed_mps: float) -> dict:
        """给定车速算一次指标与结论，纯函数式封装便于单元测试。"""
        passage = self.vehicle.simulate(self.bump, float(speed_mps), dt=self.dt)
        metrics = passage_metrics(
            passage,
            limit_mps2=self.limit_mps2,
            travel_limit_mm=self.vehicle.travel_limit_m * 1000.0,
        )
        return {
            "speed_mps": round(float(speed_mps), 3),
            "speed_kmh": round(float(speed_mps) * 3.6, 2),
            "peak_accel_mps2": metrics["peak_accel_mps2"],
            "rms_accel_mps2": metrics["rms_accel_mps2"],
            "vdv_mps1_75": metrics["vdv_mps1_75"],
            "tire_load_min_ratio": metrics["tire_load_min_ratio"],
            "suspension_travel_mm": metrics["suspension_travel_mm"],
            "comfort_limit_mps2": metrics["comfort_limit_mps2"],
            "exceeds_comfort_limit": metrics["exceeds_comfort_limit"],
            "tire_contact_lost": metrics["tire_load_min_ratio"] <= 0.0,
            "advised_max_speed_kmh": (
                None if self.advice.get("speed_mps") is None
                else round(self.advice["speed_mps"] * 3.6, 2)
            ),
            "recommendation": self._recommendation(metrics),
        }

    def _recommendation(self, metrics: dict) -> str:
        if metrics["tire_load_min_ratio"] <= 0.0:
            return "车轮离地风险，必须减速"
        if metrics["exceeds_comfort_limit"]:
            return "超过舒适阈值，建议减速"
        return "满足舒适阈值"

    # ------------------------------------------------------------------ 回调
    def on_speed(self, message: Float64) -> None:
        speed = float(message.data)
        if not np.isfinite(speed) or speed < 0.0:
            self.get_logger().warn(f"忽略非法车速：{message.data}")
            return
        self.last_speed_mps = speed
        now = self.get_clock().now().nanoseconds * 1.0e-9
        if now - self.last_publish_s < self.publish_period_s:
            return
        self.last_publish_s = now
        result = self.evaluate(speed)
        self.last_result = result
        self.publisher.publish(String(data=json.dumps(result, ensure_ascii=False)))
        if result["exceeds_comfort_limit"]:
            self.get_logger().warn(
                f"{result['speed_kmh']:.1f} km/h：峰值 "
                f"{result['peak_accel_mps2']:.2f} m/s²，{result['recommendation']}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeedBumpComfortNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
