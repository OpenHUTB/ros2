"""ROS 2 适配节点：按当前制动盘温度给出摩擦系数、停车距离与热衰退提示。

节点只做"读温度 → 查衰退曲线 → 给出结论"，热模型的积分在离线演示里跑；车上
温度通常来自制动盘温度传感器或热模型的前一帧估计。
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

from .disc import BrakeDisc, BrakeDiscSpec, FrictionFadeCurve
from .scenario import kmh_to_mps
from .vehicle import BrakingSpec, deceleration_mps2, stopping_distance_m


class BrakeThermalNode(Node):
    """订阅 ``/vehicle/brake_temperature``，发布 ``/vehicle/brake_thermal``。"""

    def __init__(self) -> None:
        super().__init__("brake_thermal_fade")
        self.declare_parameter("stop_speed_kmh", 100.0)
        self.declare_parameter("ambient_c", 25.0)
        self.declare_parameter("fade_start_c", 250.0)
        self.declare_parameter("fade_end_c", 650.0)
        self.declare_parameter("mu_cold", 0.40)
        self.declare_parameter("mu_hot", 0.22)

        self.disc = BrakeDisc(
            spec=BrakeDiscSpec(ambient_c=float(self.get_parameter("ambient_c").value)),
            curve=FrictionFadeCurve(
                mu_cold=float(self.get_parameter("mu_cold").value),
                fade_start_c=float(self.get_parameter("fade_start_c").value),
                fade_end_c=float(self.get_parameter("fade_end_c").value),
                mu_hot=float(self.get_parameter("mu_hot").value),
            ),
        )
        self.braking = BrakingSpec()
        self.stop_speed_kmh = float(self.get_parameter("stop_speed_kmh").value)

        self.create_subscription(Float64, "/vehicle/brake_temperature", self.on_temperature, 10)
        self.publisher = self.create_publisher(String, "/vehicle/brake_thermal", 10)
        self.peak_temperature_c = self.disc.spec.ambient_c
        self.get_logger().info(
            f"制动热衰退监测已启动：衰退区间 "
            f"{self.disc.curve.fade_start_c:.0f}–{self.disc.curve.fade_end_c:.0f} °C，"
            f"评估车速 {self.stop_speed_kmh:.0f} km/h"
        )

    def evaluate(self, temperature_c: float) -> dict:
        """给定盘温算一次结论，纯函数式封装便于单元测试。"""
        curve = self.disc.curve
        temperature = float(temperature_c)
        self.peak_temperature_c = max(self.peak_temperature_c, temperature)
        mu = float(curve.mu(temperature))
        decel = deceleration_mps2(temperature, self.braking, curve)
        distance = stopping_distance_m(
            kmh_to_mps(self.stop_speed_kmh), temperature, self.braking, curve
        )
        cold_distance = stopping_distance_m(
            kmh_to_mps(self.stop_speed_kmh), self.disc.spec.ambient_c, self.braking, curve
        )
        span = curve.fade_end_c - curve.fade_start_c
        fade_ratio = min(max((temperature - curve.fade_start_c) / span, 0.0), 1.0)
        return {
            "temperature_c": round(temperature, 1),
            "peak_temperature_c": round(self.peak_temperature_c, 1),
            "mu": round(mu, 3),
            "deceleration_mps2": round(decel, 2),
            "stop_distance_m": round(distance, 1),
            "cold_stop_distance_m": round(cold_distance, 1),
            "distance_growth_percent": round((distance / cold_distance - 1.0) * 100.0, 1),
            "fade_ratio": round(fade_ratio, 3),
            "in_fade": bool(temperature >= curve.fade_start_c),
            "recommendation": self._recommendation(temperature, fade_ratio),
        }

    def _recommendation(self, temperature: float, fade_ratio: float) -> str:
        curve = self.disc.curve
        if temperature >= curve.fade_end_c:
            return "严重热衰退，靠边停车自然冷却"
        if fade_ratio > 0.5:
            return "明显热衰退，减速并使用发动机制动"
        if fade_ratio > 0.0:
            return "已进入衰退区，注意增大制动距离"
        if temperature >= curve.fade_start_c - 50.0:
            return "盘温接近衰退区，建议降低制动强度"
        return "盘温正常"

    def on_temperature(self, message: Float64) -> None:
        temperature = float(message.data)
        if not temperature == temperature:  # NaN 检查
            self.get_logger().warn("忽略非法盘温：NaN")
            return
        result = self.evaluate(temperature)
        self.publisher.publish(String(data=json.dumps(result, ensure_ascii=False)))
        if result["in_fade"]:
            self.get_logger().warn(
                f"盘温 {result['temperature_c']:.0f} °C，μ={result['mu']:.2f}，"
                f"{self.stop_speed_kmh:.0f}→0 停车距离 {result['stop_distance_m']:.1f} m"
                f"（{result['distance_growth_percent']:+.1f}%），{result['recommendation']}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BrakeThermalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
