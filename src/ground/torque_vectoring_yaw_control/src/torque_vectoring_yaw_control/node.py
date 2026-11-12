"""ROS 2 节点：把扭矩矢量接到四轮驱动力指令上。

订阅
----
``/vehicle/velocity``       : ``geometry_msgs/TwistStamped``，``linear.x``=纵向车速 u、
                              ``linear.y``=侧向车速 v、``angular.z``=横摆角速度 r
``/vehicle/steer``          : ``std_msgs/Float32``，前轮转角（°，左正）
``/vehicle/drive_force_cmd``: ``std_msgs/Float32``，上层给出的**总纵向力**需求（N）
``/vehicle/accel``          : ``std_msgs/Float32``，纵向加速度（m/s²，可选，用于载荷转移）

发布
----
``/vehicle/wheel_force_cmd`` : ``std_msgs/Float32MultiArray``，四轮纵向力（N），
                               顺序固定 左前/右前/左后/右后
``/vehicle/wheel_torque_cmd``: ``std_msgs/Float32MultiArray``，四轮轮端扭矩（N·m）
``/vehicle/yaw_diag``        : ``std_msgs/String``，JSON，含参考横摆率、饱和标志、附着利用率

**为什么总纵向力由上游给**：扭矩矢量的职责只是在"总力不变"的前提下重新分配四个轮子，
纵向力需求属于驱动/制动/ACC 的权限。节点自己再闭环速度会与上层抢权限——离线仿真里
那个速度环只是为了让工况可复现，不该搬到车上。

参数
----
``mu``          路面附着系数（保守取值，宁可低估）
``enable_tv``   ``false`` 时退化为等力分配（开放式差速器），用于对照
``dead_wheels`` 失效电机，逗号分隔，``0/1/2/3`` = 左前/右前/左后/右后
``rate_hz``     控制频率，默认 100 Hz
"""

from __future__ import annotations

import json
import math

from .control import ControllerConfig, YawController
from .vehicle import VehicleSpec, WheelState

__all__ = ["TorqueVectoringNode", "parse_dead_wheels", "build", "main"]


def parse_dead_wheels(text: str) -> tuple[int, ...]:
    """把 ``"3"`` / ``"0,2"`` 这类参数解析成轮位元组，越界值直接丢弃。"""
    out = []
    for chunk in str(text).replace(";", ",").split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        try:
            idx = int(chunk)
        except ValueError:
            continue
        if 0 <= idx <= 3 and idx not in out:
            out.append(idx)
    return tuple(out)


def build(enable_tv: bool = True, mu: float = 0.95) -> tuple[VehicleSpec, YawController]:
    """构造车辆参数与控制器；``mu`` 只影响横摆率上限与附着额度。"""
    spec = VehicleSpec(mu=float(mu))
    return spec, YawController(spec, ControllerConfig(enabled=bool(enable_tv)))


class TorqueVectoringNode:  # pragma: no cover - 需要 rclpy 才实例化
    """薄封装：把 ROS 消息转成一次分配计算。"""

    def __init__(self) -> None:
        import rclpy
        from geometry_msgs.msg import TwistStamped
        from rclpy.node import Node
        from std_msgs.msg import Float32, Float32MultiArray, String

        self._node = Node("torque_vectoring_node")
        n = self._node
        n.declare_parameter("mu", 0.95)
        n.declare_parameter("enable_tv", True)
        n.declare_parameter("dead_wheels", "")
        n.declare_parameter("rate_hz", 100.0)

        mu = float(n.get_parameter("mu").value)
        enable_tv = bool(n.get_parameter("enable_tv").value)
        self.dead_wheels = parse_dead_wheels(n.get_parameter("dead_wheels").value)
        self.spec, self.controller = build(enable_tv, mu)
        self.arms = WheelState.moment_arms(self.spec.track)

        self.u = 0.0
        self.v = 0.0
        self.r = 0.0
        self.steer = 0.0
        self.ax = 0.0
        self.force_demand = 0.0
        self._have_demand = False

        n.create_subscription(TwistStamped, "/vehicle/velocity", self._on_velocity, 10)
        n.create_subscription(Float32, "/vehicle/steer", self._on_steer, 10)
        n.create_subscription(Float32, "/vehicle/accel", self._on_accel, 10)
        n.create_subscription(Float32, "/vehicle/drive_force_cmd", self._on_demand, 10)
        self.pub_force = n.create_publisher(Float32MultiArray, "/vehicle/wheel_force_cmd", 10)
        self.pub_torque = n.create_publisher(Float32MultiArray, "/vehicle/wheel_torque_cmd", 10)
        self.pub_diag = n.create_publisher(String, "/vehicle/yaw_diag", 10)

        rate = max(1.0, float(n.get_parameter("rate_hz").value))
        self.dt = 1.0 / rate
        n.create_timer(self.dt, self._tick)
        n.get_logger().info(
            f"扭矩矢量节点就绪：mu={mu:.2f}，enable_tv={enable_tv}，"
            f"失效轮={self.dead_wheels or '无'}，{rate:.0f} Hz")
        self._rclpy = rclpy

    # -------------------------------------------------------------- 回调
    def _on_velocity(self, msg) -> None:
        self.u = float(msg.twist.linear.x)
        self.v = float(msg.twist.linear.y)
        self.r = float(msg.twist.angular.z)

    def _on_steer(self, msg) -> None:
        self.steer = math.radians(float(msg.data))

    def _on_accel(self, msg) -> None:
        self.ax = float(msg.data)

    def _on_demand(self, msg) -> None:
        self.force_demand = float(msg.data)
        self._have_demand = True

    def wheel_caps(self) -> list[float]:
        """每轮纵向力额度 = min(附着椭圆, 电机能力)；失效轮归零。"""
        fy_f, fy_r, fz_f, fz_r = self.spec.axle_forces(
            self.u, self.v, self.r, self.steer, self.ax)
        caps = []
        for i, (load, fy_axle) in enumerate(((fz_f, fy_f), (fz_f, fy_f),
                                            (fz_r, fy_r), (fz_r, fy_r))):
            cap = self.spec.force_cap(load / 2.0, fy_axle / 2.0)
            if i in self.dead_wheels:
                cap = 0.0
            caps.append(min(cap, self.spec.motor_force_max()))
        return caps

    def _tick(self) -> None:
        from std_msgs.msg import Float32MultiArray, String

        if not self._have_demand:
            # 上游还没发过需求时保持零力，不要用"当前车速"猜一个目标速度——
            # 猜错的代价是一次满力制动。
            return

        out = self.controller.step(
            u=self.u, r=self.r, steer=self.steer, u_ref=0.0, ax_ff=0.0,
            caps=self.wheel_caps(), dt=self.dt, force_demand=self.force_demand,
        )
        radius = self.spec.wheel_radius
        self.pub_force.publish(Float32MultiArray(data=[float(f) for f in out.fx]))
        self.pub_torque.publish(Float32MultiArray(data=[float(f * radius) for f in out.fx]))
        self.pub_diag.publish(String(data=json.dumps({
            "yaw_rate": round(self.r, 4),
            "yaw_rate_ref": round(out.yaw_rate_ref, 4),
            "moment_cmd": round(out.moment_cmd, 1),
            "moment_achieved": round(out.moment_achieved, 1),
            "force_demand": round(out.force_demand, 1),
            "saturated": bool(out.saturated),
            "utilization": round(out.utilization, 3),
        }, ensure_ascii=False)))

    def spin(self) -> None:
        self._rclpy.spin(self._node)

    def destroy(self) -> None:
        self._node.destroy_node()


def main(argv: list[str] | None = None) -> None:  # pragma: no cover
    import rclpy

    rclpy.init(args=argv)
    node = TorqueVectoringNode()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
