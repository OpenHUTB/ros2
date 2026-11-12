"""ROS 2 节点：把走廊约束接到飞控指令上。

订阅
----
``/uav/velocity``  : ``geometry_msgs/TwistStamped``，``linear.x`` 为水平地速、``linear.z`` 为升降速度
``/uav/altitude``  : ``std_msgs/Float32``，海拔高度（m）

发布
----
``/uav/tilt_cmd``      : ``std_msgs/Float32``，倾转角指令（°，0=前飞，90=悬停，>90 为反向推力）
``/uav/thrust_cmd``    : ``std_msgs/Float32``，总推力指令（N）
``/uav/transition_diag`` : ``std_msgs/String``，JSON，含升力裕度与饱和标志

节点只做"把期望加速度换算成推力矢量"，不闭环控制姿态；姿态/角速率回路应在飞控里。
走廊参数（密度高度）通过 ``corridor_altitude`` 指定，必须与当前机场一致——
这是本模块最容易踩的坑，用海平面的表去飞高原会低估下边界。
"""

from __future__ import annotations

import json
import math

from .control import ActuatorState, Controller, ControllerConfig
from .corridor import Corridor
from .vehicle import VehicleSpec

__all__ = ["TransitionCorridorNode", "main"]


def build_controller(spec: VehicleSpec, corridor_altitude: float,
                     cfg: ControllerConfig | None = None) -> tuple[Corridor, Controller]:
    corridor = Corridor(spec, alt_m=corridor_altitude)
    return corridor, Controller(spec, corridor, cfg or ControllerConfig())


class TransitionCorridorNode:  # pragma: no cover - 需要 rclpy 才实例化
    """薄封装：把 ROS 消息转成控制器输入。"""

    def __init__(self) -> None:
        import rclpy
        from geometry_msgs.msg import TwistStamped
        from rclpy.node import Node
        from std_msgs.msg import Float32, String

        self._node = Node("transition_corridor_node")
        n = self._node
        n.declare_parameter("altitude_ref", 30.0)
        n.declare_parameter("airspeed_ref", 0.0)
        n.declare_parameter("corridor_altitude", 0.0)
        n.declare_parameter("assumed_cl", 0.9)
        n.declare_parameter("rate_hz", 50.0)

        alt = float(n.get_parameter("corridor_altitude").value)
        self.assumed_cl = float(n.get_parameter("assumed_cl").value)
        self.corridor, self.controller = build_controller(VehicleSpec(), alt)
        self.spec = self.controller.spec

        self.altitude = 0.0
        self.u = 0.0
        self.w = 0.0
        self.act = ActuatorState(tilt_deg=90.0, thrust=self.spec.weight, cl=self.assumed_cl)

        n.create_subscription(TwistStamped, "/uav/velocity", self._on_velocity, 10)
        n.create_subscription(Float32, "/uav/altitude", self._on_altitude, 10)
        self.pub_tilt = n.create_publisher(Float32, "/uav/tilt_cmd", 10)
        self.pub_thrust = n.create_publisher(Float32, "/uav/thrust_cmd", 10)
        self.pub_diag = n.create_publisher(String, "/uav/transition_diag", 10)

        rate = max(1.0, float(n.get_parameter("rate_hz").value))
        n.create_timer(1.0 / rate, self._tick)

        self.dt = 1.0 / rate
        n.get_logger().info(
            f"过渡走廊节点就绪：密度高度 {alt:.0f} m，"
            f"失速 {self.corridor.stall_speed:.2f} m/s，"
            f"悬停极限 {self.corridor.hover_speed_limit:.2f} m/s")
        self._rclpy = rclpy

    def _on_velocity(self, msg) -> None:
        self.u = float(msg.twist.linear.x)
        self.w = float(msg.twist.linear.z)

    def _on_altitude(self, msg) -> None:
        self.altitude = float(msg.data)

    def _tick(self) -> None:
        from std_msgs.msg import Float32, String

        n = self._node
        h_ref = float(n.get_parameter("altitude_ref").value)
        u_ref = float(n.get_parameter("airspeed_ref").value)
        v = math.hypot(self.u, self.w)
        rho = self.spec.density(self.altitude)

        # 升力用假定 CL 估算：节点拿不到姿态，宁可用偏保守的常数，
        # 也不要用"实测加速度反推升力"这种会把噪声放大成抖动的做法。
        lift = 0.5 * rho * v * v * self.spec.wing.area * self.assumed_cl
        drag = self.spec.drag_at(v, self.assumed_cl, rho)

        self.act, diag = self.controller.step(
            self.act, v=v, u=self.u, w=self.w, h=self.altitude, h_ref=h_ref, u_ref=u_ref,
            rho=rho, drag=drag, lift=lift, dt=self.dt,
        )

        self.pub_tilt.publish(Float32(data=float(self.act.tilt_deg)))
        self.pub_thrust.publish(Float32(data=float(self.act.thrust)))
        self.pub_diag.publish(String(data=json.dumps({
            "lift_margin_deg": round(diag["lift_margin"], 2),
            "airborne": bool(diag["lift_margin"] > 0),
            "saturated": bool(diag["saturated"]),
            "stalled": bool(diag["stalled"]),
            "tilt_deg": round(self.act.tilt_deg, 2),
            "thrust_n": round(self.act.thrust, 2),
        }, ensure_ascii=False)))

    def spin(self) -> None:
        self._rclpy.spin(self._node)

    def destroy(self) -> None:
        self._node.destroy_node()


def main(argv: list[str] | None = None) -> None:  # pragma: no cover
    import rclpy

    rclpy.init(args=argv)
    node = TransitionCorridorNode()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
