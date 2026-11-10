#!/usr/bin/env python3
"""CSL 速度控制节点。

纯 ROS 决策层：订阅桥接节点发布的目标速度与自车实际速度，
运行滞回状态机 + 加减速斜坡，输出油门/制动控制指令。
与 CARLA 完全解耦，可离线喂数据测试。

话题：
  订阅  /csl/target_speed  /csl/ego_speed  (std_msgs/Float32)
  发布  /csl/control  (std_msgs/Float32MultiArray)  [throttle, brake]
  发布  /csl/state    (std_msgs/String)   cruise/slowdown
  发布  /csl/event    (std_msgs/String)   跃迁事件（slowdown_warn / cruise_resume）

控制律（P 控制至指令速度）：
  commanded_speed = 斜坡逼近 target_speed（受 decel_max/accel_max 限幅）
  误差 e = commanded_speed - ego_speed
  e > 0 → throttle = kp_throttle * e（上限 1）；e < 0 → brake = kp_brake * -e（上限 1）
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String

from .curve_speed_core import CSLController, CSLParams


class CSLControlNode(Node):

    def __init__(self):
        super().__init__("csl_control_node")
        self.declare_parameter("cruise_speed", 8.0)
        self.declare_parameter("a_lat_max", 2.5)
        self.declare_parameter("look_ahead", 30.0)
        self.declare_parameter("v_min", 1.5)
        self.declare_parameter("decel_max", 2.0)
        self.declare_parameter("accel_max", 1.5)
        self.declare_parameter("slow_enter", 0.5)
        self.declare_parameter("slow_exit", 0.2)
        self.declare_parameter("valid_max", 30.0)
        self.declare_parameter("period", 0.05)
        self.declare_parameter("kp_throttle", 0.45)
        self.declare_parameter("kp_brake", 0.28)

        self.p = CSLParams(
            cruise_speed=float(self.get_parameter("cruise_speed").value),
            a_lat_max=float(self.get_parameter("a_lat_max").value),
            look_ahead=float(self.get_parameter("look_ahead").value),
            v_min=float(self.get_parameter("v_min").value),
            decel_max=float(self.get_parameter("decel_max").value),
            accel_max=float(self.get_parameter("accel_max").value),
            slow_enter=float(self.get_parameter("slow_enter").value),
            slow_exit=float(self.get_parameter("slow_exit").value),
            valid_max=float(self.get_parameter("valid_max").value),
        )
        self.period = float(self.get_parameter("period").value)
        self.kp_throttle = float(self.get_parameter("kp_throttle").value)
        self.kp_brake = float(self.get_parameter("kp_brake").value)

        self.ctrl = CSLController(self.p)
        self._target = self.p.cruise_speed   # 桥接未发布前的初值
        self._speed = 0.0                    # 自车实测速度
        self._last_state = self.ctrl._state

        self.sub_target = self.create_subscription(
            Float32, "csl/target_speed", self._on_target, 10)
        self.sub_speed = self.create_subscription(
            Float32, "csl/ego_speed", self._on_speed, 10)
        self.pub_control = self.create_publisher(
            Float32MultiArray, "csl/control", 10)
        self.pub_state = self.create_publisher(String, "csl/state", 10)
        self.pub_event = self.create_publisher(String, "csl/event", 10)
        self.timer = self.create_timer(self.period, self._step)

    def _on_target(self, msg):
        self._target = float(msg.data)

    def _on_speed(self, msg):
        self._speed = float(msg.data)

    def _step(self):
        """一个控制拍：状态机 + 斜坡 → P 控制油门/制动。"""
        r = self.ctrl.tick(self._target, self.period)
        e = r["commanded_speed"] - self._speed
        if e > 0:
            throttle = min(self.kp_throttle * e, 1.0)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(self.kp_brake * (-e), 1.0)
        self.pub_control.publish(
            Float32MultiArray(data=[float(throttle), float(brake)]))
        self.pub_state.publish(String(data=r["state"]))
        if r["event"] and r["state"] != self._last_state:
            self.pub_event.publish(String(data=r["event"]))
            self._last_state = r["state"]
            self.get_logger().info(
                f"状态迁移 → {r['state']}（目标 {r['target_speed']:.1f} m/s）")


def main(args=None):
    rclpy.init(args=args)
    node = CSLControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
