#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AEB 控制节点：距离/速度话题 → AEB 状态机 → 控制指令。

纯 ROS 2 节点，不依赖 CARLA 客户端——与 CARLA 世界只通过话题交互，
因此可以离线喂合成数据测试，也可以接入任意传感器源。

话题：
  订阅  /aeb/obstacle_distance  (std_msgs/Float32, 米；无障碍时为 valid_max)
        /aeb/ego_speed          (std_msgs/Float32, m/s)
  发布  /aeb/control            (std_msgs/Float32MultiArray, [throttle, steer, brake])
        /aeb/state              (std_msgs/String, cruise/warning/braking/recover)
        /aeb/event              (std_msgs/String, warn/brake/recover，仅触发时)
服务：
  /aeb/arm   (std_srvs/Trigger)  启用 AEB
  /aeb/abort (std_srvs/Trigger)  紧急中止：释放制动、回到巡航
  /aeb/reset (std_srvs/Trigger)  复位状态机与历史

决策逻辑复用 aeb_core.AEBController（滞回状态机）；相对接近速度由
前后两帧距离差分估计（estimate_closing），首帧以自车速度为兜底
（静止障碍物近似）。
"""
import os
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String
from std_srvs.srv import Trigger

# 允许以脚本方式直接运行（launch 传入时包尚未安装）
_HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from carla_aeb_braking.aeb_core import (  # noqa: E402
    CRUISE,
    AEBController,
    AEBParams,
    estimate_closing,
)

STEER = 0.0  # AEB 只管纵向（油门/制动），转向保持中性


class AEBControllerNode(Node):
    """AEB 控制节点：状态机决策 + arm/abort/reset 服务。"""

    def __init__(self):
        super().__init__("aeb_controller_node")

        # 决策参数：默认值与 config/aeb_params.yaml 一致，launch 传入 yaml 时覆盖
        defaults = AEBParams()
        fields = list(AEBParams.__dataclass_fields__)
        for name in fields:
            self.declare_parameter(name, float(getattr(defaults, name)))
        p = AEBParams(**{n: float(self.get_parameter(n).value) for n in fields})

        self.declare_parameter("start_armed", True)
        self._armed = bool(self.get_parameter("start_armed").value)

        self.ctl = AEBController(p)
        self._prev_d = None
        self._prev_t = None
        self._speed = 0.0

        self.pub_control = self.create_publisher(Float32MultiArray, "/aeb/control", 10)
        self.pub_state = self.create_publisher(String, "/aeb/state", 10)
        self.pub_event = self.create_publisher(String, "/aeb/event", 10)
        self.create_subscription(Float32, "/aeb/obstacle_distance", self._on_distance, 10)
        self.create_subscription(Float32, "/aeb/ego_speed", self._on_speed, 10)
        self.create_service(Trigger, "/aeb/arm", self._srv_arm)
        self.create_service(Trigger, "/aeb/abort", self._srv_abort)
        self.create_service(Trigger, "/aeb/reset", self._srv_reset)

        self.get_logger().info(
            f"AEB 控制器就绪（armed={self._armed}）："
            f"warn<{p.warn_enter:g}m，brake<{p.brake_enter:g}m 且接近，"
            f"滞回 {p.warn_exit:g}/{p.brake_exit:g}m")

    # ------------------------------------------------------------------ 回调
    def _on_speed(self, msg):
        self._speed = float(msg.data)

    def _on_distance(self, msg):
        d = float(msg.data)
        now = self.get_clock().now()
        closing = estimate_closing(self._prev_d, d, self._dt_s(now),
                                   fallback=self._speed)
        if not self._armed:
            # 未启用：AEB 不介入，保持巡航
            state, throttle, brake, event = (
                CRUISE, self.ctl.p.cruise_throttle, 0.0, None)
        else:
            state, throttle, brake, event = self.ctl.tick(
                d, closing, t=now.nanoseconds * 1e-9)
        self._prev_d = d
        self._prev_t = now
        self._publish(state, throttle, brake, event)

    def _dt_s(self, now):
        if self._prev_t is None:
            return 0.0
        return max((now - self._prev_t).nanoseconds * 1e-9, 1e-3)

    def _publish(self, state, throttle, brake, event):
        self.pub_control.publish(Float32MultiArray(data=[throttle, STEER, brake]))
        self.pub_state.publish(String(data=state))
        if event:
            self.pub_event.publish(String(data=event))

    # ------------------------------------------------------------------ 服务
    def _srv_arm(self, req, resp):
        self._armed = True
        resp.success = True
        resp.message = f"AEB armed（state={self.ctl.state}）"
        self.get_logger().info(resp.message)
        return resp

    def _srv_abort(self, req, resp):
        self.ctl.abort()
        # 立即释放制动、恢复巡航（不等下一帧距离）
        self._publish(CRUISE, self.ctl.p.cruise_throttle, 0.0, None)
        resp.success = True
        resp.message = "AEB aborted → cruise"
        self.get_logger().info(resp.message)
        return resp

    def _srv_reset(self, req, resp):
        self.ctl.reset()
        self._prev_d = None
        self._prev_t = None
        resp.success = True
        resp.message = "AEB controller reset"
        self.get_logger().info(resp.message)
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = AEBControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
