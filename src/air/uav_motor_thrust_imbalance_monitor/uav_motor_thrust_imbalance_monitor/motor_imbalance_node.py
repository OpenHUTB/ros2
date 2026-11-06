"""ROS 2 节点封装：把 mixer 指令与 ESC 遥测交给推力不对称判据并发布结果。

只在 `python main.py --ros` 时导入。判据逻辑全部在
motor_imbalance_math.py，本文件只做话题收发、时间戳与看门狗。

话题约定（全部使用标准消息，无自定义接口）：
  订阅 /drone/motor_commands  std_msgs/Float32MultiArray
        mixer 输出的各电机归一化指令 [0,1]（长度 = n_motors）
  订阅 /drone/esc_rpm         std_msgs/Float32MultiArray
        各 ESC 回报的实测转速（RPM）；没有转速遥测可不订阅/不上报
  发布 /drone/motor_health    std_msgs/Bool（False = 已检测到不对称）
  发布 /drone/emergency_land  std_msgs/Bool（True = 建议立即紧急降落）
  发布 /drone/commands_compensated std_msgs/Float32MultiArray
        建议下发的不对称补偿指令（已叠加原指令并封顶 [0,1]）
  发布 /drone/motor_status    std_msgs/String（JSON：状态与判据明细）

两路订阅按时间戳配对：以 mixer 指令帧为主时钟，转速帧取最近 1 个周期
内的值；超时由 10 Hz 看门狗统一处理（封顶 IMBALANCE，见判据文件）。
"""

from __future__ import annotations

import json
import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String

from .motor_imbalance_math import (
    Config,
    MotorImbalanceMonitor,
    MotorSample,
    MotorOutput,
    clamp_commands,
)


class MotorImbalanceNode(Node):
    """订阅指令与 ESC 转速，输出推力不对称检测结果。"""

    def __init__(self, **kwargs) -> None:
        # **kwargs 透传给 rclpy.Node（parameter_overrides 等），便于测试
        super().__init__("motor_imbalance_monitor", **kwargs)
        defaults = {
            "n_motors": 4,
            "rpm_max": 8000.0,
            "hover_cmd": 0.5,
            "track_err_warn_pct": 10.0,
            "track_err_emg_pct": 25.0,
            "cmd_dev_warn_pct": 15.0,
            "cmd_dev_emg_pct": 40.0,
            "near_hover_band": 0.15,
            "imbalance_limit": 3,
            "emergency_limit": 5,
            "recovery_streak": 5,
            "baseline_frames": 25,
            "ema_alpha": 0.3,
            "stale_timeout": 1.0,
            "comp_gain": 0.5,
            "comp_limit": 0.15,
            "cmd_max": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def p(name: str):
            return self.get_parameter(name).value

        cfg = Config(
            n_motors=int(p("n_motors")),
            rpm_max=float(p("rpm_max")),
            hover_cmd=float(p("hover_cmd")),
            track_err_warn_pct=float(p("track_err_warn_pct")),
            track_err_emg_pct=float(p("track_err_emg_pct")),
            cmd_dev_warn_pct=float(p("cmd_dev_warn_pct")),
            cmd_dev_emg_pct=float(p("cmd_dev_emg_pct")),
            near_hover_band=float(p("near_hover_band")),
            imbalance_limit=int(p("imbalance_limit")),
            emergency_limit=int(p("emergency_limit")),
            recovery_streak=int(p("recovery_streak")),
            baseline_frames=int(p("baseline_frames")),
            ema_alpha=float(p("ema_alpha")),
            stale_timeout=float(p("stale_timeout")),
            comp_gain=float(p("comp_gain")),
            comp_limit=float(p("comp_limit")),
            cmd_max=float(p("cmd_max")),
        )
        self._cfg = cfg
        self._monitor = MotorImbalanceMonitor(cfg)
        self._n = cfg.n_motors
        self._last_cmds: Optional[Tuple[float, List[float]]] = None
        self._last_rpm: Optional[Tuple[float, List[float]]] = None

        self._pub_health = self.create_publisher(Bool, "/drone/motor_health", 10)
        self._pub_emg = self.create_publisher(
            Bool, "/drone/emergency_land", 10
        )
        self._pub_comp = self.create_publisher(
            Float32MultiArray, "/drone/commands_compensated", 10
        )
        self._pub_status = self.create_publisher(String, "/drone/motor_status", 10)

        self.create_subscription(
            Float32MultiArray, "/drone/motor_commands", self._on_cmds, 10
        )
        self.create_subscription(
            Float32MultiArray, "/drone/esc_rpm", self._on_rpm, 10
        )
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info(
            "电机推力不对称监测已启动，配置: " + json.dumps(
                {"n_motors": self._n, "rpm_max": cfg.rpm_max,
                 "hover_cmd": cfg.hover_cmd},
                ensure_ascii=False,
            )
        )

    # --- 订阅回调 -------------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmds(self, msg: Float32MultiArray) -> None:
        t = self._now()
        cmds = [float(x) for x in msg.data]
        self._last_cmds = (t, cmds)
        self._try_update(t)

    def _on_rpm(self, msg: Float32MultiArray) -> None:
        self._last_rpm = (self._now(), [float(x) for x in msg.data])
        # 转速帧不触发判据：以指令帧为主时钟，避免一拍两判

    def _try_update(self, t: float) -> None:
        if self._last_cmds is None:
            return
        ct, cmds = self._last_cmds
        rpm = None
        if self._last_rpm is not None:
            rt, rvals = self._last_rpm
            # 转速帧过旧（> 2 个判据周期）就不采用，交给看门狗
            if t - rt <= 2.0 * 0.1 + 1e-6:
                rpm = rvals
        out = self._monitor.update(MotorSample(t=ct, commands=cmds, rpm=rpm))
        self._publish(out, cmds)

    # --- 发布与看门狗 ---------------------------------------------------

    def _publish(self, out: MotorOutput, cmds: Optional[List[float]]) -> None:
        health = Bool()
        health.data = bool(out.healthy)
        self._pub_health.publish(health)

        emg = Bool()
        emg.data = bool(out.emergency_land)
        self._pub_emg.publish(emg)

        if cmds is not None:
            comp = Float32MultiArray()
            comp.data = [float(x) for x in clamp_commands(
                cmds, out.compensation, self._cfg)]
            self._pub_comp.publish(comp)

        def rnd(v: float):
            return round(v, 2) if math.isfinite(v) else None

        status = String()
        status.data = json.dumps(
            {
                "state": out.state.value,
                "reason": out.reason,
                "healthy": out.healthy,
                "emergency_land": out.emergency_land,
                "worst_motor": out.worst_motor,
                "track_err_pct": [rnd(v) for v in out.track_err_pct],
                "cmd_dev_pct": [rnd(v) for v in out.cmd_dev_pct],
                "rpm_available": out.rpm_available,
            },
            ensure_ascii=False,
        )
        self._pub_status.publish(status)
        if out.state.value == "EMERGENCY" and out.emergency_land:
            self.get_logger().warn(
                f"电机 {out.worst_motor} 推力不对称超出补偿能力，"
                "建议立即紧急降落"
            )

    def _watchdog(self) -> None:
        if self._monitor.last_seen_t is None:
            return
        now = self._now()
        if now - self._monitor.last_seen_t <= self._cfg.stale_timeout:
            return
        out = self._monitor.note_timeout(now)
        if out is not None:
            if out.state.value == "IMBALANCE":
                self.get_logger().warn(
                    "动力遥测超时：判据降级为 IMBALANCE（断流不等同动力失效，"
                    "不触发紧急降落）"
                )
            self._publish(out, None)


def main(args=None) -> int:
    rclpy.init(args=args)
    node = MotorImbalanceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
