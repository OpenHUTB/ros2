#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通过性核算的 ROS 2 节点：算一次、把报告与结构化结果发出来。

订阅：
    evaluate   std_msgs/Empty   收到一条就重新核算一次（改场景文件后不用重启）
发布：
    report     std_msgs/String  终端同款报告文本（transient_local，后来者也能拿到）
    summary    std_msgs/String  同一份结果的 JSON（字段见 report.to_json_payload）

节点本身不做任何几何计算，全部调用 passability_check 里的纯函数，
所以离线测试覆盖到的就是节点实际跑的代码。
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String

from .report import evaluate, format_report, to_json_payload
from .scenario import ScenarioError, load_scenario
from .vehicle import VehicleError, get_vehicle


class PassabilityNode(Node):
    def __init__(self):
        super().__init__("passability_check")
        self.declare_parameter("vehicle", "sedan")
        self.declare_parameter("scenario", "garage_ramp")
        self.declare_parameter("scenario_file", "")
        self.declare_parameter("samples", 41)
        self.declare_parameter("steps", 480)
        self.declare_parameter("safety_margin_m", -1.0)
        self.declare_parameter("publish_hz", 1.0)
        self.declare_parameter("report_topic", "report")
        self.declare_parameter("summary_topic", "summary")
        self.declare_parameter("evaluate_topic", "evaluate")

        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.report_pub = self.create_publisher(
            String, str(self.get_parameter("report_topic").value), qos)
        self.summary_pub = self.create_publisher(
            String, str(self.get_parameter("summary_topic").value), qos)
        self.create_subscription(
            Empty, str(self.get_parameter("evaluate_topic").value), self.on_evaluate, 10)

        self.report_text = ""
        self.summary_text = ""
        self.result = "UNKNOWN"
        self.evaluate_once()

        hz = float(self.get_parameter("publish_hz").value)
        if hz > 0.0:
            self.create_timer(1.0 / hz, self.publish)

    # ------------------------------------------------------------ 参数
    def _scenario(self):
        path = str(self.get_parameter("scenario_file").value or "")
        scenario = load_scenario(path) if path else \
            load_scenario(str(self.get_parameter("scenario").value))
        margin = float(self.get_parameter("safety_margin_m").value)
        if margin >= 0.0:
            scenario.safety_margin_m = margin
        return scenario

    # ------------------------------------------------------------ 核算
    def evaluate_once(self):
        try:
            vehicle = get_vehicle(str(self.get_parameter("vehicle").value))
            scenario = self._scenario()
            report, assertions = evaluate(
                vehicle, scenario,
                samples=int(self.get_parameter("samples").value),
                steps=int(self.get_parameter("steps").value),
            )
        except (VehicleError, ScenarioError) as error:
            self.get_logger().error("参数错误：%s" % error)
            self.result = "ERROR"
            # 清空上一次的结果，免得 publish() 把过期的报告当成新结果再发一遍
            self.report_text = ""
            self.summary_text = ""
            return False

        header = "通过性核算（%s / %s）" % (vehicle.name, scenario.name)
        self.report_text = format_report(vehicle, scenario, report, assertions, header)
        payload = to_json_payload(vehicle, scenario, report, assertions)
        self.summary_text = json.dumps(payload, ensure_ascii=False)
        self.result = payload["result"]

        # 只有数值判据不通过才是 FAILED（warn）；PASS_WITH_NOTE 是"能过但
        # 角度判据偏保守"，用 info 并带上提示，不制造假告警
        level = self.get_logger().warn if self.result == "FAILED" else self.get_logger().info
        hint = ""
        if self.result == "PASS_WITH_NOTE":
            hint = "（数值复核能过，角度判据偏保守，详见报告备注）"
        level("%s：接近角 %.2f°，离去角 %.2f°，纵向通过角 %.2f°，最大侵入 %.3f m，结论 %s%s"
              % (scenario.title, vehicle.approach_deg, vehicle.departure_deg,
                 vehicle.breakover_deg, report["max_depth"], self.result, hint))
        for item in assertions:
            if not item["passed"]:
                self.get_logger().warn("  [FAIL] %s %s" % (item["type"], item["detail"]))
        return True

    def on_evaluate(self, _message):
        self.get_logger().info("收到 evaluate，重新核算")
        self.evaluate_once()
        self.publish()

    def publish(self):
        if not self.report_text:
            return
        self.report_pub.publish(String(data=self.report_text))
        self.summary_pub.publish(String(data=self.summary_text))


def main(args=None):
    rclpy.init(args=args)
    node = PassabilityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
