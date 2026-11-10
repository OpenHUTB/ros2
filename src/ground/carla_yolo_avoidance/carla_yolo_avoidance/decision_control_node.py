# -*- coding: utf-8 -*-
"""节点③：decision_control_node

职责（对应作业要求 3-③）：
    1. 订阅 YOLO 检测结果（JSON）；
    2. 取出"本车道内最近的车辆距离"，按阈值做避障决策；
    3. 发布车辆控制指令 [油门, 刹车, 转向]：
         距离 < 安全阈值(20 m) -> 刹车；
         距离 >= 安全阈值      -> 正常前进；
       只控制油门与刹车，转向恒为 0（不做转向控制，符合作业要求）；
    4. 同时发布一条人类可读的决策状态字符串，方便 `ros2 topic echo` 排查。

决策函数本身在 decision_logic.py 里，是纯函数，可被单元测试直接调用。
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from . import config, decision_logic


class DecisionControlNode(Node):
    """订阅检测结果，下发油门/刹车指令。"""

    def __init__(self):
        super().__init__('decision_control_node')

        self.publisher_control = self.create_publisher(
            Float32MultiArray, config.TOPIC_CONTROL_CMD, 10
        )
        self.publisher_status = self.create_publisher(
            String, config.TOPIC_DECISION_STATUS, 10
        )
        self.subscription_detections = self.create_subscription(
            String, config.TOPIC_DETECTIONS, self.on_detections, 10
        )

        # 最近一次有效检测
        self._min_distance_m = None
        self._nearest = None                   # 本车道最近的检测（仅用于调试查看）
        self._last_detection_time = None       # None = 从未收到过检测结果
        self._last_vehicle_count = 0
        self._last_state = None

        # 定时按固定频率下发指令（即使没有新检测结果也要持续下发，保证车辆行为连续）
        self.create_timer(1.0 / config.DECISION_PUBLISH_HZ, self.publish_control)

        self.get_logger().info(
            '节点就绪：订阅 {}，发布 {} / {}'.format(
                config.TOPIC_DETECTIONS, config.TOPIC_CONTROL_CMD, config.TOPIC_DECISION_STATUS
            )
        )
        self.get_logger().info(
            '避障阈值：安全距离 {:.1f} m，急刹距离 {:.1f} m，检测超时 {:.1f} s'.format(
                config.SAFE_DISTANCE_M,
                config.FULL_BRAKE_DISTANCE_M,
                config.DETECTION_STALE_TIMEOUT_S,
            )
        )

    # ------------------------------------------------------------------
    # 订阅：检测结果
    # ------------------------------------------------------------------
    def on_detections(self, msg):
        """解析 JSON 检测结果，更新"本车道最近距离"。"""
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError) as exc:
            self.get_logger().warn('检测结果不是合法 JSON，已忽略：{}'.format(exc))
            return

        detections = payload.get('detections') or []
        min_distance, nearest = decision_logic.select_nearest_in_lane(detections)

        self._min_distance_m = min_distance
        self._last_vehicle_count = int(payload.get('vehicle_count') or len(detections))
        # 收到（哪怕为空）都算"有数据"，用于区分"无目标"与"感知挂掉"
        self._last_detection_time = time.monotonic()
        self._nearest = nearest

    # ------------------------------------------------------------------
    # 定时：决策 + 下发
    # ------------------------------------------------------------------
    def publish_control(self):
        """计算决策并发布控制指令。"""
        age = None
        if self._last_detection_time is not None:
            age = time.monotonic() - self._last_detection_time

        decision = decision_logic.decide(
            min_distance_m=self._min_distance_m,
            detection_age_s=age,
            has_detection=self._last_detection_time is not None,
        )

        control_msg = Float32MultiArray()
        control_msg.data = [float(v) for v in decision.as_list()]
        self.publisher_control.publish(control_msg)

        status_msg = String()
        status_msg.data = '[{}] throttle={:.2f} brake={:.2f} steer={:.2f} | vehicles={} | {}'.format(
            decision.state,
            decision.throttle,
            decision.brake,
            decision.steer,
            self._last_vehicle_count,
            decision.reason,
        )
        self.publisher_status.publish(status_msg)

        # 只在状态切换时打印，避免刷屏
        if decision.state != self._last_state:
            self._last_state = decision.state
            if decision.state == 'CRUISE':
                self.get_logger().info('决策 -> 正常前进：{}'.format(decision.reason))
            elif decision.state == 'SLOW_DOWN':
                self.get_logger().warn('决策 -> 减速刹车：{}'.format(decision.reason))
            else:
                self.get_logger().error('决策 -> 刹车/失效保护：{}'.format(decision.reason))


def main(args=None):
    """节点③的启动函数（供 main.py 调用）。"""
    rclpy.init(args=args)
    node = None
    try:
        node = DecisionControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
