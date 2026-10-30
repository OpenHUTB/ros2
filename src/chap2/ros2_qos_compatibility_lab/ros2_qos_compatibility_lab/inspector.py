"""ROS 2 node that reports publisher/subscriber QoS compatibility."""

from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node

from .compatibility import EndpointQoS, compare_qos


def _endpoint_qos(endpoint_info: object) -> EndpointQoS:
    profile = endpoint_info.qos_profile
    return EndpointQoS(
        reliability=profile.reliability,
        durability=profile.durability,
    )


def _endpoint_id(endpoint_info: object) -> Tuple[object, ...]:
    return (
        endpoint_info.node_namespace,
        endpoint_info.node_name,
        tuple(endpoint_info.endpoint_gid),
    )


def _policy_text(value: object) -> str:
    name = getattr(value, "name", None)
    if name is not None:
        return str(name).lower()
    text = str(value)
    return text.rsplit(".", 1)[-1].lower()


class QosCompatibilityInspector(Node):
    """Discover endpoints and compare their offered/requested QoS."""

    def __init__(self) -> None:
        super().__init__("qos_compatibility_inspector")
        self.declare_parameter("scan_period_sec", 2.0)
        self.declare_parameter("topic_filter", "")
        self.declare_parameter("include_compatible", True)

        scan_period = float(self.get_parameter("scan_period_sec").value)
        if scan_period <= 0.0:
            self.get_logger().warning(
                "scan_period_sec must be positive; using 2.0 seconds"
            )
            scan_period = 2.0
        self._topic_filter = str(
            self.get_parameter("topic_filter").value
        )
        self._include_compatible = bool(
            self.get_parameter("include_compatible").value
        )
        self._reports: Dict[Tuple[object, ...], bool] = {}
        self.create_timer(scan_period, self._scan)
        self.get_logger().info(
            "QoS inspector started: "
            f"scan_period={scan_period:.1f}s "
            f"topic_filter={self._topic_filter or '*'}"
        )

    def _scan(self) -> None:
        current_reports: Dict[Tuple[object, ...], bool] = {}
        topic_names_and_types = self.get_topic_names_and_types()

        for topic_name, _topic_types in topic_names_and_types:
            if self._topic_filter not in topic_name:
                continue
            publishers = self.get_publishers_info_by_topic(topic_name)
            subscribers = self.get_subscriptions_info_by_topic(topic_name)
            for publisher in publishers:
                for subscriber in subscribers:
                    self._inspect_pair(
                        topic_name,
                        publisher,
                        subscriber,
                        current_reports,
                    )

        self._reports = current_reports

    def _inspect_pair(
        self,
        topic_name: str,
        publisher: object,
        subscriber: object,
        current_reports: Dict[Tuple[object, ...], bool],
    ) -> None:
        pair_id = (
            topic_name,
            _endpoint_id(publisher),
            _endpoint_id(subscriber),
        )
        try:
            report = compare_qos(
                _endpoint_qos(publisher),
                _endpoint_qos(subscriber),
            )
        except ValueError as error:
            self.get_logger().error(
                f"{topic_name}: unable to inspect QoS: {error}"
            )
            return

        current_reports[pair_id] = report.compatible
        previous_verdict: Optional[bool] = self._reports.get(pair_id)
        if previous_verdict is report.compatible:
            return
        if report.compatible and not self._include_compatible:
            return

        publisher_label = (
            f"{publisher.node_namespace}{publisher.node_name}"
        )
        subscriber_label = (
            f"{subscriber.node_namespace}{subscriber.node_name}"
        )
        details = (
            f"publisher={publisher_label} "
            f"subscriber={subscriber_label} "
            f"reliability={_policy_text(publisher.qos_profile.reliability)}"
            f"->{_policy_text(subscriber.qos_profile.reliability)} "
            f"durability={_policy_text(publisher.qos_profile.durability)}"
            f"->{_policy_text(subscriber.qos_profile.durability)}"
        )
        if report.compatible:
            self.get_logger().info(
                f"QoS COMPATIBLE | {topic_name} | {details}"
            )
            return

        self.get_logger().warning(
            f"QoS INCOMPATIBLE | {topic_name} | {details}"
        )
        for issue in report.issues:
            self.get_logger().warning(
                f"  {issue.policy}: publisher offers "
                f"{issue.publisher_value}, subscriber requests "
                f"{issue.subscriber_value}. {issue.recommendation}"
            )


def main(args: Optional[List[str]] = None) -> None:
    """Run the QoS compatibility inspector."""

    rclpy.init(args=args)
    node: Optional[QosCompatibilityInspector] = None
    try:
        node = QosCompatibilityInspector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
