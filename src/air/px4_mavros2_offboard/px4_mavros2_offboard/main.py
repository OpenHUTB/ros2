"""Main entry for the ROS 2 Humble MAVROS2 Offboard demo."""

from __future__ import annotations

import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .state_machine import OffboardStateMachine, OffboardTelemetry


class Px4Mavros2Offboard(Node):
    """Expose guarded start/abort services over MAVROS2."""

    def __init__(self) -> None:
        super().__init__("px4_mavros2_offboard")
        self.altitude = float(self.declare_parameter("altitude", 3.0).value)
        self.prestream_seconds = float(
            self.declare_parameter("prestream_seconds", 2.0).value
        )
        self.offboard_timeout = float(
            self.declare_parameter("offboard_timeout", 10.0).value
        )
        self.arm_timeout = float(self.declare_parameter("arm_timeout", 8.0).value)
        self.takeoff_timeout = float(
            self.declare_parameter("takeoff_timeout", 25.0).value
        )
        self.hover_seconds = float(
            self.declare_parameter("hover_seconds", 10.0).value
        )
        self.landing_timeout = float(
            self.declare_parameter("landing_timeout", 30.0).value
        )
        self.state_timeout = float(
            self.declare_parameter("state_timeout", 1.5).value
        )
        self.altitude_tolerance = float(
            self.declare_parameter("altitude_tolerance", 0.3).value
        )
        self.setpoint_rate = float(
            self.declare_parameter("setpoint_rate", 20.0).value
        )
        self.local_frame = str(
            self.declare_parameter("local_frame", "map").value
        )

        if self.setpoint_rate <= 0.0:
            raise ValueError("setpoint_rate must be positive")

        self._machine = OffboardStateMachine(
            altitude=self.altitude,
            prestream_seconds=self.prestream_seconds,
            offboard_timeout=self.offboard_timeout,
            arm_timeout=self.arm_timeout,
            takeoff_timeout=self.takeoff_timeout,
            hover_seconds=self.hover_seconds,
            landing_timeout=self.landing_timeout,
            state_timeout=self.state_timeout,
            altitude_tolerance=self.altitude_tolerance,
        )
        self._state: Optional[State] = None
        self._state_received = 0.0
        self._position: Optional[Tuple[float, float, float]] = None
        self._last_status = ""
        self._last_requests = {
            "offboard": -1e9,
            "arm": -1e9,
            "land": -1e9,
        }
        self._state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._setpoint_publisher = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10
        )
        self._status_publisher = self.create_publisher(
            String, "/uav/offboard/status", 10
        )
        self.create_subscription(
            State,
            "/mavros/state",
            self._on_state,
            self._state_qos,
        )
        self.create_subscription(
            Odometry,
            "/mavros/local_position/odom",
            self._on_position,
            qos_profile_sensor_data,
        )
        self._arming_client = self.create_client(
            CommandBool, "/mavros/cmd/arming"
        )
        self._mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.create_service(
            Trigger, "/uav/offboard/start", self._handle_start
        )
        self.create_service(
            Trigger, "/uav/offboard/abort", self._handle_abort
        )
        self.create_timer(1.0 / self.setpoint_rate, self._on_timer)

        self.get_logger().info(
            "PX4 Offboard safety node ready; call /uav/offboard/start "
            "after MAVROS state and local position are available"
        )

    def _on_state(self, message: State) -> None:
        self._state = message
        self._state_received = time.monotonic()

    def _on_position(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self._position = (position.x, position.y, position.z)

    def _telemetry(self, now: float) -> OffboardTelemetry:
        if self._state is None or self._position is None:
            return OffboardTelemetry(
                now=now,
                connected=False,
                mode="",
                armed=False,
                altitude=0.0,
                state_age=float("inf"),
            )
        return OffboardTelemetry(
            now=now,
            connected=bool(self._state.connected),
            mode=str(self._state.mode),
            armed=bool(self._state.armed),
            altitude=float(self._position[2]),
            state_age=now - self._state_received,
        )

    def _on_timer(self) -> None:
        now = time.monotonic()
        telemetry = self._telemetry(now)
        step = self._machine.step(telemetry)

        if step.send_setpoint and step.target is not None:
            self._publish_setpoint(step.target)
        if step.request_offboard:
            self._request_mode("offboard", "OFFBOARD", now)
        if step.request_arm:
            self._request_arm(now)
        if step.request_land:
            self._request_mode("land", "AUTO.LAND", now)

        status = (
            f"state={step.state}; mode={telemetry.mode}; "
            f"armed={telemetry.armed}; altitude={telemetry.altitude:.2f}; "
            f"reason={step.reason}"
        )
        self._status_publisher.publish(String(data=status))
        if step.state != self._last_status:
            self.get_logger().info(status)
            self._last_status = step.state

    def _publish_setpoint(self, target: Tuple[float, float, float]) -> None:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.local_frame
        message.pose.position.x = target[0]
        message.pose.position.y = target[1]
        message.pose.position.z = target[2]
        message.pose.orientation.w = 1.0
        self._setpoint_publisher.publish(message)

    def _request_mode(self, key: str, mode: str, now: float) -> None:
        if now - self._last_requests[key] < 1.0:
            return
        if not self._mode_client.service_is_ready():
            return
        self._last_requests[key] = now
        future = self._mode_client.call_async(
            SetMode.Request(custom_mode=mode)
        )
        future.add_done_callback(
            lambda result: self._log_service_result(
                f"set_mode({mode})", result
            )
        )

    def _request_arm(self, now: float) -> None:
        if now - self._last_requests["arm"] < 1.0:
            return
        if not self._arming_client.service_is_ready():
            return
        self._last_requests["arm"] = now
        future = self._arming_client.call_async(
            CommandBool.Request(value=True)
        )
        future.add_done_callback(
            lambda result: self._log_service_result("arming", result)
        )

    def _log_service_result(self, operation: str, future) -> None:
        try:
            response = future.result()
            success = getattr(response, "success", getattr(response, "mode_sent", False))
            if not success:
                self.get_logger().warning(
                    f"{operation} was rejected: {response}"
                )
        except Exception as exc:
            self.get_logger().error(f"{operation} service failed: {exc}")

    def _handle_start(self, request, response):
        del request
        if self._position is None or self._state is None:
            response.success = False
            response.message = "MAVROS state or local position is not available"
            return response
        if not self._machine.start(time.monotonic(), self._position):
            response.success = False
            response.message = f"Sequence already active in state {self._machine.state}"
            return response
        response.success = True
        response.message = (
            f"Started at {self._position}; target altitude "
            f"{self._machine.target[2]:.2f} m"
        )
        return response

    def _handle_abort(self, request, response):
        del request
        if self._machine.abort(time.monotonic(), "operator abort"):
            response.success = True
            response.message = "Abort accepted; requesting AUTO.LAND"
        else:
            response.success = False
            response.message = f"No active sequence in state {self._machine.state}"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4Mavros2Offboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
