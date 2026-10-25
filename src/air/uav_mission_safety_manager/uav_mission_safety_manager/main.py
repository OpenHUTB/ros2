import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .airsim_adapter import AirSimAdapter
from .controller import FlightController
from .demo_adapter import DemoAdapter
from .state_machine import SafetyMachine, State


class MissionNode(Node):
    def __init__(self):
        super().__init__('uav_mission_safety_manager')
        max_altitude = self.declare_parameter('max_altitude', 50.0).value
        timeout = self.declare_parameter('telemetry_timeout', 2.0).value
        operation_timeout = self.declare_parameter('operation_timeout', 120.0).value
        use_airsim = self.declare_parameter('use_airsim', False).value
        use_demo = self.declare_parameter('use_demo_adapter', False).value
        if use_airsim and use_demo:
            raise ValueError('use_airsim and use_demo_adapter are mutually exclusive')
        self.adapter_mode = 'airsim' if use_airsim else 'demo' if use_demo else 'state_only'
        host = self.declare_parameter('airsim_host', '127.0.0.1').value
        port = self.declare_parameter('airsim_port', 41451).value
        vehicle = self.declare_parameter('vehicle_name', '').value
        self.adapter = (AirSimAdapter(host, port, vehicle) if use_airsim else
                        DemoAdapter() if use_demo else None)
        if use_demo:
            self.get_logger().warning('DEMO adapter: local kinematics only; no AirSim or real vehicle')
        self.controller = FlightController(
            self.adapter, max_altitude, timeout,
            operation_timeout=operation_timeout) if self.adapter else None
        self.machine = self.controller.machine if self.controller else SafetyMachine(max_altitude, timeout)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.create_subscription(String, '~/command', self.command_cb, 10)
        # Synthetic external telemetry is deliberately ignored in AirSim mode.
        if not self.controller:
            self.create_subscription(String, '/uav/telemetry', self.telemetry_cb, 10)
        self.control_services = []
        for name, event in [('pause', 'pause'), ('resume', 'resume'),
                            ('return_home', 'return'), ('land', 'land'), ('cancel', 'cancel')]:
            self.control_services.append(self.create_service(
                Trigger, '~/' + name, lambda req, res, e=event: self.service_cb(e, res)))
        self.last_telemetry = time.monotonic()
        self.create_timer(0.2, self.timer_cb)

    def publish(self):
        status = {'state': self.machine.state.value, 'reason': self.machine.reason,
                  'adapter_mode': self.adapter_mode}
        if self.controller:
            status.update(position_ned=self.controller.position,
                          target_ned=self.controller.target,
                          target_speed=self.controller.target_speed,
                          takeoff_rpc_result=self.controller.takeoff_rpc_result,
                          command_pending=self.controller.pending is not None,
                          mission_queue=self.controller.mission_queue.status(),
                          last_error=self.controller.last_error)
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def request(self, event, **data):
        if not self.controller and event in ('mission', 'cancel'):
            raise ValueError('Waypoint missions require a flight controller')
        if self.controller:
            self.controller.request(event, **data)
        else:
            self.machine.transition(event, **data)

    def service_cb(self, event, response):
        try:
            self.request(event)
            response.success = True
            response.message = 'Accepted: ' + self.machine.state.value
        except (ValueError, RuntimeError, KeyError, TypeError) as exc:
            response.success = False
            response.message = str(exc)
        self.publish()
        return response

    def command_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or not isinstance(data.get('event'), str):
                raise ValueError('Expected JSON object with an event')
            event = data.pop('event')
            self.request(event, **data)
        except (ValueError, RuntimeError, KeyError, TypeError) as exc:
            self.get_logger().error(str(exc))
        self.publish()

    def telemetry_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.machine.check_safety(float(data['altitude']), 0, bool(data['inside_fence']))
            self.last_telemetry = time.monotonic()
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warning(str(exc))

    def timer_cb(self):
        if self.controller:
            self.controller.tick()
        elif self.machine.state in FlightController.ACTIVE:
            self.machine.check_safety(0, time.monotonic() - self.last_telemetry)
        self.publish()


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.adapter:
            node.adapter.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
