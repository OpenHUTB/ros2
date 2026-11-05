"""Read-only AirSim telemetry bridge. Does not arm or command the vehicle."""
import json
import airsim
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from telemetry_data import CHANNELS
from collect_holdout import pack_state


class LiveSource(Node):
    def __init__(self):
        super().__init__('telemetry_airsim_source')
        self.declare_parameter('host', '192.168.239.1')
        self.declare_parameter('vehicle', 'PredictionDrone')
        self.client = airsim.MultirotorClient(ip=self.get_parameter('host').value, timeout_value=1)
        self.vehicle = self.get_parameter('vehicle').value
        self.publisher = self.create_publisher(String, '/uav/telemetry', 100)
        self.failed = False
        self.create_timer(.05, self.tick)

    def tick(self):
        try:
            packed = pack_state(self.client.getMultirotorState(vehicle_name=self.vehicle))
            row = {'arrival_ns': int(packed[0]), 'source_ns': int(packed[0]), 'delivered': 1,
                   **dict(zip(CHANNELS, packed[1:]))}
            self.publisher.publish(String(data=json.dumps(row)))
            self.failed = False
        except Exception as exc:
            if not self.failed:
                self.get_logger().error('AirSim read failed; downstream watchdog will report no data: ' + str(exc))
            self.failed = True


def main():
    from ros_diagnostics import spin
    spin(LiveSource)


if __name__ == '__main__': main()
