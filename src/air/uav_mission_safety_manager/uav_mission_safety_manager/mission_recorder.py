"""Independent observer. Disk I/O never runs in the flight controller process."""
import json
import signal
from threading import Event

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from .mission_recording import StatusLog


class MissionRecorder(Node):
    def __init__(self):
        super().__init__('uav_mission_recorder')
        directory = self.declare_parameter('recording_directory', '~/.ros/uav_mission_logs').value
        topic = self.declare_parameter('status_topic', '/uav_mission_safety_manager/status').value
        self.log = StatusLog(directory, topic)
        self.failed = False
        self.subscription = self.create_subscription(String, topic, self.record, 100)
        self.get_logger().info('Recording status to ' + str(self.log.path))

    def record(self, message):
        if self.failed:
            return
        try:
            self.log.append(json.loads(message.data))
        except (ValueError, TypeError) as exc:
            self.get_logger().warning('Skipped invalid status: ' + str(exc))
        except OSError as exc:
            # Stop recording after a write failure; do not label the file complete.
            self.failed = True
            self.get_logger().error('Recording stopped after disk error: ' + str(exc))

    def finish(self):
        if self.failed:
            self.log.stream.close()
        else:
            self.log.close()


def main(args=None):
    # Defer termination until a sample (including its sequence increment) finishes.
    # Raising KeyboardInterrupt between flush and increment corrupts the footer count.
    stopping = Event()
    old_handlers = {sig: signal.signal(sig, lambda *_: stopping.set())
                    for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = MissionRecorder()
        while rclpy.ok() and not stopping.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        if node is not None:
            try:
                node.finish()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
