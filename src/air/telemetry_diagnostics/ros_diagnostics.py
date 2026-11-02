"""Replay publisher and diagnostic subscriber. Labels never enter the detector."""
import csv
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from stream_detector import load_detector


class Replay(Node):
    def __init__(self):
        super().__init__('telemetry_replay')
        self.declare_parameter('csv', '')
        self.declare_parameter('startup_delay', 3.)
        with Path(self.get_parameter('csv').value).open() as f:
            self.rows = list(csv.DictReader(f))
        self.pub = self.create_publisher(String, '/uav/telemetry', 100)
        self.events = self.create_publisher(String, '/uav/replay_status', 10)
        self.start = time.monotonic() + self.get_parameter('startup_delay').value
        self.base = int(self.rows[0]['arrival_ns'])
        self.i = 0
        self.timer = self.create_timer(.01, self.tick)

    def tick(self):
        elapsed = time.monotonic() - self.start
        while self.i < len(self.rows):
            row = self.rows[self.i]
            if (int(row['arrival_ns']) - self.base) / 1e9 > elapsed:
                return
            if int(row['delivered']):
                self.pub.publish(String(data=json.dumps(row)))
            self.i += 1
        self.events.publish(String(data=json.dumps({'complete': True, 'scheduled': self.i})))
        self.timer.cancel()


class Diagnostics(Node):
    def __init__(self):
        super().__init__('telemetry_diagnostics')
        self.declare_parameter('model', '')
        self.declare_parameter('log', '')
        self.declare_parameter('timeout_s', .15)
        self.detector = load_detector(self.get_parameter('model').value)
        self.pub = self.create_publisher(String, '/uav/anomaly_score', 100)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 100)
        self.create_subscription(String, '/uav/telemetry', self.receive, 100)
        self.create_subscription(String, '/uav/replay_status', self.end, 10)
        self.last_receive = None
        self.last_arrival = None
        self.started_at = time.monotonic()
        self.finished = False
        path = self.get_parameter('log').value
        if path:
            Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.log = open(path, 'w') if path else None
        self.create_timer(.05, self.timeout)

    def receive(self, msg):
        try:
            row = json.loads(msg.data)
        except (ValueError, TypeError):
            self.emit(self.detector.step({}))
            return
        result = self.detector.step(row)
        if result['reason'] == 'invalid_payload':
            self.emit(result)
            return
        self.last_receive = time.monotonic()
        self.last_arrival = int(row['arrival_ns'])
        self.emit(result)

    def timeout(self):
        if self.finished:
            return
        if self.last_receive is None:
            if time.monotonic() - self.started_at > 5:
                self.emit({'arrival_ns': 0, 'score': None, 'threshold': self.detector.config['threshold'],
                           'alarm': True, 'reason': 'no_data_received', 'data_valid': False,
                           'model_ready': False})
            return
        elapsed = time.monotonic() - self.last_receive
        if elapsed > self.get_parameter('timeout_s').value:
            # No ground-truth label or special injected packet is required for dropout detection.
            result = self.detector.step({'arrival_ns': self.last_arrival + int(elapsed * 1e9),
                                         'delivered': 0})
            self.emit(result)

    def emit(self, result):
        self.pub.publish(String(data=json.dumps(result)))
        status = DiagnosticStatus()
        status.name = 'uav/telemetry_quality'
        status.hardware_id = 'airsim_replay'
        status.level = DiagnosticStatus.WARN if result['alarm'] or not result['model_ready'] else DiagnosticStatus.OK
        if result['reason'] in ('invalid_payload', 'no_data_received'):
            status.level = DiagnosticStatus.ERROR
        status.message = result['reason']
        status.values = [KeyValue(key=k, value=str(v)) for k, v in result.items()]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostics.publish(message)
        if self.log:
            self.log.write(json.dumps(result) + '\n'); self.log.flush()

    def end(self, msg):
        try:
            self.finished = bool(json.loads(msg.data).get('complete'))
        except (ValueError, AttributeError):
            self.get_logger().warning('Ignoring malformed replay status')

    def destroy_node(self):
        if self.log:
            self.log.close()
        super().destroy_node()


def spin(cls):
    rclpy.init(); node = cls()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


def replay_main(): spin(Replay)
def diagnostics_main(): spin(Diagnostics)
