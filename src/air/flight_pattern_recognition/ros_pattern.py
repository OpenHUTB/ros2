"""ROS2 transports observations only; collection labels never reach the recognizer."""
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from pattern_data import KEYS, read_flight
from pattern_stream import Recognizer


class Replay(Node):
    def __init__(self):
        super().__init__('flight_pattern_replay')
        self.declare_parameter('files', '')
        self.declare_parameter('speed', 1.)
        self.declare_parameter('startup_delay', 3.)
        files = self.get_parameter('files').value.split(';')
        speed = float(self.get_parameter('speed').value)
        if speed <= 0:
            raise ValueError('Replay speed must be positive')
        self.scheduled = []
        offset = 0.
        for index, name in enumerate(files):
            _, stamps, values = read_flight(name)
            for stamp, z in zip(stamps, values):
                payload = dict(timestamp_ns=int(stamp), segment_id='segment_%03d' % index,
                               **{key: float(value) for key, value in zip(KEYS, z)})
                self.scheduled.append((offset + float(stamp - stamps[0]) / 1e9 / speed, payload))
            offset += float(stamps[-1] - stamps[0]) / 1e9 / speed + .5
        self.pub = self.create_publisher(String, '/uav/pattern/telemetry', 100)
        self.status = self.create_publisher(String, '/uav/pattern/replay_status', 10)
        self.start = time.monotonic() + self.get_parameter('startup_delay').value
        self.i = 0
        self.done = False
        self.finish_time = None
        self.create_timer(.005, self.tick)

    def tick(self):
        now = time.monotonic()
        if self.finish_time is not None:
            if now >= self.finish_time:
                self.done = True
            return
        # Resume after a VM pause without flooding the DDS queue with old samples.
        # Move the replay wall-clock origin; recorded simulation timestamps stay exact.
        if self.i < len(self.scheduled):
            lag = now - self.start - self.scheduled[self.i][0]
            if lag > .15:
                self.start += lag
        while self.i < len(self.scheduled) and self.scheduled[self.i][0] <= now - self.start:
            self.pub.publish(String(data=json.dumps(self.scheduled[self.i][1], allow_nan=False)))
            self.i += 1
        if self.i == len(self.scheduled):
            self.status.publish(String(data=json.dumps({'complete': True, 'samples': self.i})))
            self.finish_time = now + 1.


class PatternNode(Node):
    def __init__(self):
        super().__init__('flight_pattern_recognizer')
        self.declare_parameter('model', '')
        self.declare_parameter('log', '')
        self.recognizer = Recognizer(self.get_parameter('model').value)
        self.predictions = self.create_publisher(String, '/uav/pattern/prediction', 100)
        self.events = self.create_publisher(String, '/uav/pattern/task_events', 100)
        self.status = self.create_publisher(String, '/uav/pattern/status', 10)
        self.create_subscription(String, '/uav/pattern/telemetry', self.receive, 100)
        self.create_subscription(String, '/uav/pattern/replay_status', self.end, 10)
        self.create_timer(.25, self.check_timeout)
        self.last_receive = None
        self.timed_out = False
        self.finished = False
        path = self.get_parameter('log').value
        self.log = None
        if path:
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            self.log = open(path, 'w', buffering=1)

    def send_status(self, state):
        self.status.publish(String(data=json.dumps({'state': state})))

    def receive(self, message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError('Expected object')
            result = self.recognizer.update(payload)
        except (TypeError, ValueError, KeyError):
            self.recognizer.reset()
            self.send_status('invalid_telemetry_history_reset')
            return
        self.last_receive = time.monotonic()
        self.timed_out = False
        self.finished = False
        if result is None:
            self.send_status('collecting_7_seconds_of_history')
            return
        self.predictions.publish(String(data=json.dumps(result, allow_nan=False)))
        self.send_status('recognizing')
        if self.log:
            self.log.write(json.dumps(result, allow_nan=False) + '\n')
        if result['event']:
            self.events.publish(String(data=json.dumps(result['event'], allow_nan=False)))
            self.get_logger().info('Task event: ' + result['event']['label'])

    def end(self, message):
        try:
            complete = json.loads(message.data).get('complete', False)
        except (TypeError, ValueError, AttributeError):
            return
        if complete:
            self.finished = True
            self.send_status('replay_complete')

    def check_timeout(self):
        if self.finished or self.timed_out:
            return
        if self.last_receive is not None and time.monotonic() - self.last_receive > 1.:
            self.recognizer.reset()
            self.timed_out = True
            self.send_status('no_data_history_reset')

    def destroy_node(self):
        if self.log:
            self.log.close()
        super().destroy_node()


def run(node_type):
    rclpy.init()
    node = None
    try:
        node = node_type()
        while rclpy.ok() and not getattr(node, 'done', False):
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def replay_main():
    run(Replay)


def recognize_main():
    run(PatternNode)
