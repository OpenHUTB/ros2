"""Run with ROS_DOMAIN_ID=76 after sourcing ROS2; isolated from the demo."""
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from ros_pattern import PatternNode
from pattern_data import KEYS, read_flight, windows


def main():
    root = Path(__file__).resolve().parents[1]
    rclpy.init(args=['--ros-args', '-p', 'model:='+str(root/'models/mlp_42')])
    recognizer = PatternNode()
    probe = Node('flight_pattern_integration_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(recognizer)
    executor.add_node(probe)
    statuses, predictions, events = [], [], []
    probe.create_subscription(String, '/uav/pattern/status', lambda m: statuses.append(json.loads(m.data)['state']), 100)
    probe.create_subscription(String, '/uav/pattern/prediction', lambda m: predictions.append(json.loads(m.data)), 100)
    probe.create_subscription(String, '/uav/pattern/task_events', lambda m: events.append(json.loads(m.data)), 100)
    publisher = probe.create_publisher(String, '/uav/pattern/telemetry', 100)

    def pump(seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.005)

    try:
        pump(1.)
        path = root/'data/final_raw/episode_300.csv'
        _, stamps, values = read_flight(path)
        for stamp, z in zip(stamps, values):
            message = dict(timestamp_ns=int(stamp), segment_id='integration', **dict(zip(KEYS, z.tolist())))
            publisher.publish(String(data=json.dumps(message)))
            pump(.012)
        pump(.2)
        expected = len(windows(path))
        assert len(predictions) == expected, (len(predictions), expected)
        assert events and all(e['label'] in ['line', 'circle', 'figure_eight', 'climb', 'stop_go', 'uncertain'] for e in events)
        publisher.publish(String(data='{broken json'))
        pump(.1)
        assert statuses[-1] == 'invalid_telemetry_history_reset'
        message['vx'] = float('nan')
        publisher.publish(String(data=json.dumps(message)))
        pump(.1)
        assert statuses[-1] == 'invalid_telemetry_history_reset'
        assert not recognizer.recognizer.samples
        message['vx'] = 0.
        publisher.publish(String(data=json.dumps(message)))
        pump(.1)
        assert statuses[-1] == 'collecting_7_seconds_of_history'
        pump(1.4)
        assert statuses[-1] == 'no_data_history_reset'
        assert not recognizer.recognizer.samples
        output = {'real_ros_messages': len(stamps), 'predictions': len(predictions), 'events': len(events),
                  'malformed_json_rejected': True, 'nan_rejected': True, 'wall_timeout_resets_history': True,
                  'transport': 'rclpy publishers/subscribers, isolated ROS_DOMAIN_ID=76'}
        (root/'results/ros_integration.json').write_text(json.dumps(output, indent=2))
        print(json.dumps(output))
    finally:
        executor.shutdown()
        recognizer.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
