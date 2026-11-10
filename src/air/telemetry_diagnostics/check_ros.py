"""Observe actual ROS topics and save a standalone verification record and plot."""
import json
import time
import argparse
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['replay', 'live'], default='replay')
    args = parser.parse_args()
    rclpy.init(); node = Node('telemetry_flow_check')
    messages = []; diagnostics = []; states = []
    node.create_subscription(String, '/uav/anomaly_score', lambda m: messages.append(json.loads(m.data)), 100)
    node.create_subscription(String, '/uav/telemetry', lambda m: states.append(json.loads(m.data)), 100)
    node.create_subscription(DiagnosticArray, '/diagnostics', lambda m: diagnostics.append(m), 100)
    end = time.monotonic() + 25
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
    node.destroy_node(); rclpy.shutdown()
    if not messages or not states or not diagnostics:
        raise RuntimeError('Missing actual ROS2 messages')
    report = {'telemetry_messages': len(states), 'score_messages': len(messages),
              'diagnostic_arrays': len(diagnostics),
              'alarm_messages': sum(m['alarm'] for m in messages),
              'reasons': sorted(set(m['reason'] for m in messages)),
              'source': ('Actual ROS2 subscribers, recorded AirSim data with software injection' if args.mode == 'replay'
                         else 'Actual ROS2 subscribers, live read-only AirSim bridge; vehicle landed')}
    out = Path('results'); out.mkdir(exist_ok=True)
    (out / 'ros_flow_check.json').write_text(json.dumps(report, indent=2))
    times = [(m['arrival_ns'] - messages[0]['arrival_ns']) / 1e9 for m in messages]
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(times, [m['score'] for m in messages], label='/uav/anomaly_score')
    ax.axhline(messages[0]['threshold'], ls='--', color='orange', label='Threshold')
    ax.scatter([t for t, m in zip(times, messages) if m['alarm']],
               [0 for m in messages if m['alarm']], color='red', marker='|', label='Diagnostic alarm')
    ax.set(xlabel='Replay simulation seconds', ylabel='Score', title='Actual ROS2 topic observation')
    ax.legend(); ax.grid(alpha=.2); fig.tight_layout(); fig.savefig(out / 'ros_topic_trace.png', dpi=150)
    print(json.dumps(report))


if __name__ == '__main__': main()
