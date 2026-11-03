"""A real ROS2 subscriber window; no hard-coded recognition results."""
import json
from collections import deque
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pattern_data import CLASSES


class Dashboard(Node):
    def __init__(self):
        super().__init__('flight_pattern_dashboard')
        self.latest = None
        self.state = 'Waiting for ROS2 telemetry'
        self.predictions = []
        self.events = deque(maxlen=7)
        self.xy = deque(maxlen=400)
        self.segment = None
        self.samples = 0
        self.create_subscription(String, '/uav/pattern/telemetry', self.telemetry, 100)
        self.create_subscription(String, '/uav/pattern/prediction', self.prediction, 100)
        self.create_subscription(String, '/uav/pattern/task_events', self.event, 100)
        self.create_subscription(String, '/uav/pattern/status', self.status, 100)

    def telemetry(self, msg):
        payload = json.loads(msg.data)
        if payload['segment_id'] != self.segment:
            self.segment = payload['segment_id']
            self.xy.clear()
        self.xy.append((payload['px'], payload['py']))
        self.samples += 1

    def prediction(self, msg):
        self.latest = json.loads(msg.data)
        self.predictions.append(self.latest)

    def event(self, msg):
        self.events.append(json.loads(msg.data))

    def status(self, msg):
        self.state = json.loads(msg.data)['state']


def main():
    rclpy.init()
    node = Dashboard()
    plt.style.use('ggplot')
    figure, axes = plt.subplots(2, 2, figsize=(12, 7.4), constrained_layout=True)
    figure.canvas.manager.set_window_title('UAV Flight Pattern Recognition - ROS2')

    def update(_):
        # Drain the queue without blocking the GUI.
        for __ in range(50):
            rclpy.spin_once(node, timeout_sec=0)
        for ax in axes.flat:
            ax.clear()
        figure.suptitle('UAV FLIGHT PATTERN RECOGNITION | ROS2\n' + node.state, fontsize=15, weight='bold')
        ax = axes[0, 0]
        if node.xy:
            x, y = zip(*node.xy)
            ax.plot(x, y, color='#16758c', linewidth=2)
            ax.scatter(x[-1], y[-1], color='#e58b31')
        ax.set(title='Received AirSim trajectory (NED)', xlabel='North / m', ylabel='East / m')
        ax.axis('equal')
        ax = axes[0, 1]
        if node.latest:
            ax.barh(CLASSES, node.latest['probabilities'], color='#16758c')
            ax.set_title('Current: %s | confidence %.3f' % (node.latest['label'], node.latest['confidence']))
        else:
            ax.set_title('Collecting causal history')
        ax.set_xlim(0, 1)
        ax.set_xlabel('Softmax score (not calibrated probability)')
        ax = axes[1, 0]
        labels = CLASSES + ['uncertain']
        if node.predictions:
            ax.step(range(len(node.predictions)), [labels.index(r['label']) for r in node.predictions], where='post', color='#16758c')
        ax.set_yticks(range(len(labels))); ax.set_yticklabels(labels)
        ax.set(title='Predicted task timeline', xlabel='Published prediction index')
        ax.set_ylim(-.5, 5.5)
        ax = axes[1, 1]
        ax.axis('off')
        lines = ['AUTOMATIC TASK LOG', 'ROS telemetry samples: %d' % node.samples,
                 'Predictions: %d | events shown: %d' % (len(node.predictions), len(node.events)), '']
        for event in node.events:
            lines.append('%s  ->  %s (%.2f)' % (event['segment_id'], event['label'], event['confidence']))
        lines += ['', 'Labels are inferred from telemetry.', 'Collection labels / commands are not model inputs.']
        ax.text(0, .98, '\n'.join(lines), va='top', fontsize=10, family='monospace')

    animation = FuncAnimation(figure, update, interval=200, cache_frame_data=False)
    try:
        plt.show()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
