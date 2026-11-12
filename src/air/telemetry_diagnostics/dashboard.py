"""Interactive window displaying actual ROS2 diagnostic messages."""
import json
from collections import deque
from pathlib import Path
import tkinter as tk
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Dashboard(Node):
    def __init__(self):
        super().__init__('telemetry_dashboard')
        self.rows = deque(maxlen=800)
        self.latest = None
        self.finished = False
        self.create_subscription(String, '/uav/anomaly_score', self.receive, 100)
        self.create_subscription(String, '/uav/replay_status', self.on_status, 10)
        self.root = tk.Tk(); self.root.title('OpenHUTB UAV Telemetry Diagnostics')
        self.root.geometry('1000x650')
        self.label = tk.Label(self.root, text='Waiting for ROS2 telemetry...', font=('Arial', 15))
        self.label.pack(fill='x')
        self.figure = Figure(figsize=(10, 6)); self.axes = self.figure.subplots(2, 1)
        self.canvas = FigureCanvasTkAgg(self.figure, self.root)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.after(100, self.tick)

    def receive(self, message):
        self.latest = json.loads(message.data)
        if self.latest['arrival_ns'] > 0:
            self.rows.append(self.latest)

    def on_status(self, message):
        self.finished = bool(json.loads(message.data).get('complete'))

    def tick(self):
        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0)
        if self.rows:
            rows = list(self.rows); latest = rows[-1]
            t = [(r['arrival_ns'] - rows[0]['arrival_ns']) / 1e9 for r in rows]
            for ax in self.axes: ax.clear(); ax.grid(alpha=.2)
            self.axes[0].plot(t, [r['score'] for r in rows], label='Neural anomaly score')
            self.axes[0].axhline(latest['threshold'], color='orange', ls='--', label='Validation threshold')
            self.axes[0].legend(); self.axes[0].set_ylabel('Score')
            self.axes[1].step(t, [int(r['alarm']) for r in rows], where='post', color='red')
            self.axes[1].set(xlabel='Replay seconds', ylabel='Alarm', ylim=(-.1, 1.2))
            self.figure.tight_layout(); self.canvas.draw_idle()
        if self.latest:
            latest = self.latest
            title = 'REPLAY COMPLETE' if self.finished else ('ALARM' if latest['alarm'] else 'MONITORING')
            self.label.config(text=title + ' | ' + latest['reason'], fg='red' if latest['alarm'] else 'darkgreen')
        self.root.after(200, self.tick)

    def close(self):
        Path('results').mkdir(exist_ok=True)
        self.figure.savefig('results/dashboard_export.png', dpi=150)
        self.root.destroy()


def main():
    rclpy.init(); node = Dashboard()
    try: node.root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
