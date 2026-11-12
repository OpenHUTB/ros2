"""Live window: observed vs reconstructed position, with ground truth for display.

The truth topic is published by the replay node only; the estimator never sees it.
"""
import json
from collections import deque

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Dashboard(Node):
    def __init__(self):
        super().__init__('impute_dashboard')
        self.rows = deque(maxlen=600)
        self.truth = {}
        self.create_subscription(String, '/impute/state', self.receive, 100)
        self.create_subscription(String, '/impute/ground_truth', self.receive_truth, 100)
        self.root = tk.Tk()
        self.root.title('OpenHUTB UAV Telemetry Imputation')
        self.root.geometry('1000x640')
        self.label = tk.Label(self.root, text='Waiting for ROS2 replay...', font=('Arial', 14))
        self.label.pack(fill='x')
        self.figure = Figure(figsize=(10, 5.6))
        self.axes = self.figure.subplots(2, 1)
        self.canvas = FigureCanvasTkAgg(self.figure, self.root)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.after(150, self.tick)

    def receive(self, message):
        try:
            self.rows.append(json.loads(message.data))
        except Exception:
            pass

    def receive_truth(self, message):
        try:
            payload = json.loads(message.data)
            self.truth[round(float(payload['stamp_s']), 3)] = payload['truth']
        except Exception:
            pass

    def tick(self):
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0)
        if len(self.rows) > 2:
            rows = list(self.rows)
            t = [r['stamp_s'] for r in rows]
            truth = [self.truth.get(round(r['stamp_s'], 3)) for r in rows]
            for ax in self.axes:
                ax.clear(); ax.grid(alpha=.25)
            known = [(ti, tr) for ti, tr in zip(t, truth) if tr is not None]
            if known:
                kt, kv = zip(*known)
                self.axes[0].plot(kt, [v[2] for v in kv], 'k-', lw=1.1, alpha=.55, label='truth z')
                self.axes[0].plot(kt, [v[0] for v in kv], 'k--', lw=0.9, alpha=.35, label='truth x')
            self.axes[0].plot(t, [r['reconstructed'][2] for r in rows], color='#4a90d9', lw=1.4,
                              label='reconstructed z')
            self.axes[0].plot(t, [r['reconstructed'][0] for r in rows], color='#3a9e6f', lw=1.2,
                              label='reconstructed x')
            self.axes[0].set_ylabel('Position (m)')
            self.axes[0].legend(fontsize=7, ncol=2, loc='upper left')
            if known:
                errs = []
                for r in rows:
                    tr = self.truth.get(round(r['stamp_s'], 3))
                    if tr is not None:
                        errs.append((r['stamp_s'], float(np.linalg.norm(
                            np.subtract(r['reconstructed'][:3], tr[:3])))))
                if errs:
                    et, ev = zip(*errs)
                    self.axes[1].plot(et, ev, color='#c0392b', lw=1.2)
                    mean_err = float(np.mean(ev))
                else:
                    mean_err = float('nan')
            else:
                mean_err = float('nan')
            self.axes[1].set(xlabel='Replay seconds', ylabel='3-D error (m)')
            self.figure.tight_layout(); self.canvas.draw_idle()
            hidden = sum(1 for r in rows if not r['observed'])
            self.label.config(text='MONITORING | frames %d | reconstructed frames %d | mean error %.3f m'
                                   % (len(rows), hidden, mean_err))
        self.root.after(150, self.tick)

    def close(self):
        from pathlib import Path
        out = Path('results'); out.mkdir(exist_ok=True)
        self.figure.savefig(out / 'dashboard_export.png', dpi=150)
        self.root.destroy()


def main():
    rclpy.init(); node = Dashboard()
    try:
        node.root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
