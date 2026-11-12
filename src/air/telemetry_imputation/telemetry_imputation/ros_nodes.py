"""ROS2 replay with packet loss, and online reconstruction."""
import argparse
import json
import os
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from . import data
from .evaluate import load_model, predict

ROOT = Path(os.environ.get('IMPUTE_PROJECT_ROOT', Path(__file__).resolve().parents[1]))
DT = 0.05


class Replay(Node):
    """Publish a recorded flight, dropping a 1.5 s burst every 6 s."""

    def __init__(self):
        super().__init__('impute_replay')
        self.pub = self.create_publisher(String, '/impute/observation', 100)
        self.truth_pub = self.create_publisher(String, '/impute/ground_truth', 100)
        raw = np.loadtxt(ROOT / 'flights' / 'flight_70000.csv', delimiter=',', skiprows=1)
        self.t = raw[:, 0]
        self.state = raw[:, 1:7].astype(float)
        self.i = 0
        self.start = time.monotonic()
        self.finished = False
        self.timer = self.create_timer(DT, self.tick)

    def tick(self):
        if time.monotonic() - self.start < 2.0:
            return
        if self.i >= len(self.t):
            self.finished = True
            return
        stamp = float(self.t[self.i])
        dropped = (int(stamp / 6.0) % 2 == 1) and (stamp % 6.0 < 0.75)
        truth = String()
        truth.data = json.dumps({'stamp_s': stamp,
                                 'truth': [float(x) for x in self.state[self.i]]})
        self.truth_pub.publish(truth)
        if not dropped:
            msg = String()
            msg.data = json.dumps({'stamp_s': stamp, 'delivered': 1,
                                   'state': [float(x) for x in self.state[self.i]]})
            self.pub.publish(msg)
        self.i += 1


class ImputerNode(Node):
    """Reconstruct the newest slot at 20 Hz, whether or not a packet arrived."""

    def __init__(self):
        super().__init__('impute_estimator')
        self.pub = self.create_publisher(String, '/impute/state', 100)
        self.create_subscription(String, '/impute/observation', self.receive, 100)
        self.backend, self.payload, self.cfg = load_model(str(ROOT / 'models/imputer_42'))
        self.slots = deque(maxlen=data.WINDOW)
        self.pending = None
        self.count = 0
        self.reports = []
        self.last_stamp = None
        self.since = 0
        self.warmup = 20
        self.finished = False
        self.last_rx = time.monotonic()
        self.create_timer(DT, self.tick)
        self.create_timer(1.0, self.watch)

    def receive(self, message):
        payload = json.loads(message.data)
        self.last_stamp = float(payload['stamp_s'])
        self.since = 0
        self.pending = np.asarray(payload['state'], float)
        self.last_rx = time.monotonic()
        self.count += 1

    def tick(self):
        if self.pending is not None:
            self.slots.append(self.pending)
            self.pending = None
            self.since = 0
        else:
            self.slots.append(None)
            self.since += 1
        if self.last_stamp is None:
            return
        if sum(1 for s in self.slots if s is not None) < self.warmup:
            return
        self.emit()

    def emit(self):
        filled = [s if s is not None else np.zeros(6) for s in self.slots]
        mask = np.array([s is not None for s in self.slots])
        arr = np.asarray(filled, np.float32)[None]
        n = arr.shape[1]
        thrust = np.full((1, n), 11.77, np.float32)
        dts = np.full((1, n), DT, np.float32)
        x = np.concatenate([arr, thrust[:, :, None], mask[None, :, None].astype(np.float32),
                            dts[:, :, None]], axis=2)
        pred = predict(self.backend, self.payload, self.cfg, x, arr, mask[None])[0]
        stamp = self.last_stamp + self.since * DT
        out = {'stamp_s': float(stamp), 'reconstructed': [float(v) for v in pred[-1]],
               'observed': bool(mask[-1])}
        msg = String(); msg.data = json.dumps(out); self.pub.publish(msg)
        self.reports.append(out)

    def watch(self):
        if self.count and time.monotonic() - self.last_rx > 3:
            self.finished = True


def run(cls):
    rclpy.init()
    node = cls()
    try:
        while rclpy.ok() and not getattr(node, 'finished', False):
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def replay():
    run(Replay)


def imputer():
    run(ImputerNode)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('role', choices=['replay', 'imputer'])
    a = parser.parse_args()
    replay() if a.role == 'replay' else imputer()
