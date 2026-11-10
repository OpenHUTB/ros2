"""订阅关节指令，记录单程时延、丢包与乱序，结束时导出 CSV。

用法::

    ros2 run joint_qos_latency_eval joint_qos_sub \
      --ros-args -p qos_profile:=best_effort_depth1 -p csv_path:=results/be1.csv
"""

import csv
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .protocol import decode
from .qos_profiles import DEFAULT_PROFILE, make
from .stats import summarize

CSV_FIELDS = ('seq', 'stamp_ns', 'recv_ns', 'latency_ms', 'gap')


class JointCommandSubscriber(Node):

    def __init__(self):
        super().__init__('joint_qos_sub')
        self.declare_parameter('qos_profile', DEFAULT_PROFILE)
        self.declare_parameter('duration', 10.0)
        self.declare_parameter('topic', '/joint_commands')
        self.declare_parameter('csv_path', '')

        profile_name = str(self.get_parameter('qos_profile').value)
        self.duration = float(self.get_parameter('duration').value)
        topic = str(self.get_parameter('topic').value)
        self.csv_path = str(self.get_parameter('csv_path').value)

        self.rows = []
        self._last_seq = None
        self._finalized = False
        self._subscription = self.create_subscription(
            Float64MultiArray, topic, self._on_message, make(profile_name))

        self._deadline = (time.monotonic() + self.duration
                          if self.duration > 0.0 else float('inf'))

        self.get_logger().info(
            'subscribe %s, qos=%s, duration=%.1f s, csv=%r'
            % (topic, profile_name, self.duration, self.csv_path))

    def _on_message(self, msg):
        try:
            seq, stamp_ns, _positions = decode(msg.data)
        except ValueError as exc:
            self.get_logger().warning('bad payload: %s' % exc)
            return

        recv_ns = self.get_clock().now().nanoseconds
        # 首条消息的 gap 记为 seq+1，这样“开头漏掉的报文”也能算进 missing
        gap = seq + 1 if self._last_seq is None else seq - self._last_seq
        self.rows.append({
            'seq': seq,
            'stamp_ns': stamp_ns,
            'recv_ns': recv_ns,
            'latency_ms': (recv_ns - stamp_ns) / 1e6,
            'gap': gap,
        })
        self._last_seq = seq

    def write_csv(self, path=None):
        target = path or self.csv_path
        if not target:
            return None
        directory = os.path.dirname(target)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(target, 'w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
            writer.writeheader()
            for row in self.rows:
                writer.writerow({key: row[key] for key in CSV_FIELDS})
        return target

    def finalize(self):
        """落 CSV 并打印一行摘要；shutdown 之后日志可能不可用，故容错。"""
        if self._finalized:
            return None
        self._finalized = True
        written = self.write_csv()
        summary = summarize(self.rows)
        try:
            if summary['count'] == 0:
                # 零消息也要落 CSV（仅表头），否则该场景会在汇总表里整行消失，
                # 让读者无法区分“跑了但没收到”和“根本没跑”。
                self.get_logger().warning(
                    'received 0 messages, csv contains header only')
            self.get_logger().info(
                'received %d, missing %d, p99 %.3f ms, max %.3f ms, jitter %.3f ms'
                % (summary['count'], summary['missing'], summary['latency_p99_ms'],
                   summary['latency_max_ms'], summary['jitter_ms']))
            if written:
                self.get_logger().info('csv written to %s' % written)
        except Exception:  # pylint: disable=broad-except
            pass
        return written

    def deadline_reached(self):
        return time.monotonic() >= self._deadline


def main(args=None):
    rclpy.init(args=args)
    node = JointCommandSubscriber()
    try:
        while rclpy.ok() and not node.deadline_reached():
            rclpy.spin_once(node, timeout_sec=0.001)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
