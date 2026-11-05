"""ROS 2 节点：启动后跑完对比实验并自动退出。"""

import os
import threading

import rclpy
from rclpy.node import Node

from uav_bc_bench.bench import run_benchmark


class BenchNode(Node):
    """把 run_benchmark 放进工作线程，避免阻塞 executor。"""

    def __init__(self):
        super().__init__('uav_bc_bench')
        self.declare_parameter('dataset_dir', os.path.expanduser('~/.ros/uav_depth_track_nn/data'))
        self.declare_parameter('output_dir', os.path.expanduser('~/.ros/uav_bc_bench'))
        self.declare_parameter('epochs', 30)
        self.declare_parameter('seeds', 3)
        self.declare_parameter('batch_size', 32)
        self.declare_parameter('structures', 'mlp,cnn,cnn_wide')
        self.declare_parameter('augment', True)
        self.declare_parameter('image_size', 64)

        self._done = False
        self._error = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self.create_timer(1.0, self._on_timer)

    def _run(self):
        try:
            structures = [s for s in str(self.get_parameter('structures').value).split(',') if s]
            result = run_benchmark(
                str(self.get_parameter('dataset_dir').value),
                str(self.get_parameter('output_dir').value),
                epochs=int(self.get_parameter('epochs').value),
                seeds=int(self.get_parameter('seeds').value),
                structures=structures,
                augment=bool(self.get_parameter('augment').value),
                image_size=int(self.get_parameter('image_size').value),
                batch_size=int(self.get_parameter('batch_size').value),
                log=self.get_logger().info,
            )
            self.get_logger().info('对比实验完成：\n' + result['table'])
        except Exception as exc:  # noqa: BLE001 - 任何异常都要让节点正常退出
            self._error = exc
            self.get_logger().error(f'对比实验失败：{exc}')
        finally:
            self._done = True

    def _on_timer(self):
        if self._done:
            rclpy.shutdown()


def main(args=None):
    """节点入口。"""
    rclpy.init(args=args)
    node = BenchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
