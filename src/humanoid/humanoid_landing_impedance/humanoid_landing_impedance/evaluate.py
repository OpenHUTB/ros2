"""落地实验评估节点。

订阅 landing_sample 采样序列，到达评估截止时间后把样本写入 CSV，
供 landing_report 汇总计算指标与绘图。每个 launch 组合
（控制器模式 × 跌落高度）对应一个 CSV 文件。
"""

import csv
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

SAMPLE_KEYS = ('t', 'grf', 'base_vz', 'base_z', 'base_pitch', 'knee_flexion')


class LandingEvaluateNode(Node):
    """采样收集节点：截止后落盘 CSV 并退出。"""

    def __init__(self):
        super().__init__('landing_evaluate')
        self.declare_parameter('controller_mode', 'impedance')
        self.declare_parameter('drop_height', 0.05)
        self.declare_parameter('duration', 3.0)
        self.declare_parameter('deadline_margin', 8.0)
        self.declare_parameter('out_dir', 'results')

        self.mode = self.get_parameter('controller_mode').value
        self.drop_height = float(self.get_parameter('drop_height').value)
        self.duration = float(self.get_parameter('duration').value)
        self.out_dir = self.get_parameter('out_dir').value
        margin = float(self.get_parameter('deadline_margin').value)

        self.samples = []
        self.done = False
        self.subscription = self.create_subscription(
            Float64MultiArray, 'landing_sample', self.sample_callback, 10)
        self.deadline_timer = self.create_timer(
            self.duration + margin, self.finalize)
        self.get_logger().info(
            '评估节点已启动: 模式 {}，跌落 {:.2f} m，截止 {:.1f} s'.format(
                self.mode, self.drop_height, self.duration + margin))

    def sample_callback(self, message):
        if len(message.data) >= len(SAMPLE_KEYS):
            self.samples.append(
                {key: float(value)
                 for key, value in zip(SAMPLE_KEYS, message.data)})

    def finalize(self):
        """到达截止时间：写 CSV 并置退出标志（由主循环轮询退出）。"""
        if self.done:
            return
        self.deadline_timer.cancel()
        if not self.samples:
            self.get_logger().error('未收到任何采样，请检查仿真节点是否正常')
        else:
            os.makedirs(self.out_dir, exist_ok=True)
            path = os.path.join(
                self.out_dir,
                'samples_{}_{:.2f}.csv'.format(self.mode, self.drop_height))
            with open(path, 'w', newline='') as handle:
                writer = csv.DictWriter(handle, fieldnames=SAMPLE_KEYS)
                writer.writeheader()
                writer.writerows(self.samples)
            self.get_logger().info(
                '已保存 {} 条采样到 {}'.format(len(self.samples), path))
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = LandingEvaluateNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
