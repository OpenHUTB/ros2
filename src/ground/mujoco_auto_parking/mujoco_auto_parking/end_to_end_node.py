"""ROS 2 node: front camera image -> end-to-end velocity/yaw control."""

from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image

from .end_to_end_networks import VisuomotorController
from .resources import resolve_package_file


class EndToEndNode(Node):
    """Drive the car from images alone using the trained CNN."""

    def __init__(self) -> None:
        super().__init__('end_to_end_node')
        self.declare_parameter(
            'weights_file', 'models/e2e_small_cnn_bc.npz'
        )
        self.declare_parameter('architecture', 'small_cnn')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('nominal_speed', 0.45)
        package_share = Path(
            get_package_share_directory('mujoco_auto_parking')
        )
        weights = resolve_package_file(
            package_share, str(self.get_parameter('weights_file').value)
        )
        self._controller = VisuomotorController(
            architecture=str(self.get_parameter('architecture').value),
            weights_path=weights,
            nominal_speed=float(self.get_parameter('nominal_speed').value),
        )
        self._publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._on_image,
            10,
        )
        self._count = 0
        self.get_logger().info(
            f'端到端视觉控制节点已启动，权值: {weights}'
        )

    def _on_image(self, message: Image) -> None:
        if message.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().warning(
                f'不支持的图像编码: {message.encoding}'
            )
            return
        buffer = np.frombuffer(message.data, dtype=np.uint8)
        row = int(message.step)
        image = buffer.reshape(int(message.height), row)
        image = image[:, : int(message.width) * 3]
        image = image.reshape(int(message.height), int(message.width), 3)
        if message.encoding == 'bgr8':
            image = image[..., ::-1]
        result = self._controller.predict_action(image)
        command = Twist()
        command.linear.x = float(result['v'])
        command.angular.z = float(result['wz'])
        self._publisher.publish(command)
        self._count += 1
        if self._count % 50 == 0:
            self.get_logger().info(
                f'端到端：v={result["v"]:.2f} m/s, '
                f'wz={result["wz"]:.2f} rad/s, '
                f'推理 {result["latency_ms"]:.1f} ms'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = EndToEndNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
