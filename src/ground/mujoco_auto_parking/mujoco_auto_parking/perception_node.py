"""ROS 2 node: front camera -> neural parking-slot perception."""

from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

from .parking_perception import ImagePerceptionNetwork
from .resources import resolve_package_file


class PerceptionNode(Node):
    """Publish the target slot pose estimated from the front camera."""

    def __init__(self) -> None:
        super().__init__('parking_perception_node')
        self.declare_parameter(
            'weights_file', 'models/perception.npz'
        )
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/parking/perception')
        package_share = Path(
            get_package_share_directory('mujoco_auto_parking')
        )
        weights = resolve_package_file(
            package_share, str(self.get_parameter('weights_file').value)
        )
        self._network = ImagePerceptionNetwork(weights)
        self._publisher = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter('output_topic').value),
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._on_image,
            10,
        )
        self.get_logger().info(
            f'车位视觉感知节点已启动，权值: {weights}'
        )

    def _on_image(self, message: Image) -> None:
        if message.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().warning(
                f'不支持的图像编码: {message.encoding}'
            )
            return
        try:
            buffer = np.frombuffer(message.data, dtype=np.uint8)
            row = int(message.step)
            channels = 3
            image = buffer.reshape(int(message.height), row)
            image = image[:, : int(message.width) * channels]
            image = image.reshape(
                int(message.height), int(message.width), channels
            )
            if message.encoding == 'bgr8':
                image = image[..., ::-1]
            dx, dy, dtheta = self._network.predict(image)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'感知推理失败: {exc}')
            return
        output = Float64MultiArray()
        output.data = [float(dx), float(dy), float(dtheta), 1.0]
        self._publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PerceptionNode()
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
