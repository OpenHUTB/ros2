"""Mock camera node.

Publishes a synthetic RGB frame on /camera/image_raw so that the
perception pipeline can be tested without a real camera or simulator.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class MockCamera(Node):

    def __init__(self):
        super().__init__('mock_camera')

        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('frame_id', 'camera_link')

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value

        if self.width <= 0 or self.height <= 0:
            self.get_logger().fatal(
                'width and height must be positive, got %d x %d'
                % (self.width, self.height))
            raise ValueError('invalid image size')

        if self.fps <= 0:
            self.get_logger().fatal('fps must be positive, got %r' % (self.fps,))
            raise ValueError('invalid fps')
            
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)

        # background: a vertical colour gradient, built once
        self.base = bytearray(self.width * self.height * 3)
        for y in range(self.height):
            level = int(40 + 60 * y / max(1, self.height - 1))
            row = bytes((level, level // 2, 255 - level)) * self.width
            start = y * self.width * 3
            self.base[start:start + len(row)] = row

        self.band = 0
        self.timer = self.create_timer(1.0 / self.fps, self.publish_frame)
        self.get_logger().info(
            'Mock camera started: %d x %d @ %.1f Hz, publishing /camera/image_raw'
            % (self.width, self.height, self.fps))

    def publish_frame(self):
        frame = bytearray(self.base)

        # a bright band sweeping down the image, so the topic shows motion
        bright = bytes((255, 200, 40)) * self.width
        top = self.band
        bottom = min(self.band + 12, self.height)
        for y in range(top, bottom):
            start = y * self.width * 3
            frame[start:start + len(bright)] = bright
        self.band = (self.band + 4) % self.height

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = self.width * 3
        msg.data = bytes(frame)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
