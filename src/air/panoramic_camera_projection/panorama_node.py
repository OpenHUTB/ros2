"""Optional ROS 2 node that stitches six synchronized image topics."""

from __future__ import annotations

from collections import defaultdict
from typing import Dict, Optional

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from .stitcher import FACE_NAMES, stitch_cubemap
except ImportError:
    from stitcher import FACE_NAMES, stitch_cubemap


class PanoramaNode(Node):
    def __init__(self) -> None:
        super().__init__("panorama_stitcher")
        self.declare_parameter("width", 1024)
        self.declare_parameter("height", 512)
        self.bridge = CvBridge()
        self.frames: Dict[int, Dict[str, Image]] = defaultdict(dict)
        self.publisher = self.create_publisher(Image, "panorama/image", 10)
        for face in FACE_NAMES:
            self.create_subscription(
                Image,
                f"camera/{face}/image",
                lambda message, name=face: self.on_image(name, message),
                10,
            )

    def on_image(self, face: str, message: Image) -> None:
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        bucket = self.frames[stamp]
        bucket[face] = message
        if len(bucket) == len(FACE_NAMES):
            try:
                faces = {
                    name: self.bridge.imgmsg_to_cv2(image, "bgr8")
                    for name, image in bucket.items()
                }
                width = int(self.get_parameter("width").value)
                height = int(self.get_parameter("height").value)
                panorama = stitch_cubemap(faces, (height, width))
                output = self.bridge.cv2_to_imgmsg(panorama, "bgr8")
                output.header = message.header
                self.publisher.publish(output)
            finally:
                self.frames.pop(stamp, None)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PanoramaNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
