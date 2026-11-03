"""ROS 2 visualization node for the synthetic underwater sonar scene."""

import os

import matplotlib

if not os.environ.get("DISPLAY") and os.name != "nt":
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node

from underwater_sonar_mapping.scene import create_scene, draw_scene


class UnderwaterSonarMapping(Node):
    def __init__(self):
        super().__init__("underwater_sonar_mapping")
        self.declare_parameter("save_path", "")

        self.scene = create_scene()
        plt.ion()
        self.figure = plt.figure(figsize=(12, 9))
        self.axis = self.figure.add_subplot(111, projection="3d")
        draw_scene(self.axis, self.scene)

        save_path = str(self.get_parameter("save_path").value).strip()
        if save_path:
            output = os.path.abspath(os.path.expanduser(save_path))
            os.makedirs(os.path.dirname(output), exist_ok=True)
            self.figure.savefig(output, dpi=150, bbox_inches="tight")
            self.get_logger().info(f"Saved sonar scene to {output}")
        else:
            self.figure.canvas.draw()
            self.figure.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = UnderwaterSonarMapping()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()