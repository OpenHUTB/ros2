"""ROS 2 wrapper: publish camera frames, run the CNN, publish control commands."""

from __future__ import annotations

import numpy as np

from pix2cmd.dataset import IMG, render, road_profile
from pix2cmd.train import closed_loop_drive, train


def main(args=None) -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from std_msgs.msg import Float32MultiArray, String
    except ImportError as exc:  # pragma: no cover - depends on the ROS installation
        raise SystemExit(
            "ROS 2 Python packages are required: install ROS 2 and source its setup.bash"
        ) from exc

    class Pix2CmdNode(Node):
        """Train once, then drive the synthetic S-curve through the network."""

        def __init__(self):
            super().__init__("pix2cmd_end2end_driving")
            self.declare_parameter("epochs", 8)
            self.declare_parameter("n_train", 800)
            self.declare_parameter("steps", 240)
            self.declare_parameter("publish_images", True)

            epochs = int(self.get_parameter("epochs").value)
            n_train = int(self.get_parameter("n_train").value)
            self.steps = int(self.get_parameter("steps").value)

            self.get_logger().info(f"training CNN ({n_train} samples, {epochs} epochs) ...")
            self.net, self.history, _ = train(epochs=epochs, n_train=n_train, n_val=max(64, n_train // 4),
                                              log=lambda *_a, **_k: None)
            last = self.history[-1]
            self.get_logger().info(
                f"trained: val MAE steering={last['val_mae_steering']:.4f} "
                f"throttle={last['val_mae_throttle']:.4f}")

            self.publish_images = bool(self.get_parameter("publish_images").value)
            self.image_pub = self.create_publisher(Image, "/pix2cmd/camera", 10)
            self.cmd_pub = self.create_publisher(Float32MultiArray, "/pix2cmd/cmd", 10)
            self.state_pub = self.create_publisher(String, "/pix2cmd/state", 10)

            self.frames, self.metrics = closed_loop_drive(self.net, steps=self.steps)
            self.index = 0
            self.create_timer(0.1, self.tick)
            self.get_logger().info(
                f"closed-loop ready: mean|error|={self.metrics['mean_abs_lateral_error_m']} m, "
                f"max={self.metrics['max_abs_lateral_error_m']} m, loop over {len(self.frames)} frames")

        def tick(self):
            if not self.frames:
                return
            image, steer, throttle, error = self.frames[self.index % len(self.frames)]
            self.index += 1

            cmd = Float32MultiArray()
            cmd.data = [float(steer), float(throttle)]
            self.cmd_pub.publish(cmd)

            state = String()
            state.data = f"steer={steer:+.3f} throttle={throttle:.3f} lateral_error={error:+.3f}m"
            self.state_pub.publish(state)

            if self.publish_images:
                msg = Image()
                msg.height = IMG
                msg.width = IMG
                msg.encoding = "mono8"
                msg.step = IMG
                msg.data = (np.clip(image, 0, 1) * 255).astype(np.uint8).tobytes()
                self.image_pub.publish(msg)

    rclpy.init(args=args)
    node = Pix2CmdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
