"""ROS 2 检测节点：样本目录单次执行或订阅巡航图像。"""
import json

from .detector import DEFAULT_MODEL, FacadeDetector, annotate, asset_directory
from .image_transport import decode_image
from .main import run


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    class DetectionNode(Node):
        def __init__(self):
            super().__init__("town10_facade_defect_detection")
            defaults = {"mode": "samples", "input": str(asset_directory() / "samples"),
                        "output": "town10_detections", "model": str(DEFAULT_MODEL), "confidence": 0.25,
                        "input_topic": "/town10/image_raw", "annotated_topic": "/town10/defects/image",
                        "detections_topic": "/town10/defects/detections"}
            for name, value in defaults.items():
                self.declare_parameter(name, value)
            self.get_value = lambda name: self.get_parameter(name).value
            self.mode = self.get_value("mode")
            if self.mode == "samples":
                run(self.get_value("input"), self.get_value("output"), self.get_value("model"),
                    float(self.get_value("confidence")))
            elif self.mode == "live":
                self.detector = FacadeDetector(self.get_value("model"), float(self.get_value("confidence")))
                self.image_pub = self.create_publisher(Image, self.get_value("annotated_topic"), 1)
                self.results_pub = self.create_publisher(String, self.get_value("detections_topic"), 10)
                self.subscription = self.create_subscription(Image, self.get_value("input_topic"), self.callback,
                                                             qos_profile_sensor_data)
                self.get_logger().info(f"订阅图像: {self.get_value('input_topic')}")
            else:
                raise ValueError("mode 仅支持 samples 或 live")

        def callback(self, message):
            try:
                image = decode_image(message)
                detections = self.detector.predict(image)
                annotated = annotate(image, detections)
                output = Image()
                output.header = message.header
                output.height, output.width = annotated.shape[:2]
                output.encoding, output.is_bigendian = "bgr8", 0
                output.step, output.data = output.width * 3, annotated.tobytes()
                self.image_pub.publish(output)
                self.results_pub.publish(String(data=json.dumps({
                    "stamp": {"sec": message.header.stamp.sec, "nanosec": message.header.stamp.nanosec},
                    "frame_id": message.header.frame_id, "width": output.width, "height": output.height,
                    "n_defects": sum(d["is_defect"] for d in detections), "detections": detections},
                    ensure_ascii=False)))
            except (ValueError, RuntimeError) as error:
                self.get_logger().error(str(error))

    rclpy.init(args=args)
    node = None
    try:
        node = DetectionNode()
        if node.mode == "live":
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
