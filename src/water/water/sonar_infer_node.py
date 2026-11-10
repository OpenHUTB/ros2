"""声呐图像 CNN 推理节点：订阅相机图像，定时推理并发布带标注的结果图。"""

import json
import os

import cv2
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from water.sonar_model import load_model


class SonarInferNode(Node):

    def __init__(self):
        super().__init__("sonar_infer_node")

        # ---------------- 可配置参数（ros2 run ... --ros-args -p xxx:=yyy） ----------------
        self.declare_parameter("image_topic", "/sonar/camera/image_raw")
        self.declare_parameter("result_topic", "/sonar/result")
        # 空字符串表示使用 install 目录下的 share/water/weights/sonar_cnn_weights.pth
        self.declare_parameter("weight_file", "")
        self.declare_parameter("input_size", 256)
        self.declare_parameter("infer_period", 0.5)   # 秒，2 Hz 推理，避免 15 Hz 图像把 CPU 打满
        # 仅在缺少 classes.json 时作为回退；默认顺序与 ImageFolder 字母序一致
        self.declare_parameter("classes", ["pipe", "rock", "sand"])
        self.declare_parameter("device", "cpu")

        self.input_size = int(self.get_parameter("input_size").value)
        self.device = str(self.get_parameter("device").value)

        weight_file = str(self.get_parameter("weight_file").value)
        if not weight_file:
            weight_file = os.path.join(
                get_package_share_directory("water"), "weights", "sonar_cnn_weights.pth")
        if not os.path.isfile(weight_file):
            raise FileNotFoundError(
                f"未找到权重文件: {weight_file}。请先按 "
                "docs/water/rov_gazebo_sonar_cnn.md「数据集与权重说明」采集数据并训练，"
                "或用 -p weight_file:=<绝对路径> 指定。")

        # 类别顺序优先取训练时保存的 classes.json，其次才是参数
        classes_json = os.path.join(os.path.dirname(weight_file), "classes.json")
        if os.path.isfile(classes_json):
            with open(classes_json, encoding="utf-8") as f:
                self.classes = [str(c) for c in json.load(f)]
            self.get_logger().info(
                f"从 {classes_json} 读取类别顺序: {self.classes}")
        else:
            self.classes = [str(c) for c in self.get_parameter("classes").value]
            self.get_logger().warning(
                "未找到 classes.json，回退为参数 classes 的顺序: "
                f"{self.classes}；该顺序必须与训练时 ImageFolder 的字母序一致，"
                "否则类别会被标错。")

        # 权重缺失/结构不符时直接报错，绝不降级为随机权重输出假结果
        self.model = load_model(weight_file, len(self.classes), self.input_size, self.device)
        self.get_logger().info(f"权重加载成功: {weight_file} (device={self.device})")

        self.bridge = CvBridge()
        self.latest_msg = None

        image_topic = str(self.get_parameter("image_topic").value)
        result_topic = str(self.get_parameter("result_topic").value)
        self.sub_img = self.create_subscription(Image, image_topic, self.img_callback, 10)
        self.pub_result = self.create_publisher(Image, result_topic, 10)

        period = float(self.get_parameter("infer_period").value)
        self.timer = self.create_timer(period, self.infer_once)

        self.get_logger().info(f"声呐识别节点已启动，订阅 {image_topic}，发布 {result_topic}")

    def img_callback(self, msg):
        # 只缓存最新帧，推理放到定时器里做，避免回调阻塞与帧积压
        self.latest_msg = msg

    @staticmethod
    def _to_rgb(cv_img):
        # 单通道声呐/灰度图补成 3 通道，避免 permute(2,0,1) 报错
        if cv_img.ndim == 2:
            return cv2.cvtColor(cv_img, cv2.COLOR_GRAY2RGB)
        return cv_img

    def infer_once(self):
        if self.latest_msg is None:
            return

        msg = self.latest_msg
        rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")   # 与训练的 PIL RGB 对齐
        rgb = self._to_rgb(rgb)
        rgb = cv2.resize(rgb, (self.input_size, self.input_size))

        tensor = torch.from_numpy(rgb).permute(2, 0, 1).float().div(255.0)
        tensor = tensor.unsqueeze(0).to(self.device)

        with torch.no_grad():
            prob = torch.softmax(self.model(tensor), dim=1)[0]
            conf, idx = torch.max(prob, dim=0)

        label = self.classes[int(idx)]
        self.get_logger().info(f"检测目标: {label}, 置信度: {float(conf):.2f}")

        # 预览与发布用 BGR（OpenCV/ROS 图像惯例）
        vis = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        vis = cv2.putText(vis, f"{label} {float(conf):.2f}", (10, 30),
                          cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv2.LINE_AA)
        out_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        out_msg.header = msg.header
        self.pub_result.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SonarInferNode()
        rclpy.spin(node)
    except FileNotFoundError as e:
        logger = node.get_logger() if node is not None else rclpy.logging.get_logger("sonar_infer_node")
        logger.error(str(e))
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
