# -*- coding: utf-8 -*-
"""节点②：yolo_detect_node

职责（对应作业要求 3-②）：
    1. 启动时校验 ultralytics（import 失败给出友好提示）；
    2. 加载 YOLOv8 官方预训练权重（yolov8n.pt，只做推理，不训练）；
    3. 订阅图像话题，对每一帧做目标检测，只保留"车辆"类目标；
    4. 用检测框的像素宽度估算目标距离（单目针孔模型，见 distance_estimator.py）；
    5. 发布三样东西：
         - /carla/yolo/detections      (std_msgs/String, JSON)      检测框 + 距离
         - /carla/yolo/markers         (visualization_msgs/MarkerArray) 供 RViz 画框
         - /carla/yolo/image_annotated (sensor_msgs/Image)          画了框的图

说明：本项目刻意不使用自定义 msg，也不需要额外安装 vision_msgs，
只用 ROS 2 Humble 自带的消息包，减少复现时踩坑的概率。
检测框信息用 JSON 字符串承载，`ros2 topic echo` 就能直接看懂。
"""

import json
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from . import carla_env, config, detector, distance_estimator, image_utils


def make_image_qos():
    """图像流 QoS：Best Effort + 只保留最新一帧，避免 YOLO 推理慢时消息堆积。"""
    return QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class YoloDetectNode(Node):
    """订阅图像，做 YOLOv8 推理，发布检测框与距离。"""

    def __init__(self):
        super().__init__('yolo_detect_node')

        # ---- 0) 环境检测：import ultralytics 失败会打印友好提示并退出 ----
        self.get_logger().info('环境检查：{}'.format(carla_env.describe_environment()))
        YOLO = carla_env.import_ultralytics_or_exit()

        # ---- 1) 加载模型（只推理，不训练）----
        # 权重文件可通过 ROS 参数覆盖，默认值写在 config/params.yaml 里
        self.declare_parameter(config.PARAM_YOLO_MODEL, config.YOLO_WEIGHTS)
        model_path = self.get_parameter(config.PARAM_YOLO_MODEL).get_parameter_value().string_value
        if not model_path:
            model_path = config.YOLO_WEIGHTS

        self.get_logger().info('正在加载 YOLOv8 权重 {} ...'.format(model_path))
        self.model = YOLO(model_path)
        self.get_logger().info('模型加载完成，类别数 = {}'.format(len(self.model.names)))

        # 焦距：由 config 里的图像宽度与视场角算出，用于单目测距
        self.focal_px = distance_estimator.default_focal_px()
        self.get_logger().info('单目测距参数：focal = {:.1f} px'.format(self.focal_px))

        # ---- 2) ROS 接口 ----
        image_qos = make_image_qos()
        self.publisher_detections = self.create_publisher(String, config.TOPIC_DETECTIONS, 10)
        self.publisher_markers = self.create_publisher(
            MarkerArray, config.TOPIC_DETECTION_MARKERS, 10
        )
        self.publisher_annotated = self.create_publisher(
            Image, config.TOPIC_ANNOTATED_IMAGE, image_qos
        )
        self.subscription_image = self.create_subscription(
            Image, config.TOPIC_CAMERA_IMAGE, self.on_image, image_qos
        )

        self._frames = 0
        self._last_log_time = time.monotonic()
        self._last_infer_ms = 0.0

        self.get_logger().info(
            '节点就绪：订阅 {}，发布 {} / {} / {}'.format(
                config.TOPIC_CAMERA_IMAGE,
                config.TOPIC_DETECTIONS,
                config.TOPIC_DETECTION_MARKERS,
                config.TOPIC_ANNOTATED_IMAGE,
            )
        )

    # ------------------------------------------------------------------
    # 主流程
    # ------------------------------------------------------------------
    def on_image(self, msg):
        """图像回调：推理 -> 组装检测结果 -> 发布。"""
        try:
            frame = image_utils.ros_image_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn('图像解码失败，跳过该帧：{}'.format(exc))
            return

        start = time.monotonic()
        try:
            detections = self.infer(frame)
        except Exception as exc:
            self.get_logger().error('YOLO 推理失败，跳过该帧：{}'.format(exc))
            return
        self._last_infer_ms = (time.monotonic() - start) * 1000.0

        self._frames += 1
        self.publish_detections(msg, frame, detections)
        self.publish_markers(detections, msg.header.stamp)
        self.publish_annotated_image(frame, detections, msg.header.stamp)

        self.log_statistics(detections)

    def infer(self, frame):
        """对一帧图像做 YOLOv8 推理，返回车辆检测结果列表。

        返回:
            list[dict]，每项包含 class / class_id / confidence / bbox /
            width_px / distance_m / lateral_m（结构见 detector.py）
        """
        # 只把非空参数传给 ultralytics，避免 device='' 这类非法值
        kwargs = {
            'source': frame,
            'conf': config.YOLO_CONF_THRESHOLD,
            'iou': config.YOLO_IOU_THRESHOLD,
            'imgsz': config.YOLO_IMG_SIZE,
            'max_det': config.YOLO_MAX_DETECTIONS,
            'verbose': False,
        }
        if config.YOLO_DEVICE:
            kwargs['device'] = config.YOLO_DEVICE

        results = self.model.predict(**kwargs)
        if not results:
            return []

        result = results[0]
        boxes = getattr(result, 'boxes', None)
        if boxes is None or len(boxes) == 0:
            return []

        names = getattr(result, 'names', self.model.names)
        return detector.build_detections(
            boxes_xyxy=boxes.xyxy.cpu().numpy(),
            confidences=boxes.conf.cpu().numpy(),
            class_ids=boxes.cls.cpu().numpy().astype(int),
            names=names,
            image_width_px=frame.shape[1],
            focal_px=self.focal_px,
            max_detections=config.YOLO_MAX_DETECTIONS,
        )

    @staticmethod
    def is_vehicle(class_id, class_name):
        """判断该类别是否属于"车辆"（转发到 detector，保持单一实现）。"""
        return detector.is_vehicle(class_id, class_name)

    # ------------------------------------------------------------------
    # 发布
    # ------------------------------------------------------------------
    def publish_detections(self, source_msg, frame, detections):
        """发布 JSON 格式的检测框信息。"""
        distances = [d['distance_m'] for d in detections if d['distance_m'] is not None]

        payload = {
            'frame_id': config.CAMERA_FRAME,
            'stamp': {
                'sec': int(source_msg.header.stamp.sec),
                'nanosec': int(source_msg.header.stamp.nanosec),
            },
            'image_width': int(frame.shape[1]),
            'image_height': int(frame.shape[0]),
            'vehicle_count': len(detections),
            'min_distance_m': min(distances) if distances else None,
            'infer_ms': round(self._last_infer_ms, 2),
            'detections': detections,
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.publisher_detections.publish(msg)

    def publish_markers(self, detections, stamp):
        """发布 RViz 用的三维框（俯视图下能看出距离远近）。"""
        markers = MarkerArray()

        # 先发一个 DELETEALL，清掉上一帧的残留框
        clear = Marker()
        clear.header.frame_id = config.MAP_FRAME
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for index, det in enumerate(detections):
            distance = det['distance_m']
            if distance is None:
                continue
            lateral = det['lateral_m'] or 0.0

            # 立方体框：x = 前方距离, y = 横向偏移
            box = Marker()
            box.header.frame_id = config.MAP_FRAME
            box.header.stamp = stamp
            box.ns = 'yolo_boxes'
            box.id = index
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = float(distance)
            box.pose.position.y = float(lateral)
            box.pose.position.z = 0.8
            box.pose.orientation.w = 1.0
            box.scale.x = 4.5   # 车长（仅用于可视化）
            box.scale.y = float(distance_estimator.real_width_of_class(
                det['class_id'], det['class']
            ))
            box.scale.z = 1.5   # 车高（仅用于可视化）
            # 颜色按距离分级：红=急刹区，黄=减速区，绿=安全区
            box.color.a = 0.55
            if distance < config.FULL_BRAKE_DISTANCE_M:
                box.color.r, box.color.g, box.color.b = 1.0, 0.0, 0.0
            elif distance < config.SAFE_DISTANCE_M:
                box.color.r, box.color.g, box.color.b = 1.0, 0.8, 0.0
            else:
                box.color.r, box.color.g, box.color.b = 0.1, 1.0, 0.1
            markers.markers.append(box)

            # 文字标签：类别 + 距离 + 置信度
            label = Marker()
            label.header.frame_id = config.MAP_FRAME
            label.header.stamp = stamp
            label.ns = 'yolo_labels'
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(distance)
            label.pose.position.y = float(lateral)
            label.pose.position.z = 2.6
            label.scale.z = 1.0
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = '{} {:.1f}m {:.2f}'.format(
                det['class'], distance, det['confidence']
            )
            markers.markers.append(label)

        self.publisher_markers.publish(markers)

    def publish_annotated_image(self, frame, detections, stamp):
        """发布画了检测框的图，RViz 里可以直接看。"""
        canvas = frame.copy()

        for det in detections:
            x_min, y_min, x_max, y_max = (int(v) for v in det['bbox'])
            distance = det['distance_m']

            if distance is None:
                color = (160, 160, 160)
                text = '{} {:.2f}'.format(det['class'], det['confidence'])
            else:
                if distance < config.FULL_BRAKE_DISTANCE_M:
                    color = (0, 0, 255)      # BGR 红：危险
                elif distance < config.SAFE_DISTANCE_M:
                    color = (0, 215, 255)    # BGR 黄：注意
                else:
                    color = (0, 255, 0)      # BGR 绿：安全
                text = '{} {:.1f}m {:.2f}'.format(det['class'], distance, det['confidence'])

            cv2.rectangle(canvas, (x_min, y_min), (x_max, y_max), color, 2)
            cv2.putText(
                canvas, text, (x_min, max(15, y_min - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA
            )

        # 左上角写一行汇总信息
        distances = [d['distance_m'] for d in detections if d['distance_m'] is not None]
        summary = 'vehicles={} min_dist={} infer={:.0f}ms'.format(
            len(detections),
            'N/A' if not distances else '{:.1f}m'.format(min(distances)),
            self._last_infer_ms,
        )
        cv2.putText(
            canvas, summary, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
            0.65, (255, 255, 255), 2, cv2.LINE_AA
        )

        msg = image_utils.bgr_to_ros_image(canvas, config.CAMERA_FRAME, stamp)
        self.publisher_annotated.publish(msg)

    def log_statistics(self, detections):
        """按固定间隔打印统计信息，避免刷屏。"""
        now = time.monotonic()
        if now - self._last_log_time < 1.0:
            return
        elapsed = now - self._last_log_time
        self._last_log_time = now

        distances = [d['distance_m'] for d in detections if d['distance_m'] is not None]
        self.get_logger().info(
            '推理 {:.0f} ms ({:.1f} FPS) | 车辆 {} 个 | 最近距离 {}'.format(
                self._last_infer_ms,
                1.0 / elapsed if elapsed > 0 else 0.0,
                len(detections),
                'N/A' if not distances else '{:.2f} m'.format(min(distances)),
            )
        )


def main(args=None):
    """节点②的启动函数（供 main.py 调用）。"""
    rclpy.init(args=args)
    node = None
    try:
        node = YoloDetectNode()
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
