"""ROS 2 node that turns camera frames into fire alarms and bounding boxes."""

from __future__ import annotations

import json
import math
import time

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

from .detector_core import (
    AlertTracker,
    Detection,
    FireDetectorConfig,
    HsvFireDetector,
    OnnxFireVerifier,
    annotate,
    apply_verifier_confidence,
)


class FireDetectorNode(Node):

    def __init__(self) -> None:
        super().__init__('fire_detector')

        self.image_topic = str(self.declare_parameter(
            'image_topic', '/camera/image_raw'
        ).value)
        self.publish_debug_image = bool(self.declare_parameter(
            'publish_debug_image', True
        ).value)
        self.process_every_n_frames = max(1, int(self.declare_parameter(
            'process_every_n_frames', 1
        ).value))
        self.min_confidence = float(self.declare_parameter(
            'min_confidence', 0.45
        ).value)
        self.min_consecutive_frames = max(1, int(self.declare_parameter(
            'min_consecutive_frames', 3
        ).value))
        self.alert_hold_sec = max(0.0, float(self.declare_parameter(
            'alert_hold_sec', 2.0
        ).value))
        self.image_timeout_sec = max(0.1, float(self.declare_parameter(
            'image_timeout_sec', 2.0
        ).value))
        self.horizontal_fov_deg = float(self.declare_parameter(
            'horizontal_fov_deg', 90.0
        ).value)
        self.model_path = str(self.declare_parameter(
            'model_path', ''
        ).value)
        self.model_input_size = int(self.declare_parameter(
            'model_input_size', 224
        ).value)
        self.model_fire_class_index = int(self.declare_parameter(
            'model_fire_class_index', 1
        ).value)
        self.model_weight = float(self.declare_parameter(
            'model_weight', 0.5
        ).value)

        config = FireDetectorConfig(
            hsv_h_min=int(self.declare_parameter('hsv_h_min', 0).value),
            hsv_h_max=int(self.declare_parameter('hsv_h_max', 35).value),
            hsv_s_min=int(self.declare_parameter('hsv_s_min', 90).value),
            hsv_v_min=int(self.declare_parameter('hsv_v_min', 110).value),
            red_h_min=int(self.declare_parameter('red_h_min', 160).value),
            red_h_max=int(self.declare_parameter('red_h_max', 179).value),
            red_s_min=int(self.declare_parameter('red_s_min', 110).value),
            red_v_min=int(self.declare_parameter('red_v_min', 90).value),
            min_area_ratio=float(self.declare_parameter(
                'min_area_ratio', 0.0015
            ).value),
            max_area_ratio=float(self.declare_parameter(
                'max_area_ratio', 0.70
            ).value),
            min_confidence=0.0,
            morph_kernel=int(self.declare_parameter(
                'morph_kernel', 5
            ).value),
            max_detections=int(self.declare_parameter(
                'max_detections', 3
            ).value),
            min_aspect_ratio=float(self.declare_parameter(
                'min_aspect_ratio', 0.10
            ).value),
            max_aspect_ratio=float(self.declare_parameter(
                'max_aspect_ratio', 8.0
            ).value),
        )
        self.detector = HsvFireDetector(config)

        self.verifier = None
        self.backend = 'hsv'
        if self.model_path:
            try:
                self.verifier = OnnxFireVerifier(
                    self.model_path,
                    input_size=self.model_input_size,
                    fire_class_index=self.model_fire_class_index,
                )
                self.backend = 'hsv+onnx'
                self.get_logger().info(
                    f'Loaded ONNX fire verifier: {self.model_path}'
                )
            except Exception as exc:  # pragma: no cover - depends on model
                self.get_logger().warning(
                    f'Failed to load ONNX model, using HSV only: {exc!r}'
                )

        self.bridge = CvBridge()
        self.alert = False
        self.alert_tracker = AlertTracker(
            min_consecutive_frames=self.min_consecutive_frames,
            hold_sec=self.alert_hold_sec,
        )
        self.last_frame_time: float | None = None
        self.frame_count = 0
        self.processed_count = 0
        self.fps = 0.0
        self._frame_counter = 0
        self._fps_started = time.monotonic()
        self._fps_frames = 0
        self._last_bbox: list[float] = []

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.alert_pub = self.create_publisher(
            Bool, '/fire_detection/alert', 10
        )
        self.bbox_pub = self.create_publisher(
            Float32MultiArray, '/fire_detection/bbox', 10
        )
        self.bearing_pub = self.create_publisher(
            Float32, '/fire_detection/target_bearing', 10
        )
        self.debug_image_pub = self.create_publisher(
            Image, '/fire_detection/debug_image', qos_profile_sensor_data
        )
        self.status_pub = self.create_publisher(
            String, '/fire_detection/status', 10
        )

        self.create_timer(0.5, self.watchdog_callback)
        self.create_timer(1.0, self.heartbeat_callback)
        self.get_logger().info(
            f'Fire detector ready: image={self.image_topic}, '
            f'backend={self.backend}'
        )

    def image_callback(self, msg: Image) -> None:
        self._frame_counter += 1
        if self._frame_counter % self.process_every_n_frames != 0:
            return

        now = self._now_sec()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to convert camera image: {exc!r}',
                throttle_duration_sec=5.0,
            )
            return

        self.last_frame_time = now
        self.frame_count += 1
        self.processed_count += 1
        self._fps_frames += 1

        detections = self.detector.detect(frame)
        if self.verifier is not None and detections:
            verified: list[Detection] = []
            for detection in detections:
                roi = frame[
                    detection.y_min:detection.y_max,
                    detection.x_min:detection.x_max,
                ]
                model_confidence = self.verifier.predict(roi)
                candidate = apply_verifier_confidence(
                    detection, model_confidence, self.model_weight
                )
                if candidate.confidence >= self.min_confidence:
                    verified.append(candidate)
            detections = sorted(
                verified, key=lambda det: det.confidence, reverse=True
            )
        else:
            detections = [
                det for det in detections
                if det.confidence >= self.min_confidence
            ]

        best = detections[0] if detections else None
        self._last_bbox = self._bbox_data(best) if best is not None else []
        alert = self.alert_tracker.update(now, best is not None)
        self._set_alert(alert, force=False)

        bbox_msg = Float32MultiArray()
        bbox_msg.data = self._last_bbox
        self.bbox_pub.publish(bbox_msg)

        if best is not None:
            fov_rad = math.radians(self.horizontal_fov_deg)
            bearing = (best.center_x - 0.5) * fov_rad
            self.bearing_pub.publish(Float32(data=float(bearing)))
        else:
            self.bearing_pub.publish(Float32(data=float('nan')))

        if self.publish_debug_image:
            rendered = annotate(frame, detections, self.alert)
            try:
                debug_image = self.bridge.cv2_to_imgmsg(
                    rendered, encoding='bgr8'
                )
                debug_image.header = msg.header
                self.debug_image_pub.publish(debug_image)
            except Exception as exc:  # pragma: no cover - cv_bridge only
                self.get_logger().warning(
                    f'Failed to publish debug image: {exc!r}',
                    throttle_duration_sec=5.0,
                )

    def watchdog_callback(self) -> None:
        if self.last_frame_time is None:
            return
        if self._now_sec() - self.last_frame_time <= self.image_timeout_sec:
            return
        self.alert_tracker.reset()
        self._last_bbox = []
        self._set_alert(False, force=False)
        empty = Float32MultiArray()
        self.bbox_pub.publish(empty)
        self.bearing_pub.publish(Float32(data=float('nan')))

    def heartbeat_callback(self) -> None:
        now = time.monotonic()
        elapsed = now - self._fps_started
        if elapsed >= 1.0:
            self.fps = self._fps_frames / elapsed
            self._fps_frames = 0
            self._fps_started = now

        self._set_alert(self.alert, force=True)

        status = {
            'alert': self.alert,
            'confidence': self._last_bbox[4] if self._last_bbox else 0.0,
            'bbox': self._last_bbox,
            'frames': self.frame_count,
            'fps': round(self.fps, 2),
            'backend': self.backend,
            'image_timeout': (
                self.last_frame_time is not None
                and self._now_sec() - self.last_frame_time
                > self.image_timeout_sec
            ),
        }
        self.status_pub.publish(String(data=json.dumps(status)))

    def _set_alert(self, value: bool, force: bool) -> None:
        value = bool(value)
        if not force and value == self.alert:
            return
        if value != self.alert:
            if value:
                self.get_logger().warning('Fire alarm raised')
            else:
                self.get_logger().info('Fire alarm cleared')
        self.alert = value
        self.alert_pub.publish(Bool(data=value))

    @staticmethod
    def _bbox_data(detection: Detection) -> list[float]:
        """Pixel box followed by normalised centre, confidence and area."""

        return [
            float(detection.x_min),
            float(detection.y_min),
            float(detection.x_max),
            float(detection.y_max),
            detection.confidence,
            detection.center_x,
            detection.center_y,
            detection.area_ratio,
        ]

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def destroy_node(self) -> None:
        try:
            self.alert_pub.publish(Bool(data=False))
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FireDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
