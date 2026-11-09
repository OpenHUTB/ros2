import csv
import ctypes
import os
import time
from ctypes import wintypes
from datetime import datetime

import cv2
import numpy as np
import rclpy
import torch
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from ultralytics import YOLO
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


class YoloNode(Node):
    """仅接收和发布 ROS 消息的 YOLO 检测节点。"""

    def __init__(self):
        super().__init__('yolo_detector')

        # ============================================================
        # ROS 参数
        # ============================================================
        self.declare_parameter('model_path', 'yolo11n.pt')
        self.declare_parameter('confidence', 0.30)
        self.declare_parameter('image_size', 640)

        self.declare_parameter(
            'input_image_topic',
            '/openhutb/camera/rgb',
        )
        self.declare_parameter(
            'detections_topic',
            '/openhutb/detections',
        )
        self.declare_parameter(
            'annotated_image_topic',
            '/openhutb/yolo/image',
        )
        self.declare_parameter(
            'capture_metric_topic',
            '/openhutb/metrics/airsim_capture_ms',
        )

        self.declare_parameter('show_window', True)
        self.declare_parameter('start_topmost', True)
        self.declare_parameter(
            'window_name',
            'OpenHUTB + YOLO',
        )
        self.declare_parameter(
            'output_dir',
            'output',
        )

        # ============================================================
        # 读取参数
        # ============================================================
        self.confidence = float(
            self.get_parameter('confidence').value
        )
        self.image_size = int(
            self.get_parameter('image_size').value
        )
        self.show_window = bool(
            self.get_parameter('show_window').value
        )
        self.topmost = bool(
            self.get_parameter('start_topmost').value
        )
        self.window_name = str(
            self.get_parameter('window_name').value
        )

        self.last_capture_ms = 0.0

        # ============================================================
        # YOLO
        # ============================================================
        model_path = str(
            self.get_parameter('model_path').value
        )

        if torch.cuda.is_available():
            self.device = 0
            self.gpu_name = torch.cuda.get_device_name(0)
        else:
            self.device = 'cpu'
            self.gpu_name = 'CPU'

        self.get_logger().info(
            f'Loading YOLO model: {model_path}'
        )

        self.model = YOLO(model_path)

        self.get_logger().info(
            f'YOLO ready; device={self.gpu_name}'
        )

        # ============================================================
        # ROS 话题
        # ============================================================
        image_topic = str(
            self.get_parameter('input_image_topic').value
        )
        detections_topic = str(
            self.get_parameter('detections_topic').value
        )
        annotated_topic = str(
            self.get_parameter('annotated_image_topic').value
        )
        metric_topic = str(
            self.get_parameter('capture_metric_topic').value
        )

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.capture_sub = self.create_subscription(
            Float32,
            metric_topic,
            self.capture_metric_callback,
            10,
        )

        self.detection_pub = self.create_publisher(
            Detection2DArray,
            detections_topic,
            10,
        )

        self.annotated_pub = self.create_publisher(
            Image,
            annotated_topic,
            qos_profile_sensor_data,
        )

        # ============================================================
        # 统计信息
        # ============================================================
        self.frame_id = 0
        self.fps = 0.0
        self.fps_counter = 0
        self.fps_start = time.time()

        self.fps_values = []
        self.inference_values = []

        # ============================================================
        # 输出设置
        # ============================================================
        output_dir = str(
            self.get_parameter('output_dir').value
        )

        self.screenshot_dir = os.path.join(
            output_dir,
            'screenshots',
        )

        os.makedirs(
            self.screenshot_dir,
            exist_ok=True,
        )
        os.makedirs(
            output_dir,
            exist_ok=True,
        )

        self.csv_path = os.path.join(
            output_dir,
            'results.csv',
        )

        self.csv_file = open(
            self.csv_path,
            'w',
            newline='',
            encoding='utf-8-sig',
        )

        self.csv_writer = csv.writer(
            self.csv_file
        )

        self.csv_writer.writerow([
            'frame',
            'time',
            'fps',
            'inference_ms',
            'airsim_capture_ms',
            'objects',
        ])

        # ============================================================
        # OpenCV 窗口
        # ============================================================
        if self.show_window:
            cv2.namedWindow(
                self.window_name,
                cv2.WINDOW_NORMAL,
            )

            cv2.resizeWindow(
                self.window_name,
                1280,
                720,
            )

            # 让 HighGUI / Qt 后端先处理一个事件周期，确保可以取得
            # Windows 原生窗口句柄。
            cv2.waitKeyEx(1)
            time.sleep(0.05)

            self._apply_topmost()

            self.get_logger().info(
                'OpenCV keys: '
                'Q=quit, S=screenshot, T=toggle topmost'
            )

    # ================================================================
    # ROS 回调
    # ================================================================
    def capture_metric_callback(
        self,
        msg: Float32,
    ):
        self.last_capture_ms = float(
            msg.data
        )

    # ================================================================
    # 图像转换
    # ================================================================
    @staticmethod
    def ros_image_to_bgr(
        msg: Image,
    ):
        if msg.encoding not in (
            'bgr8',
            'rgb8',
        ):
            raise ValueError(
                f'Unsupported image encoding: {msg.encoding}'
            )

        expected = int(
            msg.height * msg.step
        )

        array = np.frombuffer(
            msg.data,
            dtype=np.uint8,
            count=expected,
        )

        frame = array.reshape(
            (
                msg.height,
                msg.step // 3,
                3,
            )
        )[:, :msg.width, :]

        if msg.encoding == 'rgb8':
            frame = cv2.cvtColor(
                frame,
                cv2.COLOR_RGB2BGR,
            )

        return frame.copy()

    @staticmethod
    def bgr_to_ros_image(
        frame,
        header,
    ):
        msg = Image()

        msg.header = header
        msg.height = int(
            frame.shape[0]
        )
        msg.width = int(
            frame.shape[1]
        )
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = int(
            frame.shape[1] * 3
        )
        msg.data = frame.tobytes()

        return msg

    # ================================================================
    # 主图像回调
    # ================================================================
    def image_callback(
        self,
        msg: Image,
    ):
        try:
            frame = self.ros_image_to_bgr(
                msg
            )
        except Exception as exc:
            self.get_logger().error(
                f'Image conversion failed: {exc}'
            )
            return

        # ------------------------------------------------------------
        # YOLO 推理
        # ------------------------------------------------------------
        results = self.model.predict(
            source=frame,
            conf=self.confidence,
            imgsz=self.image_size,
            device=self.device,
            verbose=False,
        )

        result = results[0]

        inference_ms = float(
            result.speed.get(
                'inference',
                0.0,
            )
        )

        object_count = (
            len(result.boxes)
            if result.boxes is not None
            else 0
        )

        # ------------------------------------------------------------
        # 发布检测结果
        # ------------------------------------------------------------
        detection_array = Detection2DArray()
        detection_array.header = msg.header

        names = result.names

        if result.boxes is not None:
            for index, box in enumerate(
                result.boxes
            ):
                x1, y1, x2, y2 = [
                    float(x)
                    for x in (
                        box.xyxy[0]
                        .detach()
                        .cpu()
                        .tolist()
                    )
                ]

                class_index = int(
                    box.cls[0]
                    .detach()
                    .cpu()
                    .item()
                )

                confidence = float(
                    box.conf[0]
                    .detach()
                    .cpu()
                    .item()
                )

                class_name = str(
                    names[class_index]
                )

                detection = Detection2D()
                detection.header = msg.header
                detection.id = (
                    f'{self.frame_id}-{index}'
                )

                cx = (
                    x1 + x2
                ) / 2.0
                cy = (
                    y1 + y2
                ) / 2.0

                # vision_msgs/BoundingBox2D 的中心点表示方式在不同
                # ROS 2 发行版之间有所变化。
                if hasattr(
                    detection.bbox.center,
                    'position',
                ):
                    detection.bbox.center.position.x = cx
                    detection.bbox.center.position.y = cy

                    if hasattr(
                        detection.bbox.center,
                        'theta',
                    ):
                        detection.bbox.center.theta = 0.0
                else:
                    detection.bbox.center.x = cx
                    detection.bbox.center.y = cy

                    if hasattr(
                        detection.bbox.center,
                        'theta',
                    ):
                        detection.bbox.center.theta = 0.0

                detection.bbox.size_x = (
                    x2 - x1
                )
                detection.bbox.size_y = (
                    y2 - y1
                )

                hypothesis = (
                    ObjectHypothesisWithPose()
                )

                hypothesis.hypothesis.class_id = (
                    class_name
                )
                hypothesis.hypothesis.score = (
                    confidence
                )

                detection.results.append(
                    hypothesis
                )

                detection_array.detections.append(
                    detection
                )

        self.detection_pub.publish(
            detection_array
        )

        # ------------------------------------------------------------
        # 绘制检测结果
        # ------------------------------------------------------------
        annotated = result.plot()

        # ------------------------------------------------------------
        # FPS
        # ------------------------------------------------------------
        self.frame_id += 1
        self.fps_counter += 1

        elapsed = (
            time.time()
            - self.fps_start
        )

        if elapsed >= 1.0:
            self.fps = (
                self.fps_counter
                / elapsed
            )

            self.fps_counter = 0
            self.fps_start = time.time()

        if self.fps > 0:
            self.fps_values.append(
                self.fps
            )

        self.inference_values.append(
            inference_ms
        )

        # ------------------------------------------------------------
        # 信息叠加层
        # ------------------------------------------------------------
        cv2.rectangle(
            annotated,
            (15, 15),
            (540, 190),
            (0, 0, 0),
            -1,
        )

        info = [
            f'FPS: {self.fps:.1f}',
            f'YOLO: {inference_ms:.1f} ms',
            f'AirSim: {self.last_capture_ms:.1f} ms',
            f'Objects: {object_count}',
            f'GPU: {self.gpu_name}',
        ]

        y = 45

        for text in info:
            cv2.putText(
                annotated,
                text,
                (30, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

            y += 32

        # ------------------------------------------------------------
        # 发布标注后的图像
        # ------------------------------------------------------------
        self.annotated_pub.publish(
            self.bgr_to_ros_image(
                annotated,
                msg.header,
            )
        )

        # ------------------------------------------------------------
        # CSV
        # ------------------------------------------------------------
        self.csv_writer.writerow([
            self.frame_id,
            datetime.now().strftime(
                '%Y-%m-%d %H:%M:%S'
            ),
            round(
                self.fps,
                2,
            ),
            round(
                inference_ms,
                2,
            ),
            round(
                self.last_capture_ms,
                2,
            ),
            object_count,
        ])

        if self.frame_id % 10 == 0:
            self.csv_file.flush()

        # ------------------------------------------------------------
        # OpenCV 显示与键盘处理
        # ------------------------------------------------------------
        if self.show_window:
            cv2.imshow(
                self.window_name,
                annotated,
            )

            key = cv2.waitKeyEx(1)

            if key != -1:
                key = key & 0xFF

                if key in (
                    ord('q'),
                    ord('Q'),
                ):
                    self.get_logger().info(
                        'Q pressed; '
                        'shutting down YOLO node.'
                    )

                    rclpy.shutdown()

                elif key in (
                    ord('s'),
                    ord('S'),
                ):
                    filename = (
                        datetime.now()
                        .strftime(
                            'yolo_%Y%m%d_%H%M%S.jpg'
                        )
                    )

                    path = os.path.join(
                        self.screenshot_dir,
                        filename,
                    )

                    cv2.imwrite(
                        path,
                        annotated,
                    )

                    self.get_logger().info(
                        f'Screenshot saved: {path}'
                    )

                elif key in (
                    ord('t'),
                    ord('T'),
                ):
                    self.topmost = (
                        not self.topmost
                    )

                    self._apply_topmost()

    # ================================================================
    # Windows 窗口置顶
    # ================================================================
    def _apply_topmost(
        self,
    ):
        if not self.show_window:
            return

        # ------------------------------------------------------------
        # Windows：使用原生 Win32 API。
        #
        # OpenCV 的 WND_PROP_TOPMOST 在部分 Qt / HighGUI Windows 构建中
        # 不可靠，因此改用 SetWindowPos。
        # ------------------------------------------------------------
        if os.name == 'nt':
            try:
                user32 = ctypes.windll.user32

                user32.FindWindowW.argtypes = [
                    wintypes.LPCWSTR,
                    wintypes.LPCWSTR,
                ]
                user32.FindWindowW.restype = (
                    wintypes.HWND
                )

                user32.SetWindowPos.argtypes = [
                    wintypes.HWND,
                    wintypes.HWND,
                    ctypes.c_int,
                    ctypes.c_int,
                    ctypes.c_int,
                    ctypes.c_int,
                    wintypes.UINT,
                ]
                user32.SetWindowPos.restype = (
                    wintypes.BOOL
                )

                hwnd = user32.FindWindowW(
                    None,
                    self.window_name,
                )

                if not hwnd:
                    # 再处理一个 HighGUI 事件周期后重试。
                    cv2.waitKeyEx(1)
                    time.sleep(0.02)

                    hwnd = user32.FindWindowW(
                        None,
                        self.window_name,
                    )

                if not hwnd:
                    self.get_logger().warning(
                        'Cannot find native window handle '
                        f'for "{self.window_name}"'
                    )
                    return

                hwnd_topmost = wintypes.HWND(
                    -1
                )
                hwnd_notopmost = wintypes.HWND(
                    -2
                )

                insert_after = (
                    hwnd_topmost
                    if self.topmost
                    else hwnd_notopmost
                )

                swp_nosize = 0x0001
                swp_nomove = 0x0002
                swp_noactivate = 0x0010

                result = user32.SetWindowPos(
                    hwnd,
                    insert_after,
                    0,
                    0,
                    0,
                    0,
                    (
                        swp_nomove
                        | swp_nosize
                        | swp_noactivate
                    ),
                )

                if result:
                    self.get_logger().info(
                        'Window topmost: '
                        f'{"ON" if self.topmost else "OFF"}'
                    )
                else:
                    error_code = (
                        ctypes.get_last_error()
                    )

                    self.get_logger().warning(
                        'SetWindowPos failed; '
                        f'WinError={error_code}'
                    )

                return

            except Exception as exc:
                self.get_logger().warning(
                    'Win32 topmost failed: '
                    f'{exc}'
                )

        # ------------------------------------------------------------
        # 非 Windows 平台的后备方案。
        # ------------------------------------------------------------
        if hasattr(
            cv2,
            'WND_PROP_TOPMOST',
        ):
            try:
                cv2.setWindowProperty(
                    self.window_name,
                    cv2.WND_PROP_TOPMOST,
                    (
                        1
                        if self.topmost
                        else 0
                    ),
                )

                self.get_logger().info(
                    'Window topmost: '
                    f'{"ON" if self.topmost else "OFF"}'
                )

            except cv2.error as exc:
                self.get_logger().warning(
                    'OpenCV topmost failed: '
                    f'{exc}'
                )

    # ================================================================
    # 清理资源
    # ================================================================
    def destroy_node(
        self,
    ):
        try:
            self.csv_file.close()
        except Exception:
            pass

        if self.show_window:
            cv2.destroyAllWindows()

        super().destroy_node()


def main(args=None):
    rclpy.init(
        args=args
    )

    node = YoloNode()

    try:
        rclpy.spin(
            node
        )
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
