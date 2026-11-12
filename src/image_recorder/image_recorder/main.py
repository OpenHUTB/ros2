import signal

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

try:  # Humble 及更新的 rclpy 在 SIGINT 关闭 context 时会抛这个异常
    from rclpy.executors import ExternalShutdownException
except ImportError:  # 更老的 rclpy 版本可能没有该异常类，用占位类保证 import 不失败
    class ExternalShutdownException(Exception):
        pass

from image_recorder.capture import make_capture_path, normalize_topic


class ImageRecorder(Node):
    """订阅相机话题，按回车保存当前帧。

    参数：
        image_topic  订阅的图像话题，默认 Carla 前视相机
        output_dir   保存目录，默认当前目录
        image_format 保存格式（jpg/jpeg/png），默认 jpg
        show_window  是否弹窗显示（无图形界面的环境设为 False）
    """

    def __init__(self):
        super().__init__('image_recorder_node')
        self.image_topic = normalize_topic(
            self.declare_parameter('image_topic',
                                   '/carla/ego_vehicle/camera/rgb/front/image')
            .value)
        self.output_dir = self.declare_parameter('output_dir', '.').value
        self.image_format = str(
            self.declare_parameter('image_format', 'jpg').value)
        self.show_window = bool(
            self.declare_parameter('show_window', True).value)

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self._save_index = 0
        self.get_logger().info(
            f'Node started, topic={self.image_topic}, output_dir={self.output_dir}, '
            f'show_window={self.show_window}')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self.show_window:
                cv2.imshow("Image Recorder", cv_image)
                key = cv2.waitKey(1)
            else:
                # 无窗口时无法按键，改为每帧都保存，便于录制数据集
                key = 13
            if key == 13:
                path = make_capture_path(self.output_dir,
                                         extension=self.image_format,
                                         index=self._save_index)
                if cv2.imwrite(str(path), cv_image):
                    self._save_index += 1
                    self.get_logger().info(f'Saved as {path}')
                else:
                    self.get_logger().error(f'Failed to write image: {path}')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def destroy_node(self):
        if self.show_window:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    image_recorder = ImageRecorder()
    try:
        rclpy.spin(image_recorder)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Humble 及以上：SIGINT 关闭 context 时 spin 抛 ExternalShutdownException；
        # Foxy：抛 KeyboardInterrupt。两种都按正常退出处理。
        pass
    finally:
        # ros2 launch 退出时 SIGINT 会同时发给 launch 和节点，清理阶段再收到一次
        # KeyboardInterrupt 会打印 traceback 并以 -2 退出；这里屏蔽掉。
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        image_recorder.destroy_node()
        try:
            rclpy.try_shutdown()
        except AttributeError:          # 老版本 rclpy 没有 try_shutdown
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
