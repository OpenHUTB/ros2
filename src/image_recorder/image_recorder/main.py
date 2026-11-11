import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageRecorder(Node):
    def __init__(self):
        super().__init__('image_recorder_node')
        self.subscription = self.create_subscription(
            Image,
            '/carla/ego_vehicle/camera/rgb/front/image', 
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info('Node started, waiting for images...')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Image Recorder", cv_image)
            key = cv2.waitKey(1)
            if key == 13:
                cv2.imwrite('capture.jpg', cv_image)
                self.get_logger().info('Saved as capture.jpg')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    image_recorder = ImageRecorder()
    rclpy.spin(image_recorder)
    image_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
