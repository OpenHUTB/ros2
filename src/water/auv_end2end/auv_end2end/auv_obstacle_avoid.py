import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class AuvObstacleAvoid(Node):
    def __init__(self):
        super().__init__('auv_obstacle_avoid')

        # 速度发布器
        self.vel_pub = self.create_publisher(Twist, '/auv/cmd_vel', 10)
        # 相机订阅
        self.img_sub = self.create_subscription(
            Image, '/auv/camera/camera/image_raw', self.img_cb, 10)

        self.bridge = CvBridge()

        # 避障参数
        self.obstacle_thresh = 22       # 暗色阈值：大幅降低，只识别纯黑墙体，过滤水下背景
        self.trigger_ratio = 0.12       # 触发避障阈值
        self.recover_ratio = 0.06       # 恢复直行阈值
        self.forward_speed = 2.2        # 直行巡航速度
        self.side_speed = 1.8           # 横向避让速度
        self.is_avoiding = False        # 避障状态标记
        self.avoid_start_time = 0.0     # 避障开始时间（秒）
        self.max_avoid_duration = 2.0   # 最大避让时长，超过强制恢复直行

    def img_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            height, width = cv_img.shape[:2]

            # 取画面中上部区域，剔除海底地面干扰
            roi = cv_img[int(height*0.15):int(height*0.7), :]
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            
            # === 调试用：打印画面平均灰度，方便调阈值 ===
            avg_gray = np.mean(gray)
            self.get_logger().info(f'画面平均灰度: {avg_gray:.1f}')

            obstacle_pixels = np.sum(gray < self.obstacle_thresh)
            total_pixels = gray.size
            ratio = obstacle_pixels / total_pixels

            # 实时打印占比 方便调试
            self.get_logger().info(f'障碍物占比: {ratio:.3f}')

            vel = Twist()

            # 滞环状态机：判断避障触发与恢复
            if ratio > self.trigger_ratio:
                # 首次进入避障状态，记录开始时间
                if not self.is_avoiding:
                    self.avoid_start_time = self.get_clock().now().seconds_nanoseconds()[0]
                self.is_avoiding = True
            elif ratio < self.recover_ratio:
                self.is_avoiding = False

            # 超时强制恢复直行，防止持续横向跑偏
            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            if self.is_avoiding and (current_time - self.avoid_start_time) > self.max_avoid_duration:
                self.is_avoiding = False
                self.get_logger().info('=== 避障超时 强制恢复直行 ===')

            if self.is_avoiding:
                # 触发避障：减速+右移避让
                vel.linear.x = self.forward_speed * 0.9
                vel.linear.y = -self.side_speed
                self.get_logger().info('=== 触发避障 右移避让 ===')
            else:
                # 无障碍物/已躲开/超时：恢复直行
                vel.linear.x = self.forward_speed
                vel.linear.y = 0.0

            self.vel_pub.publish(vel)

        except Exception as e:
            self.get_logger().error(f'图像处理失败: {str(e)}')


def main():
    rclpy.init()
    node = AuvObstacleAvoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

