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
        self.obstacle_thresh = 75    # 暗色阈值 覆盖深灰-纯黑
        self.ratio_thresh = 0.08    # 触发阈值 更低更容易触发
        self.forward_speed = 0.5     # 直行速度
        self.side_speed = 0.55        # 横向避让速度

    def img_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            height, width = cv_img.shape[:2]
            
            # 取画面中间到下部的大范围区域 确保障碍物进入视野就检测到
            roi = cv_img[int(height*0.15):int(height*0.9), :]
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            obstacle_pixels = np.sum(gray < self.obstacle_thresh)
            total_pixels = gray.size
            ratio = obstacle_pixels / total_pixels

            # 实时打印占比 方便调试
            self.get_logger().info(f'障碍物占比: {ratio:.3f}')

            vel = Twist()
            if ratio > self.ratio_thresh:
                # 检测到障碍物：减速+右移绕行
                vel.linear.x = self.forward_speed * 0.8
                vel.linear.y = -self.side_speed
                self.get_logger().info('=== 触发避障 右移避让 ===')
            else:
                # 无障碍物：直行
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

