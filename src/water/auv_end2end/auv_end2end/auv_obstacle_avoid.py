import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time


class AuvObstacleAvoid(Node):
    def __init__(self):
        super().__init__('auv_obstacle_avoid')

        # 速度发布器
        self.vel_pub = self.create_publisher(Twist, '/auv/cmd_vel', 10)

        # 相机订阅
        self.img_sub = self.create_subscription(
            Image,
            '/auv/camera/camera/image_raw',
            self.img_cb,
            10)

        self.bridge = CvBridge()

        # 避障参数
        self.obstacle_thresh = 22
        self.trigger_ratio = 0.22
        self.recover_ratio = 0.18
        self.forward_speed = 4.5
        self.side_speed = 2.8
        self.is_avoiding = False
        self.avoid_start_time = 0.0
        self.max_avoid_duration = 3.0
        self.avoid_direction = 1




    def img_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            height, width = cv_img.shape[:2]

            # 取画面中上部区域，剔除海底地面干扰
            roi = cv_img[int(height*0.15):int(height*0.7), :]
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

            # === 拆分左右双区域，分别计算障碍物占比 ===
            mid_w = width // 2
            roi_left = gray[:, :mid_w]   # 左半画面
            roi_right = gray[:, mid_w:]  # 右半画面

            # 左区域障碍物占比
            left_obstacle = np.sum(roi_left < self.obstacle_thresh)
            left_total = roi_left.size
            left_ratio = left_obstacle / left_total

            # 右区域障碍物占比
            right_obstacle = np.sum(roi_right < self.obstacle_thresh)
            right_total = roi_right.size
            right_ratio = right_obstacle / right_total

            # 调试用占比日志，默认不打印
            self.get_logger().debug(f'左占比: {left_ratio:.3f} | 右占比: {right_ratio:.3f}')

            vel = Twist()

            # 双向避障状态机
            if left_ratio > self.trigger_ratio or right_ratio > self.trigger_ratio:
                # 每次都更新避让方向，保证变量永远有值
                if left_ratio > right_ratio:
                    self.avoid_direction = -1  # 左侧障碍物多 → 向右避让
                else:
                    self.avoid_direction = 1   # 右侧障碍物多 → 向左避让

                # 首次进入避障状态，只打印一次日志
                if not self.is_avoiding:
                    self.avoid_start_time = self.get_clock().now().seconds_nanoseconds()[0]
                    self.is_avoiding = True
                    if self.avoid_direction == -1:
                        self.get_logger().info('=== 触发避障 向右避让 ===')
                    else:
                        self.get_logger().info('=== 触发避障 向左避让 ===')

                # 短超时兜底：最多避让1.5秒就回正，防止躲太远
                current_time = self.get_clock().now().seconds_nanoseconds()[0]
                if self.is_avoiding and (current_time - self.avoid_start_time) > self.max_avoid_duration:
                    self.is_avoiding = False
                    self.get_logger().info('=== 避让结束 恢复直行 ===')

            else:
                # 两侧都低于恢复阈值 → 障碍物躲开了，恢复直行
                if left_ratio < self.recover_ratio and right_ratio < self.recover_ratio and self.is_avoiding:
                    self.is_avoiding = False
                    self.get_logger().info('=== 障碍物清除 恢复直行 ===')

            # 速度输出
            if self.is_avoiding:
                # 避障：减速 + 横向避让
                vel.linear.x = self.forward_speed * 0.9
                vel.linear.y = self.avoid_direction * self.side_speed
            else:
                # 无障碍物：恢复直行
                vel.linear.x = self.forward_speed
                vel.linear.y = 0.0

            self.vel_pub.publish(vel)

        except Exception as e:
            self.get_logger().error(f'图像处理失败: {str(e)}')


def main():
    rclpy.init()
    node = AuvObstacleAvoid()
    rclpy.spin(node)
    rclpy.destroy_node(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()



