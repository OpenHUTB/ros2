import rclpy
from rclpy.executors import ExternalShutdownException
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
            Image,
            '/auv/camera/camera/image_raw',
            self.img_cb,
            10)
        self.bridge = CvBridge()

        # 避障参数
        self.obstacle_thresh = 22      # 灰度阈值：低于该值判为暗色障碍像素
        self.trigger_ratio = 0.22      # 触发阈值：暗像素面积占比超过该值触发避障
        self.recover_ratio = 0.18      # 恢复阈值：双侧占比均低于该值恢复直行
        self.forward_speed = 4.5       # 直行巡航速度
        self.side_speed = 2.8          # 横向避让速度
        self.is_avoiding = False       # 避障状态标记
        self.avoid_start_time = 0.0    # 避障开始时间（秒）
        self.max_avoid_duration = 3.0  # 单次最大避让时长，超时进入冷却期
        self.avoid_direction = 1       # 避让方向默认值
        # 冷却期参数：超时后强制直行，避免反复触发
        self.is_cooldown = False       # 冷却状态标记
        self.cooldown_start_time = 0.0  # 冷却开始时间（秒）
        self.avoid_cooldown = 2.0      # 冷却时长，期间强制直行

    def img_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            height, width = cv_img.shape[:2]

            # 取画面中上部区域，剔除海底地面干扰
            roi = cv_img[int(height * 0.15):int(height * 0.7), :]
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

            # === 拆分左右双区域，分别计算障碍物占比 ===
            mid_w = width // 2
            roi_left = gray[:, :mid_w]  # 左半画面
            roi_right = gray[:, mid_w:]  # 右半画面

            # 左区域障碍物占比
            left_obstacle = np.sum(roi_left < self.obstacle_thresh)
            left_total = roi_left.size
            left_ratio = left_obstacle / left_total

            # 右区域障碍物占比
            right_obstacle = np.sum(roi_right < self.obstacle_thresh)
            right_total = roi_right.size
            right_ratio = right_obstacle / right_total

            self.get_logger().debug(
                f'左占比: {left_ratio:.3f} | 右占比: {right_ratio:.3f}')

            current_time = self.get_clock().now().nanoseconds / 1e9

            # 触发条件：任一侧超过触发阈值
            obstacle_detected = (
                left_ratio > self.trigger_ratio
                or right_ratio > self.trigger_ratio
            )
            # 恢复条件：两侧都低于恢复阈值
            obstacle_cleared = (
                left_ratio < self.recover_ratio
                and right_ratio < self.recover_ratio
            )

            # 双向避障状态机（仅在非冷却期触发）
            if not self.is_cooldown and obstacle_detected:
                # 每次都更新避让方向，保证变量永远有值
                if left_ratio > right_ratio:
                    self.avoid_direction = -1  # 左侧障碍物多 → 向右避让
                else:
                    self.avoid_direction = 1   # 右侧障碍物多 → 向左避让

                # 首次进入避障状态，只打印一次日志
                if not self.is_avoiding:
                    self.avoid_start_time = current_time
                    self.is_avoiding = True
                    if self.avoid_direction == -1:
                        self.get_logger().info('=== 触发避障 向右避让 ===')
                    else:
                        self.get_logger().info('=== 触发避障 向左避让 ===')

            # 障碍物躲开且双侧低于恢复阈值 → 恢复直行
            elif obstacle_cleared and self.is_avoiding:
                self.is_avoiding = False
                self.get_logger().info('=== 障碍物清除 恢复直行 ===')

            # 超时兜底：不依赖占比是否回落到恢复阈值，
            # 占比落在 (恢复阈值, 触发阈值] 的滞环带内也按时结束避让
            avoid_elapsed = current_time - self.avoid_start_time
            if self.is_avoiding and avoid_elapsed > self.max_avoid_duration:
                self.is_avoiding = False
                self.is_cooldown = True
                self.cooldown_start_time = current_time
                self.get_logger().info('=== 避让超时 进入冷却期 强制直行 ===')

            # 冷却期结束：恢复正常避障检测
            cooldown_elapsed = current_time - self.cooldown_start_time
            if self.is_cooldown and cooldown_elapsed > self.avoid_cooldown:
                self.is_cooldown = False
                self.get_logger().info('=== 冷却结束 恢复避障检测 ===')

            # 速度输出
            vel = Twist()
            if self.is_avoiding:
                # 避障：减速 + 横向避让
                vel.linear.x = self.forward_speed * 0.9
                vel.linear.y = self.avoid_direction * self.side_speed
            else:
                # 无障碍物/冷却期：恢复直行
                vel.linear.x = self.forward_speed
                vel.linear.y = 0.0

            self.vel_pub.publish(vel)

        except Exception as e:
            self.get_logger().error(f'图像处理失败: {str(e)}')

    def destroy_node(self):
        # 退出前发布零速度，避免 AUV 保持最后一帧速度继续运动
        stop_vel = Twist()
        self.vel_pub.publish(stop_vel)
        self.get_logger().info('避障节点停止，发布零速度指令')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AuvObstacleAvoid()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
