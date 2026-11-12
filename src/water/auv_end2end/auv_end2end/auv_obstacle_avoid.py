import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from auv_end2end import avoid_logic


class AuvObstacleAvoid(Node):
    EVENT_LOG = {
        avoid_logic.AvoidStateMachine.EVENT_ENTER_RIGHT:
            '=== 触发避障 向右避让 ===',
        avoid_logic.AvoidStateMachine.EVENT_ENTER_LEFT:
            '=== 触发避障 向左避让 ===',
        avoid_logic.AvoidStateMachine.EVENT_CLEARED:
            '=== 障碍物清除 恢复直行 ===',
        avoid_logic.AvoidStateMachine.EVENT_TIMEOUT:
            '=== 避让超时 进入冷却期 强制直行 ===',
        avoid_logic.AvoidStateMachine.EVENT_COOLDOWN_END:
            '=== 冷却结束 恢复避障检测 ===',
        avoid_logic.AvoidStateMachine.EVENT_EMERGENCY:
            '=== 冷却期紧急重触发 恢复避让 ===',
    }

    def __init__(self):
        super().__init__('auv_obstacle_avoid')
        self.vel_pub = self.create_publisher(Twist, '/auv/cmd_vel', 10)
        self.img_sub = self.create_subscription(
            Image,
            '/auv/camera/camera/image_raw',
            self.img_cb,
            10)
        self.bridge = CvBridge()

        # 全部参数通过 ROS 参数暴露，可在 launch 或命令行覆盖
        self.declare_parameter('obstacle_thresh', 22)
        self.declare_parameter('adaptive_thresh', True)
        self.declare_parameter('trigger_ratio', 0.22)
        self.declare_parameter('recover_ratio', 0.18)
        self.declare_parameter('forward_speed', 4.5)
        self.declare_parameter('side_speed', 2.8)
        self.declare_parameter('max_avoid_duration', 3.0)
        self.declare_parameter('avoid_cooldown', 2.0)
        self.declare_parameter('min_avoid_duration', 0.3)
        self.declare_parameter('emergency_factor', 1.5)

        self.obstacle_thresh = self.get_parameter('obstacle_thresh').value
        self.adaptive_thresh = self.get_parameter('adaptive_thresh').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.side_speed = self.get_parameter('side_speed').value

        self.fsm = avoid_logic.AvoidStateMachine(
            trigger_ratio=self.get_parameter('trigger_ratio').value,
            recover_ratio=self.get_parameter('recover_ratio').value,
            max_avoid_duration=self.get_parameter(
                'max_avoid_duration').value,
            cooldown=self.get_parameter('avoid_cooldown').value,
            min_avoid_duration=self.get_parameter(
                'min_avoid_duration').value,
            emergency_factor=self.get_parameter('emergency_factor').value,
        )

    def img_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            roi = avoid_logic.split_roi(cv_img)
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

            left_ratio, right_ratio, thresh = avoid_logic.analyze(
                gray,
                fixed_thresh=self.obstacle_thresh,
                adaptive=self.adaptive_thresh)

            self.get_logger().debug(
                f'左占比: {left_ratio:.3f} | 右占比: {right_ratio:.3f} '
                f'| 阈值: {thresh}')

            now = self.get_clock().now().nanoseconds / 1e9
            out = self.fsm.update(now, left_ratio, right_ratio)

            if out.event:
                self.get_logger().info(self.EVENT_LOG[out.event])

            vel = Twist()
            if out.is_avoiding:
                # 避障：减速 + 横向平移
                vel.linear.x = self.forward_speed * 0.9
                vel.linear.y = out.direction * self.side_speed
            else:
                vel.linear.x = self.forward_speed
                vel.linear.y = 0.0
            self.vel_pub.publish(vel)

        except Exception as e:
            self.get_logger().error(f'图像处理失败: {str(e)}')

    def publish_stop(self):
        # planar_move 没有 cmd_vel 超时，退出前必须显式发零速度
        self.vel_pub.publish(Twist())
        self.get_logger().info('避障节点停止，发布零速度指令')


def main(args=None):
    rclpy.init(args=args)
    node = AuvObstacleAvoid()

    def handle_sigint(signum, frame):
        # rclpy 默认的 SIGINT 处理会先关闭 context，之后 publish 会失败，
        # 这里只抛出异常，把发零速度的时机留到 finally
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, handle_sigint)
    try:
        # 用带超时的 spin_once 循环，收到 SIGINT 后最多 0.1 s 就能返回；
        # rclpy.spin 会一直阻塞在 C 层，launch 等不到退出会升级成 SIGKILL，
        # 那样 finally 里的零速度指令就发不出去
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 若 context 已被外部关闭（如 launch 统一退出），此时 publish 会失败，
        # 但仍要保证节点正常销毁
        try:
            node.publish_stop()
        except Exception:
            pass
        node.destroy_node()
        # 外部可能已经关闭 context，重复 shutdown 会抛异常
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
