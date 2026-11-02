import math
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from service_interfaces.srv import MoveForward


class MoveForwardServer(Node):

    def __init__(self):
        super().__init__('move_forward_server')

        self.callback_group = ReentrantCallbackGroup()

        self.publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.current_pose = None

        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10,
            callback_group=self.callback_group
        )

        self.service = self.create_service(
            MoveForward,
            '/move_forward',
            self.handle_move_forward,
            callback_group=self.callback_group
        )

        self.get_logger().info(
            'MoveForward service is ready.'
        )

    def pose_callback(self, msg):
        self.current_pose = msg

    def handle_move_forward(self, request, response):
        distance = request.distance

        if distance <= 0:
            response.success = False
            response.message = 'Distance must be greater than 0.'
            return response

        wait_start = time.time()

        while self.current_pose is None:
            if time.time() - wait_start > 3.0:
                response.success = False
                response.message = 'Waiting for turtle pose timed out.'
                return response

            time.sleep(0.05)

        start_x = self.current_pose.x
        start_y = self.current_pose.y

        self.get_logger().info(
            f'Start position: x={start_x:.2f}, y={start_y:.2f}'
        )

        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.0

        self.get_logger().info(
            f'Moving forward {distance:.2f} meters...'
        )

        start_time = time.time()
        moved_distance = 0.0

        while True:
            self.publisher.publish(twist)

            if self.current_pose is not None:
                current_x = self.current_pose.x
                current_y = self.current_pose.y

                moved_distance = math.sqrt(
                    (current_x - start_x) ** 2 +
                    (current_y - start_y) ** 2
                )

                if moved_distance >= distance:
                    break

            if time.time() - start_time > 20.0:
                stop_twist = Twist()
                self.publisher.publish(stop_twist)

                response.success = False
                response.message = (
                    f'Movement timed out after moving '
                    f'{moved_distance:.2f} meters.'
                )

                self.get_logger().warn(response.message)
                return response

            time.sleep(0.05)

        stop_twist = Twist()
        self.publisher.publish(stop_twist)

        response.success = True
        response.message = (
            f'Moved {moved_distance:.2f} meters.'
        )

        self.get_logger().info(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)

    node = MoveForwardServer()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
