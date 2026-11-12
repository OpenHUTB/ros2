import sys

import rclpy
from rclpy.node import Node

from service_interfaces.srv import MoveForward


class MoveForwardClient(Node):

    def __init__(self):
        super().__init__('move_forward_client')

        self.client = self.create_client(
            MoveForward,
            '/move_forward'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'Waiting for /move_forward service...'
            )

    def send_request(self, distance):
        request = MoveForward.Request()
        request.distance = distance

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result()


def main(args=None):
    rclpy.init(args=args)

    node = MoveForwardClient()

    if len(sys.argv) < 2:
        node.get_logger().error(
            'Usage: ros2 run service_demo client <distance>'
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    distance = float(sys.argv[1])

    response = node.send_request(distance)

    node.get_logger().info(
        f'Success: {response.success}'
    )
    node.get_logger().info(
        f'Message: {response.message}'
    )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
