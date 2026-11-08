#!/usr/bin/env python3
"""CARLA Town04 parking lot scene generation node."""

import carla
import time
import rclpy
from rclpy.node import Node


class ParkingLotNode(Node):
    def __init__(self):
        super().__init__('parking_lot_node')

        self.declare_parameter('host', '192.168.43.1')
        self.declare_parameter('port', 2000)
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value

        self.get_logger().info(f'Connecting to CARLA {host}:{port} ...')
        client = carla.Client(host, port)
        client.set_timeout(30.0)

        self.get_logger().info('Loading Town04 ...')
        world = client.load_world('Town04')
        time.sleep(2)

        cx = 280.1
        cy = -223.7
        cz = 0.3
        slot_width = 2.8
        slot_length = 5.5
        num_slots = 6

        self.get_logger().info('Drawing parking slots ...')
        for i in range(num_slots):
            sy = cy + i * slot_width
            p1 = carla.Location(cx - slot_length / 2, sy - slot_width / 2, cz)
            p2 = carla.Location(cx + slot_length / 2, sy - slot_width / 2, cz)
            p3 = carla.Location(cx + slot_length / 2, sy + slot_width / 2, cz)
            p4 = carla.Location(cx - slot_length / 2, sy + slot_width / 2, cz)

            world.debug.draw_line(p1, p2, thickness=0.1,
                                  color=carla.Color(0, 255, 0), life_time=0)
            world.debug.draw_line(p2, p3, thickness=0.1,
                                  color=carla.Color(0, 255, 0), life_time=0)
            world.debug.draw_line(p3, p4, thickness=0.1,
                                  color=carla.Color(0, 255, 0), life_time=0)
            world.debug.draw_line(p4, p1, thickness=0.1,
                                  color=carla.Color(0, 255, 0), life_time=0)

            world.debug.draw_string(
                carla.Location(cx, sy, cz + 0.5),
                f"P{i + 1}",
                color=carla.Color(255, 255, 0),
                life_time=0
            )

        self.get_logger().info(f'{num_slots} slots drawn')

        spectator = world.get_spectator()
        spectator.set_transform(carla.Transform(
            carla.Location(x=cx, y=cy, z=15),
            carla.Rotation(pitch=-90, yaw=0)
        ))
        self.get_logger().info('Camera moved to top view')
        self.get_logger().info('Parking lot scene generated! Node staying alive ...')


def main(args=None):
    rclpy.init(args=args)
    node = ParkingLotNode()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
