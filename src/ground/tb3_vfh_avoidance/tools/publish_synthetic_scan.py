"""向 /scan 发布仿真激光数据，用于无 Gazebo 环境下联调避障节点。

场景脚本（每 4 s 切换一次）：
1. 开阔区：机器人位于 (1, 1, 45°)，前方无障碍 -> 期望 cruise；
2. 面壁：机器人位于 (2.8, 4.0, 90°)，正对 U 形陷阱左墙 -> 期望 avoid/blocked；
3. 窄巷：机器人位于 (4.1, 3.2, 0°)，位于 U 形陷阱内部 -> 期望绕行/转向。

用法::

    python tools/publish_synthetic_scan.py [--seconds 16]
"""

import argparse
import math
import os
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tb3_vfh_avoidance.sim_world import make_maze  # noqa: E402

SCENES = [
    ('开阔区', (1.0, 1.0, math.radians(45.0))),
    ('面壁', (2.8, 4.0, math.radians(90.0))),
    ('窄巷', (4.1, 3.2, 0.0)),
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=16.0)
    parser.add_argument('--topic', default='/scan')
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.node.Node('vfh_scan_sim')
    world = make_maze()
    pub = node.create_publisher(LaserScan, args.topic, 10)

    import time
    start = time.time()
    scene_index = 0
    print('开始发布仿真扫描（%.0f s，%d 个场景）' % (args.seconds, len(SCENES)))
    while rclpy.ok() and time.time() - start < args.seconds:
        name, (x, y, theta) = SCENES[scene_index % len(SCENES)]
        ranges, angle_min, inc = world.scan(x, y, theta)
        msg = LaserScan()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'
        msg.angle_min = angle_min
        msg.angle_max = angle_min + inc * (len(ranges) - 1)
        msg.angle_increment = inc
        msg.range_min = 0.12
        msg.range_max = 3.5
        msg.ranges = [float(r) for r in ranges]
        pub.publish(msg)
        if scene_index != (int((time.time() - start) // 4) % len(SCENES)):
            scene_index = int((time.time() - start) // 4) % len(SCENES)
            print('切换场景 -> %s pose=%s' % (name, (round(x, 2), round(y, 2))))
        time.sleep(0.1)

    node.destroy_node()
    rclpy.shutdown()
    print('发布结束')


if __name__ == '__main__':
    main()
