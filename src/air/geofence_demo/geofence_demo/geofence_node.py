# Copyright 2026 Your Name
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped
from std_msgs.msg import String

class GeofenceNode(Node):
    def __init__(self):
        super().__init__('geofence_node')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.geofence_sub = self.create_subscription(PolygonStamped, '/geofence', self.geofence_callback, 10)
        self.status_pub = self.create_publisher(String, '/geofence_status', 10)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.polygon_points = []
        self.get_logger().info('🛡️ 禁飞区检测节点已启动，等待数据...')

    def geofence_callback(self, msg):
        self.polygon_points = [(p.x, p.y) for p in msg.polygon.points]
        self.get_logger().info(f'📥 收到新的禁飞区配置，共 {len(self.polygon_points)} 个顶点')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        if not self.polygon_points:
            return

        # 射线法 (Ray Casting) 检测
        inside = False
        n = len(self.polygon_points)
        p1x, p1y = self.polygon_points[0]
        for i in range(1, n + 1):
            p2x, p2y = self.polygon_points[i % n]
            if self.current_y > min(p1y, p2y):
                if self.current_y <= max(p1y, p2y):
                    if self.current_x <= max(p1x, p2x):
                        if p1y != p2y:
                            xints = (self.current_y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or self.current_x <= xints:
                            inside = not inside
            p1x, p1y = p2x, p2y

        status_msg = String()
        if inside:
            status_msg.data = f'🚨 [CRITICAL] 闯入禁飞区！坐标: ({self.current_x:.1f}, {self.current_y:.1f})'
            self.get_logger().error(status_msg.data)
        else:
            status_msg.data = f'✅ [SAFE] 安全。坐标: ({self.current_x:.1f}, {self.current_y:.1f})'
            self.get_logger().info(status_msg.data)
            
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GeofenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
