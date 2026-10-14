import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class MusculoskeletalRobotAvoidNode(Node):
    def __init__(self):
        super().__init__("musculoskeletal_avoid_node")
        # 订阅机器人位姿话题
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        # 发布速度控制指令话题
        self.vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel",10)
        self.get_logger().info("✅肌肉骨骼人形机器人避障节点启动成功")

    def pose_callback(self, msg:Pose):
        vel_msg = Twist()
        safe_threshold = 1.0
        # 边界障碍物检测逻辑
        if msg.x < safe_threshold or msg.x > (11-safe_threshold) or msg.y < safe_threshold or msg.y > (11-safe_threshold):
            # 靠近障碍物：停止前进，原地旋转避开障碍
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.4
        else:
            # 处于安全区域，保持匀速直行
            vel_msg.linear.x = 0.3
            vel_msg.angular.z = 0.0
        self.vel_pub.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MusculoskeletalRobotAvoidNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
