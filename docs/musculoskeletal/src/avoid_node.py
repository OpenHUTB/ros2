import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtlePoseAvoid(Node):
    def __init__(self):
        super().__init__("turtle_pose_avoid")
        # 订阅机器人位姿话题
        self.pose_sub = self.create_subscription(Pose,"/turtle1/pose",self.pose_callback,10)
        # 发布速度控制指令
        self.vel_pub = self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.twist = Twist()
        self.get_logger().info("✅肌肉骨骼人形机器人避障节点启动成功")

    def pose_callback(self, msg):
        # 仿真空间范围 x∈[0,11], y∈[0,11]
        x = msg.x
        y = msg.y
        threshold = 1.0 # 安全距离阈值

        # 边界障碍物判断逻辑
        if x < threshold or x > (11-threshold) or y < threshold or y > (11-threshold):
            # 靠近障碍：停止前进，原地旋转
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.4
        else:
            # 安全区域，直行
            self.twist.linear.x = 0.3
            self.twist.angular.z = 0.0
        self.vel_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePoseAvoid()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
