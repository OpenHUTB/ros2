import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class AuvVelController(Node):
    def __init__(self):
        super().__init__('auv_vel_ctrl')
        # 等待Gazebo服务
        self.cli = self.create_client(SetModelState, '/gazebo/set_model_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待Gazebo服务...')
        
        # 订阅速度指令
        self.sub = self.create_subscription(
            Twist, '/auv/cmd_vel', self.cmd_cb, 10)
        
        self.model_name = 'auv'

    def cmd_cb(self, msg):
        req = SetModelState.Request()
        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = 'world'
        
        # 设置速度
        state.twist.linear.x = msg.linear.x
        state.twist.angular.z = msg.angular.z
        
        req.model_state = state
        self.cli.call_async(req)

def main():
    rclpy.init()
    node = AuvVelController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
