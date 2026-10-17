import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import threading


class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_uav_control')
        
        # 1. 创建发布者，发布到 /cmd_vel 话题
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("WASD控制：W前进 S后退 A左转 D右转，空格停止，Ctrl+C退出")
        
        # 2. 初始化速度消息对象
        self.msg = Twist()
        
        # 3. 定义速度上限（边界参数）
        self.MAX_LINEAR = 0.2   # 最大线速度 0.2 m/s
        self.MAX_ANGULAR = 0.3  # 最大角速度 0.3 rad/s

        # 4. 启动键盘监听线程 (daemon=True 表示主程序退出时该线程自动结束)
        self.key_thread = threading.Thread(target=self.key_loop, daemon=True)
        self.key_thread.start()

        # 5. 创建一个 10Hz (0.1秒) 的定时器，持续发布速度指令，防止飞控触发看门狗
        self.timer = self.create_timer(0.1, self.timer_callback)

    def get_key(self):
        """读取终端单个字符，无阻塞式设置"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def key_loop(self):
        """键盘按键读取与速度映射逻辑"""
        while rclpy.ok():
            key = self.get_key()
            
            if key == 'w' or key == 'W':
                linear_x = self.MAX_LINEAR
                angular_z = 0.0
            elif key == 's' or key == 'S':
                linear_x = -self.MAX_LINEAR
                angular_z = 0.0
            elif key == 'a' or key == 'A':
                linear_x = 0.0
                angular_z = self.MAX_ANGULAR
            elif key == 'd' or key == 'D':
                linear_x = 0.0
                angular_z = -self.MAX_ANGULAR
            elif key == ' ':
                linear_x = 0.0
                angular_z = 0.0
            elif key == '\x03': # 捕获 Ctrl+C
                break
            else:
                # 遇到无效按键，跳过本次循环，保持当前速度
                continue

            # 边界钳位保护（确保即使参数配错，也不会输出超限速度）
            linear_x = max(-self.MAX_LINEAR, min(linear_x, self.MAX_LINEAR))
            angular_z = max(-self.MAX_ANGULAR, min(angular_z, self.MAX_ANGULAR))

            # 更新目标速度（仅更新缓存，由定时器负责发布）
            self.msg.linear.x = linear_x
            self.msg.angular.z = angular_z

    def timer_callback(self):
        """定时器回调，以固定频率发送 Twist 消息"""
        self.pub.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    
    try:
        rclpy.spin(node) # 主线程进入 ROS 事件循环
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前发送一次零速度，确保安全（Fail-safe 保护）
        node.get_logger().info("节点退出，发送零速度指令...")
        node.msg.linear.x = 0.0
        node.msg.angular.z = 0.0
        node.pub.publish(node.msg)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
