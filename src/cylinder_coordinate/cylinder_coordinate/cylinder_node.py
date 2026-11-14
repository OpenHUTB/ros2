import rclpy
from rclpy.node import Node
import math
from datetime import datetime

class CylinderCoordNode(Node):
    def __init__(self):
        super().__init__('cylinder_coord_node')
        # 定时器：每0.5秒执行一次
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.angle = 0.0
        self.radius = 1.0 #圆柱半径
        #打开txt文件，追加写入
        self.file = open("cylinder_data.txt", "a", encoding="utf-8")

    def timer_callback(self):
        # 圆柱坐标转直角坐标
        x = self.radius * math.cos(self.angle)
        y = self.radius * math.sin(self.angle)
        z = 0.2
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        #终端打印
        log_msg = f"时间:{timestamp}, x={x:.3f}, y={y:.3f}, z={z:.2f}"
        self.get_logger().info(log_msg)
        #写入txt
        self.file.write(log_msg + "\n")

        self.angle += 0.1
        if self.angle > 2 * math.pi:
            self.angle = 0.0

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CylinderCoordNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
