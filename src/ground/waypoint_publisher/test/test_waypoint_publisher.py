import rclpy
from waypoint_publisher.waypoint_pub import WaypointPublisher

def test_timer_callback_publishes_path():
    rclpy.init()
    node = WaypointPublisher()
    # 直接手动执行一次回调，验证不会抛字段错误
    node.timer_callback()
    # 销毁节点
    node.destroy_node()
    rclpy.shutdown()
    print("Path生成测试通过，无AttributeError")
