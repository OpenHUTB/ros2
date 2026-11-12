"""ROS 2 节点：以固定频率推进势场导航仿真，并发布位姿与轨迹。

节点本身不做渲染，也不依赖显示设备，因此在没有 X11 / Wayland 的机器上
（例如 CI、SSH 会话、Docker 容器）可以直接用 ``ros2 launch`` 或 ``ros2 run``
启动；想在屏幕上观察轨迹请用 ``python main.py``（pygame 演示入口）。

话题:
    /potential_field/pose   geometry_msgs/PoseStamped  当前位姿（map 坐标系，米）
    /potential_field/trail  nav_msgs/Path              已走过的轨迹
"""

import math


def main(args=None):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from rclpy.node import Node

    from . import config
    from .robot import Robot

    class PotentialFieldNode(Node):
        def __init__(self):
            super().__init__("potential_field_navigation")

            self.declare_parameter("sim_rate", float(config.FPS))
            self.declare_parameter("max_steps", config.MAX_STEPS)
            self.declare_parameter("px_per_meter", 100.0)

            self.max_steps = int(self.get_parameter("max_steps").value)
            self.scale = float(self.get_parameter("px_per_meter").value)
            if self.scale <= 0.0:
                raise ValueError("px_per_meter 必须大于 0")

            self.robot = Robot()
            self._tick = 0
            self._finished_logged = False

            self.pose_pub = self.create_publisher(
                PoseStamped, "/potential_field/pose", 10)
            self.path_pub = self.create_publisher(
                Path, "/potential_field/trail", 10)

            rate = float(self.get_parameter("sim_rate").value)
            if rate <= 0.0:
                raise ValueError("sim_rate 必须大于 0")
            self.timer = self.create_timer(1.0 / rate, self._on_timer)
            self.get_logger().info(
                "势场导航节点启动：sim_rate=%.1f Hz, max_steps=%d, px_per_meter=%.1f"
                % (rate, self.max_steps, self.scale))

        def _on_timer(self):
            if not self.robot.done:
                if self.robot.steps < self.max_steps:
                    self.robot.step()
                else:
                    self.robot.done = True
                    self.robot.status = "timeout"

            self._tick += 1
            self.pose_pub.publish(self._pose_msg())
            if self._tick % 10 == 0:
                self.path_pub.publish(self._path_msg())

            if self.robot.done and not self._finished_logged:
                self.path_pub.publish(self._path_msg())
                self._finished_logged = True
                self.get_logger().info(
                    "仿真结束：状态=%s, 步数=%d, 距目标=%.2f px, 最小间隙=%.2f px"
                    % (self.robot.status, self.robot.steps,
                       self.robot.distance(), self.robot.min_clearance))

        def _pose_msg(self):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.x = float(self.robot.p[0]) / self.scale
            msg.pose.position.y = float(self.robot.p[1]) / self.scale
            msg.pose.position.z = 0.0
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = math.sin(self.robot.heading / 2.0)
            msg.pose.orientation.w = math.cos(self.robot.heading / 2.0)
            return msg

        def _path_msg(self):
            msg = Path()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            for px, py in self.robot.trail:
                stamped = PoseStamped()
                stamped.header = msg.header
                stamped.pose.position.x = float(px) / self.scale
                stamped.pose.position.y = float(py) / self.scale
                stamped.pose.position.z = 0.0
                stamped.pose.orientation.w = 1.0
                msg.poses.append(stamped)
            return msg
    rclpy.init(args=args)
    node = PotentialFieldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
