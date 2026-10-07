#!/usr/bin/env python3
import os
import rclpy
import numpy as np
import cv2
import mujoco
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped

BASE = os.path.dirname(os.path.abspath(__file__))

def quat_to_euler(q):
    w, x, y, z = q
    roll = np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = np.arcsin(np.clip(2*(w*y-z*x), -1, 1))
    return roll, pitch

class UAVNode(Node):
    def __init__(self):
        super().__init__('uav_mujoco_keyboard')
        self.pub_pose = self.create_publisher(PoseStamped, '/uav/pose', 10)
        self.sub_target = self.create_subscription(Float64MultiArray, '/uav/target', self.on_target, 10)
        self.target = np.array([0.0, 0.0, 1.0])
        self.model = mujoco.MjModel.from_xml_path(os.path.join(BASE, 'models/drone.xml'))
        self.data = mujoco.MjData(self.model)
        self.g = self.model.body_mass[1] * 9.81
        self.Kp_z, self.Kd_z = 30.0, 8.0
        self.Kp_xy, self.Kd_xy = 10.0, 4.0
        self.Kp_a, self.Kd_a = 2.0, 0.5

    def on_target(self, msg):
        if len(msg.data) >= 3:
            self.target = np.array(msg.data[:3])
            self.get_logger().info(f"收到新目标: {self.target}")

    def run(self):
        renderer = mujoco.Renderer(self.model, height=480, width=640)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            for _ in range(10):
                mujoco.mj_step(self.model, self.data)
            pos = self.data.xpos[1]
            vel = self.data.cvel[1]
            roll, pitch = quat_to_euler(self.data.xquat[1])
            F = np.clip(self.g + self.Kp_z*(self.target[2]-pos[2]) - self.Kd_z*vel[5], 0, 20)
            Fx = self.Kp_xy*(self.target[0]-pos[0]) - self.Kd_xy*vel[3]
            Fy = self.Kp_xy*(self.target[1]-pos[1]) - self.Kd_xy*vel[4]
            Mx = -self.Kp_a*roll - self.Kd_a*vel[0]
            My = -self.Kp_a*pitch - self.Kd_a*vel[1]
            self.data.xfrc_applied[1] = [Fx, Fy, F, Mx, My, 0.0]
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = float(pos[0])
            msg.pose.position.y = float(pos[1])
            msg.pose.position.z = float(pos[2])
            msg.pose.orientation.w = float(self.data.xquat[1][0])
            msg.pose.orientation.x = float(self.data.xquat[1][1])
            msg.pose.orientation.y = float(self.data.xquat[1][2])
            msg.pose.orientation.z = float(self.data.xquat[1][3])
            self.pub_pose.publish(msg)
            renderer.update_scene(self.data, camera="view")
            img = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
            cv2.putText(img, f"xyz={pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow("drone", img)
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('w'): self.target[0] += 0.3
            elif key == ord('s'): self.target[0] -= 0.3
            elif key == ord('a'): self.target[1] -= 0.3
            elif key == ord('d'): self.target[1] += 0.3
            elif key == ord('r'): self.target[2] += 0.3
            elif key == ord('f'): self.target[2] -= 0.3
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    node = UAVNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
