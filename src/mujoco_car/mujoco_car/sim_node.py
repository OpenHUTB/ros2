import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

xml = """
<mujoco>
  <visual>
    <global offwidth="320" offheight="240"/>
  </visual>
  <worldbody>
    <geom name="floor" type="plane" size="10 10 0.1" rgba="0.9 0.9 0.9 1"/>
    <body name="car" pos="0 0 0.2">
      <geom type="box" size="0.3 0.2 0.1" rgba="0.2 0.6 0.8 1"/>
      <joint type="free"/>
      <camera name="cam" pos="0.2 0 0" zaxis="1 0 0"/>
      <body pos="-0.2 -0.2 0">
        <geom type="cylinder" size="0.1 0.05" euler="0 1.5708 0"/>
      </body>
      <body pos="-0.2 0.2 0">
        <geom type="cylinder" size="0.1 0.05" euler="0 1.5708 0"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="car" gear="0"/>
  </actuator>
</mujoco>
"""

class MuJoCoSimNode(Node):
    def __init__(self):
        super().__init__('mujoco_sim_node')
        self.pub_img = self.create_publisher(Image, '/rgb_image', 10)
        self.bridge = CvBridge()
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, width=320, height=240)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.timer = self.create_timer(0.033, self.update_callback)

    def update_callback(self):
        mujoco.mj_step(self.model, self.data)
        self.renderer.update_scene(self.data, camera="cam")
        rgb = self.renderer.render()
        msg = self.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
        self.pub_img.publish(msg)
        self.viewer.sync()

def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.viewer.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()