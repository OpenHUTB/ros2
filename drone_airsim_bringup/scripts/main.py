#!/usr/bin/env python3
"""AirSim drone bringup for ROS Noetic.

Publishes:
  /drone/pose              geometry_msgs/PoseStamped (NED world)
  /drone/camera/image_raw  sensor_msgs/Image (bgr8), camera "0" by default

Subscribes:
  /drone/cmd_vel           geometry_msgs/Twist (NED velocity, m/s)

Services:
  /drone/takeoff           std_srvs/Empty
  /drone/land              std_srvs/Empty
"""

import threading

import numpy as np
import cv2
import rospy
import airsim

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import Empty, EmptyResponse
from cv_bridge import CvBridge, CvBridgeError


class DroneBringup(object):
    def __init__(self):
        self.host = rospy.get_param("~host", "127.0.0.1")
        self.port = int(rospy.get_param("~port", 41451))
        self.camera = str(rospy.get_param("~camera", "0"))
        self.rate_hz = float(rospy.get_param("~rate", 5.0))

        # AirSim's msgpackrpc client is not thread-safe. All calls go through
        # this lock because rospy services and the main loop run in different
        # threads.
        self.lock = threading.RLock()

        rospy.loginfo("Connecting to AirSim at %s:%d", self.host, self.port)
        self.client = airsim.MultirotorClient(ip=self.host, port=self.port)
        with self.lock:
            self.client.confirmConnection()
            self.client.enableApiControl(True)
            self.client.armDisarm(True)
        rospy.loginfo("AirSim connected, vehicles: %s", self.client.listVehicles())

        self.bridge = CvBridge()
        self.pose_pub = rospy.Publisher("/drone/pose", PoseStamped, queue_size=10)
        self.image_pub = rospy.Publisher("/drone/camera/image_raw", Image, queue_size=1)

        rospy.Subscriber("/drone/cmd_vel", Twist, self.cmd_vel_cb, queue_size=1)
        rospy.Service("/drone/takeoff", Empty, self.takeoff_srv)
        rospy.Service("/drone/land", Empty, self.land_srv)

        self.rate = rospy.Rate(self.rate_hz)

    def cmd_vel_cb(self, msg):
        try:
            with self.lock:
                self.client.moveByVelocityAsync(
                    msg.linear.x, msg.linear.y, msg.linear.z, 0.2
                )
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "moveByVelocityAsync failed: %s", exc)

    def takeoff_srv(self, _req):
        rospy.loginfo("takeoff requested")
        with self.lock:
            self.client.takeoffAsync(timeout_sec=10).join()
        return EmptyResponse()

    def land_srv(self, _req):
        rospy.loginfo("land requested")
        with self.lock:
            self.client.landAsync(timeout_sec=10).join()
        return EmptyResponse()

    def publish_pose(self):
        with self.lock:
            state = self.client.getMultirotorState()
        p = state.kinematics_estimated.position
        q = state.kinematics_estimated.orientation
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world_ned"
        msg.pose.position.x = p.x_val
        msg.pose.position.y = p.y_val
        msg.pose.position.z = p.z_val
        msg.pose.orientation.x = q.x_val
        msg.pose.orientation.y = q.y_val
        msg.pose.orientation.z = q.z_val
        msg.pose.orientation.w = q.w_val
        self.pose_pub.publish(msg)

    def publish_camera(self):
        with self.lock:
            responses = self.client.simGetImages([
                airsim.ImageRequest(self.camera, airsim.ImageType.Scene, False, False)
            ])
        if not responses:
            return
        img = responses[0]
        if img.width == 0 or img.height == 0:
            return
        arr = np.frombuffer(img.image_data_uint8, dtype=np.uint8)
        arr = arr.reshape(img.height, img.width, 3)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(arr, encoding="bgr8"))
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5.0, "cv_bridge failed: %s", exc)

    def spin(self):
        rospy.loginfo("drone bringup running at %.1f Hz", self.rate_hz)
        while not rospy.is_shutdown():
            try:
                self.publish_pose()
                self.publish_camera()
            except Exception as exc:
                rospy.logwarn_throttle(5.0, "AirSim read failed: %s", exc)
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("drone_airsim_bringup")
    node = DroneBringup()
    node.spin()
