#!/usr/bin/env python3
"""AirSim drone bringup for ROS Noetic.

Publishes:
  /drone/pose              geometry_msgs/PoseStamped (NED world)
  /drone/camera/image_raw  sensor_msgs/Image (bgr8)
  /drone/depth/image_raw   sensor_msgs/Image (32FC1, metres)
  /drone/imu               sensor_msgs/Imu (NED body frame)

Subscribes:
  /drone/cmd_vel           geometry_msgs/Twist (NED velocity, m/s)

Services:
  /drone/takeoff           std_srvs/Empty
  /drone/land              std_srvs/Empty
"""

import threading
import time

import numpy as np
import rospy
import airsim

from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import Empty, EmptyResponse


class DroneBringup(object):
    def __init__(self):
        self.host = rospy.get_param("~host", "127.0.0.1")
        self.port = int(rospy.get_param("~port", 41451))
        self.camera = str(rospy.get_param("~camera", "0"))
        self.depth_camera = str(rospy.get_param("~depth_camera", "depth"))
        self.rate_hz = float(rospy.get_param("~rate", 10.0))
        self.image_rate_hz = float(rospy.get_param("~image_rate", 5.0))

        # Two AirSim clients: one for pose/IMU/control at 10 Hz, one for the
        # heavy RGB+depth RPC. A single client serialises every call, so the
        # image request used to drag pose/IMU down to ~3 Hz.
        self.state_lock = threading.RLock()
        self.image_lock = threading.RLock()

        rospy.loginfo("Connecting to AirSim at %s:%d", self.host, self.port)
        self.state_client = airsim.MultirotorClient(ip=self.host, port=self.port)
        self.image_client = airsim.MultirotorClient(ip=self.host, port=self.port)
        with self.state_lock:
            self.state_client.confirmConnection()
            self.state_client.enableApiControl(True)
            self.state_client.armDisarm(True)
        with self.image_lock:
            self.image_client.confirmConnection()
        rospy.loginfo(
            "AirSim connected, vehicles: %s", self.state_client.listVehicles()
        )

        self.pose_pub = rospy.Publisher("/drone/pose", PoseStamped, queue_size=10)
        self.image_pub = rospy.Publisher("/drone/camera/image_raw", Image, queue_size=1)
        self.depth_pub = rospy.Publisher("/drone/depth/image_raw", Image, queue_size=1)
        self.imu_pub = rospy.Publisher("/drone/imu", Imu, queue_size=10)

        rospy.Subscriber("/drone/cmd_vel", Twist, self.cmd_vel_cb, queue_size=1)
        rospy.Service("/drone/takeoff", Empty, self.takeoff_srv)
        rospy.Service("/drone/land", Empty, self.land_srv)

        self.rate = rospy.Rate(self.rate_hz)

    def cmd_vel_cb(self, msg):
        try:
            with self.state_lock:
                self.state_client.moveByVelocityAsync(
                    msg.linear.x, msg.linear.y, msg.linear.z, 0.2
                )
                if abs(msg.angular.z) > 1e-3:
                    self.state_client.rotateByYawRateAsync(msg.angular.z, 0.2)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "moveByVelocityAsync failed: %r", exc)

    def _wait_while_publishing(self, seconds):
        """Wait in short steps without holding the AirSim lock."""
        deadline = time.time() + seconds
        while time.time() < deadline and not rospy.is_shutdown():
            time.sleep(0.2)

    def takeoff_srv(self, _req):
        rospy.loginfo("takeoff requested")
        with self.state_lock:
            self.state_client.takeoffAsync(timeout_sec=10)
        # The command runs asynchronously in AirSim. Do not join() under the
        # lock: pose and IMU must keep publishing during the climb.
        self._wait_while_publishing(6.0)
        return EmptyResponse()

    def land_srv(self, _req):
        rospy.loginfo("land requested")
        with self.state_lock:
            self.state_client.landAsync(timeout_sec=10)
        self._wait_while_publishing(4.0)
        return EmptyResponse()

    def publish_pose(self):
        with self.state_lock:
            state = self.state_client.getMultirotorState()
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

    def publish_imu(self):
        with self.state_lock:
            data = self.state_client.getImuData()
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "drone_imu_link"

        q = data.orientation
        msg.orientation.x = q.x_val
        msg.orientation.y = q.y_val
        msg.orientation.z = q.z_val
        msg.orientation.w = q.w_val

        av = data.angular_velocity
        msg.angular_velocity.x = av.x_val
        msg.angular_velocity.y = av.y_val
        msg.angular_velocity.z = av.z_val

        la = data.linear_acceleration
        msg.linear_acceleration.x = la.x_val
        msg.linear_acceleration.y = la.y_val
        msg.linear_acceleration.z = la.z_val

        self.imu_pub.publish(msg)

    def publish_images(self):
        with self.image_lock:
            responses = self.image_client.simGetImages([
                airsim.ImageRequest(self.camera, airsim.ImageType.Scene, False, False),
                airsim.ImageRequest(
                    self.depth_camera, airsim.ImageType.DepthPlanar, True, False
                ),
            ])
        if len(responses) < 2:
            return

        rgb = responses[0]
        if rgb.width > 0 and rgb.height > 0:
            arr = np.frombuffer(rgb.image_data_uint8, dtype=np.uint8)
            arr = arr.reshape(rgb.height, rgb.width, 3)
            msg = Image()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "drone_camera"
            msg.height = rgb.height
            msg.width = rgb.width
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = rgb.width * 3
            msg.data = arr.tobytes()
            self.image_pub.publish(msg)

        depth = responses[1]
        if depth.width > 0 and depth.height > 0 and len(depth.image_data_float) > 0:
            arr = np.asarray(depth.image_data_float, dtype=np.float32)
            arr = arr.reshape(depth.height, depth.width)
            msg = Image()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "drone_camera"
            msg.height = depth.height
            msg.width = depth.width
            msg.encoding = "32FC1"
            msg.is_bigendian = 0
            msg.step = depth.width * 4
            msg.data = arr.tobytes()
            self.depth_pub.publish(msg)

    def image_loop(self):
        rate = rospy.Rate(self.image_rate_hz)
        while not rospy.is_shutdown():
            try:
                self.publish_images()
            except Exception as exc:
                rospy.logwarn_throttle(5.0, "publish_images failed: %r", exc)
            rate.sleep()

    def spin(self):
        rospy.loginfo(
            "drone bringup running: pose/imu %.1f Hz, images %.1f Hz",
            self.rate_hz,
            self.image_rate_hz,
        )
        thread = threading.Thread(target=self.image_loop, daemon=True)
        thread.start()

        while not rospy.is_shutdown():
            try:
                self.publish_pose()
            except Exception as exc:
                rospy.logwarn_throttle(5.0, "publish_pose failed: %r", exc)
            try:
                self.publish_imu()
            except Exception as exc:
                rospy.logwarn_throttle(5.0, "publish_imu failed: %r", exc)
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("drone_airsim_bringup")
    node = DroneBringup()
    node.spin()
