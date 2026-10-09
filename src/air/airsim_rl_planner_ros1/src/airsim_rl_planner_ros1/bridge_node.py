"""AirSim simulation bridge. All velocity and pose messages explicitly use NED."""
import math
import time

import airsim
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TwistStamped
from .runtime import Node
from .depth_guard import DepthGuard
qos_profile_sensor_data = 1  # ROS 1 bounded queue depth
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class AirSimBridge(Node):
    def __init__(self):
        super().__init__('airsim_bridge')
        defaults = dict(host='127.0.0.1', port=41451, vehicle_name='',
                        camera_name='front_center', publish_rate=5.0,
                        command_timeout=1.5, max_speed=1.0)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.param = lambda name: self.get_parameter(name).value
        for name in ('publish_rate', 'command_timeout', 'max_speed'):
            if not math.isfinite(self.param(name)) or self.param(name) <= 0:
                raise ValueError(name + ' must be positive and finite')
        self.client = airsim.MultirotorClient(
            ip=self.param('host'), port=self.param('port'), timeout_value=2)
        if not self.client.ping():
            raise RuntimeError('AirSim did not answer ping')
        self.vehicle = self.param('vehicle_name')
        self.cv = CvBridge()
        self.active = False
        self.last_command = 0.0
        self.holding = False
        self.target_z = None
        self.latest_velocity = np.zeros(3)
        self.guard = DepthGuard()
        self.guard_sample = None
        self.guard_reason = None
        self.rgb_pub = self.create_publisher(Image, 'drone/rgb', qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, 'drone/depth', qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseStamped, 'drone/pose_ned', qos_profile_sensor_data)
        self.collision_pub = self.create_publisher(Bool, 'drone/collision', qos_profile_sensor_data)
        self.create_subscription(TwistStamped, 'drone/cmd_vel_ned', self.command, 1)
        self.create_service(Trigger, 'drone/takeoff', self.takeoff)
        self.create_service(Trigger, 'drone/land', self.land)
        self.create_timer(1.0 / self.param('publish_rate'), self.publish_state)
        self.create_timer(0.1, self.watchdog)
        self.get_logger().info('Connected. Call drone/takeoff to enable movement; coordinates are NED.')

    def send(self, xyz):
        if np.linalg.norm(xyz) > 1e-6:
            sample = self.guard_sample
            if sample is None or time.monotonic() - sample[2] > 0.5:
                xyz, reason = np.zeros(3), 'depth stale: hover'
            else:
                xyz, reason = self.guard.filter(xyz, sample[0], sample[1], time.monotonic())
            if reason != self.guard_reason:
                self.get_logger().info('Depth guard: ' + reason)
                self.guard_reason = reason
        if np.linalg.norm(xyz) < 1e-6:
            if not self.holding:
                self.client.hoverAsync(vehicle_name=self.vehicle).join()
                self.holding = True
            return
        duration = min(0.35, max(0.01, self.param('command_timeout') -
                                  (time.monotonic() - self.last_command)))
        self.holding = False
        if abs(xyz[2]) < 1e-6:
            if self.target_z is None:
                z = self.client.getMultirotorState(vehicle_name=self.vehicle).kinematics_estimated.position.z_val
                if not math.isfinite(z):
                    raise ValueError('Invalid altitude')
                self.target_z = float(z)
            self.client.moveByVelocityZAsync(
                float(xyz[0]), float(xyz[1]), self.target_z, duration,
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=0),
                vehicle_name=self.vehicle)
        else:
            # Preserve intentional PPO ascent/descent. Capture new altitude afterwards.
            self.target_z = None
            self.client.moveByVelocityAsync(
                *map(float, xyz), duration=duration,
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=0),
                vehicle_name=self.vehicle)

    def command(self, msg):
        if not self.active:
            return
        age = (rospy.Time.now().to_nsec() -
               (msg.header.stamp.secs * 10**9 + msg.header.stamp.nsecs)) / 1e9
        values = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        if msg.header.frame_id != 'world_ned' or age < -0.1 or age > self.param('command_timeout') or not np.all(np.isfinite(values)):
            return
        try:
            norm = np.linalg.norm(values)
            values *= min(1.0, self.param('max_speed') / max(norm, 1e-9))
            self.latest_velocity = values
            self.last_command = time.monotonic() - max(0.0, age)
            self.send(values)
        except Exception as exc:
            self.get_logger().error(str(exc))

    def watchdog(self):
        if not self.active:
            return
        try:
            if time.monotonic() - self.last_command > self.param('command_timeout'):
                self.latest_velocity = np.zeros(3)
            self.send(self.latest_velocity)
        except Exception as exc:
            self.latest_velocity = np.zeros(3)
            self.last_command = 0.0
            self.get_logger().error(str(exc))

    def takeoff(self, request, response):
        try:
            self.guard = DepthGuard()
            self.guard_sample = None
            self.client.enableApiControl(True, vehicle_name=self.vehicle)
            self.client.armDisarm(True, vehicle_name=self.vehicle)
            self.client.takeoffAsync(timeout_sec=15, vehicle_name=self.vehicle).join()
            state = self.client.getMultirotorState(vehicle_name=self.vehicle)
            self.active = state.landed_state == airsim.LandedState.Flying
            self.last_command = 0.0
            self.latest_velocity = np.zeros(3)
            self.target_z = None
            self.holding = False
            if self.active:
                self.send((0, 0, 0))
            response.success = self.active
            response.message = 'Airborne' if self.active else 'Takeoff not confirmed'
        except Exception as exc:
            self.active = False
            response.message = str(exc)
        return response

    def land(self, request, response):
        self.active = False
        self.latest_velocity = np.zeros(3)
        self.target_z = None
        self.holding = False
        try:
            self.client.landAsync(timeout_sec=30, vehicle_name=self.vehicle).join()
            state = self.client.getMultirotorState(vehicle_name=self.vehicle)
            response.success = state.landed_state == airsim.LandedState.Landed
            if response.success:
                self.client.armDisarm(False, vehicle_name=self.vehicle)
                self.client.enableApiControl(False, vehicle_name=self.vehicle)
            response.message = 'Landed' if response.success else 'Landing not confirmed'
        except Exception as exc:
            response.message = str(exc)
        return response

    def publish_state(self):
        try:
            state = self.client.getMultirotorState(vehicle_name=self.vehicle)
            collision = self.client.simGetCollisionInfo(vehicle_name=self.vehicle)
            rgb, depth = self.client.simGetImages([
                airsim.ImageRequest(self.param('camera_name'), airsim.ImageType.Scene, False, False),
                airsim.ImageRequest(self.param('camera_name'), airsim.ImageType.DepthPerspective, True, False),
            ], vehicle_name=self.vehicle)
            if min(rgb.width, rgb.height, depth.width, depth.height) <= 0:
                raise ValueError('Empty camera response')
            # AirSim uncompressed Scene bytes are RGB; no compressed-image decoder.
            image = np.frombuffer(rgb.image_data_uint8, np.uint8).reshape(rgb.height, rgb.width, 3)
            distances = np.asarray(depth.image_data_float, np.float32).reshape(depth.height, depth.width)
            q = state.kinematics_estimated.orientation
            self.guard_sample = (distances, (q.x_val, q.y_val, q.z_val, q.w_val), time.monotonic())
            if collision.has_collided:
                self.latest_velocity = np.zeros(3)
                self.last_command = 0.0
                if self.active:
                    self.send((0, 0, 0))
                self.active = False
            stamp = rospy.Time.now()
            pose = PoseStamped()
            pose.header.stamp, pose.header.frame_id = stamp, 'world_ned'
            p, q = state.kinematics_estimated.position, state.kinematics_estimated.orientation
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p.x_val, p.y_val, p.z_val
            pose.pose.orientation.x, pose.pose.orientation.y = q.x_val, q.y_val
            pose.pose.orientation.z, pose.pose.orientation.w = q.z_val, q.w_val
            for publisher, array, encoding in ((self.rgb_pub, image, 'rgb8'), (self.depth_pub, distances, '32FC1')):
                msg = self.cv.cv2_to_imgmsg(array, encoding)
                msg.header.stamp, msg.header.frame_id = stamp, 'front_camera_optical'
                publisher.publish(msg)
            self.pose_pub.publish(pose)
            self.collision_pub.publish(Bool(data=collision.has_collided))
        except Exception as exc:
            self.get_logger().error('Sensor read failed: ' + str(exc))



def main():
    rospy.init_node('airsim_bridge')
    node = AirSimBridge()
    try:
        node.spin()
    finally:
        if node.active:
            try:
                node.send((0, 0, 0))
            except Exception:
                pass
        node.destroy_node()
