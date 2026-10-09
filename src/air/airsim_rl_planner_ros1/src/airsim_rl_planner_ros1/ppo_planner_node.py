"""Actual PPO inference using Auto_drone's legacy ten-element observation."""
from pathlib import Path
import time

import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TwistStamped
from .runtime import Node
qos_profile_sensor_data = 1  # ROS 1 bounded queue depth
from sensor_msgs.msg import Image
from stable_baselines3 import PPO
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from ultralytics import YOLO

from .core import observation, velocity
from .legacy_policy import load_policy


class PPOPlanner(Node):
    def __init__(self):
        super().__init__('ppo_planner')
        defaults = dict(policy_path='', yolo_path='', control_rate=2.0,
                        sensor_timeout=1.5, speed=1.0, clearance=3.0,
                        goal_ned=[93.37, 0.79, -7.56], goal_tolerance=2.0)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.param = lambda name: self.get_parameter(name).value
        for key in ('control_rate', 'sensor_timeout', 'speed', 'clearance', 'goal_tolerance'):
            if not np.isfinite(self.param(key)) or self.param(key) <= 0:
                raise ValueError(key + ' must be positive and finite')
        self.goal = np.asarray(self.param('goal_ned'), dtype=float)
        if self.goal.shape != (3,) or not np.all(np.isfinite(self.goal)):
            raise ValueError('goal_ned must contain three finite numbers')
        paths = [Path(self.param(key)).expanduser() for key in ('policy_path', 'yolo_path')]
        for path in paths:
            if not path.is_file():
                raise FileNotFoundError('Supply an existing model file: ' + str(path))
        self.policy = load_policy(str(paths[0]))
        if self.policy.observation_space.shape != (10,) or getattr(self.policy.action_space, 'n', None) != 5:
            raise ValueError('Expected Auto_drone policy with 10 observations and 5 discrete actions')
        self.yolo = YOLO(str(paths[1]))
        labels = set(self.yolo.names.values())
        if not {'building', 'tree', 'road'}.issubset(labels):
            self.get_logger().warning('YOLO lacks some legacy classes building/tree/road; their observation entries remain zero.')
        self.cv = CvBridge()
        self.data = {}
        self.enabled = False
        self.publisher = self.create_publisher(TwistStamped, 'drone/cmd_vel_ned', 1)
        for topic, typ, key in [('drone/rgb', Image, 'rgb'), ('drone/depth', Image, 'depth'),
                                ('drone/pose_ned', PoseStamped, 'pose'), ('drone/collision', Bool, 'collision')]:
            self.create_subscription(typ, topic, lambda msg, k=key: self.receive(k, msg), qos_profile_sensor_data)
        self.create_service(SetBool, 'planner/enable', self.enable)
        self.create_timer(1.0 / self.param('control_rate'), self.step)
        self.get_logger().info('PPO loaded on CPU. Call planner/enable after takeoff. Goal is a stop condition, not a policy input.')

    def receive(self, key, msg):
        self.data[key] = (msg, time.monotonic())

    def publish(self, xyz):
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'world_ned'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, xyz)
        self.publisher.publish(msg)

    def enable(self, request, response):
        self.enabled = request.data
        if not self.enabled:
            self.publish((0, 0, 0))
        response.success = True
        response.message = 'Enabled' if self.enabled else 'Stopped'
        return response

    def fresh(self):
        now = time.monotonic()
        return all(key in self.data and now - self.data[key][1] < self.param('sensor_timeout')
                   for key in ('rgb', 'depth', 'pose', 'collision'))

    def step(self):
        if not self.enabled:
            return
        try:
            if not self.fresh():
                self.publish((0, 0, 0))
                return
            pose = self.data['pose'][0]
            if pose.header.frame_id != 'world_ned':
                raise ValueError('Pose frame must be world_ned')
            p = pose.pose.position
            position = np.array([p.x, p.y, p.z])
            if not np.all(np.isfinite(position)):
                raise ValueError('Invalid position')
            if self.data['collision'][0].data or np.linalg.norm(self.goal - position) < self.param('goal_tolerance'):
                self.enabled = False
                self.publish((0, 0, 0))
                self.get_logger().info('Stopped: collision or goal tolerance reached')
                return
            rgb_msg, depth_msg = self.data['rgb'][0], self.data['depth'][0]
            rgb_stamp, depth_stamp = rgb_msg.header.stamp, depth_msg.header.stamp
            if (rgb_stamp.secs, rgb_stamp.nsecs) != (depth_stamp.secs, depth_stamp.nsecs):
                self.publish((0, 0, 0))
                return
            bgr = self.cv.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth = self.cv.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            result = self.yolo.predict(bgr, verbose=False)[0]
            names = {self.yolo.names[int(cls)] for cls in result.boxes.cls.cpu().numpy()}
            obs = observation(names, depth, self.param('clearance'))
            action, _ = self.policy.predict(obs, deterministic=True)
            # Inference itself can exceed sensor_timeout. Never issue a stale command.
            if not self.fresh():
                self.publish((0, 0, 0))
                return
            self.publish(velocity(action, self.param('speed')))
        except Exception as exc:
            self.enabled = False
            self.publish((0, 0, 0))
            self.get_logger().error('Planner disabled: ' + str(exc))



def main():
    rospy.init_node('ppo_planner')
    node = PPOPlanner()
    try:
        node.spin()
    finally:
        node.publish((0, 0, 0))
        node.destroy_node()
