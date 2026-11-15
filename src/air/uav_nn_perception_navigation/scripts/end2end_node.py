#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
端到端视觉控制节点（任务 4）

订阅机载相机图像，直接由卷积网络输出速度指令，不包含任何人工规则：

    /uav/image_raw (sensor_msgs/Image) -> 网络 -> /uav/cmd_vel (geometry_msgs/Twist)
"""

import os
import sys

import numpy as np
import rospy
import torch
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)
try:
    import rospkg
    _source_scripts = os.path.join(
        rospkg.RosPack().get_path('uav_nn_perception_navigation'), 'scripts')
    if _source_scripts not in sys.path:
        sys.path.insert(0, _source_scripts)
except Exception:
    pass

from train_end2end import IMG_H, IMG_W, End2EndNet, downsample  # noqa: E402


def resolve_model_path(model_path):
    if os.path.isabs(model_path) and os.path.exists(model_path):
        return model_path
    try:
        import rospkg
        pkg_dir = rospkg.RosPack().get_path('uav_nn_perception_navigation')
        candidate = os.path.join(pkg_dir, 'models', os.path.basename(model_path))
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass
    return os.path.abspath(os.path.join(_SCRIPT_DIR, '..', 'models',
                                        os.path.basename(model_path)))


class End2EndNode(object):
    def __init__(self):
        model_path = resolve_model_path(
            rospy.get_param('~model_path', 'end2end.pt'))
        self.rate_hz = float(rospy.get_param('~rate', 10.0))
        self.altitude = float(rospy.get_param('~altitude', 5.0))
        self.kp_alt = float(rospy.get_param('~kp_altitude', 0.8))
        self.max_speed = float(rospy.get_param('~max_speed', 3.0))

        checkpoint = torch.load(model_path, map_location='cpu')
        self.model = End2EndNet()
        self.model.load_state_dict(checkpoint['state_dict'])
        self.model.eval()
        self.action_scale = np.array(checkpoint.get('action_scale', [3.0, 2.0, 1.0]),
                                     dtype=np.float32)
        rospy.loginfo('端到端模型已加载: %s', model_path)

        self.image = None
        self.odom = None

        rospy.Subscriber('/uav/image_raw', Image, self.image_cb, queue_size=1)
        rospy.Subscriber('/uav/odom', Odometry, self.odom_cb, queue_size=1)
        self.pub = rospy.Publisher('/uav/cmd_vel', Twist, queue_size=1)

    def image_cb(self, msg):
        if msg.encoding not in ('bgr8', 'rgb8'):
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        arr = arr.reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            arr = arr[:, :, ::-1]  # old model expects BGR
        small = downsample(arr, IMG_H, IMG_W)
        if small is not None:
            self.image = small.astype(np.float32) / 255.0

    def odom_cb(self, msg):
        self.odom = msg

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.image is None or self.odom is None:
                # 数据没齐时保持沉默，避免干扰起飞
                rate.sleep()
                continue

            try:
                batch = torch.from_numpy(self.image.transpose(2, 0, 1)).unsqueeze(0)
                with torch.no_grad():
                    out = self.model(batch).squeeze(0).numpy()
                vx, vy, yaw_rate = out * self.action_scale

                cmd = Twist()
                cmd.linear.x = float(np.clip(vx, -self.max_speed, self.max_speed))
                cmd.linear.y = float(np.clip(vy, -self.max_speed, self.max_speed))
                cmd.linear.z = max(-1.5, min(1.5,
                                             self.kp_alt * (self.altitude -
                                                            self.odom.pose.pose.position.z)))
                cmd.angular.z = float(np.clip(yaw_rate, -1.0, 1.0))
                self.pub.publish(cmd)
            except Exception as exc:  # pragma: no cover
                rospy.logwarn_throttle(5.0, '端到端推理失败: %s', exc)

            rate.sleep()


def main():
    rospy.init_node('end2end_control')
    End2EndNode().run()


if __name__ == '__main__':
    main()
