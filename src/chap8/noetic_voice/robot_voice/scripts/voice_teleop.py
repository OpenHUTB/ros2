#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Exact command mapping, repeated velocity output and a monotonic watchdog."""
import math
import threading
import time
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class VoiceTeleop:
    def __init__(self):
        rospy.init_node('voice_teleop')
        self.linear_speed = float(rospy.get_param('~linear_speed', 1.0))
        self.angular_speed = float(rospy.get_param('~angular_speed', 1.0))
        self.timeout = float(rospy.get_param('~command_timeout', 1.0))
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.linear_speed, self.angular_speed, self.timeout)):
            raise ValueError('Speeds and command_timeout must be finite and positive')
        self.lock = threading.RLock()
        self.command = Twist()
        self.last_command = None
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.on_shutdown(self.cleanup)
        self.sub = rospy.Subscriber('/recognizer/output', String, self.get_voice)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.tick)

    def get_voice(self, data):
        text = data.data.strip().lower()
        cmd = Twist()
        if text in ('forward', 'move forward'):
            cmd.linear.x = self.linear_speed
        elif text in ('back', 'backward', 'move back', 'move backward'):
            cmd.linear.x = -self.linear_speed
        elif text in ('left', 'move left'):
            cmd.angular.z = self.angular_speed
        elif text in ('right', 'move right'):
            cmd.angular.z = -self.angular_speed
        elif text not in ('stop', 'halt'):
            # Unknown recognition stops instead of extending the previous action.
            rospy.logwarn('Unsupported voice command; stopping: %s', text)
        with self.lock:
            self.command = cmd
            self.last_command = time.monotonic()
            self.pub.publish(cmd)
        rospy.loginfo('Received voice command: %s', text)

    def tick(self, _event):
        with self.lock:
            if self.last_command is None or time.monotonic() - self.last_command >= self.timeout:
                self.command = Twist()
            self.pub.publish(self.command)

    def cleanup(self):
        with self.lock:
            self.command = Twist()
            self.pub.publish(self.command)


if __name__ == '__main__':
    VoiceTeleop()
    rospy.spin()
