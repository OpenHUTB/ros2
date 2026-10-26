#!/usr/bin/env python3
"""Keyboard teleop for the AirSim drone.

Keys:
  w / s : forward / backward (NED x)
  a / d : left / right (NED y)
  r / f : up / down (NED z, r is up)
  q / e : yaw left / right
  space : stop
  t     : takeoff
  l     : land
  Ctrl-C: quit

Publishes /drone/cmd_vel at ~20 Hz and calls /drone/takeoff or /drone/land.
"""

import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

HELP = """
drone teleop
  w/s: forward/back   a/d: left/right   r/f: up/down
  q/e: yaw            space: stop       t: takeoff   l: land
  Ctrl-C: quit
"""


class DroneTeleop(object):
    def __init__(self):
        self.speed = float(rospy.get_param("~speed", 3.0))
        self.yaw_rate = float(rospy.get_param("~yaw_rate", 1.0))
        self.pub = rospy.Publisher("/drone/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(float(rospy.get_param("~rate", 20.0)))
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw = 0.0
        self.takeoff = rospy.ServiceProxy("/drone/takeoff", Empty)
        self.land = rospy.ServiceProxy("/drone/land", Empty)

    def publish(self):
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.linear.z = self.vz
        msg.angular.z = self.yaw
        self.pub.publish(msg)

    def stop(self):
        self.vx = self.vy = self.vz = self.yaw = 0.0

    def handle_key(self, key):
        if key == "w":
            self.vx = self.speed
        elif key == "s":
            self.vx = -self.speed
        elif key == "a":
            self.vy = -self.speed
        elif key == "d":
            self.vy = self.speed
        elif key == "r":
            self.vz = -self.speed
        elif key == "f":
            self.vz = self.speed
        elif key == "q":
            self.yaw = self.yaw_rate
        elif key == "e":
            self.yaw = -self.yaw_rate
        elif key == " ":
            self.stop()
        elif key == "t":
            rospy.loginfo("takeoff")
            self.takeoff()
        elif key == "l":
            rospy.loginfo("land")
            self.land()
        else:
            return
        rospy.loginfo(
            "cmd vx=%.1f vy=%.1f vz=%.1f yaw=%.1f",
            self.vx,
            self.vy,
            self.vz,
            self.yaw,
        )

    def run(self):
        if not sys.stdin.isatty():
            rospy.logerr("stdin is not a TTY; run this node in a terminal")
            return

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            sys.stdout.write(HELP)
            sys.stdout.flush()
            while not rospy.is_shutdown():
                ready, _, _ = select.select([sys.stdin], [], [], 0.05)
                if ready:
                    key = sys.stdin.read(1)
                    if key == "\x03":
                        break
                    self.handle_key(key)
                self.publish()
                self.rate.sleep()
        finally:
            self.stop()
            self.publish()
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


if __name__ == "__main__":
    rospy.init_node("drone_teleop")
    DroneTeleop().run()
