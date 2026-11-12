#!/usr/bin/env python3
"""Minimal rclpy probe: wait for one /cmd_vel and one /ca/status message."""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node("ca_probe")
got = {"cmd": None, "status": None}


def on_cmd(msg):
    got["cmd"] = (msg.linear.x, msg.linear.y, msg.angular.z)
    print("CMD %.3f %.3f %.3f" % got["cmd"], flush=True)


def on_status(msg):
    got["status"] = msg.data
    print("STATUS %s" % msg.data, flush=True)


node.create_subscription(Twist, "/cmd_vel", on_cmd, 10)
node.create_subscription(String, "/ca/status", on_status, 10)
deadline = time.time() + 15.0
while time.time() < deadline and not all(got.values()):
    rclpy.spin_once(node, timeout_sec=0.5)
print("RESULT", "ok" if all(got.values()) else "timeout", flush=True)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if all(got.values()) else 1)
