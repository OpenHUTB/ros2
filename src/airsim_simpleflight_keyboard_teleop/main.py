#!/usr/bin/env python3
"""Terminal keyboard teleop for an AirSim SimpleFlight drone over ROS1."""

import argparse
import os
import select
import sys
import termios
import time
import tty

import rospy
from airsim_ros_pkgs.msg import VelCmd
from airsim_ros_pkgs.srv import (
    Land,
    LandRequest,
    Reset,
    ResetRequest,
    Takeoff,
    TakeoffRequest,
)
from geometry_msgs.msg import Twist


HELP_TEXT = """
AirSim drone keyboard control
------------------------------
  T: takeoff          L: land
  W/S: forward/back   A/D: left/right
  R/F: up/down        Q/E: yaw left/right
  Space: toggle slow mode    X: stop
  Ctrl+C: exit
"""


class KeyboardReader:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = None

    def __enter__(self):
        if os.isatty(self.fd):
            self.old_settings = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.restore()

    def restore(self):
        if self.old_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
            self.old_settings = None

    def read(self, timeout=0.01):
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if not ready:
            return None
        value = os.read(self.fd, 1)
        if value == b"\x1b":
            extra = os.read(self.fd, 2)
            return value + extra
        return value.decode("utf-8", errors="replace").lower()


def parse_args():
    parser = argparse.ArgumentParser(description=HELP_TEXT)
    parser.add_argument("--vehicle", default="SimpleFlight")
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--fast-speed", type=float, default=2.0)
    parser.add_argument("--slow-speed", type=float, default=0.5)
    parser.add_argument("--vertical-speed", type=float, default=0.8)
    parser.add_argument("--yaw-rate", type=float, default=1.0)
    parser.add_argument("--command-timeout", type=float, default=0.15)
    return parser.parse_args()


def wait_for_service(vehicle, service_name, service_type):
    name = "/airsim_node/{}/{}".format(vehicle, service_name)
    rospy.wait_for_service(name)
    return rospy.ServiceProxy(name, service_type)


def main():
    args = parse_args()
    rospy.init_node("airsim_simpleflight_keyboard_teleop", anonymous=False)

    prefix = "/airsim_node/{}".format(args.vehicle)
    vel_pub = rospy.Publisher(
        "{}/vel_cmd_body_frame".format(prefix), VelCmd, queue_size=10
    )
    takeoff_proxy = wait_for_service(args.vehicle, "takeoff", Takeoff)
    land_proxy = wait_for_service(args.vehicle, "land", Land)
    rospy.wait_for_service("/airsim_node/reset")
    reset_proxy = rospy.ServiceProxy("/airsim_node/reset", Reset)

    rate = rospy.Rate(args.rate)
    cmd = Twist()
    last_input = time.monotonic()
    previous_key = None
    slow_mode = False

    print(HELP_TEXT)
    print("Vehicle: {}".format(args.vehicle))
    print("Waiting for keys...")

    with KeyboardReader() as reader:
        while not rospy.is_shutdown():
            key = reader.read(timeout=0.01)
            now = time.monotonic()

            if key is None:
                previous_key = None

            if key == "\x03":
                break

            if key is not None and key != previous_key:
                try:
                    if key == "t":
                        takeoff_proxy(TakeoffRequest(waitOnLastTask=True))
                        print("Takeoff command sent")
                    elif key == "l":
                        land_proxy(LandRequest(waitOnLastTask=True))
                        print("Land command sent")
                    elif key == "z":
                        reset_proxy(ResetRequest(waitOnLastTask=True))
                        print("Reset command sent")
                except rospy.ServiceException as exc:
                    rospy.logwarn("Service call failed: %s", exc)

            if key is not None:
                previous_key = key
                last_input = now

                if key == " ":
                    slow_mode = not slow_mode
                    print(
                        "Slow mode: {}".format(
                            "ON" if slow_mode else "OFF"
                        )
                    )

                horizontal = args.slow_speed if slow_mode else args.fast_speed

                if key == "w":
                    cmd.linear.x = horizontal
                elif key == "s":
                    cmd.linear.x = -horizontal
                elif key == "a":
                    cmd.linear.y = -horizontal
                elif key == "d":
                    cmd.linear.y = horizontal
                elif key == "r":
                    cmd.linear.z = -args.vertical_speed
                elif key == "f":
                    cmd.linear.z = args.vertical_speed
                elif key == "q":
                    cmd.angular.z = -args.yaw_rate
                elif key == "e":
                    cmd.angular.z = args.yaw_rate
                elif key == "x":
                    cmd = Twist()

            if now - last_input > args.command_timeout:
                cmd = Twist()

            msg = VelCmd()
            msg.twist = cmd
            vel_pub.publish(msg)
            rate.sleep()

    vel_pub.publish(VelCmd())
    print("Keyboard control stopped")


if __name__ == "__main__":
    main()
