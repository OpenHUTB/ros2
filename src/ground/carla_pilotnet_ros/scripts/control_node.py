#!/usr/bin/env python3
"""控制节点

订阅 /pilotnet/control（PilotNet 输出 steering + throttle），
映射为 Carla 车辆控制指令，发布到 /vehicle/control_cmd。
"""
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class ControlNode:
    def __init__(self):
        rospy.init_node('control_node', anonymous=False)

        self.max_steer = rospy.get_param('~max_steer', 1.0)
        self.max_throttle = rospy.get_param('~max_throttle', 1.0)

        self.cmd_pub = rospy.Publisher('/vehicle/control_cmd', Twist, queue_size=10)
        rospy.Subscriber('/pilotnet/control', Float32MultiArray, self.on_control, queue_size=10)

        rospy.loginfo("Control node ready")

    def on_control(self, msg):
        if len(msg.data) < 2:
            return
        steering = max(-self.max_steer, min(self.max_steer, msg.data[0]))
        throttle = max(0.0, min(self.max_throttle, msg.data[1]))

        cmd = Twist()
        cmd.linear.x = throttle
        cmd.angular.z = steering
        self.cmd_pub.publish(cmd)

        rospy.loginfo_throttle(2.0,
            "cmd: throttle=%.3f steer=%.3f" % (throttle, steering))


if __name__ == '__main__':
    try:
        ControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
