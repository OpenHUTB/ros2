#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class VoiceTeleop:
    def __init__(self):
        rospy.init_node("voice_teleop")

        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/recognizer/output", String, self.get_voice)

        self.linear_speed = rospy.get_param("~linear_speed", 2.0)
        self.angular_speed = rospy.get_param("~angular_speed", 2.0)

        rospy.loginfo("Starting voice teleop")

    def get_voice(self, data):
        voice_text = data.data.strip().lower()
        rospy.loginfo("I said: %s", voice_text)

        twist = Twist()

        if voice_text in ["go", "forward"]:
            twist.linear.x = self.linear_speed
        elif voice_text in ["back", "backward"]:
            twist.linear.x = -self.linear_speed
        elif voice_text == "left":
            twist.angular.z = self.angular_speed
        elif voice_text == "right":
            twist.angular.z = -self.angular_speed
        elif voice_text in ["stop", "halt"]:
            pass
        else:
            return

        self.pub.publish(twist)


if __name__ == "__main__":
    VoiceTeleop()
    rospy.spin()
