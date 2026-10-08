#!/usr/bin/env python3
import pygame
import rospy
from geometry_msgs.msg import Twist

class TeleopCar:
    def __init__(self):
        rospy.init_node('vehicle_pygame_teleop', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(30)

        pygame.init()
        pygame.display.set_mode((200, 100))
        pygame.display.set_caption("车辆键盘控制")

        self.max_linear = 1.0
        self.max_angular = 1.0
        self.linear = 0.0
        self.angular = 0.0

        rospy.loginfo("车辆键盘控制节点已启动，使用方向键控制")
        rospy.loginfo("上/下：前进/后退，左/右：左转/右转，空格：急停")

    def run(self):
        while not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    return

            keys = pygame.key.get_pressed()
            self.linear = 0.0
            self.angular = 0.0

            if keys[pygame.K_UP]:
                self.linear = self.max_linear
            elif keys[pygame.K_DOWN]:
                self.linear = -self.max_linear

            if keys[pygame.K_LEFT]:
                self.angular = self.max_angular
            elif keys[pygame.K_RIGHT]:
                self.angular = -self.max_angular

            if keys[pygame.K_SPACE]:
                self.linear = 0.0
                self.angular = 0.0

            twist = Twist()
            twist.linear.x = self.linear
            twist.angular.z = self.angular
            self.pub.publish(twist)

            pygame.display.update()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        teleop = TeleopCar()
        teleop.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        pygame.quit()
