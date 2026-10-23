#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""main.py

ROS Noetic 节点入口：红绿灯启停仿真。

在 Pygame 内置道路场景中：
1. 车辆沿道路直线行驶，前方路口有红绿灯与停止线；
2. 红灯/黄灯自动减速并在停止线前停住，绿灯自动起步通过；
3. 期望速度转成 geometry_msgs/Twist 发布到 /cmd_vel；
4. 灯色可从 /traffic_light 话题读取（std_msgs/String: red/green/yellow），
   未收到话题时使用内部模拟红绿灯。

运行：
    roslaunch traffic_light_stop main.launch
或：
    rosrun traffic_light_stop main.py
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from traffic_light_sim import Simulation, DT, STOP_X
from vehicle_model import VehicleModel


class RosTrafficLightSim(Simulation):
    """带 ROS 发布的红绿灯启停仿真。"""

    def __init__(self):
        # 先初始化 ROS（在 pygame 之前）
        rospy.init_node('traffic_light_stop', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/traffic_light', String,
                                    self._on_light, queue_size=1)
        self._light_override = None      # 外部话题灯色（None=用内部模拟灯）
        rospy.on_shutdown(self._on_shutdown)
        # 再初始化 Pygame
        super().__init__()
        rospy.loginfo("红绿灯启停节点已启动。按 [空格] 开始运行。")

    def _on_light(self, msg):
        color = msg.data.strip().lower()
        if color in ('red', 'green', 'yellow'):
            self._light_override = color

    def _on_shutdown(self):
        try:
            self.pub.publish(Twist())
        except Exception:
            pass

    def step(self):
        """重写：优先使用话题灯色，并把速度发布到 /cmd_vel。"""
        if self._light_override is not None:
            color = self._light_override
            self.light.t += DT
        else:
            color = self.light.update(DT)
        a = self.controller.compute(self.model.head(), self.model.v, color)
        self.model.step(a)
        self.t += DT

        twist = Twist()
        twist.linear.x = self.model.v
        self.pub.publish(twist)

    def run(self):
        import pygame
        running = True
        rate = rospy.Rate(30)
        while running and not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        self.running = not self.running
                    elif event.key == pygame.K_r:
                        self.reset()
            if self.running:
                self.step()
            self.render(pygame.display.get_surface())
            hint = "空格: 开始/暂停   R: 重置   ESC: 退出   /cmd_vel 发布中"
            pygame.display.set_caption("Traffic Light Stop - 红绿灯启停仿真 (ROS)")
            pygame.display.flip()
            rate.sleep()
        pygame.quit()


def main():
    try:
        import pygame
        pygame.init()
        screen = pygame.display.set_mode((1200, 500))
        sim = RosTrafficLightSim()
        sim.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
