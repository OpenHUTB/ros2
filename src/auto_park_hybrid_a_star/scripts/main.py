#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""main.py

ROS Noetic 节点入口：基于 Hybrid A* + Reeds-Shepp 的垂直自动泊车。

在 Pygame 内置二维停车场场景中：
1. 按 [空格] 触发 Hybrid A* 规划从当前位姿到目标车位的无碰撞路径；
2. Pure Pursuit 控制器跟踪路径，自动完成倒车入库；
3. 同时把期望速度/转向转成 geometry_msgs/Twist 发布到 /cmd_vel。

运行：
    roslaunch auto_park_hybrid_a_star main.launch
或：
    rosrun auto_park_hybrid_a_star main.py
"""

import math
import sys

import rospy
from geometry_msgs.msg import Twist

from collision_check import VehicleFootprint
from hybrid_a_star import HybridAStar
from tracker import DiffRCSim, PurePursuit, _densify
from auto_park_sim import (ParkingSim, CAR, OBSTACLES, START, GOAL)


class RosParkingSim(ParkingSim):
    """带 ROS 发布的泊车仿真。"""

    def __init__(self):
        # 先初始化 ROS（在 pygame 之前）
        rospy.init_node('auto_park_hybrid_a_star', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.on_shutdown(self._on_shutdown)
        # 再初始化 Pygame
        super().__init__()
        rospy.loginfo("自动泊车节点已启动。按 [空格] 开始规划+泊车。")

    def step_tracking(self, dt):
        """重写：跟踪时把 (v, steer) 转成 Twist 发布。"""
        v_cmd, s_cmd, info = self.tracker.run(self.car, dt)
        self.car.step(v_cmd, s_cmd, dt)

        twist = Twist()
        twist.linear.x = self.car.v
        # 把前轮转角转成等效角速度（差速模型近似）
        twist.angular.z = self.car.v / self.car.vp.wheel_base * math.tan(self.car.steer)
        self.pub.publish(twist)

        if info.get("at_goal"):
            self.state = "done"
            err = info.get("err", 0.0)
            self.message = "泊车完成！横向误差 %.2f m" % err
            # 停车
            t = Twist()
            self.pub.publish(t)

    def _on_shutdown(self):
        try:
            self.pub.publish(Twist())
        except Exception:
            pass

    def run(self):
        import pygame
        running = True
        rate = rospy.Rate(30)
        while running and not rospy.is_shutdown():
            dt = self.clock.tick(30) / 1000.0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE and self.state == "idle":
                        self.plan()
                    elif event.key == pygame.K_r:
                        self.reset()
            if self.state == "tracking" and self.tracker is not None:
                self.step_tracking(dt)
            self.draw_road()
            self.draw_obstacles()
            self.draw_path()
            self.draw_car()
            self.draw_hud()
            pygame.display.flip()
            rate.sleep()
        pygame.quit()


def main():
    try:
        RosParkingSim().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
