#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
小海龟彩虹螺旋线
效果：画出彩色螺旋，颜色自动变化
"""

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen

def draw_rainbow_spiral():
    rospy.init_node('rainbow_spiral', anonymous=True)
    
    # 等待画笔服务
    rospy.wait_for_service('/turtle1/set_pen')
    set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
    
    # 发布速度指令
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz
    
    # 彩虹颜色列表 [R, G, B]
    colors = [
        [255, 0, 0],    # 红
        [255, 127, 0],  # 橙
        [255, 255, 0],  # 黄
        [0, 255, 0],    # 绿
        [0, 0, 255],    # 蓝
        [75, 0, 130],   # 靛
        [148, 0, 211]   # 紫
    ]
    
    color_index = 0
    step_count = 0
    
    rospy.loginfo("开始画彩虹螺旋线...")
    
    while not rospy.is_shutdown():
        twist = Twist()
        
        # 螺旋核心：线速度递增，角速度固定
        twist.linear.x = 1.0 + step_count * 0.01  # 逐渐加速
        twist.angular.z = 1.5  # 固定转向速度
        
        # 每20步换一种颜色
        if step_count % 20 == 0:
            color = colors[color_index % len(colors)]
            set_pen(color[0], color[1], color[2], 3, 0)  # 设置画笔
            color_index += 1
            rospy.loginfo(f"切换到颜色: {color}")
        
        pub.publish(twist)
        step_count += 1
        
        # 200步后停止
        if step_count >= 200:
            twist.linear.x = 0
            twist.angular.z = 0
            pub.publish(twist)
            rospy.loginfo("完成！")
            break
            
        rate.sleep()

if __name__ == '__main__':
    try:
        draw_rainbow_spiral()
    except rospy.ROSInterruptException:
        pass