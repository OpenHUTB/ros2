#!/usr/bin/env python3
import rospy
import math
from turtlesim.srv import TeleportAbsolute, SetPen

def draw_cylinder_v2():
    rospy.init_node('draw_cylinder_v2_node', anonymous=True)
    
    # 等待服务
    rospy.wait_for_service('/turtle1/teleport_absolute')
    rospy.wait_for_service('/turtle1/set_pen')
    
    teleport_abs = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
    set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
    
    # 定义开关画笔的函数
    def pen_on():
        set_pen(255, 255, 255, 3, 0) # 白色，线宽3，开启 (off=0)
    
    def pen_off():
        set_pen(0, 0, 0, 3, 1)       # 关闭 (off=1)

    rospy.loginfo("准备画圆柱体（无中间杂乱线条版）...")
    
    # 参数设置
    center_x = 5.5
    center_y = 5.5
    a = 2.0      # 椭圆长轴半径
    b = 0.8      # 椭圆短轴半径
    height = 2.5 # 圆柱体高度
    steps = 80   # 椭圆平滑度

    # 一开始先关闭画笔，瞬移到初始位置，防止留下拖影
    pen_off()
    teleport_abs(center_x + a, center_y, 0.0) 
    rospy.sleep(0.5)

    # ========== 1. 画底面椭圆 ==========
    rospy.loginfo("画底面...")
    pen_on() # 开启画笔
    for i in range(steps + 1):
        theta = 2 * math.pi * i / steps
        target_x = center_x + a * math.cos(theta)
        target_y = center_y + b * math.sin(theta)
        teleport_abs(target_x, target_y, 0.0)
        rospy.sleep(0.015) # 画线时给一点点延迟，让它看起来像在走

    # ========== 2. 画右侧竖线 ==========
    rospy.loginfo("画右侧竖线...")
    pen_off() # 关画笔，瞬移到右侧起点
    teleport_abs(center_x + a, center_y, 0.0)
    rospy.sleep(0.2)
    
    pen_on() # 开画笔，往上画
    for h in range(1, 20):
        teleport_abs(center_x + a, center_y + (height * h / 19), 0.0)
        rospy.sleep(0.02)

    # ========== 3. 画左侧竖线 ==========
    rospy.loginfo("画左侧竖线...")
    pen_off() # 关画笔，瞬移到左侧起点
    teleport_abs(center_x - a, center_y, 0.0)
    rospy.sleep(0.2)
    
    pen_on() # 开画笔，往上画
    for h in range(1, 20):
        teleport_abs(center_x - a, center_y + (height * h / 19), 0.0)
        rospy.sleep(0.02)

    # ========== 4. 画顶面椭圆 ==========
    rospy.loginfo("画顶面...")
    pen_off() # 关画笔，瞬移到顶面起点
    teleport_abs(center_x + a, center_y + height, 0.0)
    rospy.sleep(0.2)
    
    pen_on() # 开画笔，画顶面
    for i in range(steps + 1):
        theta = 2 * math.pi * i / steps
        target_x = center_x + a * math.cos(theta)
        target_y = center_y + height + b * math.sin(theta)
        teleport_abs(target_x, target_y, 0.0)
        rospy.sleep(0.015)

    rospy.loginfo("完美的圆柱体画好了！")
    # 结束时把海龟停在旁边，关掉画笔
    pen_off()
    teleport_abs(center_x + a + 1.5, center_y + height/2, 0.0)

if __name__ == '__main__':
    try:
        draw_cylinder_v2()
    except rospy.ROSInterruptException:
        pass