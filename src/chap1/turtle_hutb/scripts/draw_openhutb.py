#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS Noetic turtlesim demo.

控制小海龟自动绘制 OpenHUTB。
使用 /turtle1/pose 进行闭环位置控制，
使用 /turtle1/set_pen 控制画笔，
使用 /turtle1/teleport_absolute 在不同笔画之间移动。
"""

import math

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, TeleportAbsolute


current_pose = None

cmd_pub = None
set_pen_client = None
teleport_client = None
clear_client = None


# 使用亮黄色，与 turtlesim 蓝色背景形成明显对比
PEN_R = 255
PEN_G = 255
PEN_B = 0
PEN_WIDTH = 4


def pose_callback(msg):
    global current_pose
    current_pose = msg


def normalize_angle(angle):
    """将角度限制到 [-pi, pi]。"""

    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def stop_turtle():
    """立即停止海龟。"""

    if cmd_pub is not None:
        cmd_pub.publish(Twist())


def shutdown_handler():
    """程序退出时确保海龟停止。"""

    stop_turtle()


def set_pen(off):
    """
    设置画笔状态。

    off=True  : 关闭画笔
    off=False : 打开画笔
    """

    try:
        set_pen_client(
            PEN_R,
            PEN_G,
            PEN_B,
            PEN_WIDTH,
            1 if off else 0
        )

        rospy.sleep(0.05)

    except rospy.ServiceException as error:
        rospy.logerr("设置画笔失败: %s", error)


def teleport_to(x, y, theta=0.0):
    """关闭画笔后将海龟移动到指定位置。"""

    try:
        teleport_client(x, y, theta)
        rospy.sleep(0.12)

    except rospy.ServiceException as error:
        rospy.logerr("移动海龟失败: %s", error)


def move_to_point(target_x, target_y, tolerance=0.035):
    """
    基于 /turtle1/pose 的闭环位置控制。

    海龟首先调整方向，然后向目标点移动。
    """

    rate = rospy.Rate(40)
    start_time = rospy.Time.now()

    while not rospy.is_shutdown():

        if current_pose is None:
            rate.sleep()
            continue

        dx = target_x - current_pose.x
        dy = target_y - current_pose.y

        distance = math.hypot(dx, dy)

        if distance < tolerance:
            break

        target_theta = math.atan2(dy, dx)

        angle_error = normalize_angle(
            target_theta - current_pose.theta
        )

        cmd = Twist()

        # 朝向误差较大时先原地转向
        if abs(angle_error) > 0.20:

            cmd.linear.x = 0.0

            cmd.angular.z = 4.0 * angle_error
            cmd.angular.z = max(
                -2.8,
                min(2.8, cmd.angular.z)
            )

        else:

            # 接近目标点时自动降低速度
            cmd.linear.x = 1.8 * distance
            cmd.linear.x = max(
                0.12,
                min(1.8, cmd.linear.x)
            )

            cmd.angular.z = 4.0 * angle_error
            cmd.angular.z = max(
                -2.0,
                min(2.0, cmd.angular.z)
            )

        cmd_pub.publish(cmd)

        elapsed = (
            rospy.Time.now() - start_time
        ).to_sec()

        if elapsed > 12.0:

            rospy.logwarn(
                "移动到目标点 (%.2f, %.2f) 超时",
                target_x,
                target_y
            )

            break

        rate.sleep()

    stop_turtle()
    rospy.sleep(0.06)


def draw_polyline(points):
    """按照给定坐标依次绘制一组连续线段。"""

    if len(points) < 2:
        return

    x0, y0 = points[0]
    x1, y1 = points[1]

    heading = math.atan2(
        y1 - y0,
        x1 - x0
    )

    # 不同笔画之间关闭画笔
    set_pen(True)

    teleport_to(
        x0,
        y0,
        heading
    )

    # 到达笔画起点以后打开画笔
    set_pen(False)

    for x, y in points[1:]:

        if rospy.is_shutdown():
            return

        move_to_point(x, y)

    stop_turtle()

    # 当前笔画结束后关闭画笔
    set_pen(True)

    rospy.sleep(0.08)


def ellipse_points(
        center_x,
        center_y,
        radius_x,
        radius_y,
        start_angle=0.0,
        end_angle=2.0 * math.pi,
        count=24
):
    """
    生成椭圆上的多个点。
    用折线逼近 O、p、e 等曲线。
    """

    points = []

    for i in range(count + 1):

        ratio = float(i) / count

        angle = (
            start_angle
            + ratio * (end_angle - start_angle)
        )

        x = center_x + radius_x * math.cos(angle)
        y = center_y + radius_y * math.sin(angle)

        points.append((x, y))

    return points


# ============================================================
# Open
# ============================================================

def draw_o():
    """绘制大写 O。"""

    rospy.loginfo("正在绘制 O")

    points = ellipse_points(
        center_x=1.00,
        center_y=5.60,
        radius_x=0.48,
        radius_y=1.65,
        count=28
    )

    draw_polyline(points)


def draw_p():
    """绘制小写 p。"""

    rospy.loginfo("正在绘制 p")

    # 竖线，包括向下延伸部分
    draw_polyline([
        (1.80, 3.25),
        (1.80, 6.40)
    ])

    # p 的圆环
    points = ellipse_points(
        center_x=2.16,
        center_y=5.72,
        radius_x=0.36,
        radius_y=0.68,
        count=20
    )

    draw_polyline(points)


def draw_e():
    """绘制小写 e。"""

    rospy.loginfo("正在绘制 e")

    draw_polyline([
        (2.62, 5.25),  # 从中间横线左端附近开始
        (3.38, 5.25),  # 先画中间横线，保证像 e

        (3.30, 5.55),
        (3.08, 5.80),
        (2.78, 5.90),
        (2.52, 5.84),
        (2.30, 5.66),
        (2.16, 5.38),

        (2.18, 5.05),
        (2.34, 4.80),
        (2.58, 4.65),
        (2.88, 4.60),
        (3.14, 4.66),
        (3.36, 4.82),
        (3.48, 5.02)
    ])


def draw_n():
    """绘制小写 n。"""

    rospy.loginfo("正在绘制 n")

    draw_polyline([
        (3.95, 4.50),
        (3.95, 6.35),
        (3.95, 5.90),
        (4.20, 6.25),
        (4.47, 6.35),
        (4.70, 6.20),
        (4.83, 5.90),
        (4.85, 4.50)
    ])


# ============================================================
# Open
# ============================================================

def draw_o():
    """绘制大写 O。"""

    rospy.loginfo("正在绘制 O")

    points = ellipse_points(
        center_x=0.95,
        center_y=5.55,
        radius_x=0.45,
        radius_y=1.65,
        count=32
    )

    draw_polyline(points)


def draw_p():
    """绘制小写 p。"""

    rospy.loginfo("正在绘制 p")

    # p 的竖线，适当缩短下伸部分
    draw_polyline([
        (1.55, 3.85),
        (1.55, 6.20)
    ])

    # p 的圆肚
    points = ellipse_points(
        center_x=1.88,
        center_y=5.55,
        radius_x=0.33,
        radius_y=0.60,
        count=24
    )

    draw_polyline(points)


def draw_e():
    """绘制小写 e，让右侧开口更明显。"""

    rospy.loginfo("正在绘制 e")

    draw_polyline([
        # 先从中间向右画横线
        (2.40, 5.25),
        (3.18, 5.25),

        # 从右端向上弯
        (3.12, 5.52),
        (2.98, 5.75),
        (2.78, 5.88),
        (2.56, 5.87),
        (2.38, 5.74),

        # 左侧向下
        (2.27, 5.52),
        (2.24, 5.27),
        (2.29, 5.02),
        (2.40, 4.80),

        # 底部向右收尾
        (2.57, 4.63),
        (2.78, 4.55),
        (2.98, 4.58),
        (3.14, 4.70)
    ])
def draw_n():
    """绘制小写 n。"""

    rospy.loginfo("正在绘制 n")

    # 左竖线
    draw_polyline([
        (3.45, 4.30),
        (3.45, 6.15)
    ])

    # n 的拱形
    draw_polyline([
        (3.45, 5.80),
        (3.58, 6.02),
        (3.76, 6.16),
        (3.97, 6.17),
        (4.16, 6.08),
        (4.29, 5.88),
        (4.34, 5.62),
        (4.34, 4.30)
    ])


# ============================================================
# HUTB
# ============================================================

def draw_h():
    """绘制 H。"""

    rospy.loginfo("正在绘制 H")

    # 左竖线
    draw_polyline([
        (4.70, 3.65),
        (4.70, 7.45)
    ])

    # 右竖线
    draw_polyline([
        (5.45, 3.65),
        (5.45, 7.45)
    ])

    # 中间横线
    draw_polyline([
        (4.70, 5.55),
        (5.45, 5.55)
    ])


def draw_u():
    """
    绘制较窄的 U。
    """

    rospy.loginfo("正在绘制 U")

    draw_polyline([
        (5.80, 7.45),

        # 左侧向下
        (5.80, 4.20),

        # U 底部圆弧
        (5.84, 3.98),
        (5.94, 3.78),
        (6.09, 3.64),
        (6.28, 3.58),
        (6.47, 3.62),
        (6.62, 3.75),
        (6.72, 3.94),
        (6.76, 4.18),

        # 右侧向上
        (6.76, 7.45)
    ])


def draw_t():
    """绘制 T。"""

    rospy.loginfo("正在绘制 T")

    # 顶部横线
    draw_polyline([
        (7.10, 7.45),
        (8.05, 7.45)
    ])

    # 中间竖线
    draw_polyline([
        (7.575, 7.45),
        (7.575, 3.65)
    ])


def draw_b():
    """绘制 B。"""

    rospy.loginfo("正在绘制 B")

    # 左侧竖线
    draw_polyline([
        (8.35, 3.65),
        (8.35, 7.45)
    ])

    # 上半圆
    draw_polyline([
        (8.35, 7.45),
        (8.85, 7.45),
        (9.10, 7.40),
        (9.30, 7.26),
        (9.43, 7.05),
        (9.47, 6.80),
        (9.42, 6.55),
        (9.28, 6.34),
        (9.08, 6.21),
        (8.84, 6.16),
        (8.35, 6.16)
    ])

    # 下半圆
    draw_polyline([
        (8.35, 6.16),
        (8.88, 6.16),
        (9.15, 6.10),
        (9.37, 5.94),
        (9.51, 5.68),
        (9.56, 5.38),
        (9.52, 5.05),
        (9.38, 4.74),
        (9.17, 4.49),
        (8.90, 4.34),
        (8.35, 4.34)
    ])


def draw_openhutb():
    """依次绘制 OpenHUTB。"""

    draw_o()
    draw_p()
    draw_e()
    draw_n()

    draw_h()
    draw_u()
    draw_t()
    draw_b()


def main():

    global cmd_pub
    global set_pen_client
    global teleport_client
    global clear_client

    rospy.init_node('draw_openhutb_node')

    rospy.on_shutdown(shutdown_handler)

    cmd_pub = rospy.Publisher(
        '/turtle1/cmd_vel',
        Twist,
        queue_size=10
    )

    rospy.Subscriber(
        '/turtle1/pose',
        Pose,
        pose_callback
    )

    rospy.loginfo("等待 turtlesim 服务...")

    rospy.wait_for_service('/turtle1/set_pen')
    rospy.wait_for_service('/turtle1/teleport_absolute')
    rospy.wait_for_service('/clear')

    set_pen_client = rospy.ServiceProxy(
        '/turtle1/set_pen',
        SetPen
    )

    teleport_client = rospy.ServiceProxy(
        '/turtle1/teleport_absolute',
        TeleportAbsolute
    )

    clear_client = rospy.ServiceProxy(
        '/clear',
        Empty
    )

    rospy.loginfo("等待 /turtle1/pose ...")

    while (
        not rospy.is_shutdown()
        and current_pose is None
    ):
        rospy.sleep(0.1)

    rospy.sleep(0.5)

    try:
        clear_client()

    except rospy.ServiceException as error:
        rospy.logerr("清空画布失败: %s", error)

    rospy.loginfo("==============================")
    rospy.loginfo("开始绘制 OpenHUTB")
    rospy.loginfo("==============================")

    draw_openhutb()

    stop_turtle()
    set_pen(True)

    # 最终把海龟放到右下角，避免遮挡文字
    teleport_to(
        10.50,
        1.00,
        math.pi
    )

    rospy.loginfo("==============================")
    rospy.loginfo("OpenHUTB 绘制完成")
    rospy.loginfo("==============================")

    rospy.sleep(1.0)


if __name__ == '__main__':

    try:
        main()

    except rospy.ROSInterruptException:
        pass
