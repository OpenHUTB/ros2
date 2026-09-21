#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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


def pose_callback(msg):
    global current_pose
    current_pose = msg


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def stop_turtle():
    if cmd_pub is not None:
        cmd_pub.publish(Twist())


def shutdown_handler():
    stop_turtle()


def set_pen(off, r=255, g=255, b=0, width=4):
    try:
        set_pen_client(r, g, b, width, 1 if off else 0)
        rospy.sleep(0.05)

    except rospy.ServiceException as error:
        rospy.logerr("设置画笔失败: %s", error)


def teleport_to(x, y, theta=0.0):
    try:
        teleport_client(x, y, theta)
        rospy.sleep(0.12)

    except rospy.ServiceException as error:
        rospy.logerr("移动海龟失败: %s", error)


def move_to_point(target_x, target_y, tolerance=0.04):
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
        angle_error = normalize_angle(target_theta - current_pose.theta)

        cmd = Twist()

        if abs(angle_error) > 0.20:
            cmd.angular.z = max(-2.8, min(2.8, 4.0 * angle_error))
        else:
            cmd.linear.x = max(0.12, min(1.8, 1.8 * distance))
            cmd.angular.z = max(-2.0, min(2.0, 4.0 * angle_error))

        cmd_pub.publish(cmd)

        if (rospy.Time.now() - start_time).to_sec() > 12.0:
            rospy.logwarn("移动到目标点超时: (%.2f, %.2f)", target_x, target_y)
            break

        rate.sleep()

    stop_turtle()
    rospy.sleep(0.06)


def draw_polyline(points, r=255, g=255, b=0, width=4):
    if len(points) < 2:
        return

    x0, y0 = points[0]
    x1, y1 = points[1]
    heading = math.atan2(y1 - y0, x1 - x0)

    set_pen(True, r, g, b, width)
    teleport_to(x0, y0, heading)

    set_pen(False, r, g, b, width)

    for x, y in points[1:]:
        if rospy.is_shutdown():
            return
        move_to_point(x, y)

    stop_turtle()
    set_pen(True, r, g, b, width)
    rospy.sleep(0.08)


def ellipse_points(
        center_x,
        center_y,
        radius_x,
        radius_y,
        start_angle=0.0,
        end_angle=2.0 * math.pi,
        count=32
):
    points = []

    for i in range(count + 1):
        ratio = float(i) / count
        angle = start_angle + ratio * (end_angle - start_angle)

        x = center_x + radius_x * math.cos(angle)
        y = center_y + radius_y * math.sin(angle)

        points.append((x, y))

    return points


def draw_face():
    rospy.loginfo("正在绘制眯眼笑脸")

    # 脸部外轮廓
    face = ellipse_points(
        center_x=5.50,
        center_y=5.50,
        radius_x=3.70,
        radius_y=3.70,
        count=72
    )
    draw_polyline(face, 255, 180, 0, 7)

    # 左眼粉色眼圈
    left_eye = ellipse_points(
        center_x=4.15,
        center_y=6.45,
        radius_x=0.86,
        radius_y=1.12,
        count=32
    )
    draw_polyline(left_eye, 255, 190, 210, 5)

    # 左眼瞳孔
    left_pupil = ellipse_points(
        center_x=4.15,
        center_y=6.35,
        radius_x=0.12,
        radius_y=0.12,
        count=12
    )
    draw_polyline(left_pupil, 35, 15, 15, 12)

    # 右边半睁眼
    right_eye = [
        (6.35, 6.15),
        (6.38, 6.55),
        (6.52, 6.88),
        (6.78, 7.08),
        (7.10, 7.15),
        (7.42, 7.08),
        (7.64, 6.88),
        (7.73, 6.55),
        (7.73, 6.15),
        (6.35, 6.15)
    ]
    draw_polyline(right_eye, 255, 190, 210, 5)

    # 右眼瞳孔
    right_pupil = ellipse_points(
        center_x=7.10,
        center_y=6.35,
        radius_x=0.11,
        radius_y=0.11,
        count=12
    )
    draw_polyline(right_pupil, 35, 15, 15, 11)

    # 笑嘴
    mouth = ellipse_points(
        center_x=5.50,
        center_y=4.45,
        radius_x=1.70,
        radius_y=0.78,
        start_angle=math.pi,
        end_angle=2.0 * math.pi,
        count=30
    )
    draw_polyline(mouth, 100, 40, 10, 6)


def main():
    global cmd_pub
    global set_pen_client
    global teleport_client
    global clear_client

    rospy.init_node("draw_face_node")
    rospy.on_shutdown(shutdown_handler)

    cmd_pub = rospy.Publisher(
        "/turtle1/cmd_vel",
        Twist,
        queue_size=10
    )

    rospy.Subscriber(
        "/turtle1/pose",
        Pose,
        pose_callback
    )

    rospy.wait_for_service("/turtle1/set_pen")
    rospy.wait_for_service("/turtle1/teleport_absolute")
    rospy.wait_for_service("/clear")

    set_pen_client = rospy.ServiceProxy(
        "/turtle1/set_pen",
        SetPen
    )

    teleport_client = rospy.ServiceProxy(
        "/turtle1/teleport_absolute",
        TeleportAbsolute
    )

    clear_client = rospy.ServiceProxy("/clear", Empty)

    while not rospy.is_shutdown() and current_pose is None:
        rospy.sleep(0.1)

    clear_client()
    rospy.sleep(0.4)

    draw_face()

    set_pen(True)
    teleport_to(10.40, 1.00, math.pi)

    rospy.loginfo("表情绘制完成")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
