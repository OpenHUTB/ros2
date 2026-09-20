#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen
from std_srvs.srv import Empty

current_pose = None


def pose_callback(data):
    global current_pose
    current_pose = data


# 贝塞尔曲线生成器（保证翅膀是平滑的圆弧）
def bezier_curve(p0, p1, p2, p3, num_points=80):
    points = []
    for i in range(num_points):
        t = i / (num_points - 1.0)
        x = (1 - t) ** 3 * p0[0] + 3 * (1 - t) ** 2 * t * p1[0] + 3 * (1 - t) * t ** 2 * p2[0] + t ** 3 * p3[0]
        y = (1 - t) ** 3 * p0[1] + 3 * (1 - t) ** 2 * t * p1[1] + 3 * (1 - t) * t ** 2 * p2[1] + t ** 3 * p3[1]
        points.append((x, y))
    return points


def generate_butterfly_waypoints():
    """使用贝塞尔曲线构建真正的蝴蝶翅膀"""
    parts = {}
    cx, cy = 5.5, 5.5  # 身体中心

    # 1. 右前翅（外侧弧线 + 内侧弧线）
    right_front_outer = bezier_curve((cx, cy + 0.8), (6.5, 8.5), (9.0, 8.5), (8.8, 6.2), 60)
    right_front_inner = bezier_curve((8.8, 6.2), (8.5, 5.0), (7.0, 4.8), (cx + 0.2, cy), 60)
    parts['right_front'] = right_front_outer + right_front_inner

    # 2. 右后翅（更小巧圆润）
    right_back_outer = bezier_curve((cx + 0.2, cy - 0.2), (7.0, 4.5), (8.5, 3.0), (7.8, 2.2), 60)
    right_back_inner = bezier_curve((7.8, 2.2), (6.5, 1.5), (6.0, 3.5), (cx, cy - 1.0), 60)
    parts['right_back'] = right_back_outer + right_back_inner

    # 3. 左前翅（镜像右前翅）
    left_front_outer = bezier_curve((cx, cy + 0.8), (4.5, 8.5), (2.0, 8.5), (2.2, 6.2), 60)
    left_front_inner = bezier_curve((2.2, 6.2), (2.5, 5.0), (4.0, 4.8), (cx - 0.2, cy), 60)
    parts['left_front'] = left_front_outer + left_front_inner

    # 4. 左后翅（镜像右后翅）
    left_back_outer = bezier_curve((cx - 0.2, cy - 0.2), (4.0, 4.5), (2.5, 3.0), (3.2, 2.2), 60)
    left_back_inner = bezier_curve((3.2, 2.2), (4.5, 1.5), (5.0, 3.5), (cx, cy - 1.0), 60)
    parts['left_back'] = left_back_outer + left_back_inner

    # 5. 身体（从头部到尾部，直线）
    parts['body'] = [(cx, cy + 1.5), (cx, cy - 1.8)]

    # 6. 触角（两根弯曲的线）
    parts['left_antenna'] = [(cx, cy + 1.5), (cx - 0.5, cy + 2.5), (cx - 1.2, cy + 3.0)]
    parts['right_antenna'] = [(cx, cy + 1.5), (cx + 0.5, cy + 2.5), (cx + 1.2, cy + 3.0)]

    return parts


def draw_path(waypoints, pub, rate):
    """控制海龟极其平滑地追踪轮廓"""
    global current_pose

    for target_x, target_y in waypoints:
        while not rospy.is_shutdown():
            dx = target_x - current_pose.x
            dy = target_y - current_pose.y
            distance = math.sqrt(dx ** 2 + dy ** 2)

            if distance < 0.1:
                break

            target_theta = math.atan2(dy, dx)
            d_theta = target_theta - current_pose.theta
            d_theta = math.atan2(math.sin(d_theta), math.cos(d_theta))

            cmd = Twist()

            # 核心防画圈逻辑：遇到急转弯，线速度降到极低，原地转向
            if abs(d_theta) > 0.4:
                cmd.linear.x = 0.05
                cmd.angular.z = 4.0 * d_theta
            else:
                cmd.linear.x = 0.8 * distance
                cmd.angular.z = 3.0 * d_theta

            cmd.linear.x = min(cmd.linear.x, 0.8)
            cmd.angular.z = max(min(cmd.angular.z, 4.0), -4.0)

            pub.publish(cmd)
            rate.sleep()

    # 刹车
    pub.publish(Twist())
    rospy.sleep(0.1)


def move_turtle():
    global current_pose
    rospy.init_node('turtle_butterfly_node', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    while current_pose is None and not rospy.is_shutdown():
        rospy.sleep(0.1)

    rospy.loginfo("准备画蝴蝶...")
    rate = rospy.Rate(50)

    # 等待服务上线
    rospy.wait_for_service('/turtle1/teleport_absolute')
    rospy.wait_for_service('/turtle1/set_pen')
    rospy.wait_for_service('/clear')

    try:
        clear_bg = rospy.ServiceProxy('/clear', Empty)
        clear_bg()
    except rospy.ServiceException:
        pass

    parts = generate_butterfly_waypoints()
    # 绘制顺序：右前翅 -> 右后翅 -> 左前翅 -> 左后翅 -> 身体 -> 触角
    order = [
        ('right_front', '右前翅'), ('right_back', '右后翅'),
        ('left_front', '左前翅'), ('left_back', '左后翅'),
        ('body', '身体'), ('left_antenna', '左触角'), ('right_antenna', '右触角')
    ]

    for key, name in order:
        waypoints = parts[key]
        start_x, start_y = waypoints[0]

        # 1. 抬笔 (off=1)
        set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        set_pen(255, 255, 255, 2, 1)

        # 2. 瞬移到起点
        teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        teleport(start_x, start_y, 0)

        # 3. 强制更新本地位置，防止程序误判导致连线
        current_pose.x = start_x
        current_pose.y = start_y
        current_pose.theta = 0
        rospy.sleep(0.2)

        # 4. 落笔 (off=0) —— 使用白色画笔 (255, 255, 255)
        set_pen(255, 255, 255, 2, 0)

        rospy.loginfo(f"正在画{name}...")
        draw_path(waypoints, pub, rate)

    # 画完后抬笔，收工
    set_pen(255, 255, 255, 2, 1)
    rospy.loginfo("蝴蝶画完啦！")


if __name__ == '__main__':
    try:
        move_turtle()
    except rospy.ROSInterruptException:
        pass