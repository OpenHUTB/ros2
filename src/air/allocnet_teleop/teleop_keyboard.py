#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
teleop_keyboard.py -- AllocNet 键盘遥操作 / 航点下发节点

在 RViz 仿真中以键盘驱动一个"虚拟航点游标"，按 G 将该游标作为 Goal
发送给 AllocNet 规划器（learning_planning_node），触发时间分配与
避障轨迹快速规划。

按键
----
    W / S       前进 / 后退
    A / D       向左 / 向右平移
    I / 空格    上升
    K           下降
    Q / E       偏航角 -/+
    R           重置到初始状态
    G           发送当前航点到 AllocNet（见下方"两段式语义"）
    + / -       调整平移步长
    H           显示帮助
    Ctrl-C      退出

两段式语义（继承自上游 C++ 规划器）
-----------------------------------
`learning_planning.cpp` 的 `targetCallBack` 要求收到 **两个** 目标点才会
调用 `plan()`：第一个是起点，第二个是终点。因此：

    第 1 次按 G  -> 设置起点 (START)
    第 2 次按 G  -> 设置终点 (GOAL)，随即触发 AllocNet 规划
    第 3 次按 G  -> 清空并作为新的起点 …此后循环

本节点会在终端明确打印当前处于哪一阶段，避免误操作。

关于高度与偏航（重要）
----------------------
上游规划器 **忽略** `pose.position.z`，实际高度由
`pose.orientation.z` 作为 [0,1] 归一化比例决定：

    z_goal = z_origin + inflate_radius
             + |orientation.z| * (z_size - 2 * inflate_radius)

因此本节点把游标高度换算成该比例后写入 `orientation.z`。
`orientation.z` 被高度占用，**偏航角无法通过该消息传给规划器**
（AllocNet 的 Goal 不支持终端偏航约束）；Q/E 仅改变游标朝向，
影响 W/S/A/D 的推进方向，并在 RViz 中通过箭头显示。

终端说明
--------
`Shift` 在原始(raw)终端下不产生独立 ASCII 码，无法单独捕获，
故使用题面给出的备选键 I / K 承担升降。
"""

import math
import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

# --------------------------------------------------------------------------
# 终端显示
# --------------------------------------------------------------------------
BANNER = r"""
+--------------------------------------------------------------------------+
|            AllocNet 键盘遥操作节点  (teleop_keyboard)                     |
+--------------------------------------------------------------------------+
|   当前游标:  x={x:7.2f}  y={y:7.2f}  z={z:5.2f}  yaw={yaw:6.1f} deg
|   平移步长: {step:.2f} m      高度步长: {zstep:.2f} m
|   日志文件: {log}
+--------------------------------------------------------------------------+
|   W/S 前进/后退      A/D 左移/右移      I/空格 上升   K 下降
|   Q/E 偏航 -/+       R 重置初始状态     G 发送航点到 AllocNet
|   +/- 调整步长       H 帮助             Ctrl-C 退出
+--------------------------------------------------------------------------+
|   阶段: {phase}
+--------------------------------------------------------------------------+
"""

HELP = """
按键速查
--------
  W / S        前进 / 后退（沿当前偏航方向）
  A / D        向左 / 向右平移
  I 或 空格    上升
  K            下降
  Q / E        偏航角 -/+ （仅改变游标朝向）
  R            重置游标到初始状态
  G            发送当前航点给 AllocNet
                 第 1 次 = 起点，第 2 次 = 终点并触发规划，如此循环
  + / -        增大 / 减小平移步长
  H            显示本帮助
  Ctrl-C       退出

提示
----
  * 高度范围受地图限制，会被自动裁剪到 [z_origin+dilate, z_origin+z_size-dilate]。
  * Shift 无法在 raw 终端中单独捕获，请用 I / K 升降。
  * 上游规划器不支持终端偏航约束，Q/E 只影响游标朝向。
"""


class TeleopKeyboard(object):
    """非阻塞键盘监听 -> 虚拟航点游标 -> AllocNet Goal。"""

    def __init__(self):
        # ----- 坐标系与地图参数（需与 learning_planning.launch 保持一致）----
        self.frame_id = rospy.get_param("~frame_id", "odom")
        self.map_x_size = rospy.get_param("~map_x_size", 20.0)
        self.map_y_size = rospy.get_param("~map_y_size", 20.0)
        self.map_z_size = rospy.get_param("~map_z_size", 5.0)
        self.map_x_origin = rospy.get_param("~map_x_origin", -10.0)
        self.map_y_origin = rospy.get_param("~map_y_origin", -10.0)
        self.map_z_origin = rospy.get_param("~map_z_origin", 0.0)
        self.dilate = rospy.get_param("~inflate_radius", 0.2)

        # ----- 步长 -----
        self.step = rospy.get_param("~step", 0.5)
        self.z_step = rospy.get_param("~z_step", 0.25)
        self.yaw_step = math.radians(rospy.get_param("~yaw_step_deg", 10.0))
        self.step_min = 0.1
        self.step_max = 5.0

        # ----- 初始状态 -----
        self.home = [
            rospy.get_param("~init_x", 0.0),
            rospy.get_param("~init_y", 0.0),
            rospy.get_param("~init_z", 1.0),
            rospy.get_param("~init_yaw_deg", 0.0) * math.pi / 180.0,
        ]

        # ----- 游标状态 -----
        self.x, self.y, self.z, self.yaw = list(self.home)
        self.z = self.clamp_z(self.z)

        # 已下发的航点（用于画轨迹预览）
        self.sent_waypoints = []
        self.goal_seq = 0          # 已按 G 的次数
        self.phase = "等待第 1 次 G：设置起点 (START)"
        self.log_path = rospy.get_param("~log_path", "")

        # ----- ROS 接口 -----
        # 游标与航点用 latch=True：RViz 可能比节点晚启动，
        # 非 latched 话题会让 RViz 错过消息、显示为空白。
        self.goal_pub = rospy.Publisher(
            rospy.get_param("~goal_topic", "/move_base_simple/goal"),
            PoseStamped, queue_size=10)
        self.cursor_pub = rospy.Publisher(
            "/teleop/cursor", Marker, queue_size=10, latch=True)
        self.path_pub = rospy.Publisher(
            "/teleop/waypoints", MarkerArray, queue_size=10, latch=True)
        self.goal_marker_pub = rospy.Publisher(
            "/teleop/goal_markers", MarkerArray, queue_size=10, latch=True)

        # 无 TTY 时的远程注入通道：向 /teleop/key 发单字符即可驱动
        # （launch 场景下键盘节点读不到 stdin，见 run() 的说明）
        self.key_sub = rospy.Subscriber(
            rospy.get_param("~key_topic", "/teleop/key"),
            String, self.on_key_msg, queue_size=100)

        # 定时刷新 RViz 中的游标
        self.timer = rospy.Timer(rospy.Duration(0.1), self.on_timer)

    def on_key_msg(self, msg):
        """从话题注入按键（用于 launch / 无 TTY 场景）。"""
        data = msg.data or ""
        # 支持一次发多个字符，也支持 "space" 这种写法
        if data.strip().lower() in ("space", " "):
            data = " "
        for ch in data:
            if not self.on_key(ch):
                rospy.signal_shutdown("remote quit")
                return

    # ------------------------------------------------------------------
    # 工具函数
    # ------------------------------------------------------------------
    def clamp_z(self, z):
        """将高度裁剪到规划器可用区间。"""
        lo = self.map_z_origin + self.dilate
        hi = self.map_z_origin + self.map_z_size - self.dilate
        return max(lo, min(hi, z))

    def altitude_to_orientation_z(self, z):
        """把真实高度换算成规划器期望的 orientation.z 归一化比例。

        对应 learning_planning.cpp:198-200
            zGoal = mapBound[4] + dilateRadius
                    + |ori.z| * (mapBound[5] - mapBound[4] - 2 * dilateRadius)
        """
        span = self.map_z_size - 2.0 * self.dilate
        if span <= 0.0:
            return 0.0
        ratio = (z - self.map_z_origin - self.dilate) / span
        return max(0.0, min(1.0, ratio))

    # ------------------------------------------------------------------
    # 按键处理
    # ------------------------------------------------------------------
    def on_key(self, key):
        """处理单个按键，返回 False 表示请求退出。"""
        if key in ("w", "W"):
            self.x += self.step * math.cos(self.yaw)
            self.y += self.step * math.sin(self.yaw)
        elif key in ("s", "S"):
            self.x -= self.step * math.cos(self.yaw)
            self.y -= self.step * math.sin(self.yaw)
        elif key in ("a", "A"):
            self.x -= self.step * math.sin(self.yaw)
            self.y += self.step * math.cos(self.yaw)
        elif key in ("d", "D"):
            self.x += self.step * math.sin(self.yaw)
            self.y -= self.step * math.cos(self.yaw)
        elif key in ("i", "I", " "):
            self.z += self.z_step
        elif key in ("k", "K"):
            self.z -= self.z_step
        elif key in ("q", "Q"):
            self.yaw -= self.yaw_step
        elif key in ("e", "E"):
            self.yaw += self.yaw_step
        elif key in ("r", "R"):
            self.reset()
        elif key in ("g", "G"):
            self.send_goal()
        elif key in ("+", "="):
            self.step = min(self.step_max, self.step + 0.1)
        elif key in ("-", "_"):
            self.step = max(self.step_min, self.step - 0.1)
        elif key in ("h", "H", "?"):
            sys.stdout.write(HELP)
        elif key == "\x03":          # Ctrl-C
            return False
        else:
            return True

        self.z = self.clamp_z(self.z)
        self.x = max(self.map_x_origin,
                     min(self.map_x_origin + self.map_x_size, self.x))
        self.y = max(self.map_y_origin,
                     min(self.map_y_origin + self.map_y_size, self.y))
        self.render()
        return True

    def reset(self):
        self.x, self.y, self.z, self.yaw = list(self.home)
        self.z = self.clamp_z(self.z)
        self.sent_waypoints = []
        self.goal_seq = 0
        self.phase = "等待第 1 次 G：设置起点 (START)"
        rospy.loginfo("[teleop] 已重置到初始状态 (%.2f, %.2f, %.2f)",
                      self.x, self.y, self.z)

    # ------------------------------------------------------------------
    # 下发 Goal
    # ------------------------------------------------------------------
    def send_goal(self):
        """把当前游标作为 Goal 发给 AllocNet。"""
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        # position.z 会被规划器忽略，这里仍填真值方便其它工具消费
        msg.pose.position.z = self.z
        # 关键：高度编码进 orientation.z（见模块 docstring）
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = self.altitude_to_orientation_z(self.z)
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

        self.goal_seq += 1
        self.sent_waypoints.append((self.x, self.y, self.z))

        if self.goal_seq % 2 == 1:
            idx = len(self.sent_waypoints) - 1
            self.phase = "已设起点 #%d，等待第 2 次 G 触发规划" % (
                (self.goal_seq + 1) // 2)
            rospy.loginfo(
                "[teleop] START #%d -> (%.2f, %.2f, %.2f)  已记录起点，"
                "再按一次 G 设置终点并触发 AllocNet 规划",
                (self.goal_seq + 1) // 2, self.x, self.y, self.z)
        else:
            self.phase = "规划已触发，等待第 1 次 G 设置新起点"
            rospy.loginfo(
                "[teleop] GOAL #%d -> (%.2f, %.2f, %.2f)  "
                "已发送终点，AllocNet 开始时间分配与避障轨迹规划",
                self.goal_seq // 2, self.x, self.y, self.z)

        self.publish_waypoints()

    # ------------------------------------------------------------------
    # RViz 可视化
    # ------------------------------------------------------------------
    def make_marker(self, ns, mid, scale, color, mtype=Marker.SPHERE):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color = color
        m.pose.orientation.w = 1.0
        return m

    def publish_cursor(self):
        # 球体表示位置
        m = self.make_marker(
            "teleop_cursor", 0, 0.35,
            ColorRGBA(1.0, 0.55, 0.0, 0.95))
        m.pose.position.x = self.x
        m.pose.position.y = self.y
        m.pose.position.z = self.z
        self.cursor_pub.publish(m)

        # 箭头表示偏航
        a = self.make_marker(
            "teleop_cursor", 1, 0.5,
            ColorRGBA(1.0, 0.85, 0.2, 1.0), Marker.ARROW)
        a.pose.position.x = self.x
        a.pose.position.y = self.y
        a.pose.position.z = self.z
        a.pose.orientation.z = math.sin(self.yaw / 2.0)
        a.pose.orientation.w = math.cos(self.yaw / 2.0)
        a.scale.x = 0.8   # 箭头长度
        a.scale.y = 0.12
        a.scale.z = 0.12
        self.cursor_pub.publish(a)

    def publish_waypoints(self):
        """把已下发的航点连成折线，便于在 RViz 中预览键盘路径。"""
        arr = MarkerArray()
        for i, (wx, wy, wz) in enumerate(self.sent_waypoints):
            m = self.make_marker(
                "teleop_waypoints", i, 0.3,
                ColorRGBA(0.2, 0.9, 0.3, 0.9))
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = wz
            arr.markers.append(m)

        line = self.make_marker(
            "teleop_waypoints", 999, 0.08,
            ColorRGBA(0.2, 0.7, 1.0, 0.8), Marker.LINE_STRIP)
        for wx, wy, wz in self.sent_waypoints:
            p = PoseStamped().pose.position
            p.x, p.y, p.z = wx, wy, wz
            line.points.append(p)
        arr.markers.append(line)
        self.path_pub.publish(arr)
        self.publish_goal_markers()

    def publish_goal_markers(self):
        """发布起点/终点标记到 /teleop/goal_markers。

        此前这个 publisher 建了却从未发布过任何消息（死代码），
        RViz 里加了这个图层也是空的。
        约定：奇数下标为 START，偶数下标为 GOAL —— 因为按 G 是两段式的。
        """
        arr = MarkerArray()
        for i, (wx, wy, wz) in enumerate(self.sent_waypoints):
            is_start = (i % 2 == 0)
            color = (ColorRGBA(0.55, 0.34, 0.29, 1.0) if is_start
                     else ColorRGBA(0.89, 0.47, 0.76, 1.0))
            m = self.make_marker("teleop_goal_markers", i, 0.6, color,
                                 Marker.SPHERE)
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = wz
            arr.markers.append(m)

            # 文字标签，RViz 里能直接看出哪个是起点哪个是终点
            t = self.make_marker("teleop_goal_markers", 100 + i, 0.0, color,
                                 Marker.TEXT_VIEW_FACING)
            t.pose.position.x = wx
            t.pose.position.y = wy
            t.pose.position.z = wz + 0.6
            t.text = ("START #%d" if is_start else "GOAL #%d") % (i // 2 + 1)
            t.scale.z = 0.35
            arr.markers.append(t)

        # 清掉多余的旧标记（航点变少时 RViz 会残留）
        for j in range(len(self.sent_waypoints), len(self.sent_waypoints) + 6):
            for mid in (j, 100 + j):
                d = self.make_marker("teleop_goal_markers", mid, 0.0,
                                     ColorRGBA(0, 0, 0, 0))
                d.action = Marker.DELETE
                arr.markers.append(d)

        self.goal_marker_pub.publish(arr)

    def on_timer(self, _evt):
        self.publish_cursor()

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------
    def render(self):
        sys.stdout.write("\r" + BANNER.format(
            x=self.x, y=self.y, z=self.z,
            yaw=math.degrees(self.yaw),
            step=self.step, zstep=self.z_step,
            log=self.log_path or "(未启用)",
            phase=self.phase))
        sys.stdout.flush()

    def run(self):
        self.render()
        rospy.loginfo("[teleop] 键盘节点已启动，按 H 查看帮助，Ctrl-C 退出。")

        # 先发布一次，让 RViz 立刻能看到游标（配合 latch=True 常驻显示）
        self.publish_cursor()
        self.publish_waypoints()

        interactive = self._stdin_is_tty()
        if not interactive:
            # ⚠ 关键：从 launch 启动时 stdin 是 /dev/null，
            # 一旦 read 返回空就退出，节点会在 RViz 里凭空消失。
            # 因此这里**不退出**，改为常驻：游标继续以 10Hz 发布，
            # 按键改由 /teleop/key 话题注入（见 on_key_msg）。
            rospy.logwarn(
                "[teleop] stdin 不是终端，键盘输入不可用 —— 节点将常驻运行，"
                "游标照常发布到 /teleop/cursor。"
                "如需真实键盘操作，请在终端里 rosrun planner teleop_keyboard.py；"
                "如需脚本驱动，向 /teleop/key 发 std_msgs/String 即可。")
            while not rospy.is_shutdown():
                rospy.sleep(0.2)
            return

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while not rospy.is_shutdown():
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not rlist:
                    continue
                key = sys.stdin.read(1)
                if not key:
                    # 终端被关闭（如 ssh 断开）→ 转为常驻而不是退出
                    rospy.logwarn("[teleop] stdin 已关闭，转为常驻模式。")
                    break
                if not self.on_key(key):
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            sys.stdout.write("\n")
            rospy.loginfo("[teleop] 终端属性已恢复。")

        # 退出交互循环后仍保持节点存活，使 RViz 图层不掉线
        while not rospy.is_shutdown():
            rospy.sleep(0.2)

    @staticmethod
    def _stdin_is_tty():
        """判断 stdin 是否是可交互终端。

        `sys.stdin` 在 launch 下可能是 None，直接调用 isatty() 会抛异常，
        因此这里逐层判断。
        """
        try:
            if sys.stdin is None:
                return False
            return bool(sys.stdin.isatty())
        except (ValueError, AttributeError, OSError):
            return False


def main():
    rospy.init_node("teleop_keyboard", anonymous=False)
    node = TeleopKeyboard()
    node.run()


if __name__ == "__main__":
    main()
