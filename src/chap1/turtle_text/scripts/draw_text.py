#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""turtle_text: 在 turtlesim 中输入单词或短句，小海龟把它们画在画布上。

三种用法：
    1. 交互模式（推荐，可以反复输入）：
       roscore
       rosrun turtlesim turtlesim_node
       rosrun turtle_text draw_text.py
    2. 一键启动（turtlesim + 本节点）：
       roslaunch turtle_text draw_text.launch
    3. 指定文字自动绘制，画完退出：
       roslaunch turtle_text draw_text.launch text:="Hello World"

画布固定 11 x 11 且不能缩放，所以字号由程序自动计算：
短单词一行画完并尽量放大，较长的短语按单词折成两到三行。
大小写各有字形：小写的 x 高、上升部、下降部都按排印惯例处理。
线宽随字号自动调整，大字用粗线，小字用细线。
"""

import math
import time

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

CANVAS = 11.0        # 画布边长
MARGIN = 1.0         # 四周留白
GAP_RATIO = 0.35     # 字距，单位是“字高”
SPACE_RATIO = 0.5    # 空格宽度，单位是“字高”
LINE_RATIO = 0.7     # 行距，单位是“字高”
MAX_LINES = 3        # 最多折成几行
MAX_SCALE = 3.0      # 字高上限，避免两个字母就占满整屏
SPEED = 1.0          # 速度系数：1.0 为标准，调大画得更快，但转角会更圆
PEN_RATIO = 1.8      # 线宽 = 字高 * PEN_RATIO（像素），最小 2 像素

# 大写字母的笔画表，坐标是“大字”刻度（2~9）。
# 程序会按统一尺子重新缩放，改字号不用动这里。
RAW_LETTERS = {
    "A": [[(2.0, 2.0), (5.0, 9.0), (8.0, 2.0)], [(3.6, 4.8), (6.4, 4.8)]],
    "B": [[(3.0, 2.0), (3.0, 9.0)],
          [(3.0, 9.0), (6.0, 9.0), (7.2, 8.4), (7.6, 7.4), (7.2, 6.4), (6.0, 5.6), (3.0, 5.5)],
          [(3.0, 5.5), (6.4, 5.4), (7.6, 4.7), (8.0, 3.5), (7.5, 2.5), (6.2, 2.0), (3.0, 2.0)]],
    "C": [[(7.8, 7.6), (6.6, 8.7), (5.0, 9.0), (3.6, 8.0), (3.0, 6.4),
           (3.0, 4.6), (3.6, 3.0), (5.0, 2.0), (6.6, 2.3), (7.8, 3.4)]],
    "D": [[(3.0, 2.0), (3.0, 9.0)],
          [(3.0, 9.0), (5.8, 9.0), (7.2, 8.2), (8.0, 6.6), (8.0, 4.4), (7.2, 2.8), (5.8, 2.0), (3.0, 2.0)]],
    "E": [[(7.5, 9.0), (3.0, 9.0), (3.0, 5.5), (6.5, 5.5)],
          [(3.0, 5.5), (3.0, 2.0), (7.5, 2.0)]],
    "F": [[(7.5, 9.0), (3.0, 9.0), (3.0, 2.0)],
          [(3.0, 5.6), (6.4, 5.6)]],
    "G": [[(7.8, 7.6), (6.6, 8.7), (5.0, 9.0), (3.6, 8.0), (3.0, 6.4),
           (3.0, 4.6), (3.7, 2.9), (5.3, 2.1), (6.9, 2.5), (7.9, 3.7), (7.9, 5.4), (6.0, 5.5)]],
    "H": [[(2.5, 2.0), (2.5, 9.0)], [(7.5, 2.0), (7.5, 9.0)], [(2.5, 5.5), (7.5, 5.5)]],
    "I": [[(3.5, 9.0), (7.0, 9.0)], [(5.25, 9.0), (5.25, 2.0)], [(3.5, 2.0), (7.0, 2.0)]],
    "J": [[(3.4, 9.0), (7.8, 9.0)],
          [(6.1, 9.0), (6.1, 4.2), (5.4, 2.7), (4.1, 2.0), (2.9, 2.6), (2.6, 3.8)]],
    "K": [[(3.0, 2.0), (3.0, 9.0)], [(7.6, 9.0), (3.0, 5.0)], [(3.0, 5.0), (7.8, 2.0)]],
    "L": [[(3.0, 9.0), (3.0, 2.0), (7.5, 2.0)]],
    "M": [[(2.5, 2.0), (2.5, 9.0), (5.25, 5.0), (8.0, 9.0), (8.0, 2.0)]],
    "N": [[(2.5, 2.0), (2.5, 9.0), (8.0, 2.0), (8.0, 9.0)]],
    "O": [[(5.5, 9.0), (7.8, 7.5), (8.0, 4.5), (6.0, 2.2),
           (4.0, 2.5), (2.8, 4.8), (3.5, 7.3), (5.5, 9.0)]],
    "P": [[(3.0, 2.0), (3.0, 9.0)],
          [(3.0, 9.0), (6.1, 9.0), (7.2, 8.4), (7.7, 7.4), (7.3, 6.3), (6.3, 5.5), (3.0, 5.4)]],
    "Q": [[(5.5, 9.0), (7.8, 7.5), (8.0, 4.5), (6.0, 2.2),
           (4.0, 2.5), (2.8, 4.8), (3.5, 7.3), (5.5, 9.0)],
          [(6.0, 3.8), (8.0, 2.0)]],
    "R": [[(3.0, 2.0), (3.0, 9.0)],
          [(3.0, 9.0), (6.1, 9.0), (7.2, 8.4), (7.7, 7.4), (7.3, 6.3), (6.3, 5.5), (3.0, 5.4)],
          [(5.2, 5.4), (7.8, 2.0)]],
    "S": [[(7.6, 8.2), (6.6, 8.9), (5.2, 9.0), (3.9, 8.4), (3.4, 7.3),
           (3.8, 6.2), (5.0, 5.6), (6.4, 5.2), (7.4, 4.4), (7.6, 3.2),
           (6.8, 2.3), (5.3, 2.0), (3.9, 2.3), (3.2, 3.1)]],
    "T": [[(2.5, 9.0), (8.0, 9.0)], [(5.25, 9.0), (5.25, 2.0)]],
    "U": [[(2.8, 9.0), (2.8, 4.4), (3.6, 2.7), (5.3, 2.0),
           (6.9, 2.6), (7.8, 4.2), (7.8, 9.0)]],
    "V": [[(2.5, 9.0), (5.25, 2.0), (8.0, 9.0)]],
    "W": [[(2.4, 9.0), (3.9, 2.0), (5.5, 6.4), (7.0, 2.0), (8.4, 9.0)]],
    "X": [[(2.5, 2.0), (8.0, 9.0)], [(2.5, 9.0), (8.0, 2.0)]],
    "Y": [[(2.8, 9.0), (5.5, 5.4), (8.2, 9.0)], [(5.5, 5.4), (5.5, 2.0)]],
    "Z": [[(2.5, 9.0), (8.0, 9.0), (2.5, 2.0), (8.0, 2.0)]],
}

# 小写字母：和大写共用同一把尺子，基线同样是 y = 2.0。
# x 高（a、c、e、o 的高度）到 y = 6.9，上升部（b、h、l）到 y = 9.0，
# 下降部（g、j、p、q、y）落到 y = 0.2。
RAW_LOWER_LETTERS = {
    "a": [[(6.6, 6.2), (5.6, 6.8), (4.3, 6.9), (3.2, 6.2), (2.7, 4.9),
           (2.8, 3.6), (3.5, 2.4), (4.7, 2.0), (5.9, 2.2), (6.6, 3.0)],
          [(6.6, 6.9), (6.6, 2.0)]],
    "b": [[(3.0, 9.0), (3.0, 2.0)],
          [(3.0, 6.9), (4.6, 6.9), (5.9, 6.2), (6.6, 4.9), (6.5, 3.4),
           (5.7, 2.3), (4.4, 2.0), (3.2, 2.2), (3.0, 2.6)]],
    "c": [[(6.6, 6.2), (5.6, 6.8), (4.3, 6.9), (3.2, 6.1), (2.7, 4.8),
           (2.8, 3.4), (3.5, 2.3), (4.7, 2.0), (5.8, 2.2), (6.6, 2.8)]],
    "d": [[(6.6, 6.2), (5.6, 6.8), (4.3, 6.9), (3.2, 6.2), (2.7, 4.9),
           (2.8, 3.6), (3.5, 2.4), (4.7, 2.0), (5.9, 2.2), (6.6, 3.0)],
          [(6.6, 9.0), (6.6, 2.0)]],
    "e": [[(2.7, 4.4), (6.6, 4.3), (6.4, 5.7), (5.5, 6.6), (4.3, 6.9),
           (3.2, 6.3), (2.7, 5.0), (2.8, 3.5), (3.6, 2.3), (4.9, 2.0), (6.2, 2.4)]],
    "f": [[(5.6, 9.0), (4.6, 9.0), (3.9, 8.3), (3.7, 7.0), (3.7, 2.0)],
          [(2.5, 5.9), (5.7, 5.9)]],
    "g": [[(6.6, 6.2), (5.6, 6.8), (4.3, 6.9), (3.2, 6.2), (2.7, 4.9),
           (2.8, 3.6), (3.5, 2.4), (4.7, 2.0), (5.9, 2.2), (6.6, 3.0), (6.6, 6.2)],
          [(6.6, 4.6), (6.6, 2.6), (6.4, 1.3), (5.7, 0.5), (4.6, 0.2), (3.5, 0.5), (2.9, 1.2)]],
    "h": [[(3.0, 9.0), (3.0, 2.0)],
          [(3.0, 5.9), (3.9, 6.7), (5.2, 6.9), (6.2, 6.2), (6.5, 5.0), (6.5, 2.0)]],
    "i": [[(4.1, 6.9), (4.1, 2.0)],
          [(4.1, 8.4), (4.1, 8.5)]],
    "j": [[(5.2, 6.9), (5.2, 1.0), (4.7, 0.3), (3.6, 0.2), (2.8, 0.8)],
          [(5.2, 8.4), (5.2, 8.5)]],
    "k": [[(3.0, 9.0), (3.0, 2.0)],
          [(6.4, 6.9), (3.0, 3.9)],
          [(3.0, 3.9), (6.6, 2.0)]],
    "l": [[(4.1, 9.0), (4.1, 2.0)]],
    "m": [[(2.6, 6.9), (2.6, 2.0)],
          [(2.6, 5.9), (3.3, 6.6), (4.2, 6.9), (4.9, 6.2), (5.1, 5.0), (5.1, 2.0)],
          [(5.1, 5.6), (5.9, 6.6), (6.9, 6.9), (7.7, 6.1), (7.9, 4.9), (7.9, 2.0)]],
    "n": [[(3.0, 6.9), (3.0, 2.0)],
          [(3.0, 5.9), (3.9, 6.7), (5.2, 6.9), (6.2, 6.2), (6.5, 5.0), (6.5, 2.0)]],
    "o": [[(4.6, 6.9), (6.0, 6.2), (6.6, 4.9), (6.5, 3.4), (5.6, 2.3),
           (4.5, 2.0), (3.4, 2.4), (2.8, 3.5), (2.8, 4.9), (3.5, 6.2), (4.6, 6.9)]],
    "p": [[(3.0, 6.9), (3.0, 0.2)],
          [(3.0, 6.4), (4.3, 6.9), (5.7, 6.6), (6.6, 5.5), (6.7, 4.1),
           (6.1, 2.9), (4.9, 2.3), (3.6, 2.4), (3.0, 2.9)]],
    "q": [[(6.6, 6.2), (5.6, 6.8), (4.3, 6.9), (3.2, 6.2), (2.7, 4.9),
           (2.8, 3.6), (3.5, 2.4), (4.7, 2.0), (5.9, 2.2), (6.6, 3.0)],
          [(6.6, 6.9), (6.6, 0.2)]],
    "r": [[(3.0, 6.9), (3.0, 2.0)],
          [(3.0, 5.4), (4.0, 6.5), (5.2, 6.9), (6.2, 6.6)]],
    "s": [[(6.3, 6.1), (5.6, 6.7), (4.5, 6.9), (3.5, 6.4), (3.2, 5.6),
           (3.6, 4.8), (4.5, 4.4), (5.5, 4.1), (6.2, 3.4), (6.2, 2.6),
           (5.4, 2.0), (4.2, 2.0), (3.2, 2.5)]],
    "t": [[(4.4, 8.2), (4.4, 3.0), (5.1, 2.1), (6.2, 2.1)],
          [(2.6, 6.3), (6.2, 6.3)]],
    "u": [[(3.0, 6.9), (3.0, 3.6), (3.7, 2.4), (4.9, 2.0), (6.0, 2.4), (6.6, 3.6), (6.6, 6.9)]],
    "v": [[(2.9, 6.9), (4.7, 2.0), (6.5, 6.9)]],
    "w": [[(2.4, 6.9), (3.7, 2.0), (4.9, 5.4), (6.1, 2.0), (7.4, 6.9)]],
    "x": [[(3.0, 6.9), (6.4, 2.0)], [(3.0, 2.0), (6.4, 6.9)]],
    "y": [[(3.0, 6.9), (4.8, 2.6)],
          [(6.6, 6.9), (4.6, 1.6), (4.0, 0.6), (3.2, 0.2), (2.6, 0.4)]],
    "z": [[(3.0, 6.9), (6.4, 6.9), (3.0, 2.0), (6.4, 2.0)]],
}

BASELINE = 2.0         # 笔画表里基线的纵坐标（字母底边所在的位置）
CAP_UNITS = 7.0        # 大写字母在笔画表里的高度，对应字号 1.0
DESCENDER_RATIO = 0.3  # 下降部（g、p、q、y）预留的高度，单位是字号


def _prepare(strokes):
    """按统一的尺子缩放：基线对齐到 y=0，字高归一化成 1.0。

    这里不能用每个字母自己的外框来缩放，否则小写字母的基线会对不齐。
    """
    xs = [x for stroke in strokes for x, _ in stroke]
    x_min = min(xs)
    return [[((x - x_min) / CAP_UNITS, (y - BASELINE) / CAP_UNITS) for x, y in stroke]
            for stroke in strokes]


GLYPHS = {}
for _table in (RAW_LETTERS, RAW_LOWER_LETTERS):
    for _ch, _strokes in _table.items():
        _normalized = _prepare(_strokes)
        _width = max(x for stroke in _normalized for x, _ in stroke)
        # i、l 这类竖线字母的宽度是 0，给它们留一点占位宽度
        GLYPHS[_ch] = (_normalized, max(_width, 0.35))


def layout_line(words):
    """排好一行里的字母，返回 [(字符, 左偏移), ...] 和这一行的总宽度（单位都是字高）。"""
    items = []
    x = 0.0
    for word_index, word in enumerate(words):
        if word_index:
            x += SPACE_RATIO
        for char_index, ch in enumerate(word):
            if char_index:
                x += GAP_RATIO
            items.append((ch, x))
            x += GLYPHS[ch][1]
    return items, x


def size_for(lines, avail_w, avail_h):
    """给定折行方式，算出能塞进画布的最大字高。"""
    max_units = 0.0
    for words in lines:
        _, width_units = layout_line(words)
        max_units = max(max_units, width_units)
    count = len(lines)
    scale_w = avail_w / max_units
    scale_h = avail_h / (1.0 + DESCENDER_RATIO + (count - 1) * (1.0 + LINE_RATIO))
    return min(scale_w, scale_h, MAX_SCALE)


def split_words(words, count):
    """把单词尽量平均地分到指定行数。"""
    total = sum(len(word) for word in words)
    target = float(total) / count
    lines, current, acc = [], [], 0
    for index, word in enumerate(words):
        current.append(word)
        acc += len(word)
        words_left = len(words) - index - 1
        lines_left = count - len(lines) - 1
        if lines_left > 0 and acc >= target and words_left >= lines_left:
            lines.append(current)
            current, acc = [], 0
    if current:
        lines.append(current)
    return lines


def plan_lines(text):
    """挑一种排版：一行放得下就用一行，否则折成两到三行，取字最大的那种。"""
    words = text.split()
    avail_w = CANVAS - 2 * MARGIN
    avail_h = CANVAS - 2 * MARGIN

    best_lines = [words]
    best_scale = size_for(best_lines, avail_w, avail_h)
    if best_scale < 1.6 and len(words) > 1:
        for count in range(2, MAX_LINES + 1):
            if count > len(words):
                break
            lines = split_words(words, count)
            scale = size_for(lines, avail_w, avail_h)
            if scale > best_scale:
                best_scale, best_lines = scale, lines
    return best_scale, best_lines


class LetterDrawer(object):
    def __init__(self):
        rospy.init_node("draw_text_node")
        self.pose = None
        self.speed = rospy.get_param("~speed", SPEED)
        rospy.Subscriber("/turtle1/pose", Pose, self.on_pose)
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        # launch 一键启动时 turtlesim 可能稍后就绪，先等服务上线
        rospy.wait_for_service("/turtle1/set_pen")
        rospy.wait_for_service("/clear")
        self.set_pen_srv = rospy.ServiceProxy("/turtle1/set_pen", SetPen)
        self.clear_srv = rospy.ServiceProxy("/clear", Empty)
        # 等发布器与订阅器建立连接，否则最早的几条指令会丢掉
        while self.pub.get_num_connections() == 0 and not rospy.is_shutdown():
            time.sleep(0.1)

    def on_pose(self, msg):
        self.pose = msg

    @staticmethod
    def wrap(angle):
        """把角度归一化到 [-pi, pi]。"""
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def stop(self):
        self.pub.publish(Twist())

    def set_pen(self, down, scale=2.0):
        """down=True 落笔，down=False 抬笔；线宽随字号变化。"""
        width = int(max(2, round(PEN_RATIO * scale)))
        self.set_pen_srv(255, 220, 0, width, 0 if down else 1)
        time.sleep(0.05)

    def clear_canvas(self):
        try:
            self.clear_srv()
        except rospy.ServiceException as exc:
            rospy.logwarn("清空画布失败: %s", exc)
        time.sleep(0.3)

    def goto(self, x, y, scale, timeout=10.0):
        """闭环控制走到 (x, y)：先对准方向再直着走，线条才是直的。"""
        tol = max(0.03, 0.04 * scale)
        v_max = self.speed * max(0.6, 1.2 * scale)
        w_max = 3.5
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.pose is None:
                time.sleep(0.01)
                continue
            dx, dy = x - self.pose.x, y - self.pose.y
            dist = math.hypot(dx, dy)
            if dist < tol:
                break
            err = self.wrap(math.atan2(dy, dx) - self.pose.theta)
            cmd = Twist()
            cmd.angular.z = max(-w_max, min(w_max, 3.5 * err))
            if abs(err) < 0.15:                # 对准方向后才前进
                cmd.linear.x = max(0.25, min(v_max, 2.0 * dist))
            self.pub.publish(cmd)
            time.sleep(0.01)
        self.pub.publish(Twist())
        time.sleep(0.03)

    def draw_text(self, text):
        started = time.time()
        scale, lines = plan_lines(text)
        if scale < 0.7:
            rospy.logwarn("内容偏长，字会比较小")
        rospy.loginfo("开始绘制「%s」：字高 %.2f，%d 行", text, scale, len(lines))

        line_h = scale * (1.0 + LINE_RATIO)
        block_h = scale * (1.0 + DESCENDER_RATIO) + (len(lines) - 1) * line_h
        y_top = MARGIN + ((CANVAS - 2 * MARGIN) + block_h) / 2.0

        self.clear_canvas()
        for index, words in enumerate(lines):
            items, width_units = layout_line(words)
            x_left = (CANVAS - width_units * scale) / 2.0
            y_baseline = y_top - scale - index * line_h
            for ch, offset in items:
                for stroke in GLYPHS[ch][0]:
                    self.set_pen(down=False, scale=scale)
                    self.goto(x_left + (offset + stroke[0][0]) * scale,
                              y_baseline + stroke[0][1] * scale, scale)
                    self.set_pen(down=True, scale=scale)
                    for px, py in stroke[1:]:
                        self.goto(x_left + (offset + px) * scale,
                                  y_baseline + py * scale, scale)
        self.set_pen(down=False, scale=scale)
        self.stop()
        rospy.loginfo("绘制完成，用时 %.1f 秒", time.time() - started)


def main():
    drawer = LetterDrawer()

    # 参数模式：roslaunch ... text:="Hello World"，画完自动退出
    preset = rospy.get_param("~text", "")
    if preset:
        drawer.draw_text(preset)
        return

    # 交互模式：在终端里反复输入单词或短句
    print("输入英文单词或短句，回车后小海龟会把它们画出来；输入 q 退出。")
    print("大小写会按原样绘制（目前定义了 A-Z 与 a-z，数字和符号会被跳过）。")
    try:
        while not rospy.is_shutdown():
            text = input("文字> ").strip()
            if text.lower() in ("q", "quit", ""):
                break
            unknown = sorted(set(ch for ch in text if ch.isalpha() and ch not in GLYPHS))
            drawable = "".join(ch for ch in text if ch in GLYPHS or ch == " ")
            if not drawable.strip():
                print("没有可以画的字符，试试英文单词。")
                continue
            if unknown:
                print("这些字符没有字形，已跳过：" + " ".join(unknown))
            drawer.draw_text(drawable)
            print("完成：" + drawable.strip())
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        drawer.stop()


if __name__ == "__main__":
    main()