#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ros_scenario_runner 的 ROS 节点。

职责很窄：按时间轴把场景里的指令发到 /cmd_vel，把车辆状态记成轨迹，
跑完后逐条判定断言，把报告打到日志里并发布到 /scenario/report。
所有判决逻辑都在 scenario_core.py 里，方便离线回归。
"""

from __future__ import division, print_function, unicode_literals

import os
import sys
import threading
import traceback

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import scenario_core as core

_IS_PY2 = sys.version_info[0] < 3


def _log(level, message):
    """打日志，并且避开 Melodic 的一个坑。

    rosgraph 的日志格式化里有这样一句：msg.replace('${message}', str(record_message))，
    如果 record_message 是含非 ASCII 的 unicode，Python 2 的 str() 会按 ASCII 编码而抛
    UnicodeEncodeError。所以先把消息统一转成 byte str（Python 3 下本来就是 str）。
    """
    text = message
    if _IS_PY2 and isinstance(text, unicode):  # noqa: F821
        text = text.encode("utf-8")
    getattr(rospy, level)(text)


class ScenarioRunner(object):
    """一次场景运行的驱动器。"""

    def __init__(self):
        self.params = core.RunnerParams.from_params(rospy.get_param)
        self.scenario = core.load_scenario(self.params.scenario_file)
        self.profile = self.params.profile()

        self.pose = core.Pose(
            self.scenario.initial.x, self.scenario.initial.y, self.scenario.initial.yaw
        )
        self.speed = self.scenario.initial_speed

        self.lock = threading.Lock()
        self.samples = [
            core.Sample(0.0, self.pose.x, self.pose.y, self.pose.yaw, self.speed)
        ]
        self.started = None
        self.finished = False

        self.cmd_pub = rospy.Publisher(
            self.params.cmd_vel_topic, Twist, queue_size=10
        )
        self.report_pub = rospy.Publisher(
            self.params.report_topic, String, queue_size=1, latch=True
        )

        if self.params.use_mock_odom:
            self.odom_pub = rospy.Publisher(
                self.params.odom_topic, Odometry, queue_size=10
            )
            self.odom_sub = None
            _log(
                "loginfo",
                "内置车辆模型：轴距无关的二维运动学，最高 %.2f m/s，加速度 %.2f，减速度 %.2f"
                % (self.profile.max_speed, self.profile.max_accel, self.profile.max_decel),
            )
        else:
            self.odom_pub = None
            self.odom_sub = rospy.Subscriber(
                self.params.odom_topic, Odometry, self.on_odom, queue_size=50
            )
            _log("loginfo", "外部车辆模式：从 %s 记录轨迹" % (self.params.odom_topic,))

        _log(
            "loginfo",
            "场景 %s 已载入：时长 %.1f s，%d 条步骤，%d 条断言"
            % (
                self.scenario.name,
                self.scenario.duration,
                len(self.scenario.steps),
                len(self.scenario.expects),
            ),
        )
        rospy.on_shutdown(self.on_shutdown)

    # ------------------------------------------------------------ 基础动作

    def publish_command(self, command):
        twist = Twist()
        twist.linear.x = command.linear
        twist.angular.z = command.angular
        self.cmd_pub.publish(twist)

    def publish_odom(self):
        if self.odom_pub is None:
            return
        message = Odometry()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "map"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = self.pose.x
        message.pose.pose.position.y = self.pose.y
        quaternion = core.yaw_to_quaternion(self.pose.yaw)
        message.pose.pose.orientation.x = quaternion[0]
        message.pose.pose.orientation.y = quaternion[1]
        message.pose.pose.orientation.z = quaternion[2]
        message.pose.pose.orientation.w = quaternion[3]
        message.twist.twist.linear.x = self.speed
        self.odom_pub.publish(message)

    def elapsed(self):
        if self.started is None:
            return 0.0
        return rospy.get_time() - self.started

    def on_odom(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = core.quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        with self.lock:
            self.samples.append(
                core.Sample(
                    self.elapsed(),
                    position.x,
                    position.y,
                    yaw,
                    message.twist.twist.linear.x,
                )
            )

    # ------------------------------------------------------------ 主流程

    def run(self):
        self.started = rospy.get_time()
        period = 1.0 / self.params.publish_hz
        sub_steps = max(1, int(round(period / self.params.sim_dt)))
        rate = rospy.Rate(self.params.publish_hz)
        _log("loginfo", "开始执行场景 %s" % (self.scenario.name,))
        while not rospy.is_shutdown():
            elapsed = self.elapsed()
            if elapsed >= self.scenario.duration:
                break
            command = self.scenario.command_at(elapsed)
            self.publish_command(command)
            if self.params.use_mock_odom:
                with self.lock:
                    step = period / sub_steps
                    for _ in range(sub_steps):
                        self.speed = core.advance(
                            self.pose, self.speed, command, step, self.profile
                        )
                    self.samples.append(
                        core.Sample(
                            elapsed, self.pose.x, self.pose.y, self.pose.yaw, self.speed
                        )
                    )
                self.publish_odom()
            rate.sleep()
        if rospy.is_shutdown():
            _log("logwarn", "场景 %s 被提前中断，报告不计入结论" % (self.scenario.name,))
            return
        self.finish()

    def finish(self):
        if self.finished:
            return
        self.finished = True
        self.publish_command(core.Command(0.0, 0.0))
        with self.lock:
            samples = list(self.samples)
        trajectory = core.Trajectory(samples, self.scenario.name)
        results = core.evaluate(trajectory, self.scenario.expects)
        report = core.format_report(
            self.scenario,
            results,
            header="里程 %.2f m，峰值 %.2f m/s，采样 %d 个"
            % (trajectory.travelled(), trajectory.peak_speed(), len(samples)),
        )
        for line in report.splitlines():
            _log("loginfo", line)
        self.report_pub.publish(String(data=core.to_message_text(report)))
        _log(
            "loginfo",
            "场景 %s 结论：%s（%d/%d 条断言通过）"
            % (
                self.scenario.name,
                "PASSED" if core.all_passed(results) else "FAILED",
                sum(1 for result in results if result.passed),
                len(results),
            ),
        )

    def on_shutdown(self):
        if not self.finished:
            _log("logwarn", "收到退出信号，发布零控制量")
        self.publish_zero()

    def publish_zero(self):
        """退出发零速，连发两次并吞掉关闭阶段的异常。"""
        for _ in range(2):
            try:
                self.publish_command(core.Command(0.0, 0.0))
            except Exception as error:  # noqa: BLE001
                _log("logwarn", "发布零控制量失败：%s" % (error,))
                return
            try:
                rospy.sleep(0.05)
            except Exception:  # noqa: BLE001
                return


def main():
    rospy.init_node("ros_scenario_runner")
    try:
        runner = ScenarioRunner()
    except core.ScenarioError as error:
        _log("logerr", "场景加载失败：%s" % (error,))
        return 1
    try:
        runner.run()
    except Exception:  # noqa: BLE001
        _log("logerr", "场景执行异常：\n%s" % (traceback.format_exc(),))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
