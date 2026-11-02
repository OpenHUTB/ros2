#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
键盘控制 Panda 机械臂节点（任务②）。

按键说明：
  1-7  分别控制 7 个关节转动
  r    复位到初始姿态
  q    退出程序
"""
import sys
import tty
import termios
import rospy
import moveit_commander


class PandaKeyboardControl(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('panda_keyboard_control', anonymous=True)

        self.group = moveit_commander.MoveGroupCommander("panda_arm")
        self.group.set_max_velocity_scaling_factor(0.3)
        self.group.set_max_acceleration_scaling_factor(0.3)
        self.joint_step = 0.1

        rospy.loginfo("已连接到 move_group，规划组: %s", self.group.get_name())

    def _get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key

    def _move_joint(self, joint_index, delta):
        joint_values = self.group.get_current_joint_values()
        joint_values[joint_index] += delta
        self.group.set_joint_value_target(joint_values)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        if plan:
            rospy.loginfo("关节 %d 移动成功", joint_index + 1)
        else:
            rospy.logwarn("关节 %d 规划失败，请检查限位", joint_index + 1)

    def _reset(self):
        rospy.loginfo("正在复位到初始姿态...")
        self.group.set_named_target("ready")
        self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

    def run(self):
        rospy.loginfo("1-7: 控制对应关节转动 | r: 复位 | q: 退出")
        try:
            while not rospy.is_shutdown():
                key = self._get_key()
                if key == 'q':
                    break
                elif key == 'r':
                    self._reset()
                elif key in [str(i) for i in range(1, 8)]:
                    self._move_joint(int(key) - 1, self.joint_step)
        except Exception as e:
            rospy.logerr("运行异常: %s", str(e))
        finally:
            self.group.stop()
            self.group.clear_pose_targets()
            moveit_commander.roscpp_shutdown()
            rospy.loginfo("程序已安全退出")


if __name__ == '__main__':
    try:
        PandaKeyboardControl().run()
    except rospy.ROSInterruptException:
        pass
