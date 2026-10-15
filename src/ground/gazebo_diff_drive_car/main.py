#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py —— OpenHUTB 无人车任务统一入口节点
============================================

【任务书对应】第 (1) 项「物理仿真 + 键盘运动控制」与第 (2) 项「感知→运动控制闭环」
            的调度入口。任务书要求「每个模块必须支持 launch 启动，入口为 main.xxx」，
            因此本节点作为 launch 文件启动的顶层节点，通过模式切换统一管理
            各个功能子模块。

【设计思路】
    采用「模式机 (Mode Machine)」架构：main.py 本身不做具体算法，只负责
      1) 读取 launch 传入的 mode 参数；
      2) 根据 mode 动态实例化对应的算法节点类；
      3) 统一处理安全兜底（急停、超时停车）。

    这样做的好处是：所有功能共享同一套底盘接口与安全策略，
    切换功能只需改变 launch 参数，无需改动代码。

【运行模式】
    teleop  : 键盘遥控        —— 由 teleop_keyboard.py 提供
    laser   : 激光雷达避障    —— 由 laser_avoid.py 提供
    camera  : 深度相机避障    —— 由 depth_camera_avoid.py 提供
    idle    : 空闲（仅加载模型，不启动任何控制算法）

【作者】OpenHUTB Car Team
【平台】ROS Noetic + Gazebo 11 + Ubuntu 20.04
"""

from __future__ import print_function

import os
import sys
import time
import threading

import rospy
from geometry_msgs.msg import Twist

# ----------------------------------------------------------------------------
# 模块搜索路径加固
# ----------------------------------------------------------------------------
# 本模块的目录布局遵循 OpenHUTB/ros2 仓库约定：
#
#     gazebo_diff_drive_car/
#     ├── main.py                    ★ 本文件（统一入口，位于包根目录）
#     ├── launch/main.launch
#     └── scripts/                   ← 各功能算法节点
#         ├── teleop_keyboard.py
#         ├── laser_avoid.py
#         ├── depth_camera_avoid.py
#         └── odom_recorder.py
#
# 因此导入算法节点时需要把 scripts/ 目录加入模块搜索路径。
# 兼容两种运行场景：
#   1) 直接从源码目录运行（rosrun / roslaunch 指定脚本路径）
#   2) catkin 安装后在 devel/lib/<pkg>/ 下运行（脚本被平铺复制）
# 以下逻辑遍历若干候选目录，把实际存在的那一个加入 sys.path。
_HERE = os.path.dirname(os.path.abspath(__file__))

_CANDIDATE_DIRS = [
    os.path.join(_HERE, "scripts"),      # 源码布局：main.py 同级有 scripts/
    _HERE,                               # 安装布局：所有脚本被复制到同一目录
    os.path.join(_HERE, "..", "scripts"),
]

for _d in _CANDIDATE_DIRS:
    _d = os.path.abspath(_d)
    if os.path.isdir(_d) and _d not in sys.path:
        sys.path.insert(0, _d)


# ============================================================================
# 常量定义
# ============================================================================

NODE_NAME = "openhutb_car_main"

# 控制指令发布频率 (Hz)。与 gazebo_ros_diff_drive 的 updateRate 保持一致，
# 保证底盘控制器能持续收到指令，避免因 cmd_vel_timeout 而停车。
CONTROL_RATE = 20.0

# 指令超时时间 (s)。若超过该时间未收到新的速度指令，则认为控制链路中断，
# 主动发布零速度使车辆停止，这是无人车最基本的安全机制。
COMMAND_TIMEOUT = 1.0


# ============================================================================
# 安全层：速度指令看门狗
# ============================================================================

class VelocityWatchdog(object):
    """速度指令看门狗。

    无人车在真实与仿真环境中都必须具备「失联停车」能力。本类监控最近一次
    速度指令的时间戳，一旦发现超过 COMMAND_TIMEOUT 未更新，立即强制发送
    零速度指令。

    这是任务书「控制策略安全性」要求的体现：即使上层算法节点崩溃或卡死，
    车辆也不会失控持续运动。
    """

    def __init__(self, publisher, timeout=COMMAND_TIMEOUT):
        self._pub = publisher
        self._timeout = timeout
        self._last_cmd_time = rospy.Time.now()
        self._last_cmd = Twist()
        self._lock = threading.Lock()
        self._stopped = False

    def feed(self, twist_msg):
        """喂入一条新的速度指令：刷新计时并立即发布。

        注意：此处必须真正 publish，否则算法计算出的速度只停留在本对象内部，
        底盘控制器永远收不到指令（车辆静止不动）。
        """
        with self._lock:
            self._last_cmd = twist_msg
            self._last_cmd_time = rospy.Time.now()
            self._stopped = False
        self._pub.publish(twist_msg)

    def check(self):
        """周期性调用。返回 True 表示链路正常，False 表示已触发停车。"""
        with self._lock:
            if self._stopped:
                return False
            elapsed = (rospy.Time.now() - self._last_cmd_time).to_sec()
            if elapsed > self._timeout:
                # 超时：强制停车
                self._pub.publish(Twist())
                self._stopped = True
                rospy.logwarn(
                    "[%s] 速度指令超时 %.2fs，已触发安全停车",
                    NODE_NAME, elapsed
                )
                return False
            return True

    def last_command(self):
        with self._lock:
            return self._last_cmd


# ============================================================================
# 模式调度器
# ============================================================================

class ModeDispatcher(object):
    """功能模式调度器。

    负责根据 launch 参数实例化不同的算法节点。所有算法节点遵循统一接口：
        - 构造时接收 (cmd_publisher, watchdog)
        - 提供 start() / stop() 方法
        - 提供 update() 方法供主循环以固定频率调用
    """

    #: 支持的运行模式及说明
    SUPPORTED_MODES = {
        "teleop": "键盘遥控模式（物理仿真 + 键盘运动控制）",
        "laser":  "激光雷达避障模式（感知→运动控制闭环）",
        "camera": "深度相机避障模式（感知→运动控制闭环）",
        "idle":   "空闲模式（仅加载模型，不启动控制算法）",
    }

    def __init__(self, mode):
        if mode not in self.SUPPORTED_MODES:
            rospy.logfatal(
                "[%s] 未知模式 '%s'，支持的模式：%s",
                NODE_NAME, mode, ", ".join(self.SUPPORTED_MODES.keys())
            )
            sys.exit(1)

        self.mode = mode
        self.node = None

        # 控制指令发布者：所有算法最终都通过它输出速度
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # 安全看门狗
        self.watchdog = VelocityWatchdog(self.cmd_pub)

        rospy.loginfo("[%s] 模式：%s —— %s",
                      NODE_NAME, mode, self.SUPPORTED_MODES[mode])

        self._create_node()

    def _create_node(self):
        """根据模式创建对应的算法节点实例。"""
        if self.mode == "idle":
            return

        # 延迟导入：避免未使用模式引入不必要的依赖，
        # 同时保证即使某个模块出错也不影响其他模式的启动。
        try:
            if self.mode == "teleop":
                from teleop_keyboard import TeleopKeyboardNode
                self.node = TeleopKeyboardNode(self.watchdog)
            elif self.mode == "laser":
                from laser_avoid import LaserAvoidNode
                self.node = LaserAvoidNode(self.cmd_pub, self.watchdog)
            elif self.mode == "camera":
                from depth_camera_avoid import DepthCameraAvoidNode
                self.node = DepthCameraAvoidNode(self.cmd_pub, self.watchdog)
        except ImportError as exc:
            rospy.logfatal("[%s] 模块导入失败：%s", NODE_NAME, exc)
            rospy.logfatal("[%s] 请确认已 source catkin 工作区的 setup.bash",
                           NODE_NAME)
            sys.exit(1)

    def spin(self):
        """主循环：以固定频率驱动算法节点，并执行安全监控。

        实现说明：
          不使用 rospy.Rate，因为它以 rospy.Time.now() 为基准，而在
          use_sim_time=True 且 /clock 尚未发布时，rospy.Time.now() 可能返回 0
          或停滞，导致 Rate.sleep() 长时间阻塞，主循环无法推进。
          此处改用墙钟时间自行计算睡眠时长，保证循环稳定运行，
          同时通过 /clock 获取仿真时间仅用于算法内部计时。
        """
        period = 1.0 / CONTROL_RATE

        if self.node is not None:
            self.node.start()
            rospy.loginfo("[%s] 算法节点已启动（控制频率 %.0f Hz）",
                          NODE_NAME, CONTROL_RATE)

        ticks = 0
        while not rospy.is_shutdown():
            t_begin = time.time()

            if self.node is not None:
                try:
                    self.node.update()
                except Exception as exc:  # noqa: BLE001
                    # 算法异常不应导致整车失控，记录后停车
                    rospy.logerr("[%s] 算法更新异常：%s", NODE_NAME, exc)
                    self.cmd_pub.publish(Twist())
                    time.sleep(0.5)
                    continue

            self.watchdog.check()
            ticks += 1

            # 每 5 秒打印一次心跳，便于确认控制回路持续运行
            if ticks % int(CONTROL_RATE * 5) == 0:
                rospy.loginfo("[%s] 控制回路运行中：已执行 %d 次循环",
                              NODE_NAME, ticks)

            # 补偿本次循环耗时，保持稳定频率
            elapsed = time.time() - t_begin
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        # 退出前确保停车，避免仿真中车辆继续滑行
        if self.node is not None:
            self.node.stop()
        self.cmd_pub.publish(Twist())
        rospy.loginfo("[%s] 已退出，车辆已停止", NODE_NAME)


# ============================================================================
# 程序入口
# ============================================================================

def main():
    rospy.init_node(NODE_NAME, anonymous=False)

    mode = rospy.get_param("~mode", "idle")
    rospy.loginfo("=" * 62)
    rospy.loginfo(" OpenHUTB 无人车 —— 统一入口节点")
    rospy.loginfo(" 平台：ROS Noetic / Gazebo 11")
    rospy.loginfo("=" * 62)

    dispatcher = ModeDispatcher(mode)

    try:
        dispatcher.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
