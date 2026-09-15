#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
空域载具八叉树三维寻路 —— 主入口节点

本节点用于验证整个模块的运行环境与链路是否就绪：
  1. 输出运行环境信息（ROS 发行版、Python 版本、Gazebo 版本、octomap 版本）
  2. 订阅 /clock，确认 Gazebo 与 ROS 的仿真时钟桥接正常
  3. 调用 Gazebo 的 /gazebo/get_world_properties 服务，确认仿真器在线
  4. 发布心跳话题 /uav_status，确认节点间通信正常
  5. 运行指定时长后自动退出，便于在终端中直接观察完整输出

运行：
    roslaunch octree_uav_3d_pathfinding main.launch
"""

import os
import subprocess
import sys

import rospy
from gazebo_msgs.srv import GetWorldProperties
from std_msgs.msg import String

MODULE_NAME = 'octree_uav_3d_pathfinding'
MODULE_VERSION = '0.1.0'
TOPIC_STATUS = '/uav_status'


def read_octomap_version(distro):
    """读取已安装的 octomap 版本。"""
    try:
        import octomap
        return getattr(octomap, '__version__', 'unknown')
    except ImportError:
        pass

    path = os.path.join('/opt/ros', distro, 'share', 'octomap', 'package.xml')
    try:
        import xml.etree.ElementTree as ET
        tree = ET.parse(path)
        node = tree.getroot().find('version')
        if node is not None and node.text:
            return node.text.strip()
    except Exception:
        pass

    return '未检测到'


def read_gazebo_version():
    """读取 Gazebo 版本。"""
    try:
        out = subprocess.check_output(
            ['gazebo', '--version'], stderr=subprocess.STDOUT
        ).decode('utf-8', errors='replace').strip().splitlines()
        return out[1].strip() if len(out) > 1 else out[0].strip()
    except Exception:
        return '未检测到'


class ClockMonitor(object):
    """订阅 /clock，统计仿真时钟的发布频率。"""

    def __init__(self):
        self.count = 0
        self.last_wall = None
        self.first_wall = None
        self.rate = 0.0
        self.sub = rospy.Subscriber('/clock', rospy.AnyMsg,
                                    self._callback, queue_size=100)

    def _callback(self, _msg):
        now = rospy.get_time()
        self.count += 1
        self.last_wall = now
        if self.first_wall is None:
            self.first_wall = now

    @property
    def received(self):
        return self.count > 0

    def compute_rate(self):
        if self.count > 1 and self.first_wall is not None:
            elapsed = self.last_wall - self.first_wall
            if elapsed > 0:
                self.rate = (self.count - 1) / elapsed
        return self.rate

    def reset_stats(self):
        self.count = 0
        self.last_wall = None
        self.first_wall = None
        self.rate = 0.0


def print_environment():
    distro = os.environ.get('ROS_DISTRO', 'unknown')
    rospy.loginfo('=' * 60)
    rospy.loginfo('模块: %s  v%s', MODULE_NAME, MODULE_VERSION)
    rospy.loginfo('=' * 60)
    rospy.loginfo('ROS 发行版      : %s', distro)
    rospy.loginfo('Python 版本     : %s', sys.version.split()[0])
    rospy.loginfo('Gazebo 版本     : %s', read_gazebo_version())
    rospy.loginfo('octomap 版本    : %s', read_octomap_version(distro))
    rospy.loginfo('节点名          : %s', rospy.get_name())
    rospy.loginfo('-' * 60)


def wait_for_clock(monitor, timeout):
    start = rospy.get_time()
    while not monitor.received:
        if rospy.get_time() - start > timeout:
            rospy.logerr('[1/3] /clock 等待超时（%.1f 秒无数据）', timeout)
            rospy.logerr('      可能原因：Gazebo 未启动，或未通过 roslaunch 启动')
            return False
        rospy.sleep(0.2)
    rospy.loginfo('[1/3] 仿真时钟桥接正常：已收到 %d 帧 /clock 数据', monitor.count)
    return True


def check_gazebo_service(timeout):
    service = '/gazebo/get_world_properties'
    try:
        rospy.wait_for_service(service, timeout=timeout)
    except rospy.ROSException:
        rospy.logerr('[2/3] Gazebo 服务不可用：%s', service)
        return False
    try:
        proxy = rospy.ServiceProxy(service, GetWorldProperties)
        props = proxy()
        rospy.loginfo('[2/3] Gazebo 服务正常：当前世界包含 %d 个模型',
                      len(props.model_names))
        if props.model_names:
            rospy.loginfo('      模型列表: %s', ', '.join(props.model_names))
        rospy.loginfo('      仿真时间: %.3f 秒', props.sim_time)
        return True
    except rospy.ServiceException as exc:
        rospy.logerr('[2/3] Gazebo 服务调用失败: %s', exc)
        return False


def check_heartbeat(pub, topic):
    msg = String()
    msg.data = '{} v{} alive'.format(MODULE_NAME, MODULE_VERSION)
    pub.publish(msg)
    rospy.loginfo('[3/3] 心跳话题正常：已向 %s 发布消息', topic)
    return True


def main():
    rospy.init_node('uav_env_check', anonymous=False)

    check_duration = rospy.get_param('~check_duration', 10.0)
    clock_timeout = rospy.get_param('~clock_timeout', 30.0)
    service_timeout = rospy.get_param('~service_timeout', 30.0)
    pub_topic = rospy.get_param('~status_topic', TOPIC_STATUS)

    print_environment()

    monitor = ClockMonitor()
    status_pub = rospy.Publisher(pub_topic, String, queue_size=10)

    results = []
    results.append(('仿真时钟桥接', wait_for_clock(monitor, clock_timeout)))
    results.append(('Gazebo 服务', check_gazebo_service(service_timeout)))
    results.append(('话题通信', check_heartbeat(status_pub, pub_topic)))

    monitor.reset_stats()
    start = rospy.get_time()
    rate = rospy.Rate(2)
    while not rospy.is_shutdown() and rospy.get_time() - start < check_duration:
        msg = String()
        msg.data = '{} alive | sim={:.3f}s | clock={}'.format(
            MODULE_NAME, rospy.get_time(), monitor.count)
        status_pub.publish(msg)
        rospy.loginfo('运行中... 仿真时间 %.3fs，已收 %d 帧 /clock，订阅者 %d 个',
                      rospy.get_time(), monitor.count,
                      status_pub.get_num_connections())
        rate.sleep()

    clock_rate = monitor.compute_rate()

    rospy.loginfo('-' * 60)
    rospy.loginfo('检查结果：')
    all_ok = True
    for name, ok in results:
        rospy.loginfo('  [%s] %s', '通过' if ok else '失败', name)
        all_ok = all_ok and ok
    rospy.loginfo('  /clock 平均频率: %.1f Hz（累计 %d 帧）', clock_rate, monitor.count)
    rospy.loginfo('-' * 60)

    if all_ok:
        rospy.loginfo('环境自检全部通过，模块可以开始后续开发。')
    else:
        rospy.logwarn('存在未通过的检查项，请根据上方提示排查。')

    rospy.loginfo('节点退出。')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
